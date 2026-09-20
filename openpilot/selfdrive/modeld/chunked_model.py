"""Chunked eGPU big model artifacts: the format the repository compiler produces.

`compile_modeld.py` emits a split-format pickle (`run_policy` + one warp jit per camera
resolution) and `common/file_chunker` splits it into fixed-size pieces next to the path
modeld already loads:

    models/big_driving_<sha16>_tinygrad.pkl.chunk01of17 ... chunk17of17
    models/big_driving_<sha16>_tinygrad.pkl.chunkmanifest        (contains "17")

That is exactly what the local SCons build writes into MODELS_DIR, so nothing in the
runtime path has to change: `active_usbgpu_compiled_path()` keys off the manifest and
`open_file_chunked()` reassembles the stream. Downloading the same files instead of
compiling them locally is what lets a device get the fast (warp-on-QCOM) model without
holding the eGPU for ~20 minutes.

The server advertises the set in the model catalog, next to the precompiled artifacts:

    "chunked": {"url": "chunked/big_driving_<sha16>_tinygrad.pkl",
                "chunks": [{"size": 47185920, "sha256": "..."}, ...],
                "size": 787632259, "sha256": "..."}

Writing the manifest is the commit point: until it exists the artifact counts as absent,
so an interrupted download is never used and simply resumes (each piece keeps its own
`.part` file through `precompiled_model.download`).

This module deliberately keeps every knob in itself so existing eGPU code stays untouched:

* the delivery preference is a small JSON file in the model cache dir, not a Params key
  (a new key would need params_keys.h and a native rebuild);
* preferring the chunked delivery over an already installed precompiled artifact uses the
  existing `rejected` marker (`precompiled_model.installed()` returns None while it
  exists), so `helpers.active_usbgpu_compiled_path()` keeps its original logic;
* the catalog's optional "chunked" section is validated here, leaving
  `precompiled_model.validate_catalog()` as it is.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld.big_model import active_manifest, model_cache_dir
from openpilot.selfdrive.modeld.big_model_status import STATUS_FILENAME, write_big_model_status
from openpilot.selfdrive.modeld.firmware import ensure_firmware
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, usbgpu_present
from openpilot.selfdrive.modeld.precompiled_model import MAX_CATALOG, download, validate_catalog

BIG_MODEL_STAMP = '.big_model_build_stamp'
SOURCE_FILE = 'model_source.json'
SOURCES = ('auto', 'chunked', 'precompiled')
LOAD_TIMEOUT = 180.0
# The background downloader is started right after the boot build, which can be before
# DNS works. A failed delivery is not retried until the next boot, so wait for the host
# and retry transient errors instead of dropping the whole download.
NETWORK_WAIT_SECONDS = 120.0
DOWNLOAD_ATTEMPTS = 3
DOWNLOAD_RETRY_DELAY = 5.0
USER_AGENT = 'carrot-chunked/1'
SHA256 = re.compile(r'^[0-9a-f]{64}$')


class ChunkLoadError(RuntimeError):
  """The artifact is on disk but does not load (bad content, or a transient eGPU issue).

  Kept distinct from a download/content problem: the pieces are already verified, so the
  caller keeps them and simply retries later instead of re-fetching 787 MB.
  """

_DOWNLOADER_MARKER = '--ensure-if-egpu'


def _background_downloader_running() -> bool:
  """Is the background big-model downloader alive?

  Mirrors the check in selfdrive/ui/big_model_badge.py rather than importing it (a UI
  module has no business in modeld). See auto_install for why it matters.
  """
  try:
    for entry in os.listdir('/proc'):
      if not entry.isdigit():
        continue
      try:
        with open(f'/proc/{entry}/cmdline', 'rb') as f:
          if _DOWNLOADER_MARKER in f.read().decode('utf-8', 'ignore'):
            return True
      except OSError:
        continue
  except OSError:
    pass
  return False


_LOAD_SNIPPET = (
  'import sys;'
  'sys.path.insert(0, sys.argv[2]);'
  # Importing compile_modeld is not optional: modeld does the same, and it is what applies
  # the tinygrad patches the pickle needs (firmware search paths for the USB GPU, Buffer
  # reduce, the pkl\'s own classes).
  'import openpilot.selfdrive.modeld.compile_modeld;'
  'from openpilot.common.file_chunker import open_file_chunked;'
  'from openpilot.selfdrive.modeld.helpers import load_oob;'
  'f = open_file_chunked(sys.argv[1]);'
  'jits = load_oob(f);'
  'print("\\n".join(sorted(str(k) for k in jits)))'
)


# --------------------------------------------------------------- delivery preference

def model_source_file(directory: Path | None = None) -> Path:
  return (directory or model_cache_dir()) / SOURCE_FILE


def read_model_source(directory: Path | None = None) -> str:
  """'auto' (default), 'chunked' or 'precompiled'. Never raises."""
  try:
    value = json.loads(model_source_file(directory).read_text(encoding='utf-8'))
    source = value.get('source')
    if isinstance(source, str) and source in SOURCES:
      return source
  except (OSError, ValueError):
    pass
  return 'auto'


def write_model_source(source: str, directory: Path | None = None) -> str:
  if source not in SOURCES:
    raise ValueError('source must be one of ' + ', '.join(SOURCES))
  path = model_source_file(directory)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + '.tmp')
  temporary.write_text(json.dumps({'source': source}, indent=2) + '\n', encoding='utf-8')
  os.replace(temporary, path)
  return source


# --------------------------------------------------------------------- artifact paths

def _pkl_path(model_sha256: str, directory: Path | None = None) -> Path:
  # must stay in sync with helpers.modeld_pkl_path(usbgpu=True, model_sha256=...)
  return (directory or MODELS_DIR) / f'big_driving_{model_sha256[:16]}_tinygrad.pkl'


def chunked_pkl_path(model=None, directory: Path | None = None) -> Path | None:
  model = model or active_manifest()
  if model is None:
    return None
  return _pkl_path(model.sha256, directory)


def installed_chunked(model=None, directory: Path | None = None) -> Path | None:
  path = chunked_pkl_path(model, directory)
  if path is None:
    return None
  manifest = Path(get_manifest_path(str(path)))
  if not manifest.is_file():
    return None
  # A manifest alone is not proof: the pieces around it can be deleted or truncated
  # (manual cleanup, an interrupted copy). Counting them here keeps "installed" true
  # only when open_file_chunked() can actually reassemble the stream.
  try:
    count = int(manifest.read_text().strip())
  except ValueError:
    return None
  for index in range(count):
    if not path.with_name(_chunk_name(path.name, index, count)).is_file():
      return None
  return path


def write_big_model_stamp(model_sha256: str, directory: Path | None = None) -> None:
  """Record that the active big model is built, for launch_chffrplus's build gate.

  The value is the model sha256 (no trailing newline); the launcher compares it against
  the active model and keeps the "artifact is ready" state only while they match.
  """
  target = (directory or MODELS_DIR) / BIG_MODEL_STAMP
  target.parent.mkdir(parents=True, exist_ok=True)
  target.write_text(model_sha256)


def active_delivery(model=None) -> str | None:
  """Which delivery the active model resolves to, using the untouched helper."""
  from openpilot.selfdrive.modeld.helpers import active_usbgpu_compiled_path

  try:
    resolved = active_usbgpu_compiled_path()
  except Exception:
    return None
  if resolved is None:
    return None
  return 'chunked' if resolved.parent == MODELS_DIR else 'precompiled'


# ----------------------------------------------------------------------- model catalog

def _validate_section(value: dict, catalog_url: str) -> dict | None:
  """Validate and resolve the optional chunked block of an already validated catalog."""
  section = value.get('chunked')
  if section is None:
    return None
  if not isinstance(section, dict) or not isinstance(section.get('url'), str) or not section['url']:
    raise ValueError('invalid chunked artifact')
  chunks = section.get('chunks')
  if not isinstance(chunks, list) or not 0 < len(chunks) <= 512:
    raise ValueError('invalid chunked artifact')
  if not SHA256.fullmatch(section.get('sha256', '')):
    raise ValueError('invalid chunked artifact hash')
  if type(section.get('size')) is not int or not 0 < section['size'] <= 4 * 1024**3:
    raise ValueError('invalid chunked artifact size')
  # Per-chunk hashes let a download verify each 45 MB piece, so one bad response is
  # re-fetched on its own instead of restarting the whole 787 MB set.
  for chunk in chunks:
    if not isinstance(chunk, dict) or not SHA256.fullmatch(chunk.get('sha256', '')):
      raise ValueError('invalid chunk hash')
    if type(chunk.get('size')) is not int or not 0 < chunk['size'] <= 128 * 1024**2:
      raise ValueError('invalid chunk size')
  if sum(chunk['size'] for chunk in chunks) != section['size']:
    raise ValueError('chunk sizes do not add up')
  section['url'] = urljoin(catalog_url, section['url'])
  parsed = urlparse(section['url'])
  if parsed.scheme not in ('http', 'https') or parsed.netloc != urlparse(catalog_url).netloc:
    raise ValueError(f'chunked artifact must be served from the model server origin: {parsed.netloc}')
  return section


def _read_catalog(model) -> dict:
  catalog_url = urljoin(model.url, 'precompiled.json')
  with urlopen(Request(catalog_url, headers={'User-Agent': USER_AGENT}), timeout=8) as response:
    data = response.read(MAX_CATALOG + 1)
  if len(data) > MAX_CATALOG:
    raise ValueError('model catalog too large')
  value = validate_catalog(json.loads(data), model.sha256, catalog_url)
  _validate_section(value, catalog_url)
  return value


def _chunk_name(base: str, index: int, total: int) -> str:
  # identical to common.file_chunker.get_chunk_name, which modeld's reader relies on
  return f'{base}.chunk{index + 1:02d}of{total:02d}'


def _status(state: str, detail: str, model=None, status_values: dict | None = None, **values) -> None:
  """Write a status the HUD badge and the web pages read. Never raises.

  model_id / sha256 are always written: the background downloader has no status_values,
  and without the sha the model page cannot match the status to its card.
  """
  try:
    fields = dict(status_values or {})
    if model is not None:
      fields.setdefault('model_id', model.model_id)
      fields.setdefault('sha256', model.sha256)
    fields.update(values)
    write_big_model_status(model_cache_dir(), state, detail=detail, **fields)
  except Exception:
    pass


def _write_progress(done: int, total: int, status_values: dict | None, model) -> None:
  _status('downloading', 'chunked model', model, status_values,
          downloaded_bytes=done, total_bytes=total)


# ------------------------------------------------------------------------- installation

def _stream_sha256(path: Path, chunks: list[dict]) -> tuple[str, int]:
  digest = hashlib.sha256()
  total = 0
  for index in range(len(chunks)):
    piece = path.with_name(_chunk_name(path.name, index, len(chunks)))
    with piece.open('rb') as f:
      while data := f.read(4 * 1024 * 1024):
        digest.update(data)
        total += len(data)
  return digest.hexdigest(), total


def validate_chunked(path: Path, timeout: float = LOAD_TIMEOUT) -> None:
  """Load the artifact in a subprocess so a broken file never becomes "installed".

  Loading needs the eGPU (the pickle's buffers live on it), so callers gate this on
  usbgpu_present(). Any failure propagates: the caller drops the downloaded set and
  falls back to the other delivery or the local compiler. The subprocess output is kept
  in the raised message - a silent "exit status 1" is useless for diagnosis.
  """
  from openpilot.common.basedir import BASEDIR

  command = [sys.executable, '-c', _LOAD_SNIPPET, str(path), str(BASEDIR)]
  try:
    result = subprocess.run(command, cwd=BASEDIR, check=True, timeout=timeout, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  except subprocess.TimeoutExpired as exc:
    raise ChunkLoadError(f'chunked model load timed out after {timeout:.0f}s') from exc
  except subprocess.CalledProcessError as exc:
    detail = (exc.output or '').strip()
    raise ChunkLoadError(f'chunked model failed to load (exit {exc.returncode}): {detail[-500:]}') from exc
  if 'run_policy' not in result.stdout:
    raise ChunkLoadError(f'unexpected chunked model contents: {result.stdout.strip()[-200:]}')


def _wait_for_network(url: str, timeout: float = NETWORK_WAIT_SECONDS) -> None:
  """Wait until the model host resolves and accepts a connection.

  launch_chffrplus starts the background downloader immediately after the boot build,
  sometimes before DNS is usable; the whole delivery used to fail on the first
  urlopen and nothing retried it until the next boot.
  """
  try:
    from openpilot.selfdrive.modeld.big_model import wait_for_manifest_network
    wait_for_manifest_network(url, timeout)
  except Exception:
    pass  # a host that stays unreachable is reported by the download itself


def ensure_chunked(model=None, directory: Path | None = None, progress=None,
                   validate: bool | None = None) -> Path | None:
  """Install the chunked artifact for `model`, downloading only what is missing.

  Returns the pkl path once the manifest is written (the commit point), or None when the
  server offers no chunked package for this model. Raises on a failed verification.
  """
  model = model or active_manifest()
  if model is None:
    return None
  directory = directory or MODELS_DIR
  path = chunked_pkl_path(model, directory)
  assert path is not None
  # Firmware is tiny (~300 KB) and required before the first GPU init, so fetch it before
  # the early return: a device that installed the artifact while the eGPU was absent still
  # needs it later (and the load check below cannot pass without it). The repo's tinygrad
  # only finds the AGNOS blobs when they match its pinned hashes, which they do not on
  # this board - that is what the compile_modeld search-path patch covers.
  # download() writes its target without creating the directory, and on a fresh device
  # the repo's models/ output directory does not exist yet.
  path.parent.mkdir(parents=True, exist_ok=True)
  installed = Path(get_manifest_path(str(path))).is_file()
  if not installed:
    # The firmware manifest and the catalog are the first network calls, so wait for the
    # host BEFORE them: a cold boot reaches the background downloader before DNS works,
    # and a failed run is not retried until the next boot. Skipped when the artifact is
    # already installed so an offline device does not wait for nothing.
    _wait_for_network(urljoin(model.url, 'precompiled.json'))
  ensure_firmware(model, download)
  if installed:
    return path

  # The catalog is fetched over the network like the pieces, so retry it the same way.
  for attempt in range(DOWNLOAD_ATTEMPTS):
    try:
      # _read_catalog() already validated and resolved the section against the catalog URL
      section = _read_catalog(model).get('chunked')
      break
    except OSError as exc:
      if attempt + 1 >= DOWNLOAD_ATTEMPTS:
        raise
      print(f'chunked catalog: {exc}; retrying')
      time.sleep(DOWNLOAD_RETRY_DELAY)
  if not section:
    return None

  chunks = section['chunks']
  total_size = section['size']
  # Pieces are fetched one by one, so the per-piece free-space check inside download()
  # passes even when the whole set does not fit. Fail up front instead of running the
  # disk full two thirds through and leaving a half written set behind.
  free = shutil.disk_usage(path.parent).free
  if free < total_size + 256 * 1024**2:
    raise OSError(f'insufficient storage for chunked model: {free} free, need {total_size}')
  done = 0
  for index, chunk in enumerate(chunks):
    piece = path.with_name(_chunk_name(path.name, index, len(chunks)))
    url = f"{section['url']}.chunk{index + 1:02d}of{len(chunks):02d}"

    def report(offset: int, _size: int, base: int = done) -> None:
      if progress:
        progress(base + offset, total_size)

    # download() already verifies this piece's sha256 and resumes from its .part file.
    # Retry locally: it deletes its own partial file, so a concurrent cleanup (a second
    # installer run) surfaces as FileNotFoundError right after the transfer and must not
    # abort a multi-minute download. Network errors are transient in the same way (DNS
    # not up yet on a cold boot, a dropped link), so they retry too.
    for attempt in range(DOWNLOAD_ATTEMPTS):
      try:
        download({'url': url, 'size': chunk['size'], 'sha256': chunk['sha256']}, piece, report)
        break
      except OSError as exc:
        if attempt + 1 >= DOWNLOAD_ATTEMPTS:
          raise
        print(f'chunked piece {index + 1}/{len(chunks)}: {exc}; retrying')
        time.sleep(DOWNLOAD_RETRY_DELAY)
    done += chunk['size']

  digest, size = _stream_sha256(path, chunks)
  if digest != section['sha256'] or size != total_size:
    raise ValueError('chunked artifact hash mismatch')

  # Write the manifest first: open_file_chunked() (used by the load check and by modeld)
  # can only read the artifact through it, so a "validate then commit" order can never
  # pass. It is still the commit point - the load check takes it back on failure - but a
  # failure no longer throws the download away: the pieces are verified per piece, so a
  # retry only re-runs the stream hash and the load.
  manifest = Path(get_manifest_path(str(path)))
  temporary = manifest.with_name(manifest.name + '.tmp')
  temporary.write_text(str(len(chunks)))
  os.replace(temporary, manifest)

  # The artifact is committed: ask for the reboot that makes modeld pick it up. Done here
  # rather than after the load check below, so the HUD prompt appears the moment the
  # download finishes instead of a minute later - the check continues in parallel.
  try:
    from openpilot.selfdrive.modeld.precompiled_fetch import REBOOT_FLAG
    REBOOT_FLAG.touch()
  except Exception:
    pass  # purely cosmetic; never fail over a flag file
  _status('verifying', 'chunked model', model, None)

  try:
    if validate if validate is not None else usbgpu_present():
      validate_chunked(path)
  except ChunkLoadError as exc:
    if 'Failed to acquire lock file' not in str(exc):
      manifest.unlink(missing_ok=True)
      raise
    # modeld holds the eGPU (background download while the car is in use). Every piece is
    # hash-verified already, so accept the set and let modeld's own load be the acceptance
    # test rather than throwing away a completed download.
    print(f'chunked model installed; load check deferred (eGPU busy): {exc}')
  except Exception:
    manifest.unlink(missing_ok=True)
    raise

  write_big_model_stamp(model.sha256, directory)
  return path


def sync_precompiled_marker(model=None) -> bool:
  """Steer the untouched upstream resolution towards the chunked artifact.

  `precompiled_model.installed()` returns None while a `rejected` marker exists, so
  `helpers.active_usbgpu_compiled_path()` (precompiled first, chunked second) falls
  through to the chunked artifact. The marker content must stay equal to the catalog's
  pickle sha - that is what makes `ensure_precompiled()` skip the download instead of
  fetching the precompiled artifact again.
  """
  model = model or active_manifest()
  if model is None:
    return False
  root = model_cache_dir() / 'precompiled' / model.sha256
  marker = root / 'rejected'
  if read_model_source() == 'precompiled':
    marker.unlink(missing_ok=True)
    return False
  try:
    served = _read_catalog(model)['pickle']['sha256']
  except Exception:
    return False
  root.mkdir(parents=True, exist_ok=True)
  marker.write_text(served)
  return True


def delete_status(cache_dir: Path | None = None) -> None:
  """Clear a transient status so the UI falls back to the manifest's truth."""
  try:
    ((cache_dir or model_cache_dir()) / STATUS_FILENAME).unlink(missing_ok=True)
  except OSError:
    pass


def clear_precompiled_marker(model=None) -> bool:
  """Let the precompiled artifact count as installed again.

  Used as the fallback when a chunked install fails: a slower working model beats no
  model (and beats falling through to a ~20 minute local SCons compile).
  """
  model = model or active_manifest()
  if model is None:
    return False
  marker = model_cache_dir() / 'precompiled' / model.sha256 / 'rejected'
  if marker.is_file():
    marker.unlink()
    return True
  return False


def rearm_usbgpu() -> bool:
  """Clear the "eGPU bring-up failed" latch so every boot retries the eGPU.

  modeld is the only writer of UsbGpuStartupFailed (setting it True when the model load
  fails) and nothing ever clears it - the comment says "for the rest of this ignition
  cycle" but it is a persistent param, so one bad boot disabled the eGPU for good: red
  badge, internal model, across reboots. Re-arming here means modeld retries, and simply
  sets it again if the GPU is genuinely still unusable.
  """
  try:
    from openpilot.common.params import Params
    Params().put_bool('UsbGpuStartupFailed', False)
    return True
  except Exception:
    return False


def auto_install(manifest=None, spinner=None, status_values: dict | None = None,
                 *, allow_download: bool | None = None) -> Path | None:
  """Boot hook for manager/build.py.

  Returns the chunked pkl path when that delivery is in use, None when the caller should
  continue with the precompiled delivery or the local compiler. Never raises.

  Downloads belong to the background downloader (launch_chffrplus starts
  `big_model --ensure-if-egpu`, which calls fetch_after_onnx): while it is alive this
  only adopts an already installed set. Two writers would delete each other's .part
  files mid-transfer, and blocking the boot for ~4 minutes is the thing the background
  delivery avoids. Pass allow_download explicitly to force either behaviour.
  """
  model = manifest or active_manifest()
  if model is None:
    return None
  # Both of these must happen before the eGPU is initialized this boot, whatever this
  # function ends up deciding:
  #  * the firmware blobs have to be on disk - tinygrad uploads them on every cold init,
  #    and a cold boot that reaches the GPU check before the (few hundred KB) download
  #    finishes fails it, which modeld then treats as a permanent eGPU failure;
  #  * the failure latch has to be cleared, or that one bad boot stays bad (see rearm_usbgpu).
  ensure_firmware(model, download)
  rearm_usbgpu()
  if read_model_source() == 'precompiled':
    sync_precompiled_marker(model)
    return None
  if allow_download is None:
    allow_download = not _background_downloader_running()
  try:
    path = installed_chunked(model)
    if path is None:
      if not allow_download:
        return None
      if spinner:
        spinner.update("USB eGPU big model\nChecking chunked model")

      def progress(done: int, total: int) -> None:
        if spinner:
          spinner.update(f"USB eGPU big model\nDownloading chunked model {done * 100 // total}%")
        _write_progress(done, total, status_values, model)

      path = ensure_chunked(model, progress=progress)
  except Exception as exc:
    print(f"Chunked eGPU model unavailable: {exc}")
    # Never drop the download: each piece was verified on arrival, so the set only needs
    # another hash+load attempt (~1 minute) instead of 787 MB over the air. Un-block the
    # precompiled artifact so this boot still gets a working model; the next boot retries
    # the chunked one (the manifest was not committed, so it counts as not installed).
    clear_precompiled_marker(model)
    # Drop the half-written status instead of leaving "downloading" on screen forever:
    # without a status file the UI reports the truth (compiled/ready) from the manifest.
    delete_status()
    return None
  if path is None:
    return None
  write_big_model_stamp(model.sha256)
  sync_precompiled_marker(model)
  _status('compiled', 'downloaded chunked model', model, status_values)
  return path


def download_deferred(model=None) -> bool:
  """Is the background downloader fetching the chunked set for `model` right now?

  build_usbgpu_model uses this to stay out of the way: the artifact is already on its way,
  so the boot should neither block on it nor fall back to the precompiled set or the
  ~20 minute local SCons compile.
  """
  model = model or active_manifest()
  if model is None or read_model_source() == 'precompiled':
    return False
  if installed_chunked(model) is not None:
    return False
  return _background_downloader_running()


def remove_chunked_sha(model_sha256: str, directory: Path | None = None) -> list[str]:
  """Delete a chunked set by model sha (failed verification, or the web model manager)."""
  path = _pkl_path(model_sha256, directory)
  if not path.parent.is_dir():
    return []
  removed = []
  for candidate in sorted(path.parent.glob(f'{path.name}.*')):
    try:
      candidate.unlink()
      removed.append(candidate.name)
    except OSError:
      pass
  return removed


def remove_chunked(model=None, directory: Path | None = None) -> list[str]:
  model = model or active_manifest()
  if model is None:
    return []
  return remove_chunked_sha(model.sha256, directory)
