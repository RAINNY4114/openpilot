"""Fetch the precompiled artifacts as soon as the ONNX is in place.

build.py runs earlier in the launch sequence and skips the precompiled fetch
while the ONNX is still missing, which used to leave it until the next boot.
Calling this right after ensure_big_model() makes one boot enough.

Kept separate from big_model.py so upstream refactors of that script cannot
clobber the logic.
"""

import json
import sys
from pathlib import Path

# Touched once the download is complete, and read by the HUD badge to show
# "REBOOT". /tmp is a tmpfs: a reboot (or power loss) wipes it, so the prompt
# disappears on its own. Timestamps cannot be used for this - the clock is
# reset across power cycles.
REBOOT_FLAG = Path("/tmp/carrot-big-model-needs-reboot")

# The stock model_runtime.py only looks for AMD firmware in /lib/firmware, which is
# read-only on AGNOS. firmware.py downloads the blobs to /data/media/0/carrot/firmware,
# so without this patch tinygrad cannot find them, falls back to downloading from
# gitlab (403), and the boot validation then rejects the whole artifact set.
# Install.sh applies the same patch; this is the equivalent for the HTTP path.
_FW_OLD = '''  def fetch_fw(path, name, sha256):
    p = pathlib.Path(f"/lib/firmware/{path}/{name}.zst")
    if p.is_file():
      blob = zstandard.ZstdDecompressor().stream_reader(p.read_bytes()).read()
      if hashlib.sha256(blob).hexdigest() == sha256:
        return blob
    return _orig(path, name, sha256)'''

_FW_NEW = '''  # Firmware search path: read-only system dir plus the persistent /data dir.
  _FW_BASES = ("/lib/firmware", "/data/media/0/carrot/firmware")
  def fetch_fw(path, name, sha256):
    for base in _FW_BASES:
      p = pathlib.Path(base) / path / (name + ".zst")
      if p.is_file():
        blob = zstandard.ZstdDecompressor().stream_reader(p.read_bytes()).read()
        if hashlib.sha256(blob).hexdigest() == sha256:
          return blob
    return _orig(path, name, sha256)'''


def patch_runtime_firmware_paths(model=None, cache_dir=None) -> bool:
  """Teach the extracted runtime to look for firmware under /data as well."""
  try:
    from openpilot.selfdrive.modeld.big_model import active_manifest, model_cache_dir

    m = model or active_manifest()
    if m is None:
      return False
    root = (cache_dir or model_cache_dir()) / 'precompiled' / m.sha256
    info = json.loads((root / 'installed.json').read_text(encoding='utf-8'))
    target = root / info['runtime_directory'] / 'model_runtime.py'
    if not target.is_file():
      return False
    src = target.read_text(encoding='utf-8')
    if '_FW_BASES' in src:
      return True
    if _FW_OLD not in src:
      from openpilot.common.swaglog import cloudlog
      cloudlog.warning('firmware path patch: unexpected model_runtime.py, skipped')
      return False
    target.write_text(src.replace(_FW_OLD, _FW_NEW), encoding='utf-8')
    return True
  except Exception:
    return False


def fetch_after_onnx(manifest, reporter) -> None:
  """Download the big-model artifacts for `manifest`, reporting progress.

  Never raises: a failure here must not prevent the normal internal-GPU build.

  Both deliveries download here, in the background, for the same reason: the boot build
  must not sit on a multi-minute download, and the HUD/web pages get their progress from
  status.json either way.
  """
  source = "auto"
  fetched_chunked = False
  newly_installed = False
  try:
    from openpilot.selfdrive.modeld import chunked_model

    source = chunked_model.read_model_source()
    if source != "precompiled":
      def chunked_progress(done, total) -> None:
        # detail="chunked model" maps to the same "PKL" label as the precompiled set.
        reporter.update("downloading", model_id=manifest.model_id, sha256=manifest.sha256,
                        downloaded_bytes=done, total_bytes=total, detail="chunked model")

      # A reboot is only needed when something actually landed in this run: this function
      # runs on every boot, and touching the flag unconditionally would leave the "REBOOT"
      # prompt on screen forever.
      was_installed = chunked_model.installed_chunked(manifest) is not None
      fetched_chunked = chunked_model.ensure_chunked(manifest, progress=chunked_progress) is not None
      newly_installed = fetched_chunked and not was_installed
  except Exception as exc:
    print(f"chunked model fetch deferred: {exc}", file=sys.stderr)

  if fetched_chunked:
    # Steer the (untouched) resolution to the chunked set and prompt for the reboot that
    # makes modeld pick it up.
    try:
      chunked_model.sync_precompiled_marker(manifest)
      if newly_installed:
        REBOOT_FLAG.touch()
    except Exception:
      pass  # purely cosmetic; never fail over a flag file
    return
  if source == "chunked":
    return  # the user pinned this delivery; the boot build falls back to the local compile
  if source == "auto":
    # Chunked is unavailable: let the precompiled artifact count as installed again.
    try:
      chunked_model.clear_precompiled_marker(manifest)
    except Exception:
      pass

  try:
    from openpilot.selfdrive.modeld.precompiled_model import ensure_precompiled

    def precompiled_progress(done, total) -> None:
      # detail="pkl" is what the HUD badge keys off to label this phase "PKL"
      # instead of "ONNX" (the ONNX download is reported by big_model itself and
      # carries no detail field).
      reporter.update("downloading", model_id=manifest.model_id, sha256=manifest.sha256,
                      downloaded_bytes=done, total_bytes=total, detail="pkl")

    ensure_precompiled(manifest, progress=precompiled_progress)
    # The runtime is extracted by now; teach it where the firmware lives.
    patch_runtime_firmware_paths(manifest)
    try:
      REBOOT_FLAG.touch()
    except Exception:
      pass  # purely cosmetic; never fail over a flag file
  except Exception as exc:
    print(f"precompiled model fetch deferred: {exc}", file=sys.stderr)
