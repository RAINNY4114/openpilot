"""AMD firmware blobs the USB eGPU needs, fetched from the model server.

This lives in its own module on purpose: precompiled_model.py should only call
ensure_firmware(), so pulling upstream changes into that file stays conflict-free.

The blobs are looked up by the compiled runtime's fetch_fw patch under
/data/media/0/carrot/firmware (see model_runtime.py). Without them the eGPU never
initializes, even when the model itself is valid.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Callable
from urllib.parse import urljoin
from urllib.request import Request, urlopen

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.big_model import active_manifest

FIRMWARE_DIR = Path('/data/media/0/carrot/firmware/amdgpu')

# Optional manifest published next to the blobs. When present it takes priority,
# so shipping a model for a different GPU arch needs no code change here.
_FIRMWARE_MANIFEST = 'manifest.json'

# name -> (sha256 of the .zst exactly as served, size in bytes)
# Fallback for the current GPU (gfx1200); must match checksums.sha256 otherwise.
FIRMWARE: dict[str, tuple[str, int]] = {
  'smu_14_0_2.bin.zst':    ('4c80055939f89ce619a8aaf71882fa04d426279191f893ff32acf02dc4861476', 137291),
  'gc_12_0_0_me.bin.zst':  ('ef7fbca61215ae42dbb96048467793e7f7a06de139e4df7121be11d70750ebd6', 47877),
  'gc_12_0_0_mec.bin.zst': ('b0d5ac1728f43873f03d94f27c8e9807e42e42653fb76054577dbd08befeb43e', 82417),
  'gc_12_0_0_pfp.bin.zst': ('aae909f6c0481b2761a4193f1ed26e793676f83abb8db16e966402ecf120b8c4', 41901),
}

# Firmware sits next to the model root, one level above the model file's directory.
_FIRMWARE_PREFIX = '../firmware/amdgpu/'


def _remote_firmware(model) -> dict[str, tuple[str, int]] | None:
  """Firmware list published by the server next to the blobs, if any.

  Lets a model built for a different GPU arch ship its own firmware without a
  code change here. Any problem simply falls back to the built-in FIRMWARE.
  """
  url = urljoin(model.url, _FIRMWARE_PREFIX + _FIRMWARE_MANIFEST)
  try:
    with urlopen(Request(url, headers={'Accept-Encoding': 'identity'}), timeout=15) as response:
      if response.status != 200:
        return None
      raw = json.loads(response.read().decode('utf-8'))
  except Exception:
    return None

  files = raw.get('files') if isinstance(raw, dict) else None
  if not isinstance(files, dict):
    return None
  out: dict[str, tuple[str, int]] = {}
  for name, meta in files.items():
    if not isinstance(name, str) or not isinstance(meta, dict):
      continue
    sha, size = meta.get('sha256'), meta.get('size')
    if isinstance(sha, str) and len(sha) == 64 and isinstance(size, int) and size > 0:
      out[name] = (sha, size)
  return out or None


def _fetch(url: str, target: Path, size: int, want_hash: str) -> None:
  """Minimal verified download, used only when no downloader is supplied."""
  partial = target.with_suffix(target.suffix + '.part')
  with urlopen(Request(url, headers={'Accept-Encoding': 'identity'}), timeout=30) as response:
    if response.status != 200:
      raise OSError(f'unexpected download status {response.status}')
    with partial.open('wb') as f:
      written = 0
      while data := response.read(1024 * 1024):
        written += len(data)
        if written > size:
          raise ValueError('firmware exceeds declared size')
        f.write(data)
      f.flush()
      os.fsync(f.fileno())
  if written != size:
    raise OSError('incomplete firmware download')
  with partial.open('rb') as f:
    if hashlib.file_digest(f, 'sha256').hexdigest() != want_hash:
      partial.unlink()
      raise ValueError('firmware hash mismatch')
  os.replace(partial, target)


def _report_progress(done: int, total: int) -> None:
  """Publish the firmware phase so the HUD badge can show "FW xx%".

  The blobs are tiny (~300 KB total), so this updates once per blob rather than
  streaming. Purely cosmetic: any failure here is ignored.
  """
  try:
    from openpilot.selfdrive.modeld.big_model import model_cache_dir
    from openpilot.selfdrive.modeld.big_model_status import write_big_model_status

    write_big_model_status(model_cache_dir(), 'downloading',
                           downloaded_bytes=done, total_bytes=total,
                           detail='firmware')
  except Exception:
    pass


def ensure_firmware(model=None, download: Callable[[dict, Path], None] | None = None) -> bool:
  """Make sure every firmware blob is present and verified.

  download is called as download(artifact, target) with artifact containing
  url/size/sha256; pass precompiled_model.download to reuse its resume logic.
  Already-present blobs are skipped, so this is cheap to call every time.
  Returns True when all blobs are in place.
  """
  model = model or active_manifest()
  if model is None:
    return False

  # Prefer the server's list, so a model for another GPU arch needs no code change.
  firmware = _remote_firmware(model) or FIRMWARE

  FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
  total = sum(size for _, size in firmware.values())
  done = 0
  ok = True
  for name, (want_hash, want_size) in sorted(firmware.items()):
    target = FIRMWARE_DIR / name
    artifact = {'url': urljoin(model.url, _FIRMWARE_PREFIX + name),
                'size': want_size, 'sha256': want_hash}
    needs_download = not (target.is_file() and target.stat().st_size == want_size)
    try:
      if needs_download:
        if download is not None:
          download(artifact, target)
        else:
          _fetch(artifact['url'], target, want_size, want_hash)
    except Exception as exc:
      # Never discard an otherwise valid model over a firmware mirror problem,
      # but it must be visible: the eGPU cannot start without these.
      cloudlog.warning(f'precompiled firmware {name} unavailable: {exc}')
      ok = False
    # Only report when something was actually fetched. Otherwise a normal boot
    # with all blobs present would leave status.json stuck on "downloading".
    if needs_download:
      done += want_size
      _report_progress(done, total)
  return ok
