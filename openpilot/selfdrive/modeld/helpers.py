import io
import os
import pickle
import struct
from pathlib import Path

from openpilot.common.file_chunker import get_manifest_path
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(f):
  opcodes = f.read(struct.unpack('<q', f.read(8))[0])
  def buffers():
    while (h := f.read(8)):
      pb = pickle.PickleBuffer(bytearray(struct.unpack('<q', h)[0]))
      if f.readinto(pb) != pb.raw().nbytes:
        raise EOFError("incomplete model buffer")
      yield pb
  return pickle.load(io.BytesIO(opcodes), buffers=buffers())

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def chestnut_compiled() -> bool:
  # 激活的 chestnut bundle 已下载齐备 -> 大模型本地可加载（tinygrad runner 读的就是它）
  if _chestnut_bundle_local():
    return True
  # 回退：仓库内置的老 big_driving_* 路径（stock runner 用）
  path = modeld_pkl_path(chestnut=True)
  return (path.is_file() or Path(get_manifest_path(path)).is_file()) and all(
    (MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl').is_file() for size in ('1344x760', '1928x1208'))


def _chestnut_bundle_local() -> bool:
  """只做 stat、不算 sha256 —— ui_state 每 ~5s 调一次 chestnut_compiled()，必须廉价。"""
  try:
    from openpilot.common.file_chunker import get_chunk_name
    from openpilot.common.hardware.hw import Paths
    from openpilot.sunnypilot.models.helpers import get_active_bundle
    bundle = get_active_bundle(chestnut=True)
    if bundle is None or not bundle.models:
      return False
    artifact = bundle.models[0].artifact
    if not artifact.fileName:
      return False
    root = Paths.model_root()
    num_chunks = len(artifact.chunks)
    if num_chunks > 0:
      return all(os.path.isfile(os.path.join(root, get_chunk_name(artifact.fileName, i, num_chunks)))
                 for i in range(num_chunks))
    return os.path.isfile(os.path.join(root, artifact.fileName))
  except Exception:
    return False
