"""Narrow down the failing GPU load validation - no download, no GPU lock needed."""
import subprocess
import sys

sys.path.insert(0, '/data/openpilot')
from openpilot.common.basedir import BASEDIR  # noqa: E402
from openpilot.selfdrive.modeld.big_model import active_manifest  # noqa: E402
from openpilot.selfdrive.modeld.chunked_model import (_LOAD_SNIPPET, chunked_pkl_path,  # noqa: E402
                                                      installed_chunked)
from openpilot.selfdrive.modeld.helpers import usbgpu_present  # noqa: E402

model = active_manifest()
path = chunked_pkl_path(model)
print('  pkl        :', path)
print('  installed  :', installed_chunked(model))
print('  usbgpu     :', usbgpu_present())
print('  BASEDIR    :', BASEDIR)
print('  sys.path[:3]:', sys.path[:3])
print()
print('=== 1) 子进程只做 import（不需要 GPU）===')
imports = (
  'import sys;'
  f'sys.path.insert(0, {str(BASEDIR)!r});'
  'from openpilot.common.file_chunker import open_file_chunked;'
  'from openpilot.selfdrive.modeld.helpers import load_oob;'
  'print("imports ok:", open_file_chunked, load_oob)'
)
done = subprocess.run([sys.executable, '-c', imports], cwd=BASEDIR, text=True,
                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
print(f'  rc={done.returncode}')
print('  ' + (done.stdout or '').strip().replace('\n', '\n  '))
print()
print('=== 2) 子进程真加载（需空闲 eGPU；当前 modeld 在跑就会看到锁错误）===')
try:
  result = subprocess.run([sys.executable, '-c', _LOAD_SNIPPET, str(path), str(BASEDIR)],
                          cwd=BASEDIR, check=True, timeout=90, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  print('  rc=0')
  print('  ' + result.stdout.strip()[:300])
except subprocess.TimeoutExpired:
  print('  超时 90s')
except subprocess.CalledProcessError as exc:
  print(f'  rc={exc.returncode}')
  print('  ' + (exc.output or '').strip()[-1200:].replace('\n', '\n  '))
except Exception as exc:
  print('  其它错误:', exc)
