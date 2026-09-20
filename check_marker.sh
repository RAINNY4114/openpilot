#!/bin/bash
# 核对 rejected 标记语义：sync 写、clear 清、最终恢复为"分片优先"
export PYTHONPATH=/data/openpilot/pydeps:/data/openpilot:/data/pythonpath
PY=/usr/local/venv/bin/python3

$PY - <<'PYEOF'
import sys
sys.path.insert(0, '/data/openpilot')
from openpilot.selfdrive.modeld import chunked_model
from openpilot.selfdrive.modeld.big_model import active_manifest, model_cache_dir
from openpilot.selfdrive.modeld.helpers import active_usbgpu_compiled_path

model = active_manifest()
marker = model_cache_dir() / 'precompiled' / model.sha256 / 'rejected'

print('  初始           :', '有标记' if marker.is_file() else '无标记')
print('  sync()  ->', chunked_model.sync_precompiled_marker(model),
      '| 标记:', ('有 ' + marker.read_text()[:16] + '…') if marker.is_file() else '无')
print('  clear() ->', chunked_model.clear_precompiled_marker(model),
      '| 标记:', '有' if marker.is_file() else '无')
print('  clear() 幂等 ->', chunked_model.clear_precompiled_marker(model), '(应为 False)')
print('  sync() 恢复 ->', chunked_model.sync_precompiled_marker(model),
      '| 标记:', ('有 ' + marker.read_text()[:16] + '…') if marker.is_file() else '无')
print()
print('  解析      :', active_usbgpu_compiled_path())
print('  delivery  :', chunked_model.active_delivery())
print('  installed :', chunked_model.installed_chunked(model))
PYEOF
