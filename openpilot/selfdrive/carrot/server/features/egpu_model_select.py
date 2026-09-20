"""Web API: list, switch between and remove coexisting eGPU big models.

Companion to features/egpu_model.py (which reports the download/compile state of
the *current* model). This one manages the set of models stored side by side in
the model cache dir, so the user can pick one in the web UI.

Routes:
  GET  /api/egpu/models          list (remote catalog merged with local state)
  POST /api/egpu/models/select   choose a model; optional reboot to activate
  POST /api/egpu/models/remove   delete a downloaded model from disk
  POST /api/egpu/models/source   pick the big model delivery (auto/precompiled/chunked)
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from typing import Any
from urllib.parse import urljoin
from urllib.request import Request, urlopen

from aiohttp import web

from openpilot.selfdrive.modeld import chunked_model, model_catalog
from openpilot.selfdrive.modeld.big_model import active_manifest, model_cache_dir
from openpilot.selfdrive.modeld.model_source import MODEL_BASE

from ..services.params import HAS_PARAMS, Params

CATALOG_PATH = 'models/catalog.json'
REMOTE_TIMEOUT = 6.0


def _remote_catalog() -> dict[str, Any] | None:
  """Optional server-side list of selectable models."""
  url = urljoin(MODEL_BASE if MODEL_BASE.endswith('/') else MODEL_BASE + '/', CATALOG_PATH)
  try:
    with urlopen(Request(url, headers={'User-Agent': 'carrot-model/1'}), timeout=REMOTE_TIMEOUT) as response:
      if response.status != 200:
        return None
      value = json.loads(response.read().decode('utf-8'))
    return value if isinstance(value, dict) else None
  except Exception:
    return None


def _active_sha() -> str | None:
  try:
    manifest = active_manifest()
    return manifest.sha256 if manifest is not None else None
  except Exception:
    return None


def _modeld_pid() -> int | None:
  try:
    out = subprocess.run(['pgrep', '-f', 'modeld.modeld'], capture_output=True, text=True, timeout=5)
    return int(out.stdout.split()[0])
  except Exception:
    return None


def _running_sha() -> str | None:
  """The model modeld is actually running, or None (internal model / unknown).

  state.json's "active" is what the *next* boot will use: it is rewritten as soon as
  a newly selected model finishes downloading, while modeld keeps running the one it
  loaded at startup. modeld publishes no model id (only UsbGpu* booleans), so derive
  it: if state.json was written after modeld started, what still runs is "previous".
  """
  try:
    from openpilot.common.params import Params
    if not Params().get_bool('UsbGpuActive'):
      return None
  except Exception:
    return None

  try:
    from openpilot.selfdrive.modeld.big_model import read_state, state_path

    state = read_state()
    active, previous = state.get('active'), state.get('previous')
    if active is None:
      return previous.sha256 if previous is not None else None
    pid = _modeld_pid()
    if pid is None or previous is None:
      return active.sha256
    try:
      elapsed = float(subprocess.run(['ps', '-o', 'etimes=', '-p', str(pid)],
                                     capture_output=True, text=True, timeout=5).stdout.strip())
      started = time.time() - elapsed
    except Exception:
      return active.sha256
    return previous.sha256 if started < os.path.getmtime(state_path()) else active.sha256
  except Exception:
    return None


def _entry(label: str | None, model_id: str | None, sha: str, size, url, *, downloaded: bool,
           selected: bool, is_active: bool) -> dict[str, Any]:
  return {
    'label': label or model_id or sha[:12],
    'model_id': model_id,
    'sha256': sha,
    'size': size,
    'url': url,
    'downloaded': downloaded,
    'selected': selected,
    'active': is_active,
  }


def build_models_payload() -> dict[str, Any]:
  local = model_catalog.list_models()
  by_sha = {item['sha256']: item for item in local}
  selected = model_catalog.selected_sha()
  # "active" is what the next boot will load; "running" is what modeld loaded this
  # boot. They differ right after a switch finished downloading (until the reboot).
  active = _active_sha()
  running = _running_sha()

  models: list[dict[str, Any]] = []
  remote = _remote_catalog()
  for entry in (remote or {}).get('models', []) or []:
    if not isinstance(entry, dict):
      continue
    sha = entry.get('sha256')
    if not isinstance(sha, str) or len(sha) != 64:
      continue
    known = by_sha.pop(sha, {})
    # known comes from list_models(), which decides `downloaded` from the ONNX files
    # actually on disk. The index also holds entries pre-registered from the remote
    # catalog, so its presence alone must not count as "downloaded".
    models.append(_entry(entry.get('label'), entry.get('model_id'), sha, entry.get('size'),
                         entry.get('url'), downloaded=bool(known.get('downloaded')),
                         selected=sha == selected, is_active=sha == running))

  # Anything on disk that the server no longer lists still has to be switchable.
  for sha, known in by_sha.items():
    models.append(_entry(None, known.get('model_id'), sha, known.get('size'), known.get('url'),
                         downloaded=True, selected=sha == selected, is_active=sha == running))

  return {
    'ok': True,
    'selected': selected,
    'active': active,
    'running': running,
    'source': chunked_model.read_model_source(),
    'sources': list(chunked_model.SOURCES),
    'delivery': chunked_model.active_delivery(),
    'remote_catalog': remote is not None,
    'models': models,
  }


async def api_models(_request: web.Request) -> web.Response:
  return web.json_response(build_models_payload())


async def api_select(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({'ok': False, 'error': 'invalid JSON body'}, status=400)
  if not isinstance(body, dict):
    return web.json_response({'ok': False, 'error': 'invalid body'}, status=400)

  sha = body.get('sha256')
  if not isinstance(sha, str) or len(sha) != 64:
    return web.json_response({'ok': False, 'error': 'sha256 required'}, status=400)

  # The device only knows models it has already downloaded (register() runs after
  # a successful fetch). Record the server's entry first, so a model that is
  # listed but not yet present can be picked, and so picking one refreshes its
  # URL - that is how a server-side layout change reaches devices still holding
  # the old address.
  for entry in (_remote_catalog() or {}).get('models', []) or []:
    if isinstance(entry, dict) and entry.get('sha256') == sha:
      model_catalog.register_remote(entry)
      break

  if not model_catalog.select(sha):
    return web.json_response({'ok': False, 'error': 'unknown model'}, status=404)

  reboot = bool(body.get('reboot'))
  if reboot and HAS_PARAMS and Params is not None:
    try:
      Params().put_bool('DoReboot', True)
    except Exception:
      reboot = False
  return web.json_response({'ok': True, 'selected': sha, 'reboot_requested': reboot})


async def api_remove(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({'ok': False, 'error': 'invalid JSON body'}, status=400)
  sha = (body or {}).get('sha256') if isinstance(body, dict) else None
  if not isinstance(sha, str) or len(sha) != 64:
    return web.json_response({'ok': False, 'error': 'sha256 required'}, status=400)

  if HAS_PARAMS and Params is not None:
    try:
      if Params().get_bool('IsEngaged'):
        return web.json_response({'ok': False, 'error': 'disengage openpilot before removing a model'},
                                 status=409)
    except Exception:
      pass
  if sha == _active_sha():
    return web.json_response({'ok': False, 'error': 'cannot remove the model currently in use'},
                             status=409)
  # state.json's active is what the *next* boot loads. modeld can still be running a
  # different one (a switch finished downloading but was not rebooted yet), and the
  # web page hides the delete button for it - check it here too so the API cannot be
  # used to pull the files out from under the running model.
  if sha == _running_sha():
    return web.json_response({'ok': False, 'error': 'cannot remove the model modeld is running'},
                             status=409)
  if sha == model_catalog.selected_sha():
    return web.json_response({'ok': False, 'error': 'cannot remove the selected model'},
                             status=409)

  root = model_cache_dir()
  removed = []
  for path in (root / f'big_driving_supercombo-{sha[:16]}.onnx',):
    try:
      if path.is_file():
        path.unlink()
        removed.append(path.name)
    except OSError as exc:
      return web.json_response({'ok': False, 'error': str(exc)}, status=500)
  precompiled = root / 'precompiled' / sha
  try:
    if precompiled.is_dir():
      shutil.rmtree(precompiled)
      removed.append(str(precompiled.relative_to(root)))
  except OSError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=500)

  # Chunked artifacts live in the repository model dir (the local compiler's output).
  from openpilot.selfdrive.modeld.chunked_model import remove_chunked_sha

  try:
    removed += remove_chunked_sha(sha)
  except OSError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=500)

  value = model_catalog.read_index()
  if value['models'].pop(sha, None) is not None:
    model_catalog.write_index(value)
  return web.json_response({'ok': True, 'removed': removed})


async def api_source(request: web.Request) -> web.Response:
  """Pick how the big model is delivered (see chunked_model.read_model_source)."""
  try:
    body = await request.json()
  except Exception:
    return web.json_response({'ok': False, 'error': 'invalid JSON body'}, status=400)
  source = (body.get('source') if isinstance(body, dict) else None)
  try:
    chunked_model.write_model_source(source)
  except ValueError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=400)
  except OSError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=500)
  return web.json_response({'ok': True, 'source': source})


def register(app: web.Application) -> None:
  app.router.add_get('/api/egpu/models', api_models)
  app.router.add_post('/api/egpu/models/select', api_select)
  app.router.add_post('/api/egpu/models/remove', api_remove)
  app.router.add_post('/api/egpu/models/source', api_source)
