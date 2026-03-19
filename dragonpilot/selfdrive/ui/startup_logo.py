import os

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params

STARTUP_LOGO_ACTIVE_BRAND_KEY = "dp_startup_logo_active_brand"
STARTUP_LOGO_FORD_INDEX_KEY = "dp_startup_logo_ford_index"
STARTUP_LOGO_LINCOLN_INDEX_KEY = "dp_startup_logo_lincoln_index"
STARTUP_SPINNER_TRACK_INDEX_KEY = "dp_startup_spinner_track_index"

_DEFAULT_LOGO_CANDIDATES = (
  "../../dragonpilot/selfdrive/assets/images/spinner_comma.png",
  "../../dragonpilot/selfdrive/assets/dragonpilot.png",
)


def _safe_int(val: bytes | str | None, default: int) -> int:
  if not val:
    return default
  try:
    if isinstance(val, bytes):
      val = val.decode("utf-8", errors="ignore")
    return int(val)
  except (TypeError, ValueError):
    return default


def get_startup_logo_variant_index(key: str, params: Params | None = None) -> int:
  params = params or Params()
  return max(0, _safe_int(params.get(key), 0))


def _asset_rel_to_full_path(asset_rel_path: str) -> str:
  return os.path.normpath(os.path.join(BASEDIR, "selfdrive", "assets", asset_rel_path))


def _asset_exists(asset_rel_path: str) -> bool:
  return os.path.isfile(_asset_rel_to_full_path(asset_rel_path))


def get_startup_logo_indices(params: Params | None = None) -> tuple[int, int]:
  params = params or Params()
  ford_idx = get_startup_logo_variant_index(STARTUP_LOGO_FORD_INDEX_KEY, params)
  lincoln_idx = get_startup_logo_variant_index(STARTUP_LOGO_LINCOLN_INDEX_KEY, params)
  return ford_idx, lincoln_idx


def get_startup_logo_candidates(params: Params | None = None) -> list[str]:
  params = params or Params()
  ford_idx, lincoln_idx = get_startup_logo_indices(params)

  candidates: list[str] = []
  if ford_idx > 0 and lincoln_idx == 0:
    candidates.extend([
      f"../../dragonpilot/selfdrive/assets/ford{ford_idx}.png",
      f"../../dragonpilot/selfdrive/assets/startup_logos/ford{ford_idx}.png",
      f"../../dragonpilot/selfdrive/assets/{ford_idx}dragonpilot.png",
      f"../../dragonpilot/selfdrive/assets/startup_logos/{ford_idx}dragonpilot.png",
      f"../../dragonpilot/selfdrive/assets/images/ford{ford_idx}.png",
      f"../../dragonpilot/selfdrive/assets/images/{ford_idx}dragonpilot.png",
    ])
  elif lincoln_idx > 0 and ford_idx == 0:
    candidates.extend([
      f"../../dragonpilot/selfdrive/assets/lincoln{lincoln_idx}.png",
      f"../../dragonpilot/selfdrive/assets/startup_logos/lincoln{lincoln_idx}.png",
      f"../../dragonpilot/selfdrive/assets/{lincoln_idx}dragonpilot.png",
      f"../../dragonpilot/selfdrive/assets/startup_logos/{lincoln_idx}dragonpilot.png",
      f"../../dragonpilot/selfdrive/assets/images/lincoln{lincoln_idx}.png",
      f"../../dragonpilot/selfdrive/assets/images/{lincoln_idx}dragonpilot.png",
    ])

  candidates.extend(_DEFAULT_LOGO_CANDIDATES)

  deduped: list[str] = []
  seen: set[str] = set()
  for candidate in candidates:
    if candidate not in seen:
      deduped.append(candidate)
      seen.add(candidate)
  return deduped


def resolve_startup_logo_asset(params: Params | None = None) -> str:
  for candidate in get_startup_logo_candidates(params):
    if _asset_exists(candidate):
      return candidate

  return _DEFAULT_LOGO_CANDIDATES[-1]


def resolve_startup_logo_filename(params: Params | None = None) -> str:
  return os.path.basename(resolve_startup_logo_asset(params))


def get_spinner_track_variant_index(params: Params | None = None) -> int:
  params = params or Params()
  return max(0, _safe_int(params.get(STARTUP_SPINNER_TRACK_INDEX_KEY), 0))


def get_spinner_track_candidates(params: Params | None = None) -> list[str]:
  idx = get_spinner_track_variant_index(params)

  candidates: list[str] = []
  if idx > 0:
    candidates.extend([
      f"images/{idx}spinner_track.png",
      f"images/spinner_track{idx}.png",
    ])

  candidates.append("images/spinner_track.png")

  deduped: list[str] = []
  seen: set[str] = set()
  for candidate in candidates:
    if candidate not in seen:
      deduped.append(candidate)
      seen.add(candidate)
  return deduped


def resolve_spinner_track_asset(params: Params | None = None) -> str:
  for candidate in get_spinner_track_candidates(params):
    if _asset_exists(candidate):
      return candidate
  return "images/spinner_track.png"


def resolve_spinner_track_filename(params: Params | None = None) -> str:
  return os.path.basename(resolve_spinner_track_asset(params))
