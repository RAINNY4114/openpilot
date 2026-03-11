from __future__ import annotations

from pathlib import Path
from typing import Iterable

from cereal import custom
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths

CURRENT_SELECTOR_VERSION = 15
REQUIRED_MIN_SELECTOR_VERSION = 12

ModelManager = custom.ModelManagerSP
ModelType = custom.ModelManagerSP.Model.Type


def _enum_raw(value: object, default: int = 0) -> int:
  if hasattr(value, "raw"):
    return int(value.raw)
  if isinstance(value, int):
    return value
  try:
    return int(value)
  except (TypeError, ValueError):
    return default


def model_root() -> Path:
  root = Path(Paths.model_root())
  root.mkdir(parents=True, exist_ok=True)
  return root


def _normalize_bundle_dict(bundle: dict) -> dict:
  normalized = dict(bundle)
  if "short_name" in normalized and "internalName" not in normalized:
    normalized["internalName"] = normalized.pop("short_name")
  if "internal_name" in normalized and "internalName" not in normalized:
    normalized["internalName"] = normalized.pop("internal_name")
  if "display_name" in normalized and "displayName" not in normalized:
    normalized["displayName"] = normalized.pop("display_name")
  if "is_20hz" in normalized and "is20hz" not in normalized:
    normalized["is20hz"] = normalized.pop("is_20hz")
  if "minimum_selector_version" in normalized and "minimumSelectorVersion" not in normalized:
    normalized["minimumSelectorVersion"] = normalized.pop("minimum_selector_version")

  # Convert runner enum string to integer for capnp compatibility
  if "runner" in normalized:
    runner = normalized["runner"]
    if isinstance(runner, str):
      runner_map = {"snpe": 0, "tinygrad": 1, "stock": 2}
      normalized["runner"] = runner_map.get(runner.lower(), 2)  # default to stock

  models = normalized.get("models")
  if isinstance(models, list):
    normalized_models = []
    for model in models:
      if not isinstance(model, dict):
        normalized_models.append(model)
        continue
      model_out = dict(model)
      # Convert model type enum string to integer
      if "type" in model_out and isinstance(model_out["type"], str):
        type_map = {"supercombo": 0, "navigation": 1, "vision": 2, "policy": 3, "offPolicy": 4, "offpolicy": 4}
        model_out["type"] = type_map.get(model_out["type"], 0)
      for key in ("artifact", "metadata"):
        artifact = model_out.get(key)
        if isinstance(artifact, dict):
          art_out = dict(artifact)
          if "file_name" in art_out and "fileName" not in art_out:
            art_out["fileName"] = art_out.pop("file_name")
          if "download_uri" in art_out and "downloadUri" not in art_out:
            art_out["downloadUri"] = art_out.pop("download_uri")
          if "download_progress" in art_out and "downloadProgress" not in art_out:
            art_out["downloadProgress"] = art_out.pop("download_progress")
          download_uri = art_out.get("downloadUri")
          if isinstance(download_uri, dict):
            uri_out = dict(download_uri)
            if "url" in uri_out and "uri" not in uri_out:
              uri_out["uri"] = uri_out.pop("url")
            art_out["downloadUri"] = uri_out
          # Convert status enum string to integer
          if "downloadProgress" in art_out and isinstance(art_out["downloadProgress"], dict):
            dp = art_out["downloadProgress"]
            if "status" in dp and isinstance(dp["status"], str):
              status_map = {"notDownloading": 0, "downloading": 1, "downloaded": 2, "cached": 3, "failed": 4}
              dp["status"] = status_map.get(dp["status"], 0)
          model_out[key] = art_out
      normalized_models.append(model_out)
    normalized["models"] = normalized_models

  overrides = normalized.get("overrides")
  if isinstance(overrides, dict):
    normalized["overrides"] = [{"key": k, "value": str(v)} for k, v in overrides.items()]
  elif isinstance(overrides, list):
    normalized_overrides = []
    for item in overrides:
      if isinstance(item, dict):
        if "key" in item and "value" in item:
          normalized_overrides.append(item)
      elif isinstance(item, (list, tuple)) and len(item) == 2:
        key, value = item
        normalized_overrides.append({"key": str(key), "value": str(value)})
    if normalized_overrides:
      normalized["overrides"] = normalized_overrides

  # Convert status enum string to integer
  if "status" in normalized and isinstance(normalized["status"], str):
    status_map = {"notDownloading": 0, "downloading": 1, "downloaded": 2, "cached": 3, "failed": 4}
    normalized["status"] = status_map.get(normalized["status"], 0)

  return normalized


def _min_selector_version(bundle: dict | custom.ModelManagerSP.ModelBundle) -> int:
  if isinstance(bundle, dict):
    if "minimumSelectorVersion" in bundle and bundle["minimumSelectorVersion"] not in (None, ""):
      return int(bundle["minimumSelectorVersion"] or 0)
    if "minimum_selector_version" in bundle and bundle["minimum_selector_version"] not in (None, ""):
      return int(bundle["minimum_selector_version"] or 0)
    return 0
  return int(bundle.minimumSelectorVersion)


def is_bundle_version_compatible(bundle: dict | custom.ModelManagerSP.ModelBundle) -> bool:
  min_version = _min_selector_version(bundle)
  if min_version == 0:
    return True
  return bool(REQUIRED_MIN_SELECTOR_VERSION <= min_version <= CURRENT_SELECTOR_VERSION)


def _coerce_bundle(bundle: dict | custom.ModelManagerSP.ModelBundle | None) -> custom.ModelManagerSP.ModelBundle | None:
  if bundle is None:
    return None
  if not isinstance(bundle, dict):
    return bundle if hasattr(bundle, "to_dict") else None
  bundle = _normalize_bundle_dict(bundle)
  try:
    return custom.ModelManagerSP.ModelBundle(**bundle)
  except Exception as e:
    from openpilot.common.swaglog import cloudlog
    cloudlog.error(f"model_manager_helpers: failed to parse bundle: {e}")
    return None


def get_active_bundle(params: Params | None = None) -> custom.ModelManagerSP.ModelBundle | None:
  params = params or Params()
  bundle = params.get("ModelManager_ActiveBundle") or {}
  if not bundle:
    return None
  if not is_bundle_version_compatible(bundle):
    return None
  return _coerce_bundle(bundle)


def get_active_model_runner(params: Params | None = None, force_check: bool = False) -> int:
  params = params or Params()
  cached = params.get("ModelRunnerTypeCache")
  if cached is not None and not force_check:
    try:
      return int(cached)
    except (TypeError, ValueError):
      pass

  runner_type = _enum_raw(custom.ModelManagerSP.Runner.stock)
  if bundle := get_active_bundle(params):
    runner_type = _enum_raw(bundle.runner, runner_type)

  if cached != runner_type:
    params.put("ModelRunnerTypeCache", int(runner_type))
  return int(runner_type)


def _find_model(bundle: custom.ModelManagerSP.ModelBundle, model_type: int) -> custom.ModelManagerSP.Model | None:
  for model in bundle.models:
    if _enum_raw(model.type) == _enum_raw(model_type):
      return model
  return None


def get_tinygrad_bundle_paths(bundle: custom.ModelManagerSP.ModelBundle) -> dict[str, Path] | None:
  vision = _find_model(bundle, ModelType.vision)
  policy = _find_model(bundle, ModelType.policy)
  if vision is None or policy is None:
    return None
  if not vision.artifact.fileName or not policy.artifact.fileName:
    return None
  if not vision.metadata.fileName or not policy.metadata.fileName:
    return None

  root = model_root()
  return {
    "vision": root / vision.artifact.fileName,
    "vision_meta": root / vision.metadata.fileName,
    "policy": root / policy.artifact.fileName,
    "policy_meta": root / policy.metadata.fileName,
  }


def get_supercombo_bundle_paths(bundle: custom.ModelManagerSP.ModelBundle) -> dict[str, Path] | None:
  model = _find_model(bundle, ModelType.supercombo)
  if model is None or not model.artifact.fileName or not model.metadata.fileName:
    return None
  root = model_root()
  return {
    "model": root / model.artifact.fileName,
    "metadata": root / model.metadata.fileName,
  }


def bundle_files(bundle: custom.ModelManagerSP.ModelBundle) -> Iterable[str]:
  for model in bundle.models:
    if model.artifact.fileName:
      yield model.artifact.fileName
    if model.metadata.fileName:
      yield model.metadata.fileName
