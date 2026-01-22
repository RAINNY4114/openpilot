from __future__ import annotations

from pathlib import Path
from typing import Iterable

from cereal import custom
from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths

CURRENT_SELECTOR_VERSION = 13
REQUIRED_MIN_SELECTOR_VERSION = 12

ModelManager = custom.ModelManagerSP
ModelType = custom.ModelManagerSP.Model.Type


def model_root() -> Path:
  root = Path(Paths.model_root())
  root.mkdir(parents=True, exist_ok=True)
  return root


def is_bundle_version_compatible(bundle: dict | custom.ModelManagerSP.ModelBundle) -> bool:
  if isinstance(bundle, dict):
    min_version = int(bundle.get("minimumSelectorVersion", 0) or 0)
  else:
    min_version = int(bundle.minimumSelectorVersion)
  return bool(REQUIRED_MIN_SELECTOR_VERSION <= min_version <= CURRENT_SELECTOR_VERSION)


def _coerce_bundle(bundle: dict | custom.ModelManagerSP.ModelBundle | None) -> custom.ModelManagerSP.ModelBundle | None:
  if bundle is None:
    return None
  if isinstance(bundle, custom.ModelManagerSP.ModelBundle):
    return bundle
  try:
    return custom.ModelManagerSP.ModelBundle(**bundle)
  except Exception:
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

  runner_type = int(custom.ModelManagerSP.Runner.stock)
  if bundle := get_active_bundle(params):
    runner_type = int(bundle.runner)

  if cached != runner_type:
    params.put("ModelRunnerTypeCache", int(runner_type))
  return int(runner_type)


def _find_model(bundle: custom.ModelManagerSP.ModelBundle, model_type: int) -> custom.ModelManagerSP.Model | None:
  for model in bundle.models:
    if int(model.type) == int(model_type):
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
