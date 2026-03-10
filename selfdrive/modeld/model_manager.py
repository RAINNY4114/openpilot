#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import time
from pathlib import Path

import requests
from requests.exceptions import RequestException, HTTPError, SSLError

from cereal import messaging, custom
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.hw import Paths
from openpilot.selfdrive.modeld.model_manager_helpers import (
  CURRENT_SELECTOR_VERSION,
  REQUIRED_MIN_SELECTOR_VERSION,
  bundle_files,
  get_active_bundle,
  is_bundle_version_compatible,
)

MODEL_URL = "https://raw.githubusercontent.com/sunnypilot/sunnypilot-docs/refs/heads/gh-pages/docs/driving_models_v15.json"
CACHE_TIMEOUT_NS = int(3600 * 1e9)
CHUNK_SIZE = 128 * 1024

PARAM_LAST_SYNC = "ModelManager_LastSyncTime"
PARAM_CACHE = "ModelManager_ModelsCache"
PARAM_DOWNLOAD_INDEX = "ModelManager_DownloadIndex"
PARAM_CLEAR_CACHE = "ModelManager_ClearCache"
PARAM_DELETE_REF = "ModelManager_DeleteBundleRef"
PARAM_ACTIVE_BUNDLE = "ModelManager_ActiveBundle"


class ModelParser:
  @staticmethod
  def _parse_download_uri(download_uri_data) -> custom.ModelManagerSP.DownloadUri:
    download_uri = custom.ModelManagerSP.DownloadUri()
    download_uri_data = download_uri_data or {}
    download_uri.uri = download_uri_data.get("url", "")
    download_uri.sha256 = download_uri_data.get("sha256", "")
    return download_uri

  @staticmethod
  def _parse_artifact(artifact_data) -> custom.ModelManagerSP.Artifact:
    artifact = custom.ModelManagerSP.Artifact()
    artifact_data = artifact_data or {}
    artifact.fileName = artifact_data.get("file_name", "")
    artifact.downloadUri = ModelParser._parse_download_uri(artifact_data.get("download_uri"))
    return artifact

  @staticmethod
  def _parse_model(model_data) -> custom.ModelManagerSP.Model | None:
    if not isinstance(model_data, dict):
      return None
    model_type = model_data.get("type")
    if not model_type:
      return None
    model = custom.ModelManagerSP.Model()
    try:
      model.type = model_type
    except Exception as err:
      cloudlog.warning(f"model_manager: unknown model type '{model_type}': {err}")
      return None
    model.artifact = ModelParser._parse_artifact(model_data.get("artifact", {}))
    if metadata := model_data.get("metadata"):
      model.metadata = ModelParser._parse_artifact(metadata)
    return model

  @staticmethod
  def _parse_overrides(overrides_data: dict[str, str]) -> list[custom.ModelManagerSP.Override]:
    overrides: list[custom.ModelManagerSP.Override] = []
    for key, value in overrides_data.items():
      override = custom.ModelManagerSP.Override()
      override.key = key
      override.value = value
      overrides.append(override)
    return overrides

  @staticmethod
  def _parse_bundle(bundle) -> custom.ModelManagerSP.ModelBundle | None:
    if not isinstance(bundle, dict):
      return None
    model_bundle = custom.ModelManagerSP.ModelBundle()
    try:
      model_bundle.index = int(bundle["index"])
    except Exception as err:
      cloudlog.warning(f"model_manager: invalid bundle index '{bundle.get('index', '')}': {err}")
      return None
    model_bundle.internalName = bundle.get("short_name", "")
    model_bundle.displayName = bundle.get("display_name", "")
    models = []
    for model in bundle.get("models", []):
      parsed = ModelParser._parse_model(model)
      if parsed is not None:
        models.append(parsed)
    if not models:
      cloudlog.warning(f"model_manager: bundle '{model_bundle.index}' has no valid models")
      return None
    model_bundle.models = models
    model_bundle.status = custom.ModelManagerSP.DownloadStatus.notDownloading
    try:
      model_bundle.generation = int(bundle.get("generation", 0))
    except Exception:
      model_bundle.generation = 0
    model_bundle.environment = bundle.get("environment", "")
    runner = bundle.get("runner", custom.ModelManagerSP.Runner.snpe)
    try:
      model_bundle.runner = runner
    except Exception as err:
      cloudlog.warning(f"model_manager: unknown runner '{runner}': {err}")
      model_bundle.runner = custom.ModelManagerSP.Runner.snpe
    model_bundle.is20hz = bundle.get("is_20hz", False)
    try:
      model_bundle.minimumSelectorVersion = int(bundle.get("minimum_selector_version", 0))
    except Exception:
      model_bundle.minimumSelectorVersion = 0
    model_bundle.overrides = ModelParser._parse_overrides(bundle.get("overrides", {}))
    model_bundle.ref = bundle.get("ref", "")
    return model_bundle

  @staticmethod
  def parse_models(json_data: dict) -> list[custom.ModelManagerSP.ModelBundle]:
    found = []
    for bundle in json_data.get("bundles", []):
      parsed = ModelParser._parse_bundle(bundle)
      if parsed is not None:
        found.append(parsed)
    return [bundle for bundle in found if is_bundle_version_compatible(bundle.to_dict())]


class ModelCache:
  def __init__(self, params: Params, cache_timeout_ns: int = CACHE_TIMEOUT_NS):
    self.params = params
    self.cache_timeout_ns = cache_timeout_ns

  @staticmethod
  def _now_ns() -> int:
    # Persisted cache timestamps should use wall time, not monotonic.
    return int(time.time() * 1e9)

  def _is_expired(self) -> bool:
    raw_last_sync = self.params.get(PARAM_LAST_SYNC) or 0
    try:
      last_sync = int(raw_last_sync)
    except (TypeError, ValueError):
      last_sync = 0
    current_time = self._now_ns()
    if last_sync == 0 or current_time < last_sync:
      return True
    return bool((current_time - last_sync) >= self.cache_timeout_ns)

  def get(self) -> tuple[dict, bool]:
    try:
      cached_data = self.params.get(PARAM_CACHE) or {}
      return cached_data, self._is_expired()
    except Exception as err:
      cloudlog.exception(f"model_manager: cache read failed: {err}")
      return {}, True

  def set(self, data: dict) -> None:
    self.params.put(PARAM_CACHE, data)
    self.params.put(PARAM_LAST_SYNC, self._now_ns())


class ModelFetcher:
  def __init__(self, params: Params):
    self.params = params
    self.model_cache = ModelCache(params)
    self.model_parser = ModelParser()

  def _fetch_and_cache_models(self) -> list[custom.ModelManagerSP.ModelBundle] | None:
    try:
      response = requests.get(MODEL_URL, timeout=10)
      if response.status_code == 404:
        raise HTTPError(f"404 Not Found: {MODEL_URL}", response=response)
      response.raise_for_status()
      json_data = response.json()
      self.model_cache.set(json_data)
      return self.model_parser.parse_models(json_data)
    except (SSLError, RequestException) as err:
      cloudlog.warning(f"model_manager: fetch transport error: {err}")
    except Exception as err:
      cloudlog.exception(f"model_manager: fetch failed: {err}")
    return None

  def get_available_bundles(self) -> list[custom.ModelManagerSP.ModelBundle]:
    cached_data, is_expired = self.model_cache.get()
    if cached_data and not is_expired:
      return self.model_parser.parse_models(cached_data)
    fetched = self._fetch_and_cache_models()
    if fetched is not None:
      return fetched
    if not cached_data:
      cloudlog.warning("model_manager: no cache available")
    return self.model_parser.parse_models(cached_data)


class ModelManagerSP:
  def __init__(self):
    self.params = Params()
    self.pm = messaging.PubMaster(["modelManagerSP"])
    self.model_fetcher = ModelFetcher(self.params)
    self.available_models: list[custom.ModelManagerSP.ModelBundle] = []
    self.selected_bundle: custom.ModelManagerSP.ModelBundle | None = None
    self.active_bundle: custom.ModelManagerSP.ModelBundle | None = get_active_bundle(self.params)
    self._download_start_times: dict[str, float] = {}

  def _calculate_eta(self, filename: str, progress: float) -> int:
    if filename not in self._download_start_times or progress <= 0:
      return 60
    elapsed = time.monotonic() - self._download_start_times[filename]
    if elapsed <= 0:
      return 60
    total_est = (elapsed / progress) * 100
    eta = total_est - elapsed
    return max(1, int(eta))

  @staticmethod
  def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
      for chunk in iter(lambda: f.read(4096), b""):
        digest.update(chunk)
    return digest.hexdigest().lower()

  def _verify_file(self, path: Path, expected_hash: str) -> bool:
    if not expected_hash:
      return True
    return self._sha256(path) == expected_hash.lower()

  def _report_status(self) -> None:
    msg = messaging.new_message("modelManagerSP", valid=True)
    state = msg.modelManagerSP
    if self.selected_bundle:
      state.selectedBundle = self.selected_bundle
    if self.active_bundle:
      state.activeBundle = self.active_bundle
    state.availableBundles = self.available_models
    self.pm.send("modelManagerSP", msg)

  def _download_artifact(self, artifact: custom.ModelManagerSP.Artifact, destination_path: Path) -> None:
    if not artifact.downloadUri.uri or not artifact.fileName:
      return

    url = artifact.downloadUri.uri
    expected_hash = artifact.downloadUri.sha256 or ""
    filename = artifact.fileName
    full_path = destination_path / filename

    if full_path.exists() and self._verify_file(full_path, expected_hash):
      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.cached
      artifact.downloadProgress.progress = 100
      artifact.downloadProgress.eta = 0
      self._report_status()
      return

    artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloading
    self._download_start_times[filename] = time.monotonic()

    try:
      with requests.get(url, stream=True, timeout=10) as response:
        response.raise_for_status()
        total_size = int(response.headers.get("content-length", 0))
        bytes_downloaded = 0

        with full_path.open("wb") as f:
          for chunk in response.iter_content(chunk_size=CHUNK_SIZE):
            if self.params.get(PARAM_DOWNLOAD_INDEX) is None:
              raise InterruptedError("download cancelled")
            if not chunk:
              continue
            f.write(chunk)
            bytes_downloaded += len(chunk)
            if total_size > 0:
              progress = (bytes_downloaded / total_size) * 100
              artifact.downloadProgress.progress = progress
              artifact.downloadProgress.eta = self._calculate_eta(filename, progress)
              self._report_status()

      if not self._verify_file(full_path, expected_hash):
        raise ValueError(f"hash validation failed for {filename}")

      artifact.downloadProgress.status = custom.ModelManagerSP.DownloadStatus.downloaded
      artifact.downloadProgress.progress = 100
      artifact.downloadProgress.eta = 0
      self._report_status()
    finally:
      self._download_start_times.pop(filename, None)

  def _process_model(self, model: custom.ModelManagerSP.Model, destination_path: Path) -> None:
    if model.metadata.fileName:
      self._download_artifact(model.metadata, destination_path)
    self._download_artifact(model.artifact, destination_path)

  def _download_bundle(self, model_bundle: custom.ModelManagerSP.ModelBundle, destination_path: Path) -> None:
    self.selected_bundle = model_bundle
    self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.downloading
    destination_path.mkdir(parents=True, exist_ok=True)
    self._report_status()

    try:
      for model in self.selected_bundle.models:
        self._process_model(model, destination_path)
      self.active_bundle = self.selected_bundle
      self.active_bundle.status = custom.ModelManagerSP.DownloadStatus.downloaded
      self.params.put(PARAM_ACTIVE_BUNDLE, self.active_bundle.to_dict())
      self.selected_bundle = None
    except InterruptedError:
      cloudlog.warning("model_manager: download cancelled")
      self.selected_bundle = None
    except Exception as err:
      cloudlog.error(f"model_manager: download failed: {err}")
      if self.selected_bundle:
        self.selected_bundle.status = custom.ModelManagerSP.DownloadStatus.failed
    finally:
      self.params.remove(PARAM_DOWNLOAD_INDEX)
      self._report_status()

  def download(self, model_bundle: custom.ModelManagerSP.ModelBundle) -> None:
    self._download_bundle(model_bundle, Path(Paths.model_root()))

  def clear_model_cache(self) -> None:
    model_dir = Path(Paths.model_root())
    if not model_dir.exists():
      return
    active_files: set[str] = set()
    if self.active_bundle is not None:
      active_files = set(bundle_files(self.active_bundle))

    for entry in model_dir.iterdir():
      if entry.is_file() and entry.name not in active_files:
        try:
          entry.unlink()
        except Exception:
          pass

  def _delete_bundle_files(self, bundle: custom.ModelManagerSP.ModelBundle) -> None:
    model_dir = Path(Paths.model_root())
    if not model_dir.exists():
      return
    for filename in bundle_files(bundle):
      target = model_dir / filename
      if target.exists():
        try:
          target.unlink()
        except Exception:
          pass

  def _handle_delete_request(self) -> None:
    delete_ref = self.params.get(PARAM_DELETE_REF)
    if not delete_ref:
      return

    bundle = next((m for m in self.available_models if m.ref == delete_ref), None)
    if bundle is None and self.active_bundle is not None and self.active_bundle.ref == delete_ref:
      bundle = self.active_bundle

    if bundle is not None:
      self._delete_bundle_files(bundle)
      if self.active_bundle is not None and self.active_bundle.ref == bundle.ref:
        self.active_bundle = None
        self.params.remove(PARAM_ACTIVE_BUNDLE)
        self.params.put("ModelRunnerTypeCache", int(custom.ModelManagerSP.Runner.stock))
    self.params.remove(PARAM_DELETE_REF)

  def main_thread(self) -> None:
    rk = Ratekeeper(1, print_delay_threshold=None)

    while True:
      try:
        self.available_models = self.model_fetcher.get_available_bundles()
        self.active_bundle = get_active_bundle(self.params)

        index_to_download = self.params.get(PARAM_DOWNLOAD_INDEX)
        if index_to_download is not None:
          try:
            index_to_download = int(index_to_download)
          except (TypeError, ValueError) as err:
            cloudlog.warning(f"model_manager: invalid download index '{index_to_download}': {err}")
            self.params.remove(PARAM_DOWNLOAD_INDEX)
            index_to_download = None
          if index_to_download is not None:
            model_to_download = next((m for m in self.available_models if m.index == index_to_download), None)
            if model_to_download is not None:
              try:
                self.download(model_to_download)
              except Exception as err:
                cloudlog.exception(f"model_manager: download error: {err}")
            self.params.remove(PARAM_DOWNLOAD_INDEX)
            self.selected_bundle = None

        if self.params.get_bool(PARAM_CLEAR_CACHE):
          self.clear_model_cache()
          self.params.remove(PARAM_CLEAR_CACHE)

        self._handle_delete_request()

        self._report_status()
        rk.keep_time()

      except Exception as err:
        cloudlog.exception(f"model_manager: main loop error: {err}")
        rk.keep_time()


def main() -> None:
  cloudlog.info("model_manager: starting")
  ModelManagerSP().main_thread()


if __name__ == "__main__":
  main()
