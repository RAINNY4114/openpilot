#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections.abc import Callable
import os
os.environ['GMMU'] = '0'
import numpy as np
import threading
import time
from setproctitle import setproctitle
from tinygrad.tensor import Tensor

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.selfdrive.modeld.helpers import chestnut_present
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.cereal.services import SERVICE_LIST
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.file_chunker import open_file_chunked
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.system import sentry
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, AUTO_LC_CONFIRM_DELAY_SEC
from openpilot.selfdrive.controls.lib.auto_overtake import AutoOvertakeHelper, LANE_PREF_AUTO
# [AUTO_AVOIDANCE_WIRING] 紧急转向避让（>80km/h 段的安全兜底通道）
from openpilot.selfdrive.controls.lib.auto_avoidance import AutoAvoidanceHelper
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value
from openpilot.selfdrive.modeld.modeld import ChestnutState

from openpilot.selfdrive.modeld.compile_modeld import (
  MODELD_INPUTS,
  make_input_queues as make_stock_input_queues,
)
from openpilot.sunnypilot.modeld_v2.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState, get_curvature_from_output
from openpilot.sunnypilot.modeld_v2.parse_model_outputs import Parser
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.meta_helper import load_meta_constants
from openpilot.sunnypilot.modeld_v2.camera_offset_helper import CameraOffsetHelper
from openpilot.sunnypilot.modeld_v2.compile_modeld import (derive_frame_skip, make_split_input_queues,
                                                           make_supercombo_input_queues, nv12_copy_size,
                                                           WARP_INPUTS, POLICY_INPUTS)
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.modeld_v2.helpers import load_oob
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.selfdrive.controls.lib.relc import RoadEdgeLaneChangeController

PROCESS_NAME = "openpilot.selfdrive.modeld.modeld_tinygrad"
BIG_MODEL_TIMEOUT = 60


# ============================================================================
# [AUTO_OVERTAKE_WIRING] configuration + diagnostics
#
# The prebuilt libparams_c.so of this fork does not know the custom
# AutoOvertake* keys, so Params() raises UnknownKeyName for them.  Follow the
# convention already used by card.py::read_param_direct and
# selfdrived.py::_radar_loss_no_disable and read the param files directly.
# ============================================================================
AO_PARAM_DIRS = ("/dev/shm/params", "/data/params/d")
AO_LOG_PATH = "/data/media/0/auto_overtake.log"
AO_LOG_MAX_BYTES = 5_000_000

AO_CFG_DEFAULTS = {
  "enabled": 0,
  "lane_preference": LANE_PREF_AUTO,
  "confirm_delay_sec": AUTO_LC_CONFIRM_DELAY_SEC,
  "min_cruise_kph": 60.0,  # [AO_CITY_60] 与 AutoOvertakeMinCruiseKph / OVERTAKE_MIN_SPEED_FLOOR_MS 对齐
  "lane_prob_min": 0.20,
}


def _ao_read_raw(key):
  for d in AO_PARAM_DIRS:
    try:
      with open(os.path.join(d, key)) as f:
        return f.read().strip()
    except Exception:
      continue
  return None


def _ao_read_num(key, default, cast=float):
  raw = _ao_read_raw(key)
  if not raw:
    return default
  try:
    return cast(float(raw))
  except (TypeError, ValueError):
    return default


# ============================================================================
# [AUTO_AVOIDANCE_WIRING] config + diagnostics
#
# 与 AO 同一套约定：fork 的 libparams_c.so 不认识自定义键，
# 因此直接读参数文件，避免 Params() 抛 UnknownKeyName。
# ============================================================================
AA_LOG_PATH = "/data/media/0/auto_avoidance.log"
AA_LOG_MAX_BYTES = 5_000_000

AA_CFG_DEFAULTS = {
  "enabled": 1,
  # ------------------------------------------------------------------
  # [AA_STABLE_RATIO] 逃逸车道稳定性判定模式。
  #
  # strict : 要求逃逸车道"连续 stable_sec 秒"安全，任何一帧不合格即归零。
  #          实测在隧道内 laneLineProbs / BSM 抖动时，计时器只能累积
  #          到 ~0.03s 便被反复打断，导致 left_stable 恒为 False，
  #          紧急转向避让永远无法从 idle 进入 avoiding。
  # ratio  : 滑动窗口容抖模式，窗口内 OK 占比 >= ratio 即可通过。
  #          这是这次修复要验证的路径。
  # ------------------------------------------------------------------
  "stable_mode": "ratio",
  "stable_ratio": 0.60,
  "stable_window_sec": 0.60,
  # ------------------------------------------------------------------
  # [AA_OBS_HOLD] 障碍物输入短时保持（秒）。
  #
  # 背景：本机 radarTracks 只有约 8.3 Hz，而 AA 跑在 modeld ~20 Hz。
  # 更糟的是接线侧 abs(y) <= 1.8 的筛选在点云漂移帧会返回 None，
  # 使 obstacle_in_path 频繁掉到 False → AA 内部 _critical_frames
  # 被反复清零，永远攒不满 CRITICAL_CONFIRM_FRAMES=3。
  # 实测后果：障碍物 t=27.5s 就出现（TTC 0.125s），AA 却迟到
  # t=32.5s 才触发，晚了约 5 秒。
  #
  # 这里做的是"保持最近一次有效障碍读数 hold_sec 秒"，覆盖 3-4 个
  # 雷达周期，从而让 CRITICAL 确认能够连续累积。
  # 0 == 关闭（行为与旧版完全一致）。
  # ------------------------------------------------------------------
  "obs_hold_sec": 0.45,
  # ------------------------------------------------------------------
  # [AA_BRAKE_FALLBACK] 纵向兜底（刹车）参数。
  #
  # brake_max  ：请求减速度硬上限（m/s^2）。0 == 完全关闭，
  #              行为与旧版（纵向不介入）完全一致。默认 0.0。
  # brake_gain ：需求缩放系数，1.0 为标准。
  # brake_ttc_ref：TTC 参考值（秒）。TTC >= 该值时不请求刹车；
  #                TTC 越小请求越大，TTC<=0 时达到 brake_max。
  #
  # 注意：本会话只做"接口预留"，该请求值目前**不会**被转发到
  #       任何执行器，仅用于日志与 /data/params/d/AvoidBrakeReq 观察。
  # ------------------------------------------------------------------
  "brake_gain": 1.0,
  "brake_max": 0.0,
  "brake_ttc_ref": 3.0,
}


def _aa_read_cfg():
  cfg = dict(AA_CFG_DEFAULTS)
  cfg["enabled"] = _ao_read_num("AutoAvoidanceEnabled", cfg["enabled"], int) != 0
  # 允许通过 /data/params/d/AvoidStableMode 热切换 strict / ratio，
  # 便于不改代码就回滚到旧行为做 A/B 对比。
  try:
    _raw = _ao_read_raw("AvoidStableMode")
    if _raw:
      _m = str(_raw).strip().lower()
      if _m in ("strict", "ratio"):
        cfg["stable_mode"] = _m
  except Exception:
    pass
  # 障碍物保持秒数，可通过 /data/params/d/AvoidObsHold 热调（0 == 关闭）。
  try:
    _raw = _ao_read_raw("AvoidObsHold")
    if _raw:
      _v = float(_raw)
      if _v >= 0.0:
        cfg["obs_hold_sec"] = _v
  except Exception:
    pass
  # [AA_BRAKE_FALLBACK] 纵向兜底参数热读。
  # AvoidBrakeMax 默认未创建 -> 保持 0.0（完全关闭，与旧版一致）。
  try:
    _raw = _ao_read_raw("AvoidBrakeMax")
    if _raw:
      _v = float(_raw)
      if _v >= 0.0:
        cfg["brake_max"] = _v
  except Exception:
    pass
  try:
    _raw = _ao_read_raw("AvoidBrakeGain")
    if _raw:
      _v = float(_raw)
      if _v >= 0.0:
        cfg["brake_gain"] = _v
  except Exception:
    pass
  try:
    _raw = _ao_read_raw("AvoidBrakeTtcRef")
    if _raw:
      _v = float(_raw)
      if _v > 0.0:
        cfg["brake_ttc_ref"] = _v
  except Exception:
    pass
  return cfg


_aa_log_state = {"t": 0.0}


def _ao_write_param(key, value):
  """把字符串写入 /data/params/d（与 AvoidObsHold 等自定义参数同目录）。

  注意：不用 AO_PARAM_DIRS[0]（/dev/shm/params 是 tmpfs，重启即丢），
  直接写持久目录，便于外部取证时与其它自定义参数并列查看。
  fork 的 libparams_c.so 不认识自定义键，因此直接写文件，
  避免 Params().put 抛 UnknownKeyName。写失败静默忽略。
  """
  for _d in ("/data/params/d", "/dev/shm/params"):
    try:
      if not os.path.isdir(_d):
        continue
      _p = os.path.join(_d, str(key))
      with open(_p, "w") as f:
        f.write(str(value))
      return True
    except Exception:
      continue
  return False


# [AA_BRAKE_FALLBACK] 上次发布的刹车请求值（避免每帧都写盘）
_aa_brake_pub = {"v": -1.0}


def _aa_log(fields):
  """把 AA 的每帧状态写入 /data/media/0/auto_avoidance.log（1Hz）。"""
  try:
    import json as _json
    now = time.monotonic()
    if now - _aa_log_state["t"] < 1.0:
      return
    _aa_log_state["t"] = now
    try:
      if os.path.getsize(AA_LOG_PATH) > AA_LOG_MAX_BYTES:
        os.replace(AA_LOG_PATH, AA_LOG_PATH + ".1")
    except Exception:
      pass
    with open(AA_LOG_PATH, "a") as f:
      f.write(_json.dumps(fields, separators=(",", ":")) + "\n")
  except Exception:
    pass


def _ao_read_cfg():
  cfg = dict(AO_CFG_DEFAULTS)
  cfg["enabled"] = _ao_read_num("AutoOvertakeEnabled", cfg["enabled"], int) != 0
  cfg["lane_preference"] = _ao_read_num("AutoOvertakeLanePref", cfg["lane_preference"], int)
  cfg["confirm_delay_sec"] = _ao_read_num("AutoOvertakeConfirmSec", cfg["confirm_delay_sec"])
  cfg["min_cruise_kph"] = _ao_read_num("AutoOvertakeMinCruiseKph", cfg["min_cruise_kph"])
  cfg["lane_prob_min"] = _ao_read_num("AutoOvertakeLaneProbMin", cfg["lane_prob_min"])
  return cfg


_ao_log_t = [-1e9]


def _ao_log(fields):
  """Append one diagnostic line per second.  Never raises."""
  now = time.monotonic()
  if now - _ao_log_t[0] < 1.0:
    return
  _ao_log_t[0] = now
  try:
    if os.path.exists(AO_LOG_PATH) and os.path.getsize(AO_LOG_PATH) > AO_LOG_MAX_BYTES:
      with open(AO_LOG_PATH, "w") as f:
        f.write("")
    with open(AO_LOG_PATH, "a") as f:
      f.write(time.strftime("%H:%M:%S") + " " + " ".join(f"{k}={v}" for k, v in fields.items()) + "\n")
  except Exception:
    pass


def _pkl_exists(path):
  from openpilot.common.file_chunker import get_manifest_path
  return os.path.exists(path) or os.path.exists(get_manifest_path(path))


def _find_driving_pkl(bundle):
  if (override := os.environ.get('COMBINED_MODEL_PKL')) and _pkl_exists(override):
    return override
  if bundle is None or not bundle.models:
    return None
  from openpilot.common.hardware.hw import Paths
  model_root = Paths.model_root()

  pkl_name = bundle.models[0].artifact.fileName
  pkl_path = os.path.join(model_root, pkl_name)
  if _pkl_exists(pkl_path):
    return pkl_path
  return None


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


class ModelState(ModelStateBase):
  inputs: dict[str, np.ndarray]
  prev_desire: np.ndarray

  def __init__(self, cam_w: int, cam_h: int, chestnut: bool = False):
    ModelStateBase.__init__(self)

    env_pkl = os.environ.get('COMBINED_MODEL_PKL')
    if env_pkl and os.path.exists(env_pkl):
      model_bundle = None
    else:
      model_bundle = get_active_bundle(chestnut=chestnut)
    self.generation = model_bundle.generation if model_bundle is not None else None
    overrides = {override.key: override.value for override in model_bundle.overrides} if model_bundle else {}

    self.LAT_SMOOTH_SECONDS = float(overrides.get('lat', ".0"))
    self.LONG_SMOOTH_SECONDS = float(overrides.get('long', ".0"))
    self.MIN_LAT_CONTROL_SPEED = 0.3
    self.PLANPLUS_CONTROL: float = 1.0
    self.chestnut = chestnut

    pkl_path = _find_driving_pkl(model_bundle)
    assert pkl_path is not None, f"No driving pkl found for {'chestnut' if chestnut else 'small model'} — all models must be compiled with compile_modeld.py"
    self._init_combined(pkl_path, cam_w, cam_h, model_bundle)

  def _init_combined(self, pkl_path, cam_w, cam_h, bundle):
    cloudlog.warning(f"loading combined pkl: {pkl_path}")
    jits = load_oob(open_file_chunked(pkl_path))

    metadata = jits['metadata']
    self.WARP_DEV = metadata.get('warp_dev', 'QCOM') if COMMA_HARDWARE else 'CPU'
    self.DEV = ('AMD' if self.chestnut else 'QCOM') if COMMA_HARDWARE else 'CPU'
    self.QUEUE_DEV = self.DEV
    self.is_run_model = 'run_model' in jits

    nv12_info = get_nv12_info(cam_w, cam_h)
    self.frame_copy_size = nv12_copy_size(*nv12_info[:3])
    self.full_frames: dict = {}
    self._blob_cache: dict = {}
    self.frame_buffers: dict = {}

    if self.is_run_model or 'model' in metadata:
      model_metadata = metadata.get('model', metadata)
      self.input_shapes = model_metadata['input_shapes']
      self.vision_output_slices = model_metadata['output_slices']
      self.policy_output_slices = {}
      self._policy_slices_list = []
      self._combined_model_type = 'supercombo'
      self._vision_input_names = [key for key in self.input_shapes if 'img' in key]
      self.frame_skip = derive_frame_skip({}, self.input_shapes)
      if self.is_run_model:
        self.input_queues, self.numpy_inputs, self.frame_buffers = make_stock_input_queues(
          self.input_shapes, self.frame_skip, device=self.DEV, frame_copy_size=self.frame_copy_size)
        self.frame_views, self.npy = self.frame_buffers, self.numpy_inputs
        self.run_model, self.run_policy, self.warp = jits['run_model'][(cam_w, cam_h)], None, None
      else:
        self.input_queues, self.numpy_inputs = make_supercombo_input_queues(self.input_shapes, self.frame_skip, device=self.QUEUE_DEV)
        self.run_model, self.run_policy, self.warp = None, jits['run_policy'], jits[(cam_w, cam_h)]
    else:
      self.run_model, self.run_policy, self.warp = None, jits['run_policy'], jits[(cam_w, cam_h)]
      vision_metadata = metadata['vision']
      policy_keys = [k for k in metadata if k not in ('vision', 'warp_dev')]
      self._combined_model_type = 'split' if policy_keys == ['policy'] else 'multi_policy'
      self.vision_output_slices = vision_metadata['output_slices']
      self._policy_keys = policy_keys
      self._policy_slices_list = [metadata[k]['output_slices'] for k in policy_keys]
      self.policy_output_slices = self._policy_slices_list[0]
      self._has_on_policy = any('on' in k.lower() for k in policy_keys)
      self._vision_input_names = [key for key in vision_metadata['input_shapes'] if 'img' in key]
      first_policy_meta = metadata[policy_keys[0]]
      frame_skip = derive_frame_skip(vision_metadata['input_shapes'], first_policy_meta['input_shapes'])
      self.input_queues, self.numpy_inputs = make_split_input_queues(vision_metadata['input_shapes'],
                                                                     first_policy_meta['input_shapes'],
                                                                     frame_skip, device=self.QUEUE_DEV)

    self._desire_key = next(key for key in self.numpy_inputs if key.startswith('desire'))
    self._road_key = next(key for key in self._vision_input_names if 'big' not in key)
    self._wide_key = next(key for key in self._vision_input_names if 'big' in key)
    self.frame_buf_params = dict.fromkeys(self._vision_input_names, nv12_info)

    is_20hz = bundle.is20hz if bundle else self._combined_model_type in ('split', 'multi_policy')
    if is_20hz:
      from openpilot.sunnypilot.models.split_model_constants import SplitModelConstants
      self.constants = SplitModelConstants()
    else:
      self.constants = ModelConstants()

    self.parser = Parser()
    self.prev_desire = np.zeros(self.constants.DESIRE_LEN, dtype=np.float32)

    if self.warp is not None:
      self.full_frames = {k: Tensor(np.zeros(nv12_info[3], dtype=np.uint8), device=self.WARP_DEV).contiguous().realize() for k in self._vision_input_names}
      self.warp(**{k: self.input_queues[k] for k in WARP_INPUTS}, frame=self.full_frames[self._road_key], big_frame=self.full_frames[self._wide_key])

  def warmup(self) -> None:
    dummy_size = self.frame_copy_size if self.is_run_model else self.frame_buf_params[self._road_key][3]
    dummy_frames = {k: np.zeros(dummy_size, dtype=np.uint8) for k in self._vision_input_names}
    transforms = {k: np.eye(3, dtype=np.float32) for k in [self._road_key, self._wide_key] if k}
    dummy_inputs = {k: np.zeros(v.shape, dtype=v.dtype) for k, v in self.numpy_inputs.items() if k not in ['tfm', 'big_tfm', 'prev_feat']}
    self.run(dummy_frames, transforms, dummy_inputs)
    if self.is_run_model:
      self.input_queues, self.numpy_inputs, self.frame_buffers = make_stock_input_queues(
        self.input_shapes, self.frame_skip, device=self.DEV, frame_copy_size=self.frame_copy_size)
      self.frame_views = self.frame_buffers
      self.npy = self.numpy_inputs
    else:
      for v in self.numpy_inputs.values():
        v[:] = 0
      self.full_frames.clear()
      self._blob_cache.clear()
    self.prev_desire[:] = 0

  @property
  def mlsim(self) -> bool:
    return bool(self.generation is not None and self.generation >= 11)

  @property
  def vision_input_names(self) -> list[str]:
    return self._vision_input_names

  @property
  def desire_key(self) -> str:
    return self._desire_key

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray],
          after_enqueue: Callable[[], None] | None = None) -> dict[str, np.ndarray] | None:
    if self.is_run_model:
      for key, buf in bufs.items():
        data = buf.data if hasattr(buf, 'data') else buf
        np.copyto(self.frame_buffers[key], np.frombuffer(data, dtype=np.uint8, count=self.frame_copy_size))
    else:
      for key, buf in bufs.items():
        ptr = np.frombuffer(buf.data, dtype=np.uint8).ctypes.data
        cache_key = (key, ptr)
        if cache_key not in self._blob_cache:
          self._blob_cache[cache_key] = Tensor.from_blob(ptr, (self.frame_buf_params[key][3],), dtype='uint8', device=self.WARP_DEV)
        self.full_frames[key] = self._blob_cache[cache_key]

    desire_key = self.desire_key
    inputs[desire_key][0] = 0
    self.numpy_inputs[desire_key][:] = np.where(inputs[desire_key] - self.prev_desire > .99, inputs[desire_key], 0)
    self.prev_desire[:] = inputs[desire_key]

    for key in ('traffic_convention', 'lateral_control_params', 'action_t'):
      if key in self.numpy_inputs and key in inputs:
        self.numpy_inputs[key][:] = inputs[key]

    self.numpy_inputs['tfm'][:, :] = transforms[self._road_key].reshape(3, 3)
    self.numpy_inputs['big_tfm'][:, :] = transforms[self._wide_key].reshape(3, 3)

    if self.run_model is not None:
      outs, = self.run_model(**{k: self.input_queues[k] for k in MODELD_INPUTS})
      raw_outputs = outs
    else:
      assert self.warp is not None and self.run_policy is not None
      warped = self.warp(**{k: self.input_queues[k] for k in WARP_INPUTS}, frame=self.full_frames[self._road_key], big_frame=self.full_frames[self._wide_key])
      raw_outputs = self.run_policy(**{k: self.input_queues[k] for k in POLICY_INPUTS if k in self.input_queues}, warped=warped)

    if after_enqueue is not None:
      after_enqueue()

    if self._combined_model_type == 'supercombo':
      model_output = raw_outputs.numpy().flatten()
      if self.chestnut and not np.all(np.isfinite(model_output)):
        raise RuntimeError("model output not finite")
      sliced = {k: model_output[np.newaxis, v] for k, v in self.vision_output_slices.items()}
      outputs = self.parser.parse_outputs(sliced)
      if 'prev_feat' in self.numpy_inputs and 'hidden_state' in self.vision_output_slices:
        self.numpy_inputs['prev_feat'][:] = model_output[self.vision_output_slices['hidden_state']]
    else:
      vision_output = raw_outputs[0].numpy().flatten()
      vision_sliced = {k: vision_output[np.newaxis, v] for k, v in self.vision_output_slices.items()}
      outputs = self.parser.parse_vision_outputs(vision_sliced)

      if 'prev_feat' in self.numpy_inputs and 'hidden_state' in self.vision_output_slices:
        self.numpy_inputs['prev_feat'][:] = vision_output[self.vision_output_slices['hidden_state']]

      for i, policy_slices in enumerate(self._policy_slices_list):
        policy_output = raw_outputs[i + 1].numpy().flatten()
        policy_sliced = {k: policy_output[np.newaxis, v] for k, v in policy_slices.items()}
        parsed = self.parser.parse_policy_outputs(policy_sliced)
        if ('off' in self._policy_keys[i]
          and self._has_on_policy
          and any('plan' in self._policy_slices_list[j] for j, k in enumerate(self._policy_keys) if 'on' in k.lower())):

          parsed.pop('plan', None)

        outputs.update(parsed)

      if 'planplus' in outputs and 'plan' in outputs:
        outputs['plan'] = outputs['plan'] + outputs['planplus']

    if 'desired_curvature' in outputs and 'prev_desired_curv' in self.numpy_inputs:
      buf = self.numpy_inputs['prev_desired_curv']
      buf[0, :-1] = buf[0, 1:]
      buf[0, -1, :] = outputs['desired_curvature'][0, :] if not self.mlsim else 0

    return outputs

  def get_action_from_model(self, model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                            lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
    if 'action' not in model_output:
      plan = model_output['plan'][0]
      desired_accel = get_accel_from_plan(plan[:, Plan.VELOCITY][:, 0], plan[:, Plan.ACCELERATION][:, 0], self.constants.T_IDXS,
                                          action_t=long_action_t)

      curvature_plan = (plan + (self.PLANPLUS_CONTROL - 1.0) * model_output['planplus'][0]
                        if 'planplus' in model_output and self.PLANPLUS_CONTROL != 1.0 else plan)
      desired_curvature = get_curvature_from_output(model_output, curvature_plan, v_ego, lat_action_t, self.mlsim)
    else:
      desired_accel = model_output['action'][0, 1]
      desired_curvature = model_output['action'][0, 0] / (max(1.0, v_ego))**2

    stop = v_ego < 0.3 and desired_accel < 0.1
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, self.LONG_SMOOTH_SECONDS)

    if self.generation is not None and self.generation >= 10: # smooth curvature for post FOF models
      if v_ego > self.MIN_LAT_CONTROL_SPEED:
        desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, self.LAT_SMOOTH_SECONDS)
      else:
        desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature), desiredAcceleration=float(desired_accel), shouldStop=bool(stop))


def main(demo=False):
  cloudlog.warning("modeld init")

  sentry.set_tag("daemon", PROCESS_NAME)
  cloudlog.bind(daemon=PROCESS_NAME)
  setproctitle(PROCESS_NAME)
  config_realtime_process(7, 54)

  CHESTNUT = chestnut_present()
  if CHESTNUT:
    os.environ['HCQDEV_WAIT_TIMEOUT_MS'] = '3000'

  params = Params()
  params.put_bool("ChestnutLoading", CHESTNUT)
  params.remove("ChestnutActive")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_NARROW_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_NARROW_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_NARROW_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  cloudlog.warning("loading model")
  st = time.monotonic()

  model = None
  if CHESTNUT:
    big_model = None
    def load_big():
      nonlocal big_model
      try:
        m = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=True)
        m.warmup()
        big_model = m
      except Exception:
        cloudlog.exception("chestnut load failed")
    loader = threading.Thread(target=load_big, daemon=True)
    loader.start()
    loader.join(BIG_MODEL_TIMEOUT)
    model = big_model
    if model is None:
      params.put_bool("ChestnutModelError", True)
    params.put_bool("ChestnutActive", model is not None)
    if model is not None:
      params.remove("ChestnutModelError")

  small_model = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=False) if model is None or CHESTNUT else None
  if model is None:
    model = small_model
  params.put_bool("ChestnutLoading", False)
  assert model is not None
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # messaging
  pub_socks = ["modelV2", "drivingModelData", "cameraOdometry", "modelDataV2SP"] + (["chestnutState"] if CHESTNUT else [])
  pm = PubMaster(pub_socks)
  sm = SubMaster(["deviceState", "carState", "narrowRoadCameraState", "extrinsicsCalibration", "driverMonitoringState", "carControl", "lateralDelay", "radarState", "radarTracks"])  # [AUTO_AVOIDANCE_WIRING] 订阅障碍物点云

  publish_state = PublishState()
  chestnut_state = ChestnutState(pm, model.chestnut) if CHESTNUT else None

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / model.constants.MODEL_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()
  camera_offset_helper = CameraOffsetHelper()


  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + model.LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  DH = DesireHelper()
  # [AUTO_OVERTAKE_WIRING]
  AO = AutoOvertakeHelper()
  # [AUTO_AVOIDANCE_WIRING] 紧急转向避让 helper（输出 lane_request -> DesireHelper）
  AA = AutoAvoidanceHelper()
  # [AA_OBS_HOLD] 障碍物短时保持状态（跨帧），用于对抗 radarTracks 8.3Hz 稀疏性。
  # 结构：{"d": float|None, "vr": float|None, "y": float|None, "t": float}
  _aa_obs_hold = {"d": None, "vr": None, "y": None, "t": 0.0}
  aa_cfg = _aa_read_cfg()
  ao_cfg = _ao_read_cfg()
  meta_constants = load_meta_constants()
  RELC = RoadEdgeLaneChangeController()

  while True:
    # Keep receiving frames until we are at least 1 frame ahead of previous extra frame
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      # Keep receiving extra frames until frame id matches main camera
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}),\
                       extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})")

    else:
      # Use single camera
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["narrowRoadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    if sm.frame % 60 == 0:
      model.lat_delay = get_lat_delay(params, sm["lateralDelay"].lateralDelay)
      model.PLANPLUS_CONTROL = params.get("PlanplusControl", return_default=True)
      camera_offset_helper.set_offset(params.get("CameraOffset", return_default=True))
      ao_cfg = _ao_read_cfg()  # [AUTO_OVERTAKE_WIRING]
      aa_cfg = _aa_read_cfg()  # [AUTO_AVOIDANCE_WIRING]
    lat_delay = model.lat_delay + model.LAT_SMOOTH_SECONDS
    if sm.updated["extrinsicsCalibration"] and sm.seen['narrowRoadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["extrinsicsCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['narrowRoadCameraState'].sensor))]
      main_intrinsics = dc.wide_road.intrinsics if main_wide_camera else dc.narrow_road.intrinsics
      model_transform_main = get_warp_matrix(device_from_calib_euler, main_intrinsics, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.wide_road.intrinsics, True).astype(np.float32)
      model_transform_main, model_transform_extra = camera_offset_helper.update(model_transform_main, model_transform_extra, sm, main_wide_camera)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(model.constants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < model.constants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}

    frame_delay = DT_MDL # compensate for time passed since the frame was captured: current_time - timestamp_eof is 50ms on average
    action_delay = DT_MDL / 2 # middle of the interval between model output (current state) and next frame (expected state)
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay

    inputs:dict[str, np.ndarray] = {
      model.desire_key: vec_desire,
      'traffic_convention': traffic_convention,
    }

    if 'lateral_control_params' in model.numpy_inputs:
      inputs['lateral_control_params'] = np.array([v_ego, lat_delay], dtype=np.float32)

    if 'action_t' in model.numpy_inputs:
      inputs['action_t'] = np.array([lat_action_t, long_action_t], dtype=np.float32)

    mt1 = time.perf_counter()
    try:
      send_chestnut = (chestnut_state is not None and
                       run_count % round(model.constants.MODEL_FREQ / SERVICE_LIST['chestnutState'].frequency) == 0)
      model_output = model.run(bufs, transforms, inputs, chestnut_state.send if send_chestnut else None)
    except Exception:
      if not params.get_bool("ChestnutActive"):
        raise
      cloudlog.exception("chestnut failed, falling back to small")
      params.put_bool("ChestnutModelError", True)
      params.put_bool("ChestnutActive", False)
      assert small_model is not None
      model = small_model
      if chestnut_state is not None:
        chestnut_state.big = False
      run_count = 0
      model_output = None
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')
      mdv2sp_send = messaging.new_message('modelDataV2SP')

      action = model.get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
      prev_action = action
      fill_model_msg(drivingdata_send, modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen, meta_constants)
      modelv2_send.modelV2.big = model.chestnut

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      left_edge, right_edge = RELC.update_and_fill(modelv2_send.modelV2, mdv2sp_send.modelDataV2SP, v_ego)
      # ------------------------------------------------------------------
      # [AUTO_OVERTAKE_WIRING] automatic overtaking decision layer.
      #
      # OEM Delphi radar (radarState.leadOne) is the ONLY overtake trigger.
      # The helper returns the LaneChangeDirection it wants to move to;
      # DesireHelper turns that into a normal lane change after a confirmation
      # delay, so the turn signal is visible and the driver can always cancel.
      # ------------------------------------------------------------------
      ao_dir = log.LaneChangeDirection.none
      ao_fields = {"en": int(bool(ao_cfg["enabled"]))}
      if ao_cfg["enabled"] and sm.seen['radarState']:
        try:
          cs = sm['carState']
          lead = sm['radarState'].leadOne
          lane_probs = list(modelv2_send.modelV2.laneLineProbs)
          lp_left = float(lane_probs[0]) if len(lane_probs) > 0 else 1.0
          lp_right = float(lane_probs[3]) if len(lane_probs) > 3 else 1.0
          v_cruise = float(cs.cruiseState.speed) if cs.cruiseState.enabled else 0.0
          # vRel is the most reliable field; vLead/vLeadK are noisy on this car.
          v_lead = max(0.0, v_ego + float(lead.vRel))
          left_ok = (not bool(left_edge)) and lp_left >= ao_cfg["lane_prob_min"]
          right_ok = (not bool(right_edge)) and lp_right >= ao_cfg["lane_prob_min"]
          # ------------------------------------------------------------
          # [AO_SAFETY_TIGHTEN] rear approach zone from the Ford BSM
          # payload (published by carstate.py as AOBsmZone / AOBsmFault).
          # ------------------------------------------------------------
          _lz, _rz, _rf = None, None, None
          try:
            _packed = _ao_read_raw("AOBsmZone")
            if _packed:
              _pv = int(float(_packed))
              _lz = _pv // 100
              _rz = _pv % 100
            _f = _ao_read_raw("AOBsmFault")
            if _f:
              _fv = int(float(_f))
              _rf = bool(_fv % 10) or bool(_fv // 10)
          except Exception:
            _lz, _rz, _rf = None, None, None

          # ------------------------------------------------------------
          # [AO_MR76_WIRE] 读取 card.py 发布的 MR76 相邻车道占用。
          #
          # card.py 与 modeld.py 是两个进程，跨进程用 params 文件传递
          # （与 AOBsmZone 同一套约定）。语义：
          #   AOMr76Fresh = 0  未接入 / stale / 未使能
          #   AOMr76Fresh = 1  数据可用 -> AOMr76Lane 十位=左 个位=右
          #
          # ★ 传参约定（与 auto_overtake 的门逻辑严格对应）：
          #   available=None  -> 该侧"根本未接入"，门必须放行（不阻断）
          #   available=True  -> 已接入且可用，valid/age 参与严格判定
          #   available=False -> 已接入但当前不可用 -> fail-closed 否决
          # 因此 AOMr76Fresh=0 时传 available=None（未接入语义），
          # 而不是传 False —— 后者会被 _sensor_fresh 直接判死并 veto。
          # ------------------------------------------------------------
          _mr76_fresh = _ao_read_num("AOMr76Fresh", 0.0) != 0.0
          _mr76_lane = int(_ao_read_num("AOMr76Lane", 0.0))
          _mr76_left_occ = bool((_mr76_lane // 10) % 10) if _mr76_fresh else False
          _mr76_right_occ = bool(_mr76_lane % 10) if _mr76_fresh else False

          ao_dir = AO.update(
            enabled=True,
            lc_state=DH.lane_change_state,
            v_ego=v_ego,
            v_cruise=v_cruise,
            lead_present=bool(lead.present),
            lead_d=float(lead.dRel),
            v_lead=v_lead,
            left_ok=left_ok,
            right_ok=right_ok,
            is_rhd=bool(is_rhd),
            manual_blinker=bool(cs.leftBlinker or cs.rightBlinker),
            bsm_available=True,
            left_bsm=bool(cs.leftBlindspot),
            right_bsm=bool(cs.rightBlindspot),
            left_rear_zone=_lz,
            right_rear_zone=_rz,
            rear_sensor_fault=_rf,
            lane_preference=int(ao_cfg["lane_preference"]),
            min_cruise_speed=float(ao_cfg["min_cruise_kph"]) * CV.KPH_TO_MS,
            # ----------------------------------------------------------
            # [AO_LANE_CONF_GATE] 把原始车道线概率与本车参数下限交给 AO，
            # 让 AO 内部那道不可绕过的硬门能与参数取较严者。
            # 背景：AutoOvertakeLaneProbMin 在设备上被设成 0.0，导致
            # 上面的 lp >= ao_cfg["lane_prob_min"] 恒真、筛选完全失效。
            # ----------------------------------------------------------
            left_lane_prob=lp_left,
            right_lane_prob=lp_right,
            lane_prob_min=float(ao_cfg["lane_prob_min"]),
            # ----------------------------------------------------------
            # [AO_JUNCTION_GATE] 路口/红绿灯距离（米），None = 未知。
            # 当前 Custom.AmapNavi 没有路口字段，故传 None；
            # AO 内部回落到"车道线断段"启发式。一旦 capnp 增加该字段，
            # 只需在此处改为真实值即可，AO 侧无需改动。
            # ----------------------------------------------------------
            junction_dist=None,
            # ----------------------------------------------------------
            # [AO_MR76_WIRE] MR76 相邻车道占用（来自 card.py 的 params）。
            #   fresh=1 -> available=True, age=0.0, obstacle=真实占用位
            #   fresh=0 -> available=None（"未接入"语义，门放行不阻断）
            #
            # LiDAR 当前未接入，lidar_* 一律不传（available=None 默认值
            # 即为 None，天然走"无数据不阻断"）。
            # ----------------------------------------------------------
            mr76_left_available=(True if _mr76_fresh else None),
            mr76_right_available=(True if _mr76_fresh else None),
            mr76_left_valid=(True if _mr76_fresh else None),
            mr76_right_valid=(True if _mr76_fresh else None),
            mr76_left_age=(0.0 if _mr76_fresh else None),
            mr76_right_age=(0.0 if _mr76_fresh else None),
            mr76_left_obstacle=(_mr76_left_occ if _mr76_fresh else None),
            mr76_right_obstacle=(_mr76_right_occ if _mr76_fresh else None),
            # [AO_AUX_SENSOR_STRICT] 原为 False，会绕过 MR76/LiDAR 健康门，
            # 使变道安全退化为"仅 BSM 布尔量"。改回 True（fail closed）。
            # [AO_MR76_WIRE] 现在 MR76 数据已真正接入，此门不再空转：
            #   available=None -> 未接入，门内放行；
            #   available=False/stale -> 已接入但异常，fail-closed 否决。
            require_aux_sensors=True,
          )
          st = AO.get_state()
          ao_fields.update({
            "v": round(v_ego * 3.6, 1), "vc": round(v_cruise * 3.6, 1),
            "lp": int(bool(lead.present)), "d": round(float(lead.dRel), 1), "vl": round(v_lead, 1),
            "lok": int(left_ok), "rok": int(right_ok),
            "lbsm": int(bool(cs.leftBlindspot)), "rbsm": int(bool(cs.rightBlindspot)),
            "lp0": round(lp_left, 2), "lp3": round(lp_right, 2),
            "ao": int(ao_dir), "mode": st["mode"], "why": st["last_reason"],
            "dh": int(DH.lane_change_state), "dhd": int(DH.lane_change_direction),
            # [AO_LANE_CONF_GATE] / [AO_JUNCTION_GATE] 诊断字段
            "cf": int(bool(st.get("lane_conf_L"))) * 2 + int(bool(st.get("lane_conf_R"))),
            "jb": int(bool(st.get("junction_ban"))),
            "js": str(st.get("junction_source") or "-"),
          })
        except Exception:
          cloudlog.exception("auto overtake update failed")
          AO.reset()
          ao_dir = log.LaneChangeDirection.none
      else:
        AO.reset()
        ao_fields["why"] = "disabled" if not ao_cfg["enabled"] else "no_radarstate"
      _ao_log(ao_fields)

      # [AUTO_AVOIDANCE_WIRING] 紧急避让优先于自动超车：AA 是安全兜底，AO 是舒适性
      aa_dir = log.LaneChangeDirection.none
      aa_fields = {"en": int(bool(aa_cfg["enabled"]))}
      if aa_cfg["enabled"]:
        try:
          _cs_aa = sm['carState']
          _lead_aa = sm['radarState'].leadOne if sm.seen['radarState'] else None
          # 用 radarTracks 提供障碍物位姿（本机唯一可用的障碍物点云源）
          # [AA_OBS_HOLD] 本帧若未取到有效点，则在 obs_hold_sec 内沿用最近一次读数，
          # 以对抗 8.3Hz 稀疏性与点云漂移，保证 AA 内部 _critical_frames 能连续累积。
          _obs_d, _obs_vr, _obs_in_path = None, None, False
          _best = None
          if sm.seen.get('radarTracks', False):
            _rt = sm['radarTracks']
            for _p in _rt.points:
              _d = float(_p.dRel); _y = float(_p.yRel); _vr = float(_p.vRel)
              if _d < 1.0 or _d > 60.0:
                continue
              # [AA_LANE_TIGHTEN] 横向筛选从 1.8m 收窄到 1.0m：
              # 1.8m 会选中相邻车道的车（半个车道宽 ≈ 1.8m），导致
              # AA 把旁车当成本车道障碍物并触发自动变道语音。
              # 1.0m 只保留真正位于本车道内的目标。
              if abs(_y) > 1.0:
                continue
              if _best is None or _d < _best[0]:
                _best = (_d, _y, _vr)

          _now_aa = time.monotonic()
          _hold_s = float(aa_cfg.get("obs_hold_sec", 0.0))
          if _best is not None:
            # 有效读数：刷新保持状态
            _aa_obs_hold["d"] = _best[0]
            _aa_obs_hold["y"] = _best[1]
            _aa_obs_hold["vr"] = _best[2]
            _aa_obs_hold["t"] = _now_aa
            _obs_d, _obs_vr, _obs_in_path = _best[0], _best[2], True
          elif (
            _hold_s > 0.0
            and _aa_obs_hold["d"] is not None
            and (_now_aa - _aa_obs_hold["t"]) <= _hold_s
          ):
            # 保持窗口内：沿用最近读数
            _obs_d = _aa_obs_hold["d"]
            _obs_vr = _aa_obs_hold["vr"]
            _obs_in_path = True
          else:
            # 超出保持窗口：彻底清空
            _aa_obs_hold["d"] = None
            _aa_obs_hold["vr"] = None
            _aa_obs_hold["y"] = None
          aa_dir, _aa_brake, _aa_hazard, _aa_off = AA.update(
            enabled=True,
            obstacle_in_path=_obs_in_path,
            lc_state=DH.lane_change_state,
            v_ego=v_ego,
            left_ok=left_ok,
            right_ok=right_ok,
            is_rhd=bool(is_rhd),
            manual_blinker=bool(_cs_aa.leftBlinker or _cs_aa.rightBlinker),
            bsm_available=True,
            left_bsm_blocked=bool(_cs_aa.leftBlindspot),
            right_bsm_blocked=bool(_cs_aa.rightBlindspot),
            obstacle_dist=_obs_d,
            obstacle_rel_speed=_obs_vr,
            lead_dist=float(_lead_aa.dRel) if _lead_aa is not None else None,
            lead_rel_speed=float(_lead_aa.vRel) if _lead_aa is not None else None,
            require_aux_sensors=False,
            # [AA_STABLE_RATIO] 逃逸车道稳定性容抖模式（默认 ratio）。
            avoid_stable_mode=str(aa_cfg["stable_mode"]),
            avoid_stable_ratio=float(aa_cfg["stable_ratio"]),
            avoid_stable_window_sec=float(aa_cfg["stable_window_sec"]),
            # [AA_BRAKE_FALLBACK] 纵向兜底（默认 brake_max=0.0 == 关闭）
            avoid_brake_gain=float(aa_cfg["brake_gain"]),
            avoid_brake_max=float(aa_cfg["brake_max"]),
            avoid_brake_ttc_ref=float(aa_cfg["brake_ttc_ref"]),
          )
          _aa_st = AA.get_state()
          aa_fields.update({
            "dir": int(aa_dir), "mode": _aa_st["mode"], "why": _aa_st["last_reason"],
            "risk": int(_aa_st["risk_level"]),
            "od": round(_obs_d, 1) if _obs_d is not None else -1,
            "ovr": round(_obs_vr, 1) if _obs_vr is not None else 0,
            "v": round(v_ego * 3.6, 1),
            "sm": str(aa_cfg["stable_mode"]),
            "oh": round(float(aa_cfg.get("obs_hold_sec", 0.0)), 2),
            "brk": round(float(_aa_brake), 3),
            "bmax": round(float(aa_cfg.get("brake_max", 0.0)), 2),
          })
        except Exception:
          cloudlog.exception("auto avoidance update failed")
          AA.reset()
          _aa_obs_hold["d"] = None
          _aa_obs_hold["vr"] = None
          _aa_obs_hold["y"] = None
          aa_dir = log.LaneChangeDirection.none
          _aa_brake = 0.0
      else:
        AA.reset()
        _aa_obs_hold["d"] = None
        _aa_obs_hold["vr"] = None
        _aa_obs_hold["y"] = None
        aa_fields["why"] = "disabled"
        _aa_brake = 0.0
      _aa_log(aa_fields)
      # ------------------------------------------------------------------
      # [AA_BRAKE_FALLBACK] 镜像 AA 的刹车请求到 /data/params/d/AvoidBrakeReq
      #
      # ★ 心跳语义：只要请求值 > 0（刹车生效中），**每帧都重写文件**，
      #   即使内容与上帧完全相同（靠写入推进 mtime）。消费侧
      #   （controlsd -> aa_brake_apply.py）用 mtime 是否推进判断
      #   "写入侧是否还活着"，从而正确区分：
      #     - 持续请求（内容恒定但 mtime 一直走）-> 刹车保持
      #     - 写入侧已死 / 请求过期（mtime 停住）  -> 刹车在 0.5s 内释放
      #   若沿用"仅当值变化才写"的节流，恒定请求会因 mtime 不再推进
      #   而被误判为过期并释放 —— 这是必须避免的安全缺陷。
      #
      # 安全声明：此处**不**直接把该值转发到 CarControl / actuators，
      # 纵向落地统一由 controlsd 侧的 aa_brake_apply 模块受控消费。
      # ------------------------------------------------------------------
      try:
        _brk_val = float(_aa_brake)
        _brk_val = 0.0 if (_brk_val != _brk_val) else max(0.0, _brk_val)
        _changed = abs(_brk_val - _aa_brake_pub["v"]) >= 0.01
        if _changed or _brk_val > 0.0:
          _aa_brake_pub["v"] = _brk_val
          # ★ 写入格式 "<值>:<monotonic秒>"。时间戳即心跳：只要本进程
          #   还在写，消费侧（aa_brake_apply）就能确认"请求仍然新鲜"，
          #   从而正确区分【持续请求】与【写入侧已死】。
          #   用 monotonic 而非 mtime，避免跨时钟域与高频重写不推进 mtime
          #   的问题（实测踩过：会造成正在生效的刹车被 staleness 误杀）。
          _ao_write_param("AvoidBrakeReq", f"{_brk_val:.3f}:{time.monotonic():.3f}")
      except Exception:
        pass
      # 紧急避让优先；AA 未介入时回落到 AO
      _merged_dir = aa_dir if aa_dir != log.LaneChangeDirection.none else ao_dir
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, left_edge, right_edge,
                auto_lane_change_direction=_merged_dir,
                auto_confirm_delay_sec=float(ao_cfg["confirm_delay_sec"]))
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      mdv2sp_send.valid = modelv2_send.valid
      mdv2sp_send.modelDataV2SP.laneTurnDirection = DH.lane_turn_direction
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
      pm.send('modelDataV2SP', mdv2sp_send)
    last_vipc_frame_id = meta_main.frame_id

if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning(f"child {PROCESS_NAME} got SIGINT")
  except Exception:
    sentry.capture_exception()
    raise
