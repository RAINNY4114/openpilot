#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections.abc import Callable
from functools import partial
import gc
import os
os.environ['GMMU'] = '0'
# [port-egpu-usb] 功率上限的唯一来源是 launch_env.sh（读 /data/ford_params/am_power_limit，默认 120）。
# 这里只在完全没有环境变量时兜底，数值必须与 launch_env.sh 的默认值保持一致。
os.environ.setdefault('AM_POWER_LIMIT', '120')
from tinygrad.device import Device
import numpy as np
import threading
import time
from setproctitle import setproctitle

import openpilot.cereal.messaging as messaging
from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present, modeld_pkl_path
# [EGPU_DIAG 20261027] P1-2/P2-1：[eGPU] 结构化日志 + error_state 分类 + 按 boot 计次的重试预算
from openpilot.selfdrive.modeld.egpu_log import (
  consume_retry, classify_error, egpu, reset_retry, retry_budget, usb_snapshot, RETRY_MAX,
)
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.cereal.services import SERVICE_LIST
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.hardware.usb import cable_connected, chestnut_usb_node_ready
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
# [AUTO_OVERTAKE_WIRING] / [AUTO_AVOIDANCE_WIRING] 接线层（自包含，见 ao_aa_bridge.py）
from openpilot.selfdrive.controls.lib.ao_aa_bridge import AoAaBridge
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value
from openpilot.selfdrive.modeld.modeld import ChestnutGpuState

from openpilot.sunnypilot.modeld_v2.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState, get_curvature_from_output
from openpilot.sunnypilot.modeld_v2.parse_model_outputs import Parser
from openpilot.sunnypilot.modeld_v2.blendful import Blendful
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.meta_helper import load_meta_constants
from openpilot.sunnypilot.modeld_v2.camera_offset_helper import CameraOffsetHelper
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.modeld_v2.helpers import load_model
from openpilot.sunnypilot.modeld_v2.frame_fusion import fused_with_warp
from openpilot.sunnypilot.modeld_v2.model_adapters import get_model_adapter
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.selfdrive.controls.lib.relc import RoadEdgeLaneChangeController

PROCESS_NAME = "openpilot.selfdrive.modeld.modeld_tinygrad"

# [CHX_USB_ENUM 20261027] 大模型加载分两段：先等 chestnut 到「USB3 + 链路已训练」，
# 再让 tinygrad 打开设备编译。两段加起来才是总窗口。
#
# 为什么必须 150s：本机 chestnut 大模型 pkl 约 762 MB，正常编译 20~40s；冷启动时
# 设备可能先以 12 Mbps 枚举，要发 0xF3 上电后才重枚举到 USB3（实测窗口 3~20s）。
# 旧值 60s 会在这条链路上「刚重枚举完、还在编译」时被 join 掐断 -> 静默回退小模型，
# 且守卫线程仍存活 -> 不走 except -> 连 "chestnut load failed" 都不打印。
BIG_MODEL_TIMEOUT = 600  # [port-egpu-usb] 上游值：762MB pkl 冷启动 + 重枚举余量
# 等设备出现（含节点权限放开）的上限
CHESTNUT_USB_GUARD_TIMEOUT = 60.0
# 发过 0xF3 之后等重枚举到 USB3 的上限
CHESTNUT_USB_POWER_TIMEOUT = 40.0

# [CHX_TIMEOUT_INVARIANT] 不变量：守卫必须比 join 先结束。
# 否则守卫还在 sleep，join 已经超时返回 None —— 大模型被静默放弃且没有任何错误日志。
assert max(CHESTNUT_USB_GUARD_TIMEOUT, CHESTNUT_USB_POWER_TIMEOUT) < BIG_MODEL_TIMEOUT, \
  "chestnut USB guard must finish before BIG_MODEL_TIMEOUT"


def _pkl_exists(path):
  return os.path.exists(path)


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


# [CHESTNUT_PKL_FMT 20261027] modeld_v2 只认 SPTGPLN1 容器（sunnypilot/modeld_v2/helpers.read_parts）。
# MODELS_DIR 里那三个 762 MiB 的 big_* 是**普通 pickle**（头 \x80\x04\x95），是给旧
# selfdrive/modeld 的 load_oob/load_pickle 用的 —— 那是 10-26 换分支之前的加载器。
# modeld_v2 直接读会抛
#   ValueError: not a plain tinygrad model, recompile it with compile_modeld.py
# 所以回落前必须先验格式，否则只会白烧一次重试预算 + 一次 onroad cycle。
PKL_MAGIC = b'SPTGPLN1'


def _pkl_format(path):
  """返回 'v2' / 'legacy' / 'pickle' / 'missing' / 'unknown'。

  'v2'     = SPTGPLN1 容器，modeld_v2 能读
  'legacy' = openpilot 的 load_oob 容器：<int64 长度><pickle opcodes><若干 <int64 长度><buffer>
             （旧 selfdrive/modeld 的 load_oob 读的就是它，本机 big_driving_tinygrad.pkl 属于这类）
  'pickle' = 裸 pickle，同样不是 modeld_v2 能读的
  """
  try:
    with open(path, 'rb') as f:
      head = f.read(16)
  except OSError:
    return 'missing'
  if head[:8] == PKL_MAGIC:
    return 'v2'
  if head[:1] == b'\x80':
    return 'pickle'
  # load_oob：前 8 字节是小端 int64 的 opcodes 长度，紧随其后才是 pickle 起始符 0x80
  if len(head) >= 9 and head[8:9] == b'\x80':
    return 'legacy'
  return 'unknown'


def _resolve_chestnut_pkl(bundle):
  """返回 (pkl_path, source)。source ∈ {'override','bundle','fallback',None}。

  [CHESTNUT_PKL_FALLBACK 20261027] bundle 体系认 Paths.model_root()（/data/media/0/models），
  而本机的 762 MiB big_driving_tinygrad.pkl 只在 MODELS_DIR（.../selfdrive/modeld/models/）。
  加上 chestnut 模型目录 driving_models_chestnut_v28.json 拉不到（设备访问不了
  raw.githubusercontent.com），ModelManager_ActiveBundleChestnut 恒为 None
  ⇒ _find_driving_pkl 恒返回 None ⇒ 门禁恒 False ⇒ load_big() 线程根本不创建。
  这里在 bundle 给不出路径时回落到 chestnut_compiled() 认的那三个文件。
  注意：回落只解决「路径」，不解决「格式」—— 格式由调用方用 _pkl_format() 单独判。
  """
  if (override := os.environ.get('COMBINED_MODEL_PKL')) and _pkl_exists(override):
    return override, 'override'
  pkl = _find_driving_pkl(bundle)
  if pkl is not None:
    return pkl, 'bundle'
  if chestnut_compiled():
    p = str(modeld_pkl_path(chestnut=True))
    if _pkl_exists(p):
      return p, 'fallback'
  return None, None


def _resolve_chestnut_gate():
  """算 CHESTNUT 门禁，并把每道判据各自的结果落盘。

  旧写法是一行短路表达式：
    CHESTNUT = _find_driving_pkl(get_active_bundle(chestnut=True)) is not None and (present or cable)
  一旦 bundle 为 None，后两项**根本不求值**，日志里什么都看不到 —— 本机就是这个状态
  （2026-10-27 实测 get_active_bundle(chestnut=True) 恒为 None）。
  这里显式求值并记录，返回值与旧表达式等价，只是多了 pkl 回落与格式校验。
  """
  bundle = get_active_bundle(chestnut=True)
  pkl, pkl_src = _resolve_chestnut_pkl(bundle)
  present = chestnut_present()
  cable = cable_connected()
  compiled = chestnut_compiled()
  fmt = _pkl_format(pkl) if pkl else 'missing'

  # [CHESTNUT_GATE_20261027] 判据从 (present or cable) 收紧成 present：
  #   cable_connected() 读 /sys/class/power_supply/usb/typec_cc_orientation，那是 USB-C
  #   充电口的 CC 方向，插个充电器也是 True。2026-10-27 route 6 实测它 True 而
  #   deviceState.chestnutPresent 642/642 全 False —— 用它只会把「插了充电线」当成
  #   「硬件就绪」，然后在 chestnut_usb_node_ready() 里白等 45~60 s（这段时间没有 modelV2）。
  #   冷启动时设备即使先以 12 Mbps 枚举，chestnut_present() 也是 True，
  #   所以 present 已覆盖「慢速枚举 → 0xF3 上电 → 重枚举 USB3」那条链路。
  # [port-egpu-usb] fmt 校验已从门禁里移除。
  # 原写法把「格式不对就别浪费重试预算」和「PCIe 上电」写成了同一个条件：
  # fmt 不是 SPTGPLN1 时 gate=False -> load_big() 线程根本不创建 ->
  # chestnut_usb_node_ready() 永远不执行 -> 0xF3 永远不发 -> 底座停在 12Mbps ->
  # 格式永远修不好。完整死锁。
  # 实测 /data/egpu_diag.jsonl：CHX_USB_ENUM 0 次 / pre-power 0 次 / load_start 0 次，
  # 而 gate_pkl_bad_fmt 5 次 —— 诊断清楚知道格式不对，然后把自己关掉了。
  # 格式问题由 convert_upstream_pkl.py 单独解决，仍记录在 egpu('gate', pkl_fmt=...)
  # 与 gate_pkl_bad_fmt 事件里，只是不再充当硬件门禁。
  gate = pkl is not None and present
  egpu('gate', chestnut=gate, pkl=pkl, pkl_src=pkl_src, pkl_fmt=fmt, present=present,
       cable=cable, compiled=compiled,
       bundle_generation=getattr(bundle, 'generation', None),
       bundle_models=len(bundle.models) if bundle is not None else 0)

  if pkl is not None and fmt != 'v2':
    # 路径找到了但格式不对：老 selfdrive/modeld 的 load_oob 能读，modeld_v2 的
    # read_parts 读不了。修法是用 sunnypilot/modeld_v2/convert_upstream_pkl.py 转换。
    egpu('gate_pkl_bad_fmt', pkl=pkl, pkl_fmt=fmt, pkl_src=pkl_src)
  elif pkl is not None and not present:
    # pkl 已就绪、只差硬件：这条把「软件阻塞」和「硬件阻塞」彻底分开
    egpu('gate_wait_hw', pkl=pkl, pkl_src=pkl_src, cable=cable, usb=usb_snapshot())
  return gate


def _request_cycle_later(params, delay: float) -> None:
  """延迟 delay 秒再请求一次 onroad cycle。

  用 Timer 而不是 sleep：modeld 此刻要继续用小模型出帧，绝不能阻塞主线程。
  """
  def _fire():
    try:
      params.put_bool("OnroadCycleRequested", True)
      egpu('cycle_requested', delay=round(delay, 1))
    except Exception:
      pass

  if delay <= 0:
    _fire()
    return
  t = threading.Timer(delay, _fire)
  t.daemon = True
  t.start()
  egpu('cycle_scheduled', delay=round(delay, 1))


def _handle_big_model_failure(params) -> None:
  """大模型加载失败后的重试决策（P0-2：按 boot 计次 + 退避）。"""
  st, allowed, backoff = retry_budget()
  last_ts = float(st.get('last_ts', 0.0))
  since = (time.time() - last_ts) if last_ts > 0 else 1e9

  if not allowed:
    egpu('retry_exhausted', attempts=int(st.get('count', 0)), max=RETRY_MAX,
         since_last=round(since, 1), usb=usb_snapshot())
    cloudlog.error(f"[eGPU] chestnut retry budget exhausted ({st.get('count')}/{RETRY_MAX}) this boot")
    params.put_bool("ChestnutRetry", False)
    params.put_bool("ChestnutActive", False)
    return

  attempt = int(st.get('count', 0)) + 1
  # 第 1 次立刻请求；第 2/3 次至少要等 backoff（距上次失败不足就延后）
  delay = 0.0 if attempt == 1 else max(0.0, backoff - since)
  st = consume_retry(st)
  egpu('retry_request', attempt=attempt, max=RETRY_MAX, backoff=backoff,
       since_last=round(since, 1), delay=round(delay, 1))
  cloudlog.error(f"[eGPU] big model load failed, retry {attempt}/{RETRY_MAX} in {delay:.0f}s")
  params.put_bool("ChestnutRetry", True)
  params.put_bool("ChestnutActive", False)
  _request_cycle_later(params, delay)


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
    if pkl_path is None and chestnut:
      # [CHESTNUT_PKL_FALLBACK 20261027] 与门禁同一套回落（见 _resolve_chestnut_pkl）：
      # bundle 体系认 model_root，而本机 big pkl 在 MODELS_DIR。
      pkl_path, _src = _resolve_chestnut_pkl(model_bundle)
    assert pkl_path is not None, f"No driving pkl found for {'chestnut' if chestnut else 'small model'} — all models must be compiled with compile_modeld.py"
    self._init_combined(pkl_path, cam_w, cam_h, model_bundle)

  def _init_combined(self, pkl_path, cam_w, cam_h, bundle):
    cloudlog.warning(f"loading combined pkl: {pkl_path}")
    jits = load_model(pkl_path, extend=fused_with_warp(cam_w, cam_h, self.chestnut))

    metadata = jits['metadata']
    self.WARP_DEV = metadata.get('warp_dev', 'QCOM') if COMMA_HARDWARE else 'CPU'
    self.DEV = ('AMD' if self.chestnut else 'QCOM') if COMMA_HARDWARE else 'CPU'
    self.QUEUE_DEV = self.DEV
    self.adapter = get_model_adapter(jits, cam_w, cam_h, self.DEV, self.QUEUE_DEV, self.WARP_DEV, self.chestnut)
    self.vision_output_slices = self.adapter.vision_output_slices
    self.policy_output_slices = self.adapter.policy_output_slices
    self._policy_slices_list = self.adapter._policy_slices_list
    self._combined_model_type = self.adapter._combined_model_type
    self._vision_input_names = self.adapter._vision_input_names
    self.numpy_inputs = self.adapter.numpy_inputs
    self._policy_keys = self.adapter._policy_keys
    self._has_on_policy = self.adapter._has_on_policy
    self._desire_key = self.adapter._desire_key
    self._road_key = self.adapter._road_key
    self._wide_key = self.adapter._wide_key
    self.frame_buf_params = self.adapter.frame_buf_params

    is_20hz = bundle.is20hz if bundle else self._combined_model_type in ('split', 'multi_policy')
    if is_20hz:
      from openpilot.sunnypilot.models.split_model_constants import SplitModelConstants
      self.constants = SplitModelConstants()
    else:
      self.constants = ModelConstants()

    self.parser = Parser()
    self.prev_desire = np.zeros(self.constants.DESIRE_LEN, dtype=np.float32)

  def warmup(self) -> None:
    dummy_frames, transforms, dummy_inputs = self.adapter.get_dummy_inputs()
    self.run(dummy_frames, transforms, dummy_inputs)
    self.adapter.reset_warmup_buffers()
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
    self.adapter.copy_frames(bufs)

    desire_key = self.desire_key
    inputs[desire_key][0] = 0
    (self.numpy_inputs[desire_key].flat if self.adapter.is_native else self.numpy_inputs[desire_key])[:] = \
      np.where(inputs[desire_key] - self.prev_desire > .99, inputs[desire_key], 0)
    self.prev_desire[:] = inputs[desire_key]

    for key in ('traffic_convention', 'lateral_control_params', 'action_t'):
      if key in self.numpy_inputs and key in inputs:
        self.numpy_inputs[key][:] = inputs[key]

    if self.adapter.is_native:
      for i, key in enumerate(self._vision_input_names):
        if key in transforms:
          self.numpy_inputs['tfm'][i] = transforms[key].reshape(3, 3)
    else:
      self.numpy_inputs['tfm'][:, :] = transforms[self._road_key].reshape(3, 3)
      self.numpy_inputs['big_tfm'][:, :] = transforms[self._wide_key].reshape(3, 3)

    raw_outputs = self.adapter.run()
    if self.chestnut:
      # [EGPU_DIAG 20261027] 0=正常；正数是 USB 传输层状态码（+1=LIBUSB_TRANSFER_ERROR），
      # 负数是 libusb 调用层错误。egpucp 的旧分类器只认 -1，把 +1 当成数值失败永久拒绝。
      try:
        err = int(Device[self.DEV].error_state.host.view(fmt='q')[0])
      except Exception as e:
        err = -1000
        egpu('error_state_unavailable', err=repr(e))
      if err != 0 and err != -1000:
        cls = classify_error(err)
        egpu('error_state', raw=err, cls=cls, usb=usb_snapshot())
        raise RuntimeError(f"chestnut failed with {err} ({cls})")

    if after_enqueue is not None:
      after_enqueue()

    if self._combined_model_type == 'supercombo':
      model_output = raw_outputs.numpy().flatten()
      if self.chestnut and not np.all(np.isfinite(model_output)):
        raise RuntimeError("model output not finite")
      sliced = {k: model_output[np.newaxis, v] for k, v in self.vision_output_slices.items()}
      outputs = self.parser.parse_outputs(sliced)
      if 'prev_feat' in self.numpy_inputs and 'hidden_state' in self.vision_output_slices:
        (self.numpy_inputs['prev_feat'].flat if self.adapter.is_native else self.numpy_inputs['prev_feat'])[:] = \
          model_output[self.vision_output_slices['hidden_state']]
    else:
      vision_output = raw_outputs[0].numpy().flatten()
      vision_sliced = {k: vision_output[np.newaxis, v] for k, v in self.vision_output_slices.items()}
      outputs = self.parser.parse_vision_outputs(vision_sliced)

      if 'prev_feat' in self.numpy_inputs and 'hidden_state' in self.vision_output_slices:
        (self.numpy_inputs['prev_feat'].flat if self.adapter.is_native else self.numpy_inputs['prev_feat'])[:] = \
          vision_output[self.vision_output_slices['hidden_state']]

      for i, policy_slices in enumerate(self._policy_slices_list):
        policy_output = raw_outputs[i + 1].numpy().flatten()
        policy_sliced = {k: policy_output[np.newaxis, v] for k, v in policy_slices.items()}
        parsed = self.parser.parse_policy_outputs(policy_sliced)
        if 'off' in self._policy_keys[i] and self._has_on_policy:
          for key in ('plan', 'planplus', 'action'):
            if any(key in self._policy_slices_list[j] for j, k in enumerate(self._policy_keys) if 'on' in k.lower()):
              parsed.pop(key, None)

        outputs.update(parsed)

      if 'planplus' in outputs and 'plan' in outputs:
        outputs['plan'] = outputs['plan'] + outputs['planplus']

    if 'desired_curvature' in outputs and 'prev_desired_curv' in self.numpy_inputs:
      buf = self.numpy_inputs['prev_desired_curv']
      buf[0, :-1] = buf[0, 1:]
      buf[0, -1, :] = outputs['desired_curvature'][0, :] if not self.mlsim else 0

    return outputs

  def get_action_from_model(self, model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                            lat_action_t: float, long_action_t: float, v_ego: float,
                            blendful: Callable[[float], float] | None = None) -> log.ModelDataV2.Action:
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
    if blendful is not None:
      desired_curvature += blendful(desired_curvature)

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

  cloudlog.bind(daemon=PROCESS_NAME)
  setproctitle(PROCESS_NAME)

  CHESTNUT = _resolve_chestnut_gate()
  if CHESTNUT:
    from tinygrad.runtime.ops_amd import AMDDevice
    AMDDevice.wait_timeout_ms = 3000

  params = Params()
  params.put_bool("ChestnutLoading", CHESTNUT)
  params.remove("ChestnutActive")

  gc.disable()

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
      t_load = time.monotonic()
      egpu('load_start', timeout=BIG_MODEL_TIMEOUT, guard=CHESTNUT_USB_GUARD_TIMEOUT,
           power=CHESTNUT_USB_POWER_TIMEOUT, usb=usb_snapshot())
      try:
        # [CHX_USB_ENUM 20261027] 权限 + 上电时序 + 枚举位置三道一起判，
        # 只有确认「节点可写 + 在 USB3 侧 + 链路已训练」才交给 tinygrad。
        # 失败时明确抛错，不要静默继续 —— 静默继续只会得到一次 EACCES/EBUSY。
        if not chestnut_usb_node_ready(timeout=CHESTNUT_USB_GUARD_TIMEOUT,
                                       power_timeout=CHESTNUT_USB_POWER_TIMEOUT):
          raise RuntimeError("chestnut USB node not ready: 设备未枚举/未上电/不在 USB3 侧，"
                             "详见 [CHX_USB_ENUM] 日志")
        m = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=True)
        m.warmup()
        big_model = m
        egpu('load_ok', elapsed=round(time.monotonic() - t_load, 1))
      except Exception as e:
        egpu('load_fail', err=repr(e), elapsed=round(time.monotonic() - t_load, 1),
             usb=usb_snapshot())
        cloudlog.exception("chestnut load failed")
    loader = threading.Thread(target=load_big, daemon=True)
    loader.start()
    loader.join(BIG_MODEL_TIMEOUT)
    model = big_model
    egpu('join_done', loaded=model is not None, loader_alive=loader.is_alive(),
         elapsed=round(time.monotonic() - st, 1))
    if model is not None:
      # 成功即清零本 boot 的失败计数，下次失败还能重试
      reset_retry()
      params.put_bool("ChestnutRetry", False)
      params.put_bool("ChestnutActive", True)
    else:
      _handle_big_model_failure(params)

  small_model = ModelState(cam_w=vipc_client_main.width, cam_h=vipc_client_main.height, chestnut=False) if model is None or CHESTNUT else None
  if model is None:
    model = small_model
  params.put_bool("ChestnutLoading", False)
  assert model is not None
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  config_realtime_process(7, 54)

  # messaging
  pub_socks = ["modelV2", "drivingModelData", "cameraOdometry", "modelDataV2SP"] + (["chestnutGpuState"] if CHESTNUT else [])
  pm = PubMaster(pub_socks)
  sm = SubMaster(["deviceState", "carState", "narrowRoadCameraState", "extrinsicsCalibration", "driverMonitoringState",
                  "carControl", "controlsState", "lateralDelay", "amapNavi",
                  # [AUTO_OVERTAKE_WIRING] AO 唯一触发源（radard 融合的 leadOne）
                  # [AUTO_AVOIDANCE_WIRING] AA 障碍物点云源（card.py 发布的前向雷达）
                  "radarState", "radarTracks"])

  publish_state = PublishState()
  chestnut_state = ChestnutGpuState(pm, model.chestnut) if CHESTNUT else None

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
  meta_constants = load_meta_constants()
  RELC = RoadEdgeLaneChangeController()
  blendful = Blendful()
  blendful_enabled = False
  # [AUTO_OVERTAKE_WIRING] / [AUTO_AVOIDANCE_WIRING] 接线层实例。
  # 构造本身不读硬件、不抛异常；配置每次 update 内部按 2 s 热加载。
  AOA = AoAaBridge()
  is_rhd = bool(getattr(CP, "isRhd", False))

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
      blendful_enabled = params.get_bool("Blendful")
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
                       run_count % round(model.constants.MODEL_FREQ / SERVICE_LIST['chestnutGpuState'].frequency) == 0)
      model_output = model.run(bufs, transforms, inputs, chestnut_state.send if send_chestnut else None)
    except Exception:
      if not model.chestnut:
        raise
      cloudlog.exception("big model failed, fall back to small")
      egpu('fallback_small', usb=usb_snapshot())
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

      if not sm['carControl'].latActive:
        prev_action.desiredCurvature = sm['controlsState'].curvature
      blendful_active = desire == log.Desire.none and DH.lane_change_state == log.LaneChangeState.off
      action = model.get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego,
                                           partial(blendful.update, model_output, v_ego, blendful_active, enabled=blendful_enabled))
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
      # [AUTO_OVERTAKE_WIRING] / [AUTO_AVOIDANCE_WIRING]
      #
      # 接线层只产出两样东西：
      #   1. `direction`      —— 想变去的车道（AA 优先，AA 未介入时回落 AO）
      #   2. `AvoidBrakeReq`  —— 写盘的心跳标量，由 controlsd 的 AABrakeApply 消费
      #
      # ★ 绝不触碰横向控制链：`actuators.curvature` / `desiredCurvature` /
      #   path_angle 全部与本模块无关。用户 10-27 明确「保留现有车道居中」。
      # ★ 任何内部异常都被 ao_aa_bridge 吞掉并回落到 direction=none，
      #   等价于纯上游行为，绝不会因为 AO/AA 把 modeld 带崩。
      # ------------------------------------------------------------------
      _ao_aa = AOA.update(sm=sm, dh=DH, v_ego=v_ego, modelv2=modelv2_send.modelV2,
                          lat_active=sm['carControl'].latActive,
                          left_edge=left_edge, right_edge=right_edge, is_rhd=is_rhd)
      DH.update(sm['amapNavi'], sm['carState'], sm['carControl'].latActive, lane_change_prob, left_edge, right_edge,
                auto_lane_change_direction=_ao_aa.direction,
                auto_confirm_delay_sec=_ao_aa.confirm_delay_sec)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      mdv2sp_send.valid = modelv2_send.valid
      mdv2sp_send.modelDataV2SP.laneTurnDirection = DH.lane_turn_direction
      mdv2sp_send.modelDataV2SP.blendfulCurvature = blendful.curvature
      mdv2sp_send.modelDataV2SP.blendfulTrust = float(blendful.trust)
      mdv2sp_send.modelDataV2SP.modelCurvature = float(blendful.model_curvature)
      mdv2sp_send.modelDataV2SP.blendfulGoingAround = bool(blendful.going_around)
      mdv2sp_send.modelDataV2SP.blendfulLeftProb = blendful.l_prob
      mdv2sp_send.modelDataV2SP.blendfulRightProb = blendful.r_prob
      mdv2sp_send.modelDataV2SP.blendfulLaneWidth = float(blendful.lane_width.x)
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
    cloudlog.exception("modeld exception")
    raise
