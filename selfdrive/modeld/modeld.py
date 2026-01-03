#!/usr/bin/env python3
import os
from openpilot.system.hardware import TICI
os.environ['DEV'] = 'QCOM' if TICI else 'CPU'
USBGPU = "USBGPU" in os.environ
if USBGPU:
  os.environ['DEV'] = 'AMD'
  os.environ['AMD_IFACE'] = 'USB'
from tinygrad.tensor import Tensor
from tinygrad.dtype import dtypes
import time
import pickle
import numpy as np
import cereal.messaging as messaging
from cereal import car, log
from pathlib import Path
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.auto_avoidance import AutoAvoidanceHelper
from openpilot.selfdrive.controls.lib.auto_overtake import AutoOvertakeHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.models.commonmodel_pyx import DrivingModelFrame, CLContext
from openpilot.selfdrive.modeld.runners.tinygrad_helpers import qcom_tensor_from_opencl_address
from openpilot.selfdrive.modeld.cone_detections import decode_cone_detections
from dragonpilot.selfdrive.controls.lib.road_edge_detector import RoadEdgeDetector


PROCESS_NAME = "selfdrive.modeld.modeld"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

VISION_PKL_PATH = Path(__file__).parent / 'models/driving_vision_tinygrad.pkl'
POLICY_PKL_PATH = Path(__file__).parent / 'models/driving_policy_tinygrad.pkl'
VISION_METADATA_PATH = Path(__file__).parent / 'models/driving_vision_metadata.pkl'
POLICY_METADATA_PATH = Path(__file__).parent / 'models/driving_policy_metadata.pkl'

LAT_SMOOTH_SECONDS = 0.1
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3

# Auto lane-change safety: block auto lane changes when the target lane has a detected hazard
# within a conservative forward distance. Distance is derived from a time-gap and clamped.
AUTO_LC_TARGET_LANE_TGAP_S = float(os.getenv("DP_LINCOLN_AUTO_LC_TGAP_S", "2.0"))
AUTO_LC_TARGET_LANE_MIN_DIST_M = float(os.getenv("DP_LINCOLN_AUTO_LC_MIN_DIST_M", "25.0"))
AUTO_LC_TARGET_LANE_MAX_DIST_M = float(os.getenv("DP_LINCOLN_AUTO_LC_MAX_DIST_M", "90.0"))
AUTO_LC_TARGET_LANE_BLOCK_HOLD_SEC = float(os.getenv("DP_LINCOLN_AUTO_LC_BLOCK_HOLD_SEC", "0.8"))

# Auto-avoid trigger tightening: keep slowdown behavior, but avoid starting lane changes on weak/noisy detections.
AVOID_CONE_METRIC_MIN = float(os.getenv("DP_LINCOLN_AVOID_CONE_METRIC_MIN", "0.25"))
AVOID_VEHICLE_METRIC_MIN = float(os.getenv("DP_LINCOLN_AVOID_VEHICLE_METRIC_MIN", "0.45"))
AVOID_VEHICLE_METRIC_MIN_NO_LEAD = float(os.getenv("DP_LINCOLN_AVOID_VEHICLE_METRIC_MIN_NO_LEAD", "0.65"))
AVOID_STOPPED_LEAD_SPEED_MS = float(os.getenv("DP_LINCOLN_AVOID_STOPPED_LEAD_SPEED_MS", "3.0"))


def get_action_from_model(model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                          lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
    plan = model_output['plan'][0]
    desired_accel, should_stop = get_accel_from_plan(plan[:,Plan.VELOCITY][:,0],
                                                     plan[:,Plan.ACCELERATION][:,0],
                                                     ModelConstants.T_IDXS,
                                                     action_t=long_action_t)
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)

    desired_curvature = get_curvature_from_plan(plan[:,Plan.T_FROM_CURRENT_EULER][:,2],
                                                plan[:,Plan.ORIENTATION_RATE][:,2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
    if v_ego > MIN_LAT_CONTROL_SPEED:
      desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, LAT_SMOOTH_SECONDS)
    else:
      desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                  desiredAcceleration=float(desired_accel),
                                  shouldStop=bool(should_stop))

class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof

class InputQueues:
  def __init__ (self, model_fps, env_fps, n_frames_input):
    assert env_fps % model_fps == 0
    assert env_fps >= model_fps
    self.model_fps = model_fps
    self.env_fps = env_fps
    self.n_frames_input = n_frames_input

    self.dtypes = {}
    self.shapes = {}
    self.q = {}

  def update_dtypes_and_shapes(self, input_dtypes, input_shapes) -> None:
    self.dtypes.update(input_dtypes)
    if self.env_fps == self.model_fps:
      self.shapes.update(input_shapes)
    else:
      for k in input_shapes:
        shape = list(input_shapes[k])
        if 'img' in k:
          n_channels = shape[1] // self.n_frames_input
          shape[1] = (self.env_fps // self.model_fps + (self.n_frames_input - 1)) * n_channels
        else:
          shape[1] = (self.env_fps // self.model_fps) * shape[1]
        self.shapes[k] = tuple(shape)

  def reset(self) -> None:
    self.q = {k: np.zeros(self.shapes[k], dtype=self.dtypes[k]) for k in self.dtypes.keys()}

  def enqueue(self, inputs:dict[str, np.ndarray]) -> None:
    for k in inputs.keys():
      if inputs[k].dtype != self.dtypes[k]:
        raise ValueError(f'supplied input <{k}({inputs[k].dtype})> has wrong dtype, expected {self.dtypes[k]}')
      input_shape = list(self.shapes[k])
      input_shape[1] = -1
      single_input = inputs[k].reshape(tuple(input_shape))
      sz = single_input.shape[1]
      self.q[k][:,:-sz] = self.q[k][:,sz:]
      self.q[k][:,-sz:] = single_input

  def get(self, *names) -> dict[str, np.ndarray]:
    if self.env_fps == self.model_fps:
      return {k: self.q[k] for k in names}
    else:
      out = {}
      for k in names:
        shape = self.shapes[k]
        if 'img' in k:
          n_channels = shape[1] // (self.env_fps // self.model_fps + (self.n_frames_input - 1))
          out[k] = np.concatenate([self.q[k][:, s:s+n_channels] for s in np.linspace(0, shape[1] - n_channels, self.n_frames_input, dtype=int)], axis=1)
        elif 'pulse' in k:
          # any pulse within interval counts
          out[k] = self.q[k].reshape((shape[0], shape[1] * self.model_fps // self.env_fps, self.env_fps // self.model_fps, -1)).max(axis=2)
        else:
          idxs = np.arange(-1, -shape[1], -self.env_fps // self.model_fps)[::-1]
          out[k] = self.q[k][:, idxs]
      return out

class ModelState:
  frames: dict[str, DrivingModelFrame]
  inputs: dict[str, np.ndarray]
  output: np.ndarray
  prev_desire: np.ndarray  # for tracking the rising edge of the pulse

  def __init__(self, context: CLContext):
    with open(VISION_METADATA_PATH, 'rb') as f:
      vision_metadata = pickle.load(f)
      self.vision_input_shapes =  vision_metadata['input_shapes']
      self.vision_input_names = list(self.vision_input_shapes.keys())
      self.vision_output_slices = vision_metadata['output_slices']
      vision_output_size = vision_metadata['output_shapes']['outputs'][1]

    with open(POLICY_METADATA_PATH, 'rb') as f:
      policy_metadata = pickle.load(f)
      self.policy_input_shapes =  policy_metadata['input_shapes']
      self.policy_output_slices = policy_metadata['output_slices']
      policy_output_size = policy_metadata['output_shapes']['outputs'][1]

    self.frames = {name: DrivingModelFrame(context, ModelConstants.MODEL_RUN_FREQ//ModelConstants.MODEL_CONTEXT_FREQ) for name in self.vision_input_names}
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)

    # policy inputs
    self.numpy_inputs = {k: np.zeros(self.policy_input_shapes[k], dtype=np.float32) for k in self.policy_input_shapes}
    self.full_input_queues = InputQueues(ModelConstants.MODEL_CONTEXT_FREQ, ModelConstants.MODEL_RUN_FREQ, ModelConstants.N_FRAMES)
    for k in ['desire_pulse', 'features_buffer']:
      self.full_input_queues.update_dtypes_and_shapes({k: self.numpy_inputs[k].dtype}, {k: self.numpy_inputs[k].shape})
    self.full_input_queues.reset()

    # img buffers are managed in openCL transform code
    self.vision_inputs: dict[str, Tensor] = {}
    self.vision_output = np.zeros(vision_output_size, dtype=np.float32)
    self.policy_inputs = {k: Tensor(v, device='NPY').realize() for k,v in self.numpy_inputs.items()}
    self.policy_output = np.zeros(policy_output_size, dtype=np.float32)
    self.parser = Parser()

    with open(VISION_PKL_PATH, "rb") as f:
      self.vision_run = pickle.load(f)

    with open(POLICY_PKL_PATH, "rb") as f:
      self.policy_run = pickle.load(f)

  def slice_outputs(self, model_outputs: np.ndarray, output_slices: dict[str, slice]) -> dict[str, np.ndarray]:
    parsed_model_outputs = {k: model_outputs[np.newaxis, v] for k,v in output_slices.items()}
    return parsed_model_outputs

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
                inputs: dict[str, np.ndarray], prepare_only: bool) -> dict[str, np.ndarray] | None:
    # Model decides when action is completed, so desire input is just a pulse triggered on rising edge
    inputs['desire_pulse'][0] = 0
    new_desire = np.where(inputs['desire_pulse'] - self.prev_desire > .99, inputs['desire_pulse'], 0)
    self.prev_desire[:] = inputs['desire_pulse']

    imgs_cl = {name: self.frames[name].prepare(bufs[name], transforms[name].flatten()) for name in self.vision_input_names}

    if TICI and not USBGPU:
      # The imgs tensors are backed by opencl memory, only need init once
      for key in imgs_cl:
        if key not in self.vision_inputs:
          self.vision_inputs[key] = qcom_tensor_from_opencl_address(imgs_cl[key].mem_address, self.vision_input_shapes[key], dtype=dtypes.uint8)
    else:
      for key in imgs_cl:
        frame_input = self.frames[key].buffer_from_cl(imgs_cl[key]).reshape(self.vision_input_shapes[key])
        self.vision_inputs[key] = Tensor(frame_input, dtype=dtypes.uint8).realize()

    if prepare_only:
      return None

    self.vision_output = self.vision_run(**self.vision_inputs).contiguous().realize().uop.base.buffer.numpy()
    vision_outputs_dict = self.parser.parse_vision_outputs(self.slice_outputs(self.vision_output, self.vision_output_slices))

    self.full_input_queues.enqueue({'features_buffer': vision_outputs_dict['hidden_state'], 'desire_pulse': new_desire})
    for k in ['desire_pulse', 'features_buffer']:
      self.numpy_inputs[k][:] = self.full_input_queues.get(k)[k]
    self.numpy_inputs['traffic_convention'][:] = inputs['traffic_convention']

    self.policy_output = self.policy_run(**self.policy_inputs).contiguous().realize().uop.base.buffer.numpy()
    policy_outputs_dict = self.parser.parse_policy_outputs(self.slice_outputs(self.policy_output, self.policy_output_slices))

    combined_outputs_dict = {**vision_outputs_dict, **policy_outputs_dict}
    if SEND_RAW_PRED:
      combined_outputs_dict['raw_pred'] = np.concatenate([self.vision_output.copy(), self.policy_output.copy()])

    return combined_outputs_dict


def main(demo=False):
  cloudlog.warning("modeld init")

  if not USBGPU:
    # USB GPU currently saturates a core so can't do this yet,
    # also need to move the aux USB interrupts for good timings
    config_realtime_process(7, 54)

  st = time.monotonic()
  cloudlog.warning("setting up CL context")
  cl_context = CLContext()
  cloudlog.warning("CL context ready; loading model")
  model = ModelState(cl_context)
  cloudlog.warning(f"models loaded in {time.monotonic() - st:.1f}s, modeld starting")

  # visionipc clients
  while True:
    available_streams = VisionIpcClient.available_streams("camerad", block=False)
    if available_streams:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in available_streams and VisionStreamType.VISION_STREAM_ROAD in available_streams
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in available_streams
      break
    time.sleep(.1)

  vipc_client_main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide_camera else VisionStreamType.VISION_STREAM_ROAD
  vipc_client_main = VisionIpcClient("camerad", vipc_client_main_stream, True, cl_context)
  vipc_client_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False, cl_context)
  cloudlog.warning(f"vision stream set up, main_wide_camera: {main_wide_camera}, use_extra_client: {use_extra_client}")

  while not vipc_client_main.connect(False):
    time.sleep(0.1)
  while use_extra_client and not vipc_client_extra.connect(False):
    time.sleep(0.1)

  cloudlog.warning(f"connected main cam with buffer size: {vipc_client_main.buffer_len} ({vipc_client_main.width} x {vipc_client_main.height})")
  if use_extra_client:
    cloudlog.warning(f"connected extra cam with buffer size: {vipc_client_extra.buffer_len} ({vipc_client_extra.width} x {vipc_client_extra.height})")

  # messaging
  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry", "modelExt"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay",
                  "carParams", "customReservedRawData0", "radarState"])

  publish_state = PublishState()
  params = Params()

  # setup filter to track dropped frames
  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_RUN_FREQ)
  frame_id = 0
  last_vipc_frame_id = 0
  run_count = 0

  model_transform_main = np.zeros((3, 3), dtype=np.float32)
  model_transform_extra = np.zeros((3, 3), dtype=np.float32)
  live_calib_seen = False
  buf_main, buf_extra = None, None
  meta_main = FrameMeta()
  meta_extra = FrameMeta()


  if demo:
    CP = get_demo_car_params()
  else:
    CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  cloudlog.info("modeld got CarParams: %s", CP.brand)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  # TODO Move smooth seconds to action function
  long_delay = CP.longitudinalActuatorDelay + LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  dp_lat_lca_speed = int(params.get("dp_lat_lca_speed"))
  dp_lat_lca_auto_sec = float(params.get("dp_lat_lca_auto_sec"))
  DH = DesireHelper(dp_lat_lca_speed=dp_lat_lca_speed, dp_lat_lca_auto_sec=dp_lat_lca_auto_sec)
  RED = RoadEdgeDetector(params.get_bool("dp_lat_road_edge_detection"))
  AA = AutoAvoidanceHelper()
  AO = AutoOvertakeHelper()
  cone_in_path = False
  vehicle_in_path = False
  cone_metric = 0.0
  vehicle_metric = 0.0
  avoid_obstacle_x_offset = 0.0
  avoid_obstacle_x_valid = False
  left_lane_haz_dist_m = 0.0
  right_lane_haz_dist_m = 0.0
  left_lane_blocked_until = 0.0
  right_lane_blocked_until = 0.0
  cone_last_update_t = 0.0

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

    if sm.updated.get("customReservedRawData0", False):
      try:
        raw = sm["customReservedRawData0"]
        payload = decode_cone_detections(raw) if raw else None
        if payload is not None:
          cone_in_path = bool(payload.get("inPath", False))
          vehicle_in_path = bool(payload.get("vehicleInPath", False))
          try:
            cone_metric = float(payload.get("coneMetric", 0.0) or 0.0)
          except Exception:
            cone_metric = 0.0
          try:
            vehicle_metric = float(payload.get("vehicleMetric", 0.0) or 0.0)
          except Exception:
            vehicle_metric = 0.0
          try:
            left_lane_haz_dist_m = float(payload.get("leftLaneHazDistM", 0.0) or 0.0)
            right_lane_haz_dist_m = float(payload.get("rightLaneHazDistM", 0.0) or 0.0)
          except Exception:
            left_lane_haz_dist_m = 0.0
            right_lane_haz_dist_m = 0.0

          # Best-effort obstacle lateral position (for avoidance side preference).
          # We intentionally keep this simple: pick the closest in-path cone/vehicle bbox and use its x offset.
          avoid_obstacle_x_valid = False
          avoid_obstacle_x_offset = 0.0
          try:
            img_w = int(payload.get("imgW", 0) or 0)
            img_h = int(payload.get("imgH", 0) or 0)
            objs = payload.get("objsR", None) or payload.get("objs", None) or []
            if img_w > 0 and img_h > 0 and isinstance(objs, list):
              x_min = float(img_w) * 0.35
              x_max = float(img_w) * 0.65
              y_min = float(img_h) * 0.55
              best_y2 = -1.0
              best_cx = 0.0
              for o in objs:
                if not isinstance(o, dict):
                  continue
                try:
                  cls = int(o.get("c", -1))
                  if cls not in (0, 1, 2, 3, 5, 7, 80):
                    continue
                  x1 = float(o.get("x1", 0.0))
                  y1 = float(o.get("y1", 0.0))
                  x2 = float(o.get("x2", 0.0))
                  y2 = float(o.get("y2", 0.0))
                except Exception:
                  continue
                if not (np.isfinite(x1) and np.isfinite(y1) and np.isfinite(x2) and np.isfinite(y2)):
                  continue
                if x2 <= x1 or y2 <= y1:
                  continue
                cx = 0.5 * (x1 + x2)
                if not (x_min <= cx <= x_max and y2 >= y_min):
                  continue
                if y2 > best_y2:
                  best_y2 = y2
                  best_cx = cx
              if best_y2 > 0.0:
                avoid_obstacle_x_offset = float((best_cx - float(img_w) * 0.5) / max(float(img_w) * 0.5, 1.0))
                avoid_obstacle_x_offset = float(max(-1.0, min(1.0, avoid_obstacle_x_offset)))
                avoid_obstacle_x_valid = True
          except Exception:
            avoid_obstacle_x_valid = False
            avoid_obstacle_x_offset = 0.0
          cone_last_update_t = time.monotonic()
      except Exception:
        cloudlog.exception("failed to parse cone detections")

    if time.monotonic() - cone_last_update_t > 1.0:
      cone_in_path = False
      vehicle_in_path = False
      cone_metric = 0.0
      vehicle_metric = 0.0
      avoid_obstacle_x_offset = 0.0
      avoid_obstacle_x_valid = False
      left_lane_haz_dist_m = 0.0
      right_lane_haz_dist_m = 0.0
      left_lane_blocked_until = 0.0
      right_lane_blocked_until = 0.0

    desire = DH.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    lat_delay = sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS
    if sm.updated["liveCalibration"] and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
      model_transform_main = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics, True).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if desire >= 0 and desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    # tracked dropped frames
    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10: # let frame drops warm up
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count = run_count + 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    bufs = {name: buf_extra if 'big' in name else buf_main for name in model.vision_input_names}
    transforms = {name: model_transform_extra if 'big' in name else model_transform_main for name in model.vision_input_names}
    inputs:dict[str, np.ndarray] = {
      'desire_pulse': vec_desire,
      'traffic_convention': traffic_convention,
    }

    mt1 = time.perf_counter()
    model_output = model.run(bufs, transforms, inputs, prepare_only)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')
      model_ext_send = messaging.new_message('modelExt')

      action = get_action_from_model(model_output, prev_action, lat_delay + DT_MDL, long_delay + DT_MDL, v_ego)
      prev_action = action
      fill_model_msg(drivingdata_send, modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      RED.update(modelv2_send.modelV2.roadEdgeStds, modelv2_send.modelV2.laneLineProbs)
      model_ext_send.modelExt.leftEdgeDetected = RED.left_edge_detected
      model_ext_send.modelExt.rightEdgeDetected = RED.right_edge_detected

      cs = sm["carState"]
      lat_active = bool(sm["carControl"].latActive)
      one_blinker = cs.leftBlinker != cs.rightBlinker
      bsm_available = bool(sm.valid.get("carParams", False) and sm["carParams"].enableBsm)
      left_ok = bsm_available and (not cs.leftBlindspot) and (not RED.left_edge_detected)
      right_ok = bsm_available and (not cs.rightBlindspot) and (not RED.right_edge_detected)
      # Additional forward check: don't auto lane-change into an occupied target lane.
      # This is based on coned's YOLO detections; values are 0.0 when not available.
      now_mono = time.monotonic()
      target_lane_block_dist_m = max(AUTO_LC_TARGET_LANE_MIN_DIST_M,
                                     min(AUTO_LC_TARGET_LANE_MAX_DIST_M, float(v_ego) * AUTO_LC_TARGET_LANE_TGAP_S))
      if AUTO_LC_TARGET_LANE_BLOCK_HOLD_SEC > 0.0:
        if left_lane_haz_dist_m > 0.1 and left_lane_haz_dist_m < target_lane_block_dist_m:
          left_lane_blocked_until = max(float(left_lane_blocked_until), now_mono + AUTO_LC_TARGET_LANE_BLOCK_HOLD_SEC)
        if right_lane_haz_dist_m > 0.1 and right_lane_haz_dist_m < target_lane_block_dist_m:
          right_lane_blocked_until = max(float(right_lane_blocked_until), now_mono + AUTO_LC_TARGET_LANE_BLOCK_HOLD_SEC)

      if left_lane_haz_dist_m > 0.1 and left_lane_haz_dist_m < target_lane_block_dist_m:
        left_ok = False
      if right_lane_haz_dist_m > 0.1 and right_lane_haz_dist_m < target_lane_block_dist_m:
        right_ok = False

      if now_mono < float(left_lane_blocked_until):
        left_ok = False
      if now_mono < float(right_lane_blocked_until):
        right_ok = False

      # Block *starting* auto lane changes in curves. This does not cancel an in-progress auto lane change.
      curve_block = False
      try:
        if DH.lane_change_state == log.LaneChangeState.off:
          yaw_rate = float(getattr(cs, "yawRate", 0.0))
          if np.isfinite(yaw_rate) and v_ego > 5.0:
            k_cur = abs(yaw_rate) / max(v_ego, 0.1)
            # Speed-dependent threshold: block more aggressively as speed rises.
            v0, v1 = 22.0, 35.0  # ~80 km/h .. ~126 km/h
            k0, k1 = 0.004, 0.002
            if v_ego <= v0:
              k_th = k0
            elif v_ego >= v1:
              k_th = k1
            else:
              k_th = k0 + (k1 - k0) * (v_ego - v0) / (v1 - v0)
            curve_block = k_cur > k_th
      except Exception:
        curve_block = False

      # Highway auto-overtake (lead-based). Uses radarState (which is also populated on radarless platforms).
      lead_present = False
      lead_d = 0.0
      v_lead = 0.0
      if sm.valid.get("radarState", False):
        lead_one = sm["radarState"].leadOne
        lead_present = bool(getattr(lead_one, "status", False))
        if lead_present:
          lead_d = float(getattr(lead_one, "dRel", 0.0))
          v_lead = float(getattr(lead_one, "vLead", 0.0))
          if not np.isfinite(v_lead) or v_lead <= 0.0:
            v_rel = float(getattr(lead_one, "vRel", 0.0))
            v_lead = float(v_ego + v_rel)

      v_cruise_kph = float(getattr(cs, "vCruise", V_CRUISE_UNSET))
      v_cruise = float(v_ego if v_cruise_kph == V_CRUISE_UNSET or v_cruise_kph <= 0.0 else (v_cruise_kph * CV.KPH_TO_MS))
      cruise_enabled = bool(getattr(cs.cruiseState, "enabled", False))

      # Lane preference: 0=auto, 1=keep left, 2=keep right (HUD cycles this).
      try:
        raw_pref = params.get("dp_lincoln_lane_preference") or b"0"
        if isinstance(raw_pref, bytes):
          raw_pref = raw_pref.decode("utf-8", errors="ignore")
        lane_pref = int(str(raw_pref).strip() or "0")
      except Exception:
        lane_pref = 0
      lane_pref = lane_pref if lane_pref in (0, 1, 2) else 0

      overtake_dir = AO.update(
        enabled=params.get_bool("dp_lincoln_auto_overtake") and lat_active and cruise_enabled,
        lc_state=DH.lane_change_state,
        v_ego=v_ego,
        v_cruise=v_cruise,
        lead_present=lead_present,
        lead_d=lead_d,
        v_lead=v_lead,
        left_ok=left_ok,
        right_ok=right_ok,
        is_rhd=bool(is_rhd),
        manual_blinker=bool(one_blinker),
        bsm_available=bsm_available,
        lane_preference=lane_pref,
      )

      # Treat "vehicle in path" as an avoidance obstacle only when it's likely stopped/very slow and close enough.
      # Cones are gated by a metric threshold to avoid weak/edge detections triggering lane changes too early.
      cone_obstacle = bool(cone_in_path and (cone_metric >= AVOID_CONE_METRIC_MIN))
      vehicle_obstacle = bool(vehicle_in_path and (
        (lead_present and (v_lead < AVOID_STOPPED_LEAD_SPEED_MS) and (vehicle_metric >= AVOID_VEHICLE_METRIC_MIN)) or
        ((not lead_present) and (vehicle_metric >= AVOID_VEHICLE_METRIC_MIN_NO_LEAD))
      ))
      obstacle_in_path_for_avoid = bool(cone_obstacle or vehicle_obstacle)

      avoid_prefer_dir = log.LaneChangeDirection.none
      if avoid_obstacle_x_valid:
        if avoid_obstacle_x_offset > 0.10:
          avoid_prefer_dir = log.LaneChangeDirection.left
        elif avoid_obstacle_x_offset < -0.10:
          avoid_prefer_dir = log.LaneChangeDirection.right

      avoid_dir = AA.update(
        enabled=params.get_bool("dp_lincoln_auto_avoid") and lat_active,
        obstacle_in_path=obstacle_in_path_for_avoid,
        lc_state=DH.lane_change_state,
        v_ego=v_ego,
        left_ok=left_ok,
        right_ok=right_ok,
        is_rhd=bool(is_rhd),
        manual_blinker=bool(one_blinker),
        bsm_available=bsm_available,
        prefer_dir=avoid_prefer_dir,
      )

      # Don't start new auto lane changes in a curve (manual blinkers still allowed).
      if curve_block and not one_blinker and DH.lane_change_state == log.LaneChangeState.off:
        overtake_dir = log.LaneChangeDirection.none
        avoid_dir = log.LaneChangeDirection.none

      auto_dir = avoid_dir if avoid_dir != log.LaneChangeDirection.none else overtake_dir
      DH.update(cs, lat_active, lane_change_prob, RED.left_edge_detected, RED.right_edge_detected,
                auto_lane_change_direction=auto_dir)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      fill_pose_msg(posenet_send, model_output, meta_main.frame_id, vipc_dropped_frames, meta_main.timestamp_eof, live_calib_seen)
      pm.send('modelV2', modelv2_send)
      pm.send('drivingModelData', drivingdata_send)
      pm.send('cameraOdometry', posenet_send)
      pm.send('modelExt', model_ext_send)
    last_vipc_frame_id = meta_main.frame_id


if __name__ == "__main__":
  try:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='A boolean for demo mode.')
    args = parser.parse_args()
    main(demo=args.demo)
  except KeyboardInterrupt:
    cloudlog.warning("got SIGINT")
