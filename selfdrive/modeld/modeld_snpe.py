#!/usr/bin/env python3
import os
import time
import numpy as np
import pickle
import cereal.messaging as messaging
from cereal import car, log
from pathlib import Path
from setproctitle import setproctitle
from cereal.messaging import PubMaster, SubMaster
from msgq.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL, config_realtime_process
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.system import sentry
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, smooth_value, get_curvature_from_plan
from openpilot.selfdrive.livedelay.helpers import get_lat_delay

from openpilot.selfdrive.modeld.runners import ModelRunner, Runtime
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.models.commonmodel_pyx import ModelFrame, CLContext
from openpilot.selfdrive.modeld.model_manager_helpers import get_active_bundle, get_supercombo_bundle_paths
from dragonpilot.selfdrive.controls.lib.road_edge_detector import RoadEdgeDetector


PROCESS_NAME = "selfdrive.modeld.modeld_snpe"
SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

DEFAULT_THNEED_PATH = Path(__file__).parent / 'models/supercombo.thneed'
DEFAULT_ONNX_PATH = Path(__file__).parent / 'models/supercombo.onnx'
DEFAULT_METADATA_PATH = Path(__file__).parent / 'models/supercombo_metadata.pkl'

LAT_SMOOTH_SECONDS = 0.1
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3


def _resolve_supercombo_paths(params: Params) -> tuple[dict, Path]:
  bundle = get_active_bundle(params)
  if bundle is not None:
    paths = get_supercombo_bundle_paths(bundle)
    if paths and paths["model"].is_file() and paths["metadata"].is_file():
      model_paths = {ModelRunner.THNEED: paths["model"]}
      if DEFAULT_ONNX_PATH.is_file():
        model_paths[ModelRunner.ONNX] = DEFAULT_ONNX_PATH
      return model_paths, paths["metadata"]

  if DEFAULT_THNEED_PATH.is_file() and DEFAULT_METADATA_PATH.is_file():
    model_paths = {ModelRunner.THNEED: DEFAULT_THNEED_PATH}
    if DEFAULT_ONNX_PATH.is_file():
      model_paths[ModelRunner.ONNX] = DEFAULT_ONNX_PATH
    return model_paths, DEFAULT_METADATA_PATH

  params.remove("ModelManager_ActiveBundle")
  raise FileNotFoundError("modeld_snpe: no supercombo model available")


def _load_metadata(metadata_path: Path) -> dict:
  with metadata_path.open("rb") as f:
    return pickle.load(f)


def _prepare_inputs(model_metadata: dict) -> dict[str, np.ndarray]:
  inputs = {
    k: np.zeros(v, dtype=np.float32).flatten()
    for k, v in model_metadata['input_shapes'].items()
    if 'img' not in k
  }
  return inputs


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


class ModelState:
  frame: ModelFrame
  wide_frame: ModelFrame
  inputs: dict[str, np.ndarray]
  output: np.ndarray
  prev_desire: np.ndarray
  model: ModelRunner

  def __init__(self, context: CLContext):
    self.frame = ModelFrame(context)
    self.wide_frame = ModelFrame(context)
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)

    params = Params()
    model_paths, metadata_path = _resolve_supercombo_paths(params)
    self.model_metadata = _load_metadata(metadata_path)
    self.inputs = _prepare_inputs(self.model_metadata)
    self.output_slices = self.model_metadata['output_slices']
    net_output_size = self.model_metadata['output_shapes']['outputs'][1]
    self.output = np.zeros(net_output_size, dtype=np.float32)
    self.parser = Parser()

    overrides = {}
    if bundle := get_active_bundle(params):
      overrides = {override.key: override.value for override in bundle.overrides}
    self.LAT_SMOOTH_SECONDS = float(overrides.get('lat', f"{LAT_SMOOTH_SECONDS}"))
    self.LONG_SMOOTH_SECONDS = float(overrides.get('long', f"{LONG_SMOOTH_SECONDS}"))

    self.model = ModelRunner(model_paths, self.output, Runtime.GPU, False, context)
    self.model.addInput("input_imgs", None)
    self.model.addInput("big_input_imgs", None)
    for k, v in self.inputs.items():
      self.model.addInput(k, v)

  def slice_outputs(self, model_outputs: np.ndarray) -> dict[str, np.ndarray]:
    return {k: model_outputs[np.newaxis, v] for k, v in self.output_slices.items()}

  def run(self, buf: VisionBuf, wbuf: VisionBuf, transform: np.ndarray, transform_wide: np.ndarray,
          inputs: dict[str, np.ndarray], prepare_only: bool) -> dict[str, np.ndarray] | None:
    inputs['desire'][0] = 0
    self.inputs['desire'][:-ModelConstants.DESIRE_LEN] = self.inputs['desire'][ModelConstants.DESIRE_LEN:]
    self.inputs['desire'][-ModelConstants.DESIRE_LEN:] = np.where(inputs['desire'] - self.prev_desire > .99, inputs['desire'], 0)
    self.prev_desire[:] = inputs['desire']

    for k in self.inputs:
      if k in inputs and k != 'desire':
        self.inputs[k][:] = inputs[k]

    self.model.setInputBuffer("input_imgs", self.frame.prepare(buf, transform.flatten(), self.model.getCLBuffer("input_imgs")))
    if wbuf is not None:
      self.model.setInputBuffer("big_input_imgs", self.wide_frame.prepare(wbuf, transform_wide.flatten(), self.model.getCLBuffer("big_input_imgs")))

    if prepare_only:
      return None

    self.model.execute()
    outputs = self.parser.parse_outputs(self.slice_outputs(self.output))

    self.inputs['features_buffer'][:-ModelConstants.FEATURE_LEN] = self.inputs['features_buffer'][ModelConstants.FEATURE_LEN:]
    self.inputs['features_buffer'][-ModelConstants.FEATURE_LEN:] = outputs['hidden_state'][0, :]

    if "desired_curvature" in outputs:
      if "prev_desired_curvs" in self.inputs.keys():
        self.inputs['prev_desired_curvs'][:-1] = self.inputs['prev_desired_curvs'][1:]
        self.inputs['prev_desired_curvs'][-1] = outputs['desired_curvature'][0, 0]
      if "prev_desired_curv" in self.inputs.keys():
        self.inputs['prev_desired_curv'][:-1] = self.inputs['prev_desired_curv'][1:]
        self.inputs['prev_desired_curv'][-1:] = outputs['desired_curvature'][0, :]

    return outputs

  def get_action_from_model(self, model_output: dict[str, np.ndarray], prev_action: log.ModelDataV2.Action,
                            lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
    plan = model_output['plan'][0]
    desired_accel, should_stop = get_accel_from_plan(plan[:, Plan.VELOCITY][:, 0],
                                                     plan[:, Plan.ACCELERATION][:, 0],
                                                     ModelConstants.T_IDXS,
                                                     action_t=long_action_t)
    desired_accel = smooth_value(desired_accel, prev_action.desiredAcceleration, self.LONG_SMOOTH_SECONDS)

    desired_curvature = get_curvature_from_plan(plan[:, Plan.T_FROM_CURRENT_EULER][:, 2],
                                                plan[:, Plan.ORIENTATION_RATE][:, 2],
                                                ModelConstants.T_IDXS,
                                                v_ego,
                                                lat_action_t)
    if v_ego > MIN_LAT_CONTROL_SPEED:
      desired_curvature = smooth_value(desired_curvature, prev_action.desiredCurvature, self.LAT_SMOOTH_SECONDS)
    else:
      desired_curvature = prev_action.desiredCurvature

    return log.ModelDataV2.Action(desiredCurvature=float(desired_curvature),
                                  desiredAcceleration=float(desired_accel),
                                  shouldStop=bool(should_stop))


def main(demo: bool = False) -> None:
  cloudlog.warning("modeld_snpe init")
  config_realtime_process(7, 54)

  cl_context = CLContext()
  model = ModelState(cl_context)
  model.lat_delay = 0.0

  use_extra_client = True
  main_wide_camera = False
  while True:
    avail = VisionIpcClient.available_streams("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    if avail:
      use_extra_client = VisionStreamType.VISION_STREAM_WIDE_ROAD in avail and VisionStreamType.VISION_STREAM_ROAD in avail
      main_wide_camera = VisionStreamType.VISION_STREAM_ROAD not in avail
      break
    time.sleep(0.1)

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

  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry", "modelExt"])
  sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay"])

  publish_state = PublishState()
  params = Params()
  RED = RoadEdgeDetector(params.get_bool("dp_lat_road_edge_detection"))

  frame_dropped_filter = FirstOrderFilter(0., 10., 1. / ModelConstants.MODEL_FREQ)
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

  cloudlog.info("modeld_snpe got CarParams: %s", CP.brand)
  long_delay = CP.longitudinalActuatorDelay + model.LONG_SMOOTH_SECONDS
  prev_action = log.ModelDataV2.Action()

  DH = DesireHelper()

  while True:
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_client_main.recv()
      meta_main = FrameMeta(vipc_client_main)
      if buf_main is None:
        break

    if buf_main is None:
      cloudlog.debug("vipc_client_main no frame")
      continue

    if use_extra_client:
      while True:
        buf_extra = vipc_client_extra.recv()
        meta_extra = FrameMeta(vipc_client_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break

      if buf_extra is None:
        cloudlog.debug("vipc_client_extra no frame")
        continue

      if abs(meta_main.timestamp_sof - meta_extra.timestamp_sof) > 10000000:
        cloudlog.error(
          f"frames out of sync! main: {meta_main.frame_id} ({meta_main.timestamp_sof / 1e9:.5f}), "
          f"extra: {meta_extra.frame_id} ({meta_extra.timestamp_sof / 1e9:.5f})"
        )
    else:
      buf_extra = buf_main
      meta_extra = meta_main

    sm.update(0)
    desire = DH.desire
    v_ego = sm["carState"].vEgo
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["roadCameraState"].frameId

    lat_delay = get_lat_delay(params, sm["liveDelay"].lateralDelay, CP.steerActuatorDelay) + model.LAT_SMOOTH_SECONDS
    if sm.updated["liveCalibration"] and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      device_from_calib_euler = np.array(sm["liveCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
      model_transform_main = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics, False).astype(np.float32)
      model_transform_extra = get_warp_matrix(device_from_calib_euler, dc.ecam.intrinsics, True).astype(np.float32)
      live_calib_seen = True

    traffic_convention = np.zeros(2)
    traffic_convention[int(is_rhd)] = 1

    vec_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
    if 0 <= desire < ModelConstants.DESIRE_LEN:
      vec_desire[desire] = 1

    vipc_dropped_frames = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_dropped_filter.update(min(vipc_dropped_frames, 10))
    if run_count < 10:
      frame_dropped_filter.x = 0.
      frames_dropped = 0.
    run_count += 1

    frame_drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = vipc_dropped_frames > 0
    if prepare_only:
      cloudlog.error(f"skipping model eval. Dropped {vipc_dropped_frames} frames")

    inputs: dict[str, np.ndarray] = {
      'desire': vec_desire,
      'traffic_convention': traffic_convention,
    }

    if "lateral_control_params" in model.inputs.keys():
      inputs['lateral_control_params'] = np.array([max(v_ego, 0.), lat_delay], dtype=np.float32)

    if "driving_style" in model.inputs.keys():
      inputs['driving_style'] = np.array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], dtype=np.float32)

    if "nav_features" in model.inputs.keys():
      inputs['nav_features'] = np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32)

    if "nav_instructions" in model.inputs.keys():
      inputs['nav_instructions'] = np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32)

    mt1 = time.perf_counter()
    model_output = model.run(buf_main, buf_extra, model_transform_main, model_transform_extra, inputs, prepare_only)
    mt2 = time.perf_counter()
    model_execution_time = mt2 - mt1

    if model_output is not None:
      modelv2_send = messaging.new_message('modelV2')
      drivingdata_send = messaging.new_message('drivingModelData')
      posenet_send = messaging.new_message('cameraOdometry')
      model_ext_send = messaging.new_message('modelExt')

      action = model.get_action_from_model(model_output, prev_action, lat_delay + DT_MDL, long_delay + DT_MDL, v_ego)
      prev_action = action
      fill_model_msg(drivingdata_send, modelv2_send, model_output, action,
                     publish_state, meta_main.frame_id, meta_extra.frame_id, frame_id,
                     frame_drop_ratio, meta_main.timestamp_eof, model_execution_time, live_calib_seen)

      desire_state = modelv2_send.modelV2.meta.desireState
      l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_state[log.Desire.laneChangeRight]
      lane_change_prob = l_lane_change_prob + r_lane_change_prob
      DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)
      modelv2_send.modelV2.meta.laneChangeState = DH.lane_change_state
      modelv2_send.modelV2.meta.laneChangeDirection = DH.lane_change_direction
      drivingdata_send.drivingModelData.meta.laneChangeState = DH.lane_change_state
      drivingdata_send.drivingModelData.meta.laneChangeDirection = DH.lane_change_direction

      RED.update(modelv2_send.modelV2.roadEdgeStds, modelv2_send.modelV2.laneLineProbs)
      model_ext_send.modelExt.leftEdgeDetected = RED.left_edge_detected
      model_ext_send.modelExt.rightEdgeDetected = RED.right_edge_detected

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
    cloudlog.warning(f"child {PROCESS_NAME} got SIGINT")
  except Exception:
    sentry.capture_exception()
    raise
