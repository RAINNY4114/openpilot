import math
import numpy as np

from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from opendbc.car.ford.apa_controller import FordAPAController

from openpilot.selfdrive.controls.lib.ford_curve_controller import FordCurveController


LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert


def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(
    v_ego,
    [5, 10],
    [apply_curvature, output_curvature],
  ))


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0

    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False

    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

    # Ford / Lincoln curvature controller.
    #
    # All Ford-specific curvature processing is handled inside this
    # controller:
    #
    # - Human Turn Detection
    # - Curve Entry / Hold / Exit
    # - current-curvature error protection
    # - Ford curvature rate limiting
    # - lateral acceleration limiting
    # - anti-overshoot
    # - post-driver-reset ramp
    #
    # CarController itself only supplies vehicle state and sends the
    # resulting curvature to the Ford CAN message.
    self.curve_controller = FordCurveController(CP)

    # ------------------------------------------------------------------
    # Dual-source curvature telemetry
    #
    # Records CC.currentCurvature (steering-angle / vehicle-model source,
    # as used by bluepilot bp-7.0) against the legacy -yawRate/vEgo source.
    #
    # Written at ~1 Hz to:
    #     /data/media/0/curv_src.log
    #
    # Format (comma separated):
    #     t,vEgo,cc_curv_steer_vm,yaw_curv,diff
    # ------------------------------------------------------------------
    self._curv_src_path = "/data/media/0/curv_src.log"
    self._curv_src_frame = 0
    self._curv_src_fh = None

    # ========================================================================
    # [C2_APA][路线A] Ford APA 角度域支路 (ParkAid_Data, 0x3A8)
    #
    # 与既有 FordCurveController 并存:
    #   - 既有: curvature -> LatCtl_D_Rq (0x3D3 / 0x3D6)  不受影响
    #   - APA : curvature -> ParkAid_Data (0x3A8)          本支路
    #
    # 默认全程静默 (dp_ford_apa_enable 缺省 0), 不产生任何 CAN。
    # ========================================================================
    self.apa_controller = FordAPAController(CP)

  # ======================================================================
  # Dual-source curvature telemetry
  # ======================================================================

  def _curv_src_log(self, v_ego, cc_curv, yaw_curv):
    """Append one dual-source curvature sample at ~1 Hz.

    Fails silently: telemetry must never break lateral control.
    """
    try:
      # CarController runs at 100 Hz; STEER_STEP gates us to ~20 Hz, so
      # every 20th call here is ~1 Hz.
      self._curv_src_frame += 1
      if self._curv_src_frame % 20 != 0:
        return

      if self._curv_src_fh is None:
        self._curv_src_fh = open(self._curv_src_path, "a", buffering=1)

      import time as _time
      diff = cc_curv - yaw_curv
      self._curv_src_fh.write(
        "%.3f,%.3f,%.8f,%.8f,%.8f\n"
        % (_time.monotonic(), v_ego, cc_curv, yaw_curv, diff)
      )
    except Exception:
      pass

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (
      VisualAlert.steerRequired,
      VisualAlert.ldw,
    )
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # =======================================================================
    # ACC buttons
    # =======================================================================

    if CC.cruiseControl.cancel:
      can_sends.append(
        fordcan.create_button_msg(
          self.packer,
          self.CAN.camera,
          CS.buttons_stock_values,
          cancel=True,
        )
      )
      can_sends.append(
        fordcan.create_button_msg(
          self.packer,
          self.CAN.main,
          CS.buttons_stock_values,
          cancel=True,
        )
      )

    elif CC.cruiseControl.resume and (
      self.frame % CarControllerParams.BUTTONS_STEP
    ) == 0:
      can_sends.append(
        fordcan.create_button_msg(
          self.packer,
          self.CAN.camera,
          CS.buttons_stock_values,
          resume=True,
        )
      )
      can_sends.append(
        fordcan.create_button_msg(
          self.packer,
          self.CAN.main,
          CS.buttons_stock_values,
          resume=True,
        )
      )

    # If stock lane centering isn't off, send a button press to toggle it off.
    # The stock system checks for steering pressed, and eventually disengages
    # cruise control.
    elif (
      CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0
      and (self.frame % CarControllerParams.ACC_UI_STEP) == 0
    ):
      can_sends.append(
        fordcan.create_button_msg(
          self.packer,
          self.CAN.camera,
          CS.buttons_stock_values,
          tja_toggle=True,
        )
      )

    # =======================================================================
    # Lateral control
    # =======================================================================

    # Send steer msg at 20Hz.
    if (self.frame % CarControllerParams.STEER_STEP) == 0:

      v_ego = float(max(CS.out.vEgoRaw, 0.1))

      # Keep the existing Bronco / F-150 anti-overshoot behavior.
      #
      # This remains outside FordCurveController because it is an existing
      # vehicle-specific CarController behavior.
      if self.CP.carFingerprint in (
        CAR.FORD_BRONCO_SPORT_MK1,
        CAR.FORD_F_150_MK14,
      ):
        self.anti_overshoot_curvature_last = anti_overshoot(
          actuators.curvature,
          self.anti_overshoot_curvature_last,
          v_ego,
        )
        requested_curvature = self.anti_overshoot_curvature_last
      else:
        requested_curvature = float(actuators.curvature)

      # ---------------------------------------------------------------
      # Current vehicle curvature
      #
      # FordCurveController uses this for the RAINNY-style
      # current-curvature error protection.
      #
      # BLUEPILOT-STYLE STEERING-ANGLE SOURCE
      # -------------------------------------
      # bluepilot bp-7.0 (commit 26030f3cb) measures "current curvature"
      # from the steering angle through the vehicle model instead of the
      # RCM yaw-rate signal:
      #
      #     -VM.calc_curvature(radians(steer - angleOffset), vEgo, roll)
      #
      # On this device that exact quantity is already computed once per
      # controlsd cycle and published as `CC.currentCurvature`
      # (controlsd.py: steer_angle_without_offset -> self.VM.calc_curvature,
      #  with VehicleModel fed by lp.stiffnessFactor and lp.steerRatio).
      #
      # We therefore consume CC.currentCurvature directly so that BOTH
      # curvature consumers share a single measurement source. This keeps
      # the online-learned steerRatio (vehicleParameters.steerRatio,
      # ~15.014 on this car) consistent between controlsd and the Ford
      # curvature controller.
      #
      # The yaw-rate estimate is retained purely as a fallback for the
      # cases where CC.currentCurvature is not usable:
      #   * before controlsd has published a valid value (== 0.0),
      #   * while vehicle model params are still converging (stiffness/
      #     steer ratio clamped to their floor by controlsd).
      # ---------------------------------------------------------------

      yaw_current_curvature = (
        -float(CS.out.yawRate) / v_ego
      )

      cc_current_curvature = float(
        getattr(CC, "currentCurvature", 0.0) or 0.0
      )

      # A finite, non-zero model curvature means controlsd has a live
      # VehicleModel estimate. Fall back to yaw only while it does not.
      if math.isfinite(cc_current_curvature) and cc_current_curvature != 0.0:
        current_curvature = cc_current_curvature
      else:
        current_curvature = yaw_current_curvature

      # Dual-source telemetry. Appended at ~1 Hz to bound file growth.
      # Columns: vEgo, CC.currentCurvature (steer/VM), -yawRate/vEgo.
      # Used to validate the source switch and to re-tune CURVATURE_ERROR.
      self._curv_src_log(
        v_ego,
        cc_current_curvature,
        yaw_current_curvature,
      )

      # ---------------------------------------------------------------
      # Driver steering state
      #
      # These values are consumed by HumanTurnDetection.
      # ---------------------------------------------------------------

      steering_angle_deg = float(
        getattr(
          CS.out,
          "steeringAngleDeg",
          0.0,
        )
      )

      steering_torque_nm = float(
        getattr(
          CS.out,
          "steeringTorque",
          0.0,
        )
      )

      steering_pressed = bool(
        getattr(
          CS.out,
          "steeringPressed",
          False,
        )
      )

      # ---------------------------------------------------------------
      # Cruise state
      #
      # HTD disables itself while cruise is enabled, according to the
      # supplied FordCurveController implementation.
      # ---------------------------------------------------------------

      cruise_enabled = bool(
        getattr(
          CS.out.cruiseState,
          "enabled",
          False,
        )
      )

      # S-gear (Sport). Ford's selector position 4 ("Sport_DriveSport") is
      # reported by carstate.py as GearShifter.sport. FordCurveController uses
      # it for its Sport profile, but only when `dp_ford_sport_enable` is set.
      sport_gear = bool(
        CS.out.gearShifter == structs.CarState.GearShifter.sport
      )

      # ---------------------------------------------------------------
      # Ford Curve Controller
      #
      # IMPORTANT:
      # Do NOT apply CURVATURE_ERROR or CURVATURE_LIMITS here again.
      # FordCurveController is now the single curvature-processing stage.
      # ---------------------------------------------------------------

      apply_curvature = self.curve_controller.update(
        desired_curvature=requested_curvature,
        v_ego=v_ego,
        active=bool(CC.latActive),
        steering_angle_deg=steering_angle_deg,
        steering_torque_nm=steering_torque_nm,
        steering_pressed=steering_pressed,
        lat_active=bool(CC.latActive),
        cruise_enabled=cruise_enabled,
        current_curvature=current_curvature,
      )

      # Extract path_angle (c1) from FordCurveController (BluePilot bp-7.0)
      apply_path_angle = getattr(self.curve_controller, 'path_angle', 0.0)

      self.apply_curvature_last = float(apply_curvature)

      # =====================================================================
      # Ford lateral CAN message
      # =====================================================================

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        #
        # Ford uses four individual signals to dictate how to drive to the
        # car. Curvature alone (limited to 0.02 m^-1) can actuate the steering
        # for a large portion of any lateral movements. However, in order to
        # get further control on steer actuation, the other three signals are
        # necessary.

        mode = 1 if CC.latActive else 0
        counter = (
          self.frame // CarControllerParams.STEER_STEP
        ) % 0x10

        can_sends.append(
          fordcan.create_lat_ctl2_msg(
            self.packer,
            self.CAN,
            mode,
            0.,
            0.,
            -self.apply_curvature_last,
            0.,
            counter,
          )
        )

      else:
        # Q3 / non-CANFD Ford path.
        #
        # This is the path required for the 2018 Lincoln MKX / Nautilus
        # configuration.
        can_sends.append(
          fordcan.create_lat_ctl_msg(
            self.packer,
            self.CAN,
            CC.latActive,
            0.,
            0.,
            -self.apply_curvature_last,
            0.,
          )
        )

    # =======================================================================
    # LKA
    # =======================================================================

    # Send lka msg at 33Hz.
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(
        fordcan.create_lka_msg(
          self.packer,
          self.CAN,
        )
      )

    # =======================================================================
    # Longitudinal control
    # =======================================================================

    # Send acc msg at 50Hz.
    if (
      self.CP.openpilotLongitudinalControl
      and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0
    ):
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction
        # is very slow.
        accel = apply_creep_compensation(
          accel,
          CS.out.vEgo,
        )

        # The stock system has been seen rate limiting the brake accel to
        # 5 m/s^3, however even 3.5 m/s^3 causes some overshoot with a step
        # response.
        accel = max(
          accel,
          self.accel
          - (
            3.5
            * CarControllerParams.ACC_CONTROL_STEP
            * DT_CTRL
          ),
        )

      accel = float(
        np.clip(
          accel,
          CarControllerParams.ACCEL_MIN,
          CarControllerParams.ACCEL_MAX,
        )
      )

      gas = float(
        np.clip(
          gas,
          CarControllerParams.ACCEL_MIN,
          CarControllerParams.ACCEL_MAX,
        )
      )

      # Both gas and accel are in m/s^2.
      # accel is used solely for braking.
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to
      # compensate for the brake/pre-charge bits.
      accel_due_to_pitch = 0.0

      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = (
          math.sin(CC.orientationNED[1])
          * ACCELERATION_DUE_TO_GRAVITY
        )

      accel_pitch_compensated = (
        accel + accel_due_to_pitch
      )

      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = (
        CC.actuators.longControlState
        == LongCtrlState.stopping
      )

      can_sends.append(
        fordcan.create_acc_msg(
          self.packer,
          self.CAN,
          CC.longActive,
          gas,
          accel,
          stopping,
          self.brake_request,
          # [FIX] ACC_PRED_REQUEST_CHANNEL: stock sends the set speed here,
          # openpilot sent V_CRUISE_MAX (145) which may hold a low gear.
          v_ego_kph=(
            float(CS.out.cruiseState.speed) * 3.6
            if CS.out.cruiseState.speed > 0
            else V_CRUISE_MAX
          ),
        )
      )

      self.accel = accel
      self.gas = gas

    # =======================================================================
    # UI
    # =======================================================================

    send_ui = (
      (self.main_on_last != main_on)
      or (self.lkas_enabled_last != CC.latActive)
      or (self.steer_alert_last != steer_alert)
    )

    # Send lkas ui msg at 1Hz or if ui state changes.
    if (
      (self.frame % CarControllerParams.LKAS_UI_STEP) == 0
      or send_ui
    ):
      can_sends.append(
        fordcan.create_lkas_ui_msg(
          self.packer,
          self.CAN,
          main_on,
          CC.latActive,
          steer_alert,
          hud_control,
          CS.lkas_status_stock_values,
        )
      )

    # Send acc ui msg at 5Hz or if ui state changes.
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (
      (self.frame % CarControllerParams.ACC_UI_STEP) == 0
      or send_ui
    ):
      show_distance_bars = (
        self.frame - self.distance_bar_frame < 400
      )

      can_sends.append(
        fordcan.create_acc_ui_msg(
          self.packer,
          self.CAN,
          self.CP,
          main_on,
          CC.latActive,
          fcw_alert,
          CS.out.cruiseState.standstill,
          show_distance_bars,
          hud_control,
          CS.acc_tja_status_stock_values,
        )
      )

    # =======================================================================
    # [C2_APA][路线A] APA 角度域横向支路 (ParkAid_Data, 0x3A8)
    #
    # 只在 dp_ford_apa_enable == 2 (L2 实发) 时才产生 CAN。
    # 异常完全隔离: 任何异常都不影响既有控制回路。
    # =======================================================================
    try:
      apa_msg = self.apa_controller.update(
        frame=self.frame,
        lat_active=bool(CC.latActive),
        curvature=self.apply_curvature_last,
        v_ego=float(max(CS.out.vEgoRaw, 0.1)),
        sapp_state=getattr(CS, "apa_sapp_state", 0),
        eps_assist_limited=getattr(CS, "apa_eps_assist_limited", False),
        veh_speed_kph=getattr(CS, "apa_veh_speed_kph", 0.0),
        now_ms=int(now_nanos // 1000000),
      )
      if apa_msg is not None:
        can_sends.append(apa_msg)
    except Exception:
      pass

    # =======================================================================
    # State update
    # =======================================================================

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()

    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1

    return new_actuators, can_sends