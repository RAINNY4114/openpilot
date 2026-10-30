#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  }
  if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  }
  if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

// ---------------------------------------------------------------------------------------------
// [CHX PATH_ANGLE 20261027] c1 (LatCtlPath_An_Actl) 放行 —— 弯道 angle 模式
//
// 上游把 path_angle 钉死在 inactive sentinel 上（见 tx_hook 里的 violation 判断），因为
// openpilot 从来没用过这个信号。本补丁把它放开，作为一个**额外**的转向执行通道：
// 弯道时由 Python 侧计算 path_angle = -kappa * v * gain 下发，让 PSCM 把角度当作内部
// 位置目标处理，而不是只吃曲率前馈。
//
// 线束换算（LateralMotionControl 与 LateralMotionControl2 相同）：
//   LatCtlPath_An_Actl : 11 bit，0.0005 rad/bit，offset -0.5  ->  raw = phys * 2000 + 1000
//   物理范围 -0.5 .. 0.5235 rad，inactive（0.0 rad）= 1000
//
// 幅值上限 ±0.45 rad（约 26°）。Python 侧 path_angle 来自上游已限幅的 apply_curvature
// （|kappa| <= 0.02 m^-1），乘以 v 与增益后 |path_angle| <= 0.02 * 25 * 1.3 = 0.65 rad，
// 故 0.45 是真实可能触发的硬上限，不是摆设；超限由 path_angle_cmd_checks 的 ROC 先拦住。
#define FORD_PATH_ANGLE_MIN -0.45f
#define FORD_PATH_ANGLE_MAX 0.45f

// 变化率上限，与 Python 侧 _soft_roc_limit 对齐：
//   speeds {9,10,15,25} m/s -> {0.055, 0.055, 0.0425, 0.009} rad / 20 Hz 拍
// lookup 固定 3 点且曲线在 9~10 m/s 平坦，故 {10,15,25} 可精确复现（<10 夹到首点）。
// 乘 1.02 使固件窗口永远比 Python 实际发送的宽 2%，绝不反向拦截；再加上 +1 CAN 单位
// 与 1 m/s 的速度下限让步，留足余量。
static const AngleSteeringLimits FORD_PATH_ANGLE_LIMITS = {
  .max_angle = 1000,              // path_angle_cmd_checks 不用它，仅为结构体完整
  .angle_deg_to_can = 2000,       // 1 / 0.0005 rad per CAN unit
  .angle_rate_up_lookup = {
    {10., 15., 25.},
    {0.0561f, 0.04335f, 0.00918f}
  },
  .angle_rate_down_lookup = {
    {10., 15., 25.},
    {0.0561f, 0.04335f, 0.00918f}
  },
  .frequency = 20U,               // LateralMotionControl / LateralMotionControl2 @ 20 Hz
};

static const CurvatureSteeringLimits FORD_STEERING_LIMITS = {
  .max_curvature = 1000,              // 0.02 rad/m * curvature_to_can
  .curvature_to_can = 50000,          // CAN units per rad/m
  .frequency = 20,                    // Hz
  .max_curvature_error = 100,         // 0.002 rad/m * curvature_to_can
  .curvature_error_min_speed = 10.0,  // m/s
  .max_steer_power = 0,               // disabled, Ford has no steed power signal
};

static int desired_path_angle_last = 0;

// path_angle 的 ROC + inactive 检查。刻意与 steer_angle_cmd_checks 分开：
// 后者共用 desired_angle_last，且会拿 angle_meas 做校验，而 Ford 从不填充 angle_meas
// （它只通过 curvature_state.meas 发布实测曲率），其 inactive 分支会跟一个恒零的测量比较。
static bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if (steer_control_enabled) {
    // 速度下调 1 m/s，使变化率窗口略宽于 openpilot 自身的软 ROC，
    // 防止两条角度指令之间插进来一个更新的车速
    const float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
    const int delta_path_angle_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    const int highest_desired_path_angle = desired_path_angle_last + delta_path_angle_roc;
    const int lowest_desired_path_angle = desired_path_angle_last - delta_path_angle_roc;

    violation |= safety_max_limit_check(desired_path_angle, highest_desired_path_angle, lowest_desired_path_angle);
  }

  // PSCM 未被要求转向时，path_angle 必须停在 inactive sentinel（0.0 rad），
  // 这样陈旧值绝不可能被一帧重新接合就恢复
  if (!steer_control_enabled) {
    violation |= (desired_path_angle != 0);
    desired_path_angle_last = 0;
  } else {
    desired_path_angle_last = desired_path_angle;
  }

  return violation;
}

static void ford_rx_hook(const CANPacket_t *msg) {
  // Update in motion state from standstill signal
  if (msg_matches(msg, FORD_DesiredTorqBrk, FORD_MAIN_BUS)) {
    // Signal: VehStop_D_Stat
    vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
  }

  // Update vehicle speed
  if (msg_matches(msg, FORD_BrakeSysFeatures, FORD_MAIN_BUS)) {
    // Signal: Veh_V_ActlBrk
    UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
  }

  // Check vehicle speed against a second source
  if (msg_matches(msg, FORD_EngVehicleSpThrottle2, FORD_MAIN_BUS)) {
    // Disable controls if speeds from ABS and PCM ECUs are too far apart.
    // Signal: Veh_V_ActlEng
    float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED_2(filtered_pcm_speed);
  }

  // Update vehicle yaw rate
  if (msg_matches(msg, FORD_Yaw_Data_FD1, FORD_MAIN_BUS)) {
    // FIXME: safety can receive yaw before new vehicle speed, it should recompute meas on either received
    // Signal: VehYaw_W_Actl
    // TODO: we should use the speed which results in the closest angle measurement to the desired angle
    float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
    float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
    // convert current curvature into units on CAN for comparison with desired curvature
    update_sample(&curvature_state.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
  }

  // Update gas pedal
  if (msg_matches(msg, FORD_EngVehicleSpThrottle, FORD_MAIN_BUS)) {
    // Pedal position: (0.1 * val) in percent
    // Signal: ApedPos_Pc_ActlArb
    gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
  }

  // Update brake pedal and cruise state
  if (msg_matches(msg, FORD_EngBrakeData, FORD_MAIN_BUS)) {
    // Signal: BpedDrvAppl_D_Actl
    brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

    // Signal: CcStat_D_Actl
    unsigned int cruise_state = msg->data[1] & 0x07U;
    bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);
    pcm_cruise_check(cruise_engaged);
    acc_main_on = (cruise_state == 3U) || cruise_engaged;
  }

  if (msg_matches(msg, FORD_Steering_Data_FD1, FORD_MAIN_BUS)) {
    mads_button_press = GET_BIT(msg, 40U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = (msg->data[6] & 0xC0U) != 0U;

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny;  // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    violation |= ((msg->data[3] >> 1) & 1U) && !controls_allowed;     // Signal: CcAsllButtnResPress (resume)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Do not allow steering using Lane_Assist_Data1 (Lane-Departure Aid).
    // This message must be sent for Lane Centering to work, and can include
    // values such as the steering angle or lane curvature for debugging,
    // but the action (LkaActvStats_D2_Req) must be set to zero.
    unsigned int action = msg->data[0] >> 5;
    if (action != 0U) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);

    // [CHX PATH_ANGLE] path_angle 已放行，不再要求等于 inactive sentinel；
    // curvature_rate 与 path_offset 仍按上游保持钉死（未做安全性验证）。
    bool violation = (raw_curvature_rate != FORD_INACTIVE_CURVATURE_RATE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

    // Check angle error and steer_control_enabled
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
    violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);

    // [CHX PATH_ANGLE] c1 的 ROC 与 inactive 校验
    int desired_path_angle = (int)raw_path_angle - (int)FORD_INACTIVE_PATH_ANGLE;
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];

    // [CHX PATH_ANGLE] 同 LateralMotionControl，path_angle 放行
    bool violation = (raw_curvature_rate != FORD_CANFD_INACTIVE_CURVATURE_RATE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

    // Check angle error and steer_control_enabled
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
    violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);

    // [CHX PATH_ANGLE] c1 的 ROC 与 inactive 校验
    int desired_path_angle = (int)raw_path_angle - (int)FORD_INACTIVE_PATH_ANGLE;
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, FORD_PATH_ANGLE_LIMITS);

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  static RxCheck ford_rx_checks[] = {
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    // FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug
    // Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC
    // It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}},
    // These messages have no counter or checksum
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

#ifdef ALLOW_DEBUG
  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };
#endif

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  safety_config ret;
  if (ford_canfd) {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
#ifdef ALLOW_DEBUG
    const uint16_t FORD_PARAM_LONGITUDINAL = 1;
    if (GET_FLAG(param, FORD_PARAM_LONGITUDINAL)) {
      ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS);
    }
#endif
  } else {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS);
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};
