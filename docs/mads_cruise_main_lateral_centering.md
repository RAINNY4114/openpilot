# 巡航主开关可用 -> 自动横向居中（MADS）复刻文档

本文件是完整实现指南，目标是让其他分支直接复刻该功能，不需要对比任何分支。

---

## 目标行为
- 当巡航主开关变为可用（cruiseState.available 从 0->1），横向立即居中。
- 纵向仍保持原逻辑：必须按 SET/RES（+/-）才启用速度控制。

---

## 必须改动清单（文件级）
1) common/params_keys.h
2) cereal/custom.capnp
3) cereal/log.capnp
4) cereal/services.py + 重新生成 cereal/services.h
5) selfdrive/selfdrived/events.py
6) selfdrive/selfdrived/selfdrived.py
7) selfdrive/controls/controlsd.py
8) selfdrive/car/card.py
9) openpilot/sunnypilot/*（运行时模块）

---

## 1) 参数默认值
文件：common/params_keys.h
新增（保持 PERSISTENT，禁止 BACKUP）：
```
    {"Mads", {PERSISTENT, BOOL, "1"}},
    {"MadsMainCruiseAllowed", {PERSISTENT, BOOL, "1"}},
    {"MadsSteeringMode", {PERSISTENT, INT, "0"}},
    {"MadsUnifiedEngagementMode", {PERSISTENT, BOOL, "1"}},
```

---

## 2) cereal/custom.capnp
新增结构体（直接复制粘贴）：
```
struct ModularAssistiveDrivingSystem {
  state @0 :ModularAssistiveDrivingSystemState;
  enabled @1 :Bool;
  active @2 :Bool;
  available @3 :Bool;

  enum ModularAssistiveDrivingSystemState {
    disabled @0;
    paused @1;
    enabled @2;
    softDisabling @3;
    overriding @4;
  }
}

struct CarStateExt @0xd9ed2c19ed77ad91 {

  lkasOn @0 :Bool;

}



struct ModelExt @0xaedffd8f31e7b55d {

  leftEdgeDetected @0 :Bool;

  rightEdgeDetected @1 :Bool;

}



struct ModelManagerSP @0xf35cc4560bbf6ec2 {
  activeBundle @0 :ModelBundle;
  selectedBundle @1 :ModelBundle;
  availableBundles @2 :List(ModelBundle);

  struct DownloadUri {
    uri @0 :Text;
    sha256 @1 :Text;
  }

  enum DownloadStatus {
    notDownloading @0;
    downloading @1;
    downloaded @2;
    cached @3;
    failed @4;
  }

  struct DownloadProgress {
    status @0 :DownloadStatus;
    progress @1 :Float32;
    eta @2 :UInt32;
  }

  struct Artifact {
    fileName @0 :Text;
    downloadUri @1 :DownloadUri;
    downloadProgress @2 :DownloadProgress;
  }

  struct Model {
    type @0 :Type;
    artifact @1 :Artifact;
    metadata @2 :Artifact;

    enum Type {
      supercombo @0;
      navigation @1;
      vision @2;
      policy @3;
    }
  }

  enum Runner {
    snpe @0;
    tinygrad @1;
    stock @2;
  }

  struct Override {
    key @0 :Text;
    value @1 :Text;
  }

  struct ModelBundle {
    index @0 :UInt32;
    internalName @1 :Text;
    displayName @2 :Text;
    models @3 :List(Model);
    status @4 :DownloadStatus;
    generation @5 :UInt32;
    environment @6 :Text;
    runner @7 :Runner;
    is20hz @8 :Bool;
    ref @9 :Text;
    minimumSelectorVersion @10 :UInt32;
    overrides @11 :List(Override);
  }
}


struct OnroadEventSP @0xda96579883444c35 {
  events @0 :List(Event);

  struct Event {
    name @0 :EventName;

    # event types
    enable @1 :Bool;
    noEntry @2 :Bool;
    warning @3 :Bool;   # alerts presented only when enabled or soft disabling
    userDisable @4 :Bool;
    softDisable @5 :Bool;
    immediateDisable @6 :Bool;
    preEnable @7 :Bool;
    permanent @8 :Bool; # alerts presented regardless of openpilot state
    overrideLateral @10 :Bool;
    overrideLongitudinal @9 :Bool;
  }

  enum EventName {
    lkasEnable @0;
    lkasDisable @1;
    manualSteeringRequired @2;
    manualLongitudinalRequired @3;
    silentLkasEnable @4;
    silentLkasDisable @5;
    silentBrakeHold @6;
    silentWrongGear @7;
    silentReverseGear @8;
    silentDoorOpen @9;
    silentSeatbeltNotLatched @10;
    silentParkBrake @11;
    wrongCarModeAlertOnly @12;
    pedalPressedAlertOnly @13;
  }
}


struct SelfdriveStateSP @0x80ae746ee2596b11 {
  mads @0 :ModularAssistiveDrivingSystem;
}
```

---

## 3) cereal/log.capnp
新增消息映射：
```
    onroadEventsSP @110 :Custom.OnroadEventSP;
    selfdriveStateSP @111 :Custom.SelfdriveStateSP;
```

---

## 4) cereal/services.py / services.h
在 services.py 中加入：
```
  "selfdriveStateSP": (True, 100., 10),
  "onroadEventsSP": (True, 1., 1),
```

重新生成 services.h（避免 UTF-16 BOM）：
1) 新建临时脚本 tools/gen_services_h.py：
```python
import io, runpy, contextlib
from pathlib import Path
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    runpy.run_path("cereal/services.py", run_name="__main__")
Path("cereal/services.h").write_text(buf.getvalue(), encoding="utf-8", newline="\n")
```
2) 执行：
```
python tools/gen_services_h.py
```
3) 脚本可删除

---

## 5) openpilot/sunnypilot 运行时模块（必须存在）
这些文件用于设备端 import，避免 ModuleNotFoundError。

### openpilot/sunnypilot/mads/helpers.py
```python
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from opendbc.car import structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE


MADS_NO_ACC_MAIN_BUTTON = ("rivian", "tesla")


class MadsSteeringModeOnBrake:
  REMAIN_ACTIVE = 0
  PAUSE = 1
  DISENGAGE = 2


def read_steering_mode_param(params: Params):
  return params.get("MadsSteeringMode", return_default=True)


def set_alternative_experience(CP: structs.CarParams, params: Params):
  if params.get_bool("Mads"):
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALKA

```

### openpilot/sunnypilot/mads/state.py
```python
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import log, custom
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.state import SOFT_DISABLE_TIME
from openpilot.common.realtime import DT_CTRL

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName

ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.paused, *ACTIVE_STATES)

GEARS_ALLOW_PAUSED_SILENT = [EventNameSP.silentWrongGear, EventNameSP.silentReverseGear, EventNameSP.silentBrakeHold,
                             EventNameSP.silentDoorOpen, EventNameSP.silentSeatbeltNotLatched, EventNameSP.silentParkBrake]
GEARS_ALLOW_PAUSED = [EventName.wrongGear, EventName.reverseGear, EventName.brakeHold,
                      EventName.doorOpen, EventName.seatbeltNotLatched, EventName.parkBrake]


class StateMachine:
  def __init__(self, mads):
    self.selfdrive = mads.selfdrive
    self.ss_state_machine = mads.selfdrive.state_machine
    self._events = mads.selfdrive.events
    self._events_sp = mads.selfdrive.events_sp

    self.state = State.disabled

  def add_current_alert_types(self, alert_type):
    if not self.selfdrive.enabled:
      self.ss_state_machine.current_alert_types.append(alert_type)

  def check_contains(self, event_type: str) -> bool:
    return bool(self._events.contains(event_type) or self._events_sp.contains(event_type))

  def check_contains_in_list(self) -> bool:
    return bool(self._events.contains_in_list(GEARS_ALLOW_PAUSED) or self._events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT))

  def update(self):
    # soft disable timer and current alert types are from the state machine of openpilot
    # decrement the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state

    # ENABLED, SOFT DISABLING, PAUSED, OVERRIDING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.check_contains(ET.USER_DISABLE):
        if self._events_sp.has(EventNameSP.silentLkasDisable):
          self.state = State.paused
        else:
          self.state = State.disabled
        self.ss_state_machine.current_alert_types.append(ET.USER_DISABLE)

      elif self.check_contains(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.add_current_alert_types(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.check_contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            if not self.selfdrive.enabled:
              self.ss_state_machine.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
              self.ss_state_machine.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.check_contains(ET.OVERRIDE_LATERAL):
            self.state = State.overriding
            self.add_current_alert_types(ET.OVERRIDE_LATERAL)

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.check_contains(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.ss_state_machine.soft_disable_timer > 0:
            self.add_current_alert_types(ET.SOFT_DISABLE)

          elif self.ss_state_machine.soft_disable_timer <= 0:
            self.state = State.disabled

        # PAUSED
        elif self.state == State.paused:
          if self.check_contains(ET.ENABLE):
            if self.check_contains(ET.NO_ENTRY):
              self.add_current_alert_types(ET.NO_ENTRY)

            else:
              if self.check_contains(ET.OVERRIDE_LATERAL):
                self.state = State.overriding
              else:
                self.state = State.enabled
              self.add_current_alert_types(ET.ENABLE)

        # OVERRIDING
        elif self.state == State.overriding:
          if self.check_contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            if not self.selfdrive.enabled:
              self.ss_state_machine.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
              self.ss_state_machine.current_alert_types.append(ET.SOFT_DISABLE)
          elif not self.check_contains(ET.OVERRIDE_LATERAL):
            self.state = State.enabled
          else:
            self.ss_state_machine.current_alert_types += [ET.OVERRIDE_LATERAL]

    # DISABLED
    elif self.state == State.disabled:
      if self.check_contains(ET.ENABLE):
        if self.check_contains(ET.NO_ENTRY):
          if self.check_contains_in_list():
            self.state = State.paused
          self.add_current_alert_types(ET.NO_ENTRY)

        else:
          if self.check_contains(ET.OVERRIDE_LATERAL):
            self.state = State.overriding
          else:
            self.state = State.enabled
          self.add_current_alert_types(ET.ENABLE)

    # check if MADS is engaged and actuators are enabled
    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES
    if active:
      self.add_current_alert_types(ET.WARNING)

    return enabled, active

```

### openpilot/sunnypilot/mads/mads.py
```python
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import log, custom

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.common.params import Params
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, read_steering_mode_param, MADS_NO_ACC_MAIN_BUTTON
from openpilot.sunnypilot.mads.state import StateMachine, GEARS_ALLOW_PAUSED_SILENT

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)


class ModularAssistiveDrivingSystem:
  def __init__(self, selfdrive):
    self.CP = selfdrive.CP
    self.params = selfdrive.params

    self.enabled = False
    self.active = False
    self.available = False
    self.allow_always = False
    self.no_main_cruise = False
    self.selfdrive = selfdrive
    self.selfdrive.enabled_prev = False
    self.state_machine = StateMachine(self)
    self.events = self.selfdrive.events
    self.events_sp = self.selfdrive.events_sp
    self.disengage_on_accelerator = Params().get_bool("DisengageOnAccelerator")
    if self.CP.brand == "hyundai":
      if self.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True
    if self.CP.brand == "tesla":
      self.allow_always = True

    if self.CP.brand in MADS_NO_ACC_MAIN_BUTTON:
      self.no_main_cruise = True

    # read params on init
    self.enabled_toggle = self.params.get_bool("Mads")
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.steering_mode_on_brake = read_steering_mode_param(self.params)
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def read_params(self):
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def pedal_pressed_non_gas_pressed(self, CS: structs.CarState) -> bool:
    # ignore `pedalPressed` events caused by gas presses
    if self.events.has(EventName.pedalPressed) and not (CS.gasPressed and not self.selfdrive.CS_prev.gasPressed and self.disengage_on_accelerator):
      return True

    return False

  def should_silent_lkas_enable(self, CS: structs.CarState) -> bool:
    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE and self.pedal_pressed_non_gas_pressed(CS):
      return False

    if self.events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT):
      return False

    return True

  def block_unified_engagement_mode(self) -> bool:
    # UEM disabled
    if not self.unified_engagement_mode:
      return True

    if self.enabled:
      return True

    if self.selfdrive.enabled and self.selfdrive.enabled_prev:
      return True

    return False

  def get_wrong_car_mode(self, alert_only: bool) -> None:
    if alert_only:
      if self.events.has(EventName.wrongCarMode):
        self.replace_event(EventName.wrongCarMode, EventNameSP.wrongCarModeAlertOnly)
    else:
      self.events.remove(EventName.wrongCarMode)

  def transition_paused_state(self):
    if self.state_machine.state != State.paused:
      self.events_sp.add(EventNameSP.silentLkasDisable)

  def replace_event(self, old_event: int, new_event: int):
    self.events.remove(old_event)
    self.events_sp.add(new_event)

  def update_events(self, CS: structs.CarState):
    if not self.selfdrive.enabled and self.enabled:
      if CS.standstill:
        if self.events.has(EventName.doorOpen):
          self.replace_event(EventName.doorOpen, EventNameSP.silentDoorOpen)
          self.transition_paused_state()
        if self.events.has(EventName.seatbeltNotLatched):
          self.replace_event(EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched)
          self.transition_paused_state()
      if self.events.has(EventName.wrongGear) and (CS.vEgo < 2.5 or CS.gearShifter == GearShifter.reverse):
        self.replace_event(EventName.wrongGear, EventNameSP.silentWrongGear)
        self.transition_paused_state()
      if self.events.has(EventName.reverseGear):
        self.replace_event(EventName.reverseGear, EventNameSP.silentReverseGear)
        self.transition_paused_state()
      if self.events.has(EventName.brakeHold):
        self.replace_event(EventName.brakeHold, EventNameSP.silentBrakeHold)
        self.transition_paused_state()
      if self.events.has(EventName.parkBrake):
        self.replace_event(EventName.parkBrake, EventNameSP.silentParkBrake)
        self.transition_paused_state()

      if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
        if self.pedal_pressed_non_gas_pressed(CS):
          self.transition_paused_state()

      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)

    selfdrive_enable_events = self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable)
    set_speed_btns_enable = any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents)

    # wrongCarMode alert only or actively block control
    self.get_wrong_car_mode(selfdrive_enable_events or set_speed_btns_enable)

    if selfdrive_enable_events:
      if self.pedal_pressed_non_gas_pressed(CS):
        self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

      if self.block_unified_engagement_mode():
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)
    else:
      if self.main_enabled_toggle:
        if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
          self.events_sp.add(EventNameSP.lkasEnable)

    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel:
        if not self.selfdrive.enabled and self.selfdrive.enabled_prev:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
      if be.type == ButtonType.lkas and be.pressed and (CS.cruiseState.available or self.allow_always):
        if self.enabled:
          if self.selfdrive.enabled:
            self.events_sp.add(EventNameSP.manualSteeringRequired)
          else:
            self.events_sp.add(EventNameSP.lkasDisable)
        else:
          self.events_sp.add(EventNameSP.lkasEnable)

    if not CS.cruiseState.available and not self.no_main_cruise:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available:
        self.events_sp.add(EventNameSP.lkasDisable)

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE:
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # block lkasEnable if being sent, then send pedalPressedAlertOnly event
          if self.events_sp.contains(EventNameSP.lkasEnable):
            self.events_sp.remove(EventNameSP.lkasEnable)
            self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

    if self.should_silent_lkas_enable(CS):
      if self.state_machine.state == State.paused:
        self.events_sp.add(EventNameSP.silentLkasEnable)

    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)

  def update(self, CS: structs.CarState):
    if not self.enabled_toggle:
      return

    self.update_events(CS)

    if not self.CP.passive and self.selfdrive.initialized:
      self.enabled, self.active = self.state_machine.update()

    # Copy of previous SelfdriveD states for MADS events handling
    self.selfdrive.enabled_prev = self.selfdrive.enabled

```

### openpilot/sunnypilot/selfdrive/selfdrived/events_base.py
```python
import bisect
from enum import IntEnum
from abc import abstractmethod
from collections.abc import Callable

from cereal import log, car
import cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.system.hardware import HARDWARE

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert


# Alert priorities
class Priority(IntEnum):
  LOWEST = 0
  LOWER = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5


# Event types
class ET:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  OVERRIDE_LATERAL = 'overrideLateral'
  OVERRIDE_LONGITUDINAL = 'overrideLongitudinal'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


class Alert:
  def __init__(self,
               alert_text_1: str,
               alert_text_2: str,
               alert_status: log.SelfdriveState.AlertStatus,
               alert_size: log.SelfdriveState.AlertSize,
               priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert,
               duration: float,
               creation_delay: float = 0.):

    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.priority = priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration = int(duration / DT_CTRL)

    self.creation_delay = creation_delay

    self.alert_type = ""
    self.event_type: str | None = None

  def __str__(self) -> str:
    return f"{self.alert_text_1}/{self.alert_text_2} {self.priority} {self.visual_alert} {self.audible_alert}"

  def __gt__(self, alert2) -> bool:
    if not isinstance(alert2, Alert):
      return False
    return self.priority > alert2.priority

class AlertBase(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str, alert_status: log.SelfdriveState.AlertStatus,
               alert_size: log.SelfdriveState.AlertSize, priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert, duration: float):
    super().__init__(alert_text_1, alert_text_2, alert_status, alert_size, priority, visual_alert, audible_alert, duration)


AlertCallbackType = Callable[[car.CarParams, car.CarState, messaging.SubMaster, bool, int, log.ControlsState], Alert]


# ********** alert callback functions **********


def wrong_car_mode_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  text = "Enable Adaptive Cruise to Engage"
  if CP.brand == "honda":
    text = "Enable Main Switch to Engage"
  return NoEntryAlert(text)


class EventsBase:
  def __init__(self):
    self.events: list[int] = []
    self.static_events: list[int] = []
    self.event_counters = {}

  @property
  def names(self) -> list[int]:
    return self.events

  def __len__(self) -> int:
    return len(self.events)

  def add(self, event_name: int, static: bool = False) -> None:
    if static:
      bisect.insort(self.static_events, event_name)
    bisect.insort(self.events, event_name)

  def clear(self) -> None:
    self.event_counters = {k: (v + 1 if k in self.events else 0) for k, v in self.event_counters.items()}
    self.events = self.static_events.copy()

  def contains(self, event_type: str) -> bool:
    return any(event_type in self.get_events_mapping().get(e, {}) for e in self.events)

  def create_alerts(self, event_types: list[str], callback_args=None):
    if callback_args is None:
      callback_args = []

    ret = []
    for e in self.events:
      types = self.get_events_mapping()[e].keys()
      for et in event_types:
        if et in types:
          alert = self.get_events_mapping()[e][et]
          if not isinstance(alert, Alert):
            alert = alert(*callback_args)

          if DT_CTRL * (self.event_counters[e] + 1) >= alert.creation_delay:
            alert.alert_type = f"{self.get_event_name(e)}/{et}"
            alert.event_type = et
            ret.append(alert)
    return ret

  def add_from_msg(self, events):
    for e in events:
      bisect.insort(self.events, e.name.raw)

  def to_msg(self):
    ret = []
    for event_name in self.events:
      event = self.get_event_msg_type().new_message()
      event.name = event_name
      for event_type in self.get_events_mapping().get(event_name, {}):
        setattr(event, event_type, True)
      ret.append(event)
    return ret

  def has(self, event_name: int) -> bool:
    return event_name in self.events

  def contains_in_list(self, events_list: list[int]) -> bool:
    return any(event_name in self.events for event_name in events_list)

  def remove(self, event_name: int, static: bool = False) -> None:
    if static and event_name in self.static_events:
      self.static_events.remove(event_name)

    if event_name in self.events:
      self.event_counters[event_name] = self.event_counters[event_name] + 1
      self.events.remove(event_name)

  @abstractmethod
  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    raise NotImplementedError

  @abstractmethod
  def get_event_name(self, event: int) -> str:
    raise NotImplementedError

  @abstractmethod
  def get_event_msg_type(self):
    raise NotImplementedError


EmptyAlert = Alert("" , "", AlertStatus.normal, AlertSize.none, Priority.LOWEST,
                   VisualAlert.none, AudibleAlert.none, 0)

class NoEntryAlert(Alert):
  def __init__(self, alert_text_2: str,
               alert_text_1: str = "openpilot Unavailable",
               visual_alert: car.CarControl.HUDControl.VisualAlert=VisualAlert.none):
    if HARDWARE.get_device_type() == 'mici':
      alert_text_1, alert_text_2 = alert_text_2, alert_text_1
    super().__init__(alert_text_1, alert_text_2, AlertStatus.normal,
                     AlertSize.mid, Priority.LOW, visual_alert,
                     AudibleAlert.refuse, 3.)


class SoftDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__("TAKE CONTROL IMMEDIATELY", alert_text_2,
                     AlertStatus.userPrompt, AlertSize.full,
                     Priority.MID, VisualAlert.steerRequired,
                     AudibleAlert.warningSoft, 2.),


# less harsh version of SoftDisable, where the condition is user-triggered
class UserSoftDisableAlert(SoftDisableAlert):
  def __init__(self, alert_text_2: str):
    super().__init__(alert_text_2),
    self.alert_text_1 = "openpilot will disengage"


class ImmediateDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__("TAKE CONTROL IMMEDIATELY", alert_text_2,
                     AlertStatus.critical, AlertSize.full,
                     Priority.HIGHEST, VisualAlert.steerRequired,
                     AudibleAlert.warningImmediate, 4.),


class EngagementAlert(Alert):
  def __init__(self, audible_alert: car.CarControl.HUDControl.AudibleAlert):
    super().__init__("", "",
                     AlertStatus.normal, AlertSize.none,
                     Priority.MID, VisualAlert.none,
                     audible_alert, .2),


class NormalPermanentAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "", duration: float = 0.2, priority: Priority = Priority.LOWER, creation_delay: float = 0.):
    super().__init__(alert_text_1, alert_text_2,
                     AlertStatus.normal, AlertSize.mid if len(alert_text_2) else AlertSize.small,
                     priority, VisualAlert.none, AudibleAlert.none, duration, creation_delay=creation_delay),


class StartupAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "Always keep hands on wheel and eyes on road", alert_status=AlertStatus.normal):
    alert_size = AlertSize.mid
    if HARDWARE.get_device_type() == 'mici':
      if alert_text_2 == "Always keep hands on wheel and eyes on road":
        alert_text_2 = ""
      alert_size = AlertSize.small
    super().__init__(alert_text_1, alert_text_2,
                     alert_status, alert_size,
                     Priority.LOWER, VisualAlert.none, AudibleAlert.none, 5.),

```

### openpilot/sunnypilot/selfdrive/selfdrived/events.py
```python
import cereal.messaging as messaging
from cereal import log, car, custom
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, \
  NoEntryAlert, EngagementAlert, AlertCallbackType, wrong_car_mode_alert


AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName


# get event name from enum
EVENT_NAME_SP = {v: k for k, v in EventNameSP.schema.enumerants.items()}


class EventsSP(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_SP.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_SP

  def get_event_name(self, event: int):
    return EVENT_NAME_SP[event]

  def get_event_msg_type(self):
    return custom.OnroadEventSP.Event


EVENTS_SP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  EventNameSP.lkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventNameSP.lkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventNameSP.manualSteeringRequired: {
    ET.USER_DISABLE: Alert(
      "Automatic Lane Centering is OFF",
      "Manual Steering Required",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.disengage, 1.),
  },

  EventNameSP.manualLongitudinalRequired: {
    ET.WARNING: Alert(
      "Smart/Adaptive Cruise Control: OFF",
      "Manual Speed Control Required",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.silentLkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentLkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentBrakeHold: {
    ET.WARNING: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Active"),
  },

  EventNameSP.silentWrongGear: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: Alert(
      "Gear not D",
      "openpilot Unavailable",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0.),
  },

  EventNameSP.silentReverseGear: {
    ET.PERMANENT: Alert(
      "Reverse\nGear",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.NO_ENTRY: NoEntryAlert("Reverse Gear"),
  },

  EventNameSP.silentDoorOpen: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Door Open"),
  },

  EventNameSP.silentSeatbeltNotLatched: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Seatbelt Unlatched"),
  },

  EventNameSP.silentParkBrake: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Parking Brake Engaged"),
  },

  EventNameSP.wrongCarModeAlertOnly: {
    ET.WARNING: wrong_car_mode_alert,
  },

  EventNameSP.pedalPressedAlertOnly: {
    ET.WARNING: NoEntryAlert("Pedal Pressed")
  },
}

```

---

## 6) selfdrive/selfdrived/events.py
为 Events 增加方法（插入到 class Events 内）：
```python
  def has(self, event_name: int) -> bool:
    return event_name in self.events

  def contains_in_list(self, events_list: list[int]) -> bool:
    return any(event_name in self.events for event_name in events_list)

  def remove(self, event_name: int, static: bool = False) -> None:
    if static and event_name in self.static_events:
      self.static_events.remove(event_name)

    if event_name in self.events:
      self.event_counters[event_name] = self.event_counters.get(event_name, 0) + 1
      self.events.remove(event_name)
```

---

## 7) selfdrive/selfdrived/selfdrived.py
### 关键 imports（加入）：
```
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
    self.pm = messaging.PubMaster(['selfdriveState', 'onroadEvents', 'selfdriveStateSP', 'onroadEventsSP'])
    self.events_sp = EventsSP()
    # onroadEventsSP - logged every second or on change
      ce_send_sp = messaging.new_message('onroadEventsSP')
      ce_send_sp.onroadEventsSP.events = self.events_sp.to_msg()
      self.pm.send('onroadEventsSP', ce_send_sp)
```

### 初始化（加入）：
```
    self.events_sp = EventsSP()
    self.events_sp_prev = []
    self.mads = ModularAssistiveDrivingSystem(self)
    self.events_sp.clear()
    alerts_sp = self.events_sp.create_alerts(self.state_machine.current_alert_types, callback_args)
    mads.state = self.mads.state_machine.state
    mads.enabled = self.mads.enabled
    mads.active = self.mads.active
    mads.available = self.mads.enabled_toggle
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events_sp.names != self.events_sp_prev):
      ce_send_sp.onroadEventsSP.events = self.events_sp.to_msg()
    self.events_sp_prev = self.events_sp.names.copy()
      self.mads.update(CS)
      self.mads.read_params()
```

### update_events() 中清理 SP 事件：
```
    self.events_sp.clear()
```

### update_alerts() 合并 SP alerts：
```
    alerts_sp = self.events_sp.create_alerts(self.state_machine.current_alert_types, callback_args)
    self.AM.add_many(self.sm.frame, alerts + alerts_sp)
```

### publish_selfdriveState() 发送 SP 消息：
```
# selfdriveStateSP
    ss_sp_msg = messaging.new_message('selfdriveStateSP')
    ss_sp_msg.valid = True
    ss_sp = ss_sp_msg.selfdriveStateSP
    mads = ss_sp.mads
    mads.state = self.mads.state_machine.state
    mads.enabled = self.mads.enabled
    mads.active = self.mads.active
    mads.available = self.mads.enabled_toggle

    self.pm.send('selfdriveStateSP', ss_sp_msg)

    # onroadEventsSP - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events_sp.names != self.events_sp_prev):
      ce_send_sp = messaging.new_message('onroadEventsSP')
      ce_send_sp.valid = True
      ce_send_sp.onroadEventsSP.events = self.events_sp.to_msg()
      self.pm.send('onroadEventsSP', ce_send_sp)
    self.events_sp_prev =
```

### step() 中运行 MADS：
```
      self.mads.update(CS)
```

### params 线程读取 MADS 参数：
```
      self.mads.read_params()
```

---

## 8) selfdrive/controls/controlsd.py
### 订阅 + 横向激活逻辑：
```
                                   'selfdriveStateSP',
    self.mads_active = False
    ss_sp = self.sm['selfdriveStateSP']
    mads_available = bool(ss_sp.mads.available)
    mads_active = bool(ss_sp.mads.active) if mads_available else False
    self.mads_active = mads_active
    lat_active = mads_active if mads_available else (self.sm['selfdriveState'].active or self.alka_active)
    htd_allowed, self.htd_state = self.htd.update(lat_active, CS.steeringAngleDeg, CS.steeringTorque, CS.vEgo)
    lat_active = lat_active and htd_allowed
      lat_active
    ncs.alkaActive = self.mads_active or self.alka_active
```

---

## 9) selfdrive/car/card.py
### 允许 ALKA（MADS 打开即可）：
```
      if self.params.get_bool("dp_lat_alka"):
        dp_params |= structs.DPFlags.LatALKA
    self.CP.alternativeExperience = 0
    # dp - ALKA/MADS: allow lateral control with ACC main when enabled
    mads_enabled = self.params.get_bool("Mads")
    if (dp_params & structs.DPFlags.LatALKA) or mads_enabled:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALKA
    # dp - ALKA: publish lkas_on state from carstate
```

---

## 验证步骤
1) 只打开巡航主开关（不按 SET/RES）
   - 预期：横向居中立即工作
2) 按 SET/RES
   - 预期：纵向速度控制正常
3) 关闭 Mads 参数
   - 预期：回到旧逻辑（必须 SET/RES 才有横向）

---

## 常见错误与规避
- services.h 被写成 UTF-16 BOM -> 编译报错 “UTF-16 (LE) byte order mark detected”
  - 必须使用 UTF-8 无 BOM 重新生成。

- openpilot.sunnypilot 模块缺失 -> selfdrived 报 ModuleNotFoundError
  - 必须包含第 5) 的最小运行模块。

- 使用 BACKUP 参数 -> 嵌入端编译失败
  - 参数必须为 PERSISTENT。

- NameError: mads_active is not defined
  - 在 controlsd 中保存 self.mads_active，并在 publish() 使用 self.mads_active。

---

## 回退方案（最小）
- 删除 openpilot/sunnypilot 运行时模块
- 移除 selfdrived/controlsd/card 中 MADS 逻辑
- 回滚 capnp/services 改动
- 删除 Mads* 参数
