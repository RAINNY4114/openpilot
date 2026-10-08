#!/usr/bin/env python3
import os
import time
import ctypes
import select
import threading

import cereal.messaging as messaging
from cereal.services import SERVICE_LIST
from openpilot.common.util import sudo_write
from openpilot.common.realtime import config_realtime_process, Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.common.gpio import gpiochip_get_ro_value_fd, gpioevent_data
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware_diagnostics import append_hardware_diagnostic

from openpilot.system.sensord.sensors.i2c_sensor import Sensor
from openpilot.system.sensord.sensors.lsm6ds3_accel import LSM6DS3_Accel
from openpilot.system.sensord.sensors.lsm6ds3_gyro import LSM6DS3_Gyro
from openpilot.system.sensord.sensors.lsm6ds3_temp import LSM6DS3_Temp
from openpilot.system.sensord.sensors.mmc5603nj_magn import MMC5603NJ_Magn
from openpilot.system.sensord.imu import I2C_BUS_IMU, probe_lsm6ds3


def _sensor_diag_details(sensor: Sensor | None, service: str = "", error: Exception | None = None) -> dict[str, object]:
  details: dict[str, object] = {
    "service": service,
    "i2c_bus": I2C_BUS_IMU,
  }
  if sensor is not None:
    details["sensor_class"] = sensor.__class__.__name__
    try:
      details["i2c_address"] = hex(sensor.device_address)
    except Exception:
      details["i2c_address"] = "unknown"
  if error is not None:
    details["error"] = repr(error)
  return details


def interrupt_loop(sensors: list[tuple[Sensor, str, bool]], event) -> None:
  pm = messaging.PubMaster([service for sensor, service, interrupt in sensors if interrupt])

  # Requesting both edges as the data ready pulse from the lsm6ds sensor is
  # very short (75us) and is mostly detected as falling edge instead of rising.
  # So if it is detected as rising the following falling edge is skipped.
  try:
    fd = gpiochip_get_ro_value_fd("sensord", 0, 84)
  except Exception as e:
    append_hardware_diagnostic("sensord", "imuInterruptGpioOpenFailed", {
      "gpiochip": 0,
      "gpio_line": 84,
      "error": repr(e),
      "likely_reason": "IMU data-ready interrupt GPIO is missing or not mapped on this hardware",
    }, min_interval_sec=0.)
    raise

  # Configure IRQ affinity
  irq_path = "/proc/irq/336/smp_affinity_list"
  if not os.path.exists(irq_path):
    irq_path = "/proc/irq/335/smp_affinity_list"
  if os.path.exists(irq_path):
    sudo_write('1\n', irq_path)

  offset = time.time_ns() - time.monotonic_ns()

  poller = select.poll()
  poller.register(fd, select.POLLIN | select.POLLPRI)
  poll_timeout_count = 0
  no_poll_event_count = 0
  while not event.is_set():
    events = poller.poll(100)
    if not events:
      poll_timeout_count += 1
      cloudlog.error("poll timed out")
      append_hardware_diagnostic("sensord", "imuInterruptPollTimeout", {
        "gpiochip": 0,
        "gpio_line": 84,
        "poll_timeout_count": poll_timeout_count,
        "likely_reason": "IMU data-ready IRQ is not firing; check LSM6DS3 INT pin, GPIO mapping, power, and sensor variant",
      }, min_interval_sec=60.)
      continue
    if not (events[0][1] & (select.POLLIN | select.POLLPRI)):
      no_poll_event_count += 1
      cloudlog.error("no poll events set")
      append_hardware_diagnostic("sensord", "imuInterruptPollEventInvalid", {
        "gpiochip": 0,
        "gpio_line": 84,
        "poll_event": int(events[0][1]),
        "no_poll_event_count": no_poll_event_count,
      }, min_interval_sec=60.)
      continue

    dat = os.read(fd, ctypes.sizeof(gpioevent_data)*16)
    evd = gpioevent_data.from_buffer_copy(dat)

    cur_offset = time.time_ns() - time.monotonic_ns()
    if abs(cur_offset - offset) > 10 * 1e6:  # ms
      cloudlog.warning(f"time jumped: {cur_offset} {offset}")
      offset = cur_offset
      continue

    ts = evd.timestamp - cur_offset
    for sensor, service, interrupt in sensors:
      if interrupt:
        try:
          evt = sensor.get_event(ts)
          if not sensor.is_data_valid():
            continue
          msg = messaging.new_message(service, valid=True)
          setattr(msg, service, evt)
          pm.send(service, msg)
        except Sensor.DataNotReady:
          pass
        except Exception as e:
          append_hardware_diagnostic(
            "sensord", "imuEventProcessingFailed", _sensor_diag_details(sensor, service, e),
            dedupe_key=f"sensord:imuEventProcessingFailed:{service}", min_interval_sec=60.,
          )
          cloudlog.exception(f"Error processing {service}")


def polling_loop(sensor: Sensor, service: str, event: threading.Event) -> None:
  pm = messaging.PubMaster([service])
  rk = Ratekeeper(SERVICE_LIST[service].frequency, print_delay_threshold=None)
  while not event.is_set():
    try:
      evt = sensor.get_event()
      if not sensor.is_data_valid():
        continue
      msg = messaging.new_message(service, valid=True)
      setattr(msg, service, evt)
      pm.send(service, msg)
    except Exception as e:
      append_hardware_diagnostic(
        "sensord", "sensorPollingLoopFailed", _sensor_diag_details(sensor, service, e),
        dedupe_key=f"sensord:sensorPollingLoopFailed:{service}", min_interval_sec=60.,
      )
      cloudlog.exception(f"Error in {service} polling loop")
    rk.keep_time()

def main() -> None:
  config_realtime_process([1, ], 1)

  lsm6ds3_present, lsm6ds3_details = probe_lsm6ds3()
  if not lsm6ds3_present:
    cloudlog.warning(f"sensord: no LSM6DS3 IMU detected, running without IMU sensors: {lsm6ds3_details}")
    try:
      while True:
        time.sleep(60)
    except KeyboardInterrupt:
      pass
    return

  try:
    sensors_cfg = [
      (LSM6DS3_Accel(I2C_BUS_IMU), "accelerometer", True),
      (LSM6DS3_Gyro(I2C_BUS_IMU), "gyroscope", True),
      (LSM6DS3_Temp(I2C_BUS_IMU), "temperatureSensor", False),
    ]
  except Exception as e:
    append_hardware_diagnostic("sensord", "sensorConstructionFailed", {
      "i2c_bus": I2C_BUS_IMU,
      "error": repr(e),
      "likely_reason": "I2C bus cannot be opened or expected IMU hardware is missing",
    }, min_interval_sec=0.)
    raise
  if HARDWARE.get_device_type() == "tizi":
    try:
      sensors_cfg.append(
        (MMC5603NJ_Magn(I2C_BUS_IMU), "magnetometer", False),
      )
    except Exception as e:
      append_hardware_diagnostic("sensord", "magnetometerConstructionFailed", {
        "i2c_bus": I2C_BUS_IMU,
        "error": repr(e),
        "likely_reason": "Magnetometer I2C device is missing or not responding",
      }, min_interval_sec=0.)
      raise

  # Reset sensors
  for sensor, _, _ in sensors_cfg:
    try:
      sensor.reset()
    except Exception as e:
      append_hardware_diagnostic(
        "sensord", "sensorResetFailed", _sensor_diag_details(sensor, error=e),
        dedupe_key=f"sensord:sensorResetFailed:{sensor.__class__.__name__}", min_interval_sec=60.,
      )
      cloudlog.exception(f"Error initializing {sensor} sensor")

  # Initialize sensors
  exit_event = threading.Event()
  threads = [
    threading.Thread(target=interrupt_loop, args=(sensors_cfg, exit_event), daemon=True)
  ]
  for sensor, service, interrupt in sensors_cfg:
    try:
      sensor.init()
      if not interrupt:
        # Start polling thread for sensors without interrupts
        threads.append(threading.Thread(
          target=polling_loop,
          args=(sensor, service, exit_event),
          daemon=True
        ))
    except Exception as e:
      details = _sensor_diag_details(sensor, service, e)
      details["uses_interrupt"] = bool(interrupt)
      details["likely_reason"] = "Sensor chip ID, I2C address, or register setup failed"
      append_hardware_diagnostic("sensord", "sensorInitFailed", details, dedupe_key=f"sensord:sensorInitFailed:{service}", min_interval_sec=60.)
      cloudlog.exception(f"Error initializing {service} sensor")

  try:
    for t in threads:
      t.start()
    while any(t.is_alive() for t in threads):
      time.sleep(1)
  except KeyboardInterrupt:
    pass
  finally:
    exit_event.set()
    for t in threads:
      if t.is_alive():
        t.join()

    for sensor, _, _ in sensors_cfg:
      try:
        sensor.shutdown()
      except Exception as e:
        append_hardware_diagnostic(
          "sensord", "sensorShutdownFailed", _sensor_diag_details(sensor, error=e),
          dedupe_key=f"sensord:sensorShutdownFailed:{sensor.__class__.__name__}", min_interval_sec=60.,
        )
        cloudlog.exception("Error shutting down sensor")

if __name__ == "__main__":
  main()
