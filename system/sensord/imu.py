from __future__ import annotations

import time

try:
  import smbus2
except ImportError:
  smbus2 = None

I2C_BUS_IMU = 1
LSM6DS3_I2C_ADDRESS = 0x6A
LSM6DS3_WHO_AM_I_REG = 0x0F
LSM6DS3_WHO_AM_I_IDS = {0x69, 0x6A}


def probe_lsm6ds3(attempts: int = 10, delay: float = 0.2) -> tuple[bool, dict[str, object]]:
  details: dict[str, object] = {
    "i2c_bus": I2C_BUS_IMU,
    "i2c_address": hex(LSM6DS3_I2C_ADDRESS),
    "who_am_i_reg": hex(LSM6DS3_WHO_AM_I_REG),
    "expected_chip_ids": [hex(chip_id) for chip_id in sorted(LSM6DS3_WHO_AM_I_IDS)],
    "attempts": attempts,
  }

  if smbus2 is None:
    details["error"] = "smbus2 is not installed"
    return False, details

  for attempt in range(max(attempts, 1)):
    bus = None
    try:
      bus = smbus2.SMBus(I2C_BUS_IMU)
      chip_id = bus.read_byte_data(LSM6DS3_I2C_ADDRESS, LSM6DS3_WHO_AM_I_REG)
      details["chip_id"] = hex(chip_id)
      details["attempt"] = attempt + 1
      return chip_id in LSM6DS3_WHO_AM_I_IDS, details
    except Exception as e:
      details["error"] = repr(e)
    finally:
      if bus is not None:
        try:
          bus.close()
        except Exception:
          pass

    if attempt + 1 < attempts:
      try:
        time.sleep(delay)
      except Exception:
        pass

  return False, details
