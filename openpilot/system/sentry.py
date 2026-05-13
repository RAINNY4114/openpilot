"""Install exception handler for process crash."""
import os
from datetime import datetime
from enum import Enum

import sentry_sdk
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params
from openpilot.system.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version


class SentryProject(Enum):
  # python project
  SELFDRIVE = "https://980a0cba712a4c3593c33c78a12446e1@o273754.ingest.sentry.io/1488600"
  # native project
  SELFDRIVE_NATIVE = "https://980a0cba712a4c3593c33c78a12446e1@o273754.ingest.sentry.io/1488600"


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()

def save_exception(exc_text):
  log = "\n".join(exc_text.splitlines()) + "\n"
  Params().put("dp_dev_last_log", log)

  crash_dir = "/data/community/crashes"
  timestamp = datetime.now()
  try:
    os.makedirs(crash_dir, exist_ok=True)

    error_log_path = os.path.join(crash_dir, "error.log")
    header = f"=== Error Log (Last Updated: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}) ===\n\n"
    with open(error_log_path, "w", encoding="utf-8") as f:
      f.write(header)
      f.write(log)

    history_path = os.path.join(crash_dir, f"error_{timestamp.strftime('%Y%m%d_%H%M%S')}.log")
    with open(history_path, "w", encoding="utf-8") as f:
      f.write(log)

    cloudlog.info(f"Error log saved to {error_log_path}")
  except Exception:
    cloudlog.exception("Failed to persist error log")

def capture_exception(*args, **kwargs) -> None:
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    import traceback
    save_exception(traceback.format_exc())
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()
  # forks like to mess with this, so double check
  comma_remote = build_metadata.openpilot.comma_remote and "commaai" in build_metadata.openpilot.git_origin
  if not comma_remote or not is_registered_device() or PC:
    return False

  env = "release" if build_metadata.tested_channel else "master"
  dongle_id = Params().get("DongleId")

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  traces_sample_rate=1.0,
                  max_value_length=8192,
                  environment=env)

  sentry_sdk.set_user({"id": dongle_id})
  sentry_sdk.set_tag("dirty", build_metadata.openpilot.is_dirty)
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", build_metadata.channel)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())

  return True
