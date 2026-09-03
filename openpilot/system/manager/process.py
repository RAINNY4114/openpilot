import importlib
import os
import signal
import time
import subprocess
from collections.abc import Callable, ValuesView
from abc import ABC, abstractmethod
from multiprocessing import Process

from setproctitle import setproctitle

from openpilot.cereal import log
from opendbc.car.structs import car
import openpilot.cereal.messaging as messaging
import openpilot.system.sentry as sentry
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


# ============================================================================
# Process restart settings
# ============================================================================

# Do not restart a process immediately if it repeatedly crashes.
# This prevents a broken process from creating a very fast restart loop.
RESTART_MIN_INTERVAL = 2.0

# If a process exits very shortly after being started, wait before restarting.
RESTART_BACKOFF_INTERVAL = 5.0


def launcher(proc: str, name: str) -> None:
  try:
    # Import the process module.
    mod = importlib.import_module(proc)

    # Rename the process.
    setproctitle(proc)

    # Create new messaging context since we forked.
    messaging.reset_context()

    # Add daemon name tag to logs.
    cloudlog.bind(daemon=name)
    sentry.set_tag("daemon", name)

    # Execute the process.
    mod.main()

  except KeyboardInterrupt:
    cloudlog.warning(f"child {proc} got SIGINT")

  except Exception:
    # Can't install the crash handler because sys.excepthook
    # doesn't play nice with threads, so catch it here.
    sentry.capture_exception()
    raise


def nativelauncher(pargs: list[str], cwd: str, name: str) -> None:
  os.environ['MANAGER_DAEMON'] = name

  # Execute the process.
  os.chdir(cwd)
  os.execvp(pargs[0], pargs)


def join_process(process: Process, timeout: float) -> None:
  # Process().join(timeout) will hang due to a Python 3 bug:
  # https://bugs.python.org/issue28382
  #
  # Poll exitcode instead.
  t = time.monotonic()

  while time.monotonic() - t < timeout and process.exitcode is None:
    time.sleep(0.001)


class ManagerProcess(ABC):
  daemon = False
  sigkill = False

  should_run: Callable[[bool, Params, car.CarParams], bool]

  proc: Process | None = None
  enabled = True
  name = ""
  shutting_down = False

  # Restart bookkeeping.
  last_start_time = 0.0
  last_exit_time = 0.0
  restart_count = 0

  @abstractmethod
  def start(self) -> None:
    pass

  def _process_has_exited(self) -> bool:
    """
    Return True if a Process object exists but its child process
    has already exited.

    This is the critical condition that the original implementation
    failed to handle correctly.
    """
    return (
      self.proc is not None
      and self.proc.exitcode is not None
    )

  def _cleanup_dead_process(self) -> int | None:
    """
    Clean up a stale multiprocessing.Process object after the
    child process has exited.

    Returns the process exit code.
    """
    if self.proc is None:
      return None

    exitcode = self.proc.exitcode

    if exitcode is None:
      return None

    pid = self.proc.pid

    cloudlog.error(
      f"PROCESS EXITED: {self.name} "
      f"pid={pid} "
      f"exitcode={exitcode} "
      f"shutting_down={self.shutting_down}"
    )

    self.last_exit_time = time.monotonic()

    # Only count unexpected exits as crashes.
    if not self.shutting_down:
      self.restart_count += 1

      cloudlog.error(
        f"PROCESS CRASH/EXIT: {self.name} "
        f"restart_count={self.restart_count}"
      )

    # The multiprocessing.Process object cannot be reused.
    self.proc = None
    self.shutting_down = False

    return exitcode

  def _restart_allowed(self) -> bool:
    """
    Prevent extremely fast restart loops.

    Normal processes can restart immediately after a normal runtime.
    Processes that crash immediately receive a short delay.
    """
    now = time.monotonic()

    if self.last_start_time <= 0:
      return True

    runtime = now - self.last_start_time

    # Process lived long enough. Normal restart.
    if runtime >= RESTART_MIN_INTERVAL:
      return True

    # Process died almost immediately.
    if self.last_exit_time > 0:
      since_exit = now - self.last_exit_time

      if since_exit < RESTART_BACKOFF_INTERVAL:
        cloudlog.warning(
          f"delaying restart of {self.name}: "
          f"runtime={runtime:.3f}s "
          f"since_exit={since_exit:.3f}s"
        )
        return False

    return True

  def stop(
    self,
    retry: bool = True,
    block: bool = True,
    sig: signal.Signals | None = None
  ) -> int | None:

    if self.proc is None:
      return None

    # ------------------------------------------------------------------------
    # Process has already exited.
    # ------------------------------------------------------------------------
    if self.proc.exitcode is not None:
      ret = self.proc.exitcode

      cloudlog.info(
        f"{self.name} already dead "
        f"pid={self.proc.pid} "
        f"exitcode={ret}"
      )

      self.proc = None
      self.shutting_down = False

      return ret

    # ------------------------------------------------------------------------
    # Process is alive.
    # ------------------------------------------------------------------------
    if not self.shutting_down:
      cloudlog.info(f"killing {self.name}")

      if sig is None:
        sig = signal.SIGKILL if self.sigkill else signal.SIGINT

      self.signal(sig)
      self.shutting_down = True

      if not block:
        return None

    # Wait for process to terminate.
    join_process(self.proc, 5)

    # ------------------------------------------------------------------------
    # If process failed to die, force kill it.
    # ------------------------------------------------------------------------
    if self.proc.exitcode is None and retry:
      cloudlog.info(
        f"killing {self.name} with SIGKILL"
      )

      self.signal(signal.SIGKILL)
      self.proc.join()

    ret = self.proc.exitcode

    cloudlog.info(
      f"{self.name} is dead with {ret}"
    )

    if self.proc.exitcode is not None:
      self.shutting_down = False
      self.proc = None

    return ret

  def signal(self, sig: int) -> None:
    if self.proc is None:
      return

    # Don't signal an already exited process.
    if self.proc.exitcode is not None and self.proc.pid is not None:
      return

    # Can't signal a process without a PID.
    if self.proc.pid is None:
      return

    cloudlog.info(
      f"sending signal {sig} to {self.name}"
    )

    try:
      os.kill(self.proc.pid, sig)

    except ProcessLookupError:
      cloudlog.warning(
        f"{self.name} pid={self.proc.pid} "
        f"already disappeared"
      )

    except OSError as e:
      cloudlog.warning(
        f"failed to signal {self.name}: {e}"
      )

  def get_process_state_msg(self):
    state = log.ManagerState.ProcessState.new_message()
    state.name = self.name

    if self.proc is not None:
      state.running = (
        self.proc.exitcode is None
        and self.proc.is_alive()
      )

      state.shouldBeRunning = (
        self.proc is not None
        and not self.shutting_down
      )

      state.pid = self.proc.pid or 0
      state.exitCode = self.proc.exitcode or 0

    else:
      state.running = False
      state.shouldBeRunning = False
      state.pid = 0
      state.exitCode = 0

    return state


class NativeProcess(ManagerProcess):

  def __init__(
    self,
    name,
    cwd,
    cmdline,
    should_run,
    enabled=True,
    sigkill=False
  ):
    self.name = name
    self.cwd = cwd
    self.cmdline = cmdline
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.launcher = nativelauncher

    self.proc = None
    self.shutting_down = False

    self.last_start_time = 0.0
    self.last_exit_time = 0.0
    self.restart_count = 0

  def start(self) -> None:

    # ------------------------------------------------------------------------
    # If a previous non-blocking stop is still in progress,
    # finish stopping it first.
    # ------------------------------------------------------------------------
    if self.shutting_down:
      self.stop()

    # ------------------------------------------------------------------------
    # Existing process.
    # ------------------------------------------------------------------------
    if self.proc is not None:

      # Process is alive.
      if self.proc.exitcode is None:
        return

      # Process object exists, but child has exited.
      # The original implementation incorrectly returned here.
      self._cleanup_dead_process()

    # ------------------------------------------------------------------------
    # Prevent rapid crash/restart loops.
    # ------------------------------------------------------------------------
    if not self._restart_allowed():
      return

    cwd = os.path.join(BASEDIR, self.cwd)

    cloudlog.info(
      f"starting process {self.name}"
    )

    self.proc = Process(
      name=self.name,
      target=self.launcher,
      args=(
        self.cmdline,
        cwd,
        self.name,
      ),
    )

    try:
      self.proc.start()

      self.last_start_time = time.monotonic()
      self.shutting_down = False

      cloudlog.info(
        f"started {self.name} "
        f"pid={self.proc.pid}"
      )

    except Exception:
      cloudlog.exception(
        f"failed to start native process {self.name}"
      )

      self.proc = None
      self.shutting_down = False
      raise


class PythonProcess(ManagerProcess):

  def __init__(
    self,
    name,
    module,
    should_run,
    enabled=True,
    sigkill=False
  ):
    self.name = name
    self.module = module
    self.should_run = should_run
    self.enabled = enabled
    self.sigkill = sigkill
    self.launcher = launcher

    self.proc = None
    self.shutting_down = False

    self.last_start_time = 0.0
    self.last_exit_time = 0.0
    self.restart_count = 0

  def start(self) -> None:

    # ------------------------------------------------------------------------
    # If a previous non-blocking stop is still in progress,
    # finish stopping it first.
    # ------------------------------------------------------------------------
    if self.shutting_down:
      self.stop()

    # ------------------------------------------------------------------------
    # Existing process.
    # ------------------------------------------------------------------------
    if self.proc is not None:

      # Process is alive.
      if self.proc.exitcode is None:
        return

      # Process object exists, but child has exited.
      # The original implementation incorrectly returned here.
      self._cleanup_dead_process()

    # ------------------------------------------------------------------------
    # Prevent rapid crash/restart loops.
    # ------------------------------------------------------------------------
    if not self._restart_allowed():
      return

    cloudlog.info(
      f"starting python {self.module}"
    )

    self.proc = Process(
      name=self.name,
      target=self.launcher,
      args=(
        self.module,
        self.name,
      ),
    )

    try:
      self.proc.start()

      self.last_start_time = time.monotonic()
      self.shutting_down = False

      cloudlog.info(
        f"started {self.name} "
        f"pid={self.proc.pid}"
      )

    except Exception:
      cloudlog.exception(
        f"failed to start python process {self.name}"
      )

      self.proc = None
      self.shutting_down = False
      raise


class DaemonProcess(ManagerProcess):
  """
  Python process that has to stay running across manager restart.

  This is used for athena so you don't lose SSH access when restarting
  manager.
  """

  def __init__(
    self,
    name,
    module,
    param_name,
    enabled=True
  ):
    self.name = name
    self.module = module
    self.param_name = param_name
    self.enabled = enabled
    self.params = None

  @staticmethod
  def should_run(started, params, CP):
    return True

  def start(self) -> None:

    if self.params is None:
      self.params = Params()

    pid = self.params.get(self.param_name)

    if pid is not None:
      try:
        pid_int = int(pid)

        os.kill(pid_int, 0)

        with open(f'/proc/{pid_int}/cmdline') as f:
          if self.module in f.read():
            # Daemon is already running.
            return

      except (OSError, FileNotFoundError, ValueError):
        # Process is dead or PID is invalid.
        pass

    cloudlog.info(
      f"starting daemon {self.name}"
    )

    proc = subprocess.Popen(
      ['python', '-m', self.module],
      stdin=open('/dev/null'),
      stdout=open('/dev/null', 'w'),
      stderr=open('/dev/null', 'w'),
      preexec_fn=os.setpgrp
    )

    self.params.put(
      self.param_name,
      proc.pid,
      block=True
    )

  def stop(
    self,
    retry=True,
    block=True,
    sig=None
  ) -> None:
    # Daemon processes intentionally survive manager restart.
    pass


def ensure_running(
  procs: ValuesView[ManagerProcess],
  started: bool,
  params: Params,
  CP: car.CarParams,
  not_run: list[str] | None = None
) -> list[ManagerProcess]:

  if not_run is None:
    not_run = []

  running = []

  # --------------------------------------------------------------------------
  # Determine which processes should be running.
  # --------------------------------------------------------------------------
  for p in procs:

    should_be_running = (
      p.enabled
      and p.name not in not_run
      and p.should_run(started, params, CP)
    )

    if should_be_running:
      running.append(p)

    else:
      p.stop(block=False)

  # --------------------------------------------------------------------------
  # Start or restart processes.
  # --------------------------------------------------------------------------
  for p in running:

    # Explicitly detect a stale Process object before start().
    #
    # This is the main fix for:
    #
    #   child process exits
    #       ↓
    #   self.proc remains non-None
    #       ↓
    #   start() returns
    #       ↓
    #   process never comes back
    #
    if p.proc is not None and p.proc.exitcode is not None:

      cloudlog.error(
        f"detected dead process: {p.name} "
        f"pid={p.proc.pid} "
        f"exitcode={p.proc.exitcode}"
      )

      p._cleanup_dead_process()

    p.start()

  return running
