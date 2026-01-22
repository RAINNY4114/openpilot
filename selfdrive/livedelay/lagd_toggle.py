from cereal import log
from opendbc.car import structs

from openpilot.common.params import Params


class LagdToggle:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()
    self.lag = 0.0
    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = float(self.params.get("LagdToggleDelay", return_default=True))

  def _read_params(self) -> None:
    self.lagd_toggle = self.params.get_bool("LagdToggle")
    self.software_delay = float(self.params.get("LagdToggleDelay", return_default=True))

  def update(self, lag_msg: log.LiveDelayData) -> None:
    self._read_params()

    if self.lagd_toggle:
      self.lag = float(lag_msg.liveDelay.lateralDelay)
    else:
      self.lag = float(self.CP.steerActuatorDelay) + self.software_delay

    self.params.put_nonblocking("LagdValueCache", self.lag)
