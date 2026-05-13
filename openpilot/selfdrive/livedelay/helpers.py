from openpilot.common.params import Params


def get_lat_delay(params: Params, stock_lat_delay: float, fallback_steer_delay: float | None = None) -> float:
  cached = params.get("LagdValueCache")
  if cached is not None:
    try:
      return float(cached)
    except (TypeError, ValueError):
      pass

  if params.get_bool("LagdToggle"):
    return float(stock_lat_delay)

  if fallback_steer_delay is not None:
    sw_delay = float(params.get("LagdToggleDelay", return_default=True))
    return float(fallback_steer_delay) + sw_delay

  return float(stock_lat_delay)
