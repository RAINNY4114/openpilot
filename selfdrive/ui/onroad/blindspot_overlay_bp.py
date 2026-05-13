import time

import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app


class BlindspotOverlayBP:
  def __init__(self):
    self._params = Params()
    self._left_alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._right_alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._pulse_start_time = time.monotonic()

  def enabled(self) -> bool:
    return self._params.get_bool("ShowBlindspotOverlay")

  def render(self, rect: rl.Rectangle, blind_spot_width: int = 250):
    if not self.enabled():
      return

    sm = ui_state.sm
    if not sm.valid.get('carState', False):
      return

    car_state = sm['carState']
    left_blindspot = bool(getattr(car_state, "leftBlindspot", False))
    right_blindspot = bool(getattr(car_state, "rightBlindspot", False))

    self._left_alpha_filter.update(1.0 if left_blindspot else 0.0)
    self._right_alpha_filter.update(1.0 if right_blindspot else 0.0)

    pulse_duration = 3.0
    current_time = time.monotonic()
    pulse_phase = ((current_time - self._pulse_start_time) % pulse_duration) / pulse_duration

    edge_alpha_start = 0.75
    edge_alpha_end = 0.0

    x = int(rect.x)
    y = int(rect.y)
    h = int(rect.height)

    brightness_pulse = 0.3 + 0.7 * (0.5 + 0.5 * np.sin(pulse_phase * 2 * np.pi))

    if self._left_alpha_filter.x > 0.01:
      filter_alpha = self._left_alpha_filter.x
      edge_alpha = int(255 * edge_alpha_start * filter_alpha * brightness_pulse)
      inside_alpha = int(255 * edge_alpha_end * filter_alpha * brightness_pulse)
      edge_color = rl.Color(255, 0, 0, edge_alpha)
      inside_color = rl.Color(255, 0, 0, inside_alpha)
      rl.draw_rectangle_gradient_h(x, y, blind_spot_width, h, edge_color, inside_color)

    if self._right_alpha_filter.x > 0.01:
      filter_alpha = self._right_alpha_filter.x
      edge_alpha = int(255 * edge_alpha_start * filter_alpha * brightness_pulse)
      inside_alpha = int(255 * edge_alpha_end * filter_alpha * brightness_pulse)
      edge_color = rl.Color(255, 0, 0, edge_alpha)
      inside_color = rl.Color(255, 0, 0, inside_alpha)
      rl.draw_rectangle_gradient_h(
        x + int(rect.width) - blind_spot_width, y,
        blind_spot_width, h,
        inside_color, edge_color,
      )
