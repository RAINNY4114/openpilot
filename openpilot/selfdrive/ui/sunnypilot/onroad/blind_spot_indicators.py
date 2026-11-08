"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter


class BlindSpotIndicators:
  # [BSD_20261108] Zone-based color levels (cpv9-inspired)
  # Zone 1-2: far (yellow), Zone 3: near (orange), Zone 4: danger (red)
  # Also checks radarState.leadLeft/leadRight for side radar targets
  ZONE_COLORS = {
    0: rl.Color(255, 255, 255, 255),   # white (default)
    1: rl.Color(255, 255, 100, 255),   # pale yellow (far)
    2: rl.Color(255, 220, 80, 255),    # yellow (far)
    3: rl.Color(255, 165, 0, 255),     # orange (near)
    4: rl.Color(255, 60, 60, 255),     # red (danger)
  }

  def __init__(self):
    self._txt_blind_spot_left: rl.Texture = gui_app.texture('icons_mici/onroad/blind_spot_left.png', 108, 128)
    self._txt_blind_spot_right: rl.Texture = gui_app.texture('icons_mici/onroad/blind_spot_left.png', 108, 128, flip_x=True)

    self._blind_spot_left_alpha_filter = FirstOrderFilter(0, 0.15, 1 / gui_app.target_fps)
    self._blind_spot_right_alpha_filter = FirstOrderFilter(0, 0.15, 1 / gui_app.target_fps)
    self._left_zone = 0
    self._right_zone = 0
    self._left_side_radar = False
    self._right_side_radar = False

  def _read_bsm_zone(self):
    """Read Ford BSM Zone from AOBsmZone param file."""
    import os
    left_zone, right_zone = 0, 0
    try:
      for d in ("/dev/shm/params", "/data/params/d"):
        try:
          with open(os.path.join(d, "AOBsmZone")) as f:
            val = int(float(f.read().strip()))
            left_zone = val // 100
            right_zone = val % 100
            break
        except Exception:
          continue
    except Exception:
      pass
    return left_zone, right_zone

  def update(self) -> None:
    sm = ui_state.sm
    CS = sm['carState']

    # [BSD_20261108] Read BSM zone levels
    self._left_zone, self._right_zone = self._read_bsm_zone()

    # Also check radarState side leads
    self._left_side_radar = False
    self._right_side_radar = False
    try:
      rs = sm['radarState']
      if rs.leadLeft.present:
        self._left_side_radar = True
        # If no BSM zone but side radar detects close target, set zone 3
        if self._left_zone == 0 and float(rs.leadLeft.dRel) < 5.0:
          self._left_zone = 3
      if rs.leadRight.present:
        self._right_side_radar = True
        if self._right_zone == 0 and float(rs.leadRight.dRel) < 5.0:
          self._right_zone = 3
    except Exception:
      pass

    # Use boolean blindspot as fallback (zone 3 if no zone data)
    left_detected = CS.leftBlindspot or self._left_side_radar
    right_detected = CS.rightBlindspot or self._right_side_radar
    if left_detected and self._left_zone == 0:
      self._left_zone = 3
    if right_detected and self._right_zone == 0:
      self._right_zone = 3

    self._blind_spot_left_alpha_filter.update(1.0 if left_detected else 0.0)
    self._blind_spot_right_alpha_filter.update(1.0 if right_detected else 0.0)

  @property
  def detected(self) -> bool:
    return ui_state.blindspot and (self._blind_spot_left_alpha_filter.x > 0.01 or self._blind_spot_right_alpha_filter.x > 0.01)

  def render(self, rect: rl.Rectangle) -> None:
    if not ui_state.blindspot:
      return

    BLIND_SPOT_MARGIN_X = 20
    BLIND_SPOT_Y_OFFSET = 100

    if self._blind_spot_left_alpha_filter.x > 0.01:
      pos_x = int(rect.x + BLIND_SPOT_MARGIN_X)
      pos_y = int(rect.y + BLIND_SPOT_Y_OFFSET)
      alpha = int(255 * self._blind_spot_left_alpha_filter.x)
      zone = min(self._left_zone, 4)
      color = self.ZONE_COLORS.get(zone, rl.Color(255, 255, 255, alpha))
      color = rl.Color(color.r, color.g, color.b, alpha)
      rl.draw_texture_ex(self._txt_blind_spot_left, rl.Vector2(pos_x, pos_y), 0.0, 1.0, color)

    if self._blind_spot_right_alpha_filter.x > 0.01:
      pos_x = int(rect.x + rect.width - BLIND_SPOT_MARGIN_X - self._txt_blind_spot_right.width)
      pos_y = int(rect.y + BLIND_SPOT_Y_OFFSET)
      alpha = int(255 * self._blind_spot_right_alpha_filter.x)
      zone = min(self._right_zone, 4)
      color = self.ZONE_COLORS.get(zone, rl.Color(255, 255, 255, alpha))
      color = rl.Color(color.r, color.g, color.b, alpha)
      rl.draw_texture_ex(self._txt_blind_spot_right, rl.Vector2(pos_x, pos_y), 0.0, 1.0, color)
