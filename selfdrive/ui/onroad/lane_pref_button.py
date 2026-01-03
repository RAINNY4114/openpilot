import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


class LanePrefButton(Widget):
  """
  Single-tap HUD control that cycles the lane preference:
    0 = AUTO (default)
    1 = KEEP LEFT
    2 = KEEP RIGHT
  """

  _KEY = "dp_lincoln_lane_preference"
  _LABELS = {0: "A", 1: "L", 2: "R"}

  def __init__(self, button_size: int):
    super().__init__()
    self._params = Params()
    self._rect = rl.Rectangle(0, 0, button_size, button_size)
    self._font = gui_app.font(FontWeight.BOLD)
    # Subtle/transparent HUD styling
    self._bg = rl.Color(0, 0, 0, 110)
    self._fg = rl.Color(255, 255, 255, 255)

    # Highlight per state (subtle, no popups/voice).
    self._accent = {
      0: rl.Color(255, 255, 255, 120),   # AUTO
      1: rl.Color(0, 122, 255, 160),     # LEFT
      2: rl.Color(255, 149, 0, 160),     # RIGHT
    }

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y
    self._rect.width, self._rect.height = rect.width, rect.height

  def _update_state(self) -> None:
    # Only show for Ford/Lincoln builds, and only when an auto lane-change
    # feature is enabled (auto avoidance / auto overtake) to keep the HUD clean.
    try:
      car_params = ui_state.sm["carParams"]
      is_ford = getattr(car_params, "brand", "") == "ford"
    except Exception:
      self.set_visible(False)
      return

    try:
      auto_overtake = self._params.get_bool("dp_lincoln_auto_overtake")
      auto_avoid = self._params.get_bool("dp_lincoln_auto_avoid")
    except Exception:
      auto_overtake = False
      auto_avoid = False

    self.set_visible(is_ford and (auto_overtake or auto_avoid))

  def _get_pref(self) -> int:
    try:
      raw = self._params.get(self._KEY)
      if raw is None:
        return 0
      if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="ignore")
      v = int(str(raw).strip() or "0")
      return v if v in (0, 1, 2) else 0
    except Exception:
      return 0

  def _set_pref(self, v: int) -> None:
    v = v if v in (0, 1, 2) else 0
    self._params.put(self._KEY, str(v))

  def _handle_mouse_release(self, mouse_pos) -> bool:
    super()._handle_mouse_release(mouse_pos)
    self._set_pref((self._get_pref() + 1) % 3)
    return False

  def _render(self, rect: rl.Rectangle) -> None:
    pref = self._get_pref()
    label = self._LABELS.get(pref, "A")

    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)
    radius = float(self._rect.width / 2)

    fg = rl.Color(self._fg.r, self._fg.g, self._fg.b, 180 if self.is_pressed else 255)
    accent = self._accent.get(pref, fg)

    # Drop shadow (very subtle)
    shadow_alpha = 55 if not self.is_pressed else 80
    rl.draw_circle(center_x + 2, center_y + 3, radius, rl.Color(0, 0, 0, shadow_alpha))

    # Background (transparent radial gradient)
    bg_outer_alpha = int((150 if self.is_pressed else self._bg.a))
    bg_center_alpha = int(bg_outer_alpha * 0.55)
    rl.draw_circle_gradient(
      center_x,
      center_y,
      int(radius),
      rl.Color(0, 0, 0, bg_center_alpha),
      rl.Color(0, 0, 0, bg_outer_alpha),
    )

    # Accent ring (thicker than draw_circle_lines)
    ring_outer = max(1.0, radius - 1.0)
    ring_inner = max(0.0, ring_outer - max(3.0, radius * 0.06))
    rl.draw_ring(rl.Vector2(center_x, center_y), ring_inner, ring_outer, 0.0, 360.0, 40, accent)
    rl.draw_ring(rl.Vector2(center_x, center_y), max(0.0, ring_inner - 2.0), max(0.0, ring_inner - 1.0), 0.0, 360.0, 40, rl.Color(255, 255, 255, 35))

    font_size = int(68)
    text_metrics = measure_text_cached(self._font, label, font_size)
    text_pos = rl.Vector2(center_x - text_metrics.x / 2, center_y - text_metrics.y / 2)

    # Text shadow for readability on bright scenes
    rl.draw_text_ex(self._font, label, rl.Vector2(text_pos.x + 2, text_pos.y + 2), font_size, 0, rl.Color(0, 0, 0, 120))
    rl.draw_text_ex(self._font, label, text_pos, font_size, 0, fg)
