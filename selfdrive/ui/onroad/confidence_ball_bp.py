import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


def draw_circle_gradient(center_x: float, center_y: float, radius: int,
                         top: rl.Color, bottom: rl.Color) -> None:
  rl.draw_rectangle_gradient_v(int(center_x - radius), int(center_y - radius),
                               radius * 2, radius * 2,
                               top, bottom)

  outer_radius = int(radius * 1.5) + 2
  rl.draw_ring(rl.Vector2(center_x, center_y), radius, outer_radius,
               0.0, 360.0,
               20, rl.BLACK)


class ConfidenceBallTiciBP(Widget):
  BALL_RADIUS = 50
  BALL_WIDTH = BALL_RADIUS * 2 + 5

  def __init__(self):
    super().__init__()
    self._confidence_filter = FirstOrderFilter(-0.5, 0.5, 1 / gui_app.target_fps)

  def _update_state(self):
    if ui_state.status == UIStatus.DISENGAGED:
      self._confidence_filter.update(-0.5)
      return

    try:
      model = ui_state.sm['modelV2']
      brake_probs = list(getattr(model.meta.disengagePredictions, "brakeDisengageProbs", []) or [1.0])
      steer_probs = list(getattr(model.meta.disengagePredictions, "steerOverrideProbs", []) or [1.0])
      confidence = (1 - max(brake_probs)) * (1 - max(steer_probs))
    except Exception:
      confidence = 0.0

    self._confidence_filter.update(confidence)

  def _render(self, _):
    content_rect = rl.Rectangle(
      self.rect.x,
      self.rect.y,
      self.rect.width,
      self.rect.height,
    )

    radius = self.BALL_RADIUS
    bottom_position = content_rect.height
    top_position = 0.0
    range_height = bottom_position - top_position

    filter_min = -0.5
    filter_max = 1.0
    normalized = (self._confidence_filter.x - filter_min) / (filter_max - filter_min)
    normalized = max(0.0, min(1.0, normalized))

    dot_height = bottom_position - (normalized * range_height) + radius
    dot_height = content_rect.y + dot_height

    if ui_state.status == UIStatus.ENGAGED:
      if self._confidence_filter.x > 0.5:
        top_dot_color = rl.Color(0, 255, 204, 255)
        bottom_dot_color = rl.Color(0, 255, 38, 255)
      elif self._confidence_filter.x > 0.2:
        top_dot_color = rl.Color(255, 200, 0, 255)
        bottom_dot_color = rl.Color(255, 115, 0, 255)
      else:
        top_dot_color = rl.Color(255, 0, 21, 255)
        bottom_dot_color = rl.Color(255, 0, 89, 255)
    elif ui_state.status == UIStatus.OVERRIDE:
      top_dot_color = rl.Color(255, 255, 255, 255)
      bottom_dot_color = rl.Color(82, 82, 82, 255)
    else:
      top_dot_color = rl.Color(50, 50, 50, 255)
      bottom_dot_color = rl.Color(13, 13, 13, 255)

    ball_center_x = content_rect.x + radius
    draw_circle_gradient(ball_center_x, dot_height, radius, top_dot_color, bottom_dot_color)
