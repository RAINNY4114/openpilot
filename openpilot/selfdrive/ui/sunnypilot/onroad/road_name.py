"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.sunnypilot.onroad import cjk_font
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, FONT_SCALE
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

FONT_SIZE = 46


class RoadNameRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.road_name = ""
    self.is_metric = False
    self.font_demi = gui_app.font(FontWeight.SEMI_BOLD)

  def update(self):
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      return

    self.is_metric = ui_state.is_metric

    if sm.updated["liveMapDataSP"]:
      lmd = sm["liveMapDataSP"]
      self.road_name = lmd.roadName

  def _render(self, rect: rl.Rectangle):
    if not self.road_name or not ui_state.road_name_toggle:
      return

    text = self.road_name

    # [ROADNAME_CJK 20261027] 道路名是动态中文，而 UI 的 CJK 字体是按
    # app_<lang>.po 里的字符预子集加载的，OSM 里的「恩」「波」「桥」「路」
    # 全都没有字形 ⇒ 原来显示成 "??大道"。这里换成按码点现建的完整 CJK 字体，
    # 详见 cjk_font.py 的 docstring。拿不到就退回子集字体（会缺字，但绝不崩）。
    px = FONT_SIZE * FONT_SCALE
    cjk = cjk_font.get_font(text)

    if cjk is not None:
      def measure(t: str) -> rl.Vector2:
        return cjk_font.measure(cjk, t, px)

      def draw(t: str, origin: rl.Vector2) -> None:
        cjk_font.draw(cjk, t, origin, px, rl.Color(255, 255, 255, 200))
    else:
      def measure(t: str) -> rl.Vector2:
        return measure_text_cached(self.font_demi, t, FONT_SIZE)

      def draw(t: str, origin: rl.Vector2) -> None:
        rl.draw_text_ex(self.font_demi, t, origin, FONT_SIZE, 0, rl.Color(255, 255, 255, 200))

    text_size = measure(text)

    padding = 40
    rect_width = max(200, min(text_size.x + padding, rect.width - 40))

    road_rect = rl.Rectangle(rect.x + rect.width / 2 - rect_width / 2, rect.y - 4, rect_width, 60)

    rl.draw_rectangle_rounded(road_rect, 0.2, 10, rl.Color(0, 0, 0, 120))

    max_text_width = road_rect.width - 20
    if text_size.x > max_text_width:
      while text_size.x > max_text_width and len(text) > 3:
        text = text[:-1]
        text_size = measure(text + "...")
      text = text + "..."

    sz = measure(text)
    origin = rl.Vector2(road_rect.x + road_rect.width / 2 - sz.x / 2, road_rect.y + road_rect.height / 2 - sz.y / 2)
    draw(text, origin)
