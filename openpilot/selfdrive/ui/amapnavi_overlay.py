#!/usr/bin/env python3
"""amapnavi（外挂激光雷达 / 摄像头 BSD 模块）的 UI 显示。

由 cpv9-dev ``openpilot/selfdrive/ui/carrot.cc`` 中 AmapNavi 相关绘制移植而来
（源端约 500 行 C++ / nanovg，含 ui.cc 中订阅 amapNavi 的 1 行）。

**设计目标：不侵入现有 UI 代码**。解析、配色、布局、绘制全部集中在本模块，
现有 UI 文件各自只需加 1 行调用：

* ``ui_state.py``              - SubMaster 增加 ``"amapNavi"``
* ``onroad/model_renderer.py`` - ``_draw_carrot_overlays()`` 末尾调用 :func:`draw_barriers_c3`
* ``onroad/hud_renderer.py``   - ``_render()`` 调用 :func:`draw_bsd_panel`，
                                 状态徽标处调用 :func:`draw_ext_state_badge`
* ``mici/onroad/model_renderer.py`` - ``_draw_blindspots()`` 末尾调用 :func:`draw_barriers_mici`

**改动生效方式（踩过的坑）**：``system/manager/process.py`` 会在 manager 启动时
preimport 各进程模块，各进程再由 manager ``fork`` 出来并继承已导入的模块。
因此**只重启 UI 进程（``selfdrive.ui.ui``）不会重新加载本模块**——改完这里的
UI 代码必须重启 manager / 设备（``sudo reboot``）才会生效；只重启 UI 进程时
屏幕会一直沿用 manager 启动那一刻的旧版本，表现为"改了没反应"。

侧向标志（``amapNavi`` 的独立字段，不再按位或）::

  blindLidarL     激光-左侧方          blindLidarR     激光-右侧方
  blindLidarLf    激光-左前角          blindLidarRf    激光-右前角
  blindLidarLb    激光-左后角          blindLidarRb    激光-右后角
  blindCombinedL  综合盲区(左)         blindCombinedR  综合盲区(右)
  blindCarL       车身盲区(左)         blindCarR       车身盲区(右)
  laneBlindL      左侧实线             laneBlindR      右侧实线

  ``leftBlind`` / ``rightBlind`` 位图字段保留仅为兼容（旧代码/诊断），本模块不依赖它：
  :meth:`AmapNaviView.blind` 内部把上面的独立字段组装成原有位序，绘制逻辑无感知。

  组装位序（``blind()`` 用，与源端位图一致）::

    bit0 (1)  激光雷达盲区          bit3 (8)  实线
    bit1 (2)  综合盲区              bit4 (16) 目标在侧前方（向上箭头）
    bit2 (4)  车身盲区              bit5 (32) 目标在侧后方（向下箭头）

外挂转向灯（``amapNavi.extBlinker``，独立字段）::

  0=灭 1=左 2=右 —— 来自外挂转向灯板回传的状态（板子实际在打什么灯），
  用于在激光雷达行的雷达图标内侧画闪烁的转向箭头

设备位定义（``leftDevice`` / ``rightDevice``）::

  bit0 (1) 激光雷达在线
  bit1 (2) 摄像头在线

距离字段（``lfDrel`` / ``lbDrel`` / ``rfDrel`` / ``rbDrel``，四角 = 左前/左后/右前/右后）::

  原始单位 mm，amap_navi 发布时 /100 → dm，本模块显示时再 /10 → 米（与源端一致）

显示策略（与源端一致）::

  变道护栏：``ShowLaneInfo >= 2`` 强制常显；否则只在对应侧处于变道准备
            (``laneChangeState == preLaneChange``) 时显示  —— 见 :func:`barrier_visible_sides`
  图标箭头：``ShowLaneInfo >= 1`` 才画上下箭头

**第一行圆圈（原车前盲区）的数据来源**：源端取 ``modelV2.meta.leftFrontBlind``，
本 fork 的 MetaData 没有该字段（cereal 里搜不到 FrontBlind），因此改从
``amapNavi.lFrontBlind`` / ``rFrontBlind`` 读 —— 由
``selfdrive/carrot/amapnavi/stock_front_blind.py`` 用原车前雷达目标算出后下发；
若将来 MetaData 补上该字段，两者按 OR 合并（见 :func:`_front_blind`）。

**第一行还显示原车雷达的侧前距离**：有 ``radarState.leadLeft`` / ``leadRight``
目标时，画"浅色底 + 粗黄圈"并在旁边显示它的 ``dRel``（米，一位小数，黄色带
黑底，与第二行四角激光距离同格式）；判定为盲区时仍显示原来的黄圆 + 红箭头
图标。见 :func:`_radar_side_leads`。

**图标配色统一为全不透明**（半透明在花背景与夜景下都显淡，纯蓝在夜里尤其看不清，
已提亮一档）；「有数据但不判盲区」的样式（第一行前雷达、第二行激光雷达都是
:func:`_circle_ring`）＝ 不透明浅色底 + 16px 粗彩色环，浅底色固定所以对比度
不随摄像头画面变化；盲区实心圆（:func:`_icon_solid`）直接用不透明填充色，
不再叠深色描边/底衬。
"""

import os
import time
from dataclasses import dataclass, field

import pyray as rl

from openpilot.cereal import log
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.lib.text_draw import draw_text_ui_style, get_text_draw_pos
from openpilot.selfdrive.ui.road_markings import blindspot_barrier_quads, project_blindspot_barrier

LaneChangeState = log.LaneChangeState

# ------------------------------------------------------------------ 位定义
BLIND_LIDAR = 1
BLIND_CAMERA = 2
BLIND_STOCK_SIDE = 4
BLIND_SOLID_LINE = 8
BLIND_FRONT = 16
BLIND_REAR = 32

# 外挂转向灯板回传的转向灯状态（amapNavi.extBlinker，独立字段，不占盲区位图）
BLINKER_LEFT = 1
BLINKER_RIGHT = 2

DEVICE_LIDAR = 1
DEVICE_CAMERA = 2

SIDES = ("left", "right")

# ------------------------------------------------------------------ 配色
def _c(r: int, g: int, b: int, a: int) -> rl.Color:
  return rl.Color(r, g, b, a)


# 变道护栏（半透明，源端 alpha=60）
BARRIER_YELLOW = _c(255, 215, 0, 60)
BARRIER_BLUE = _c(0, 0, 255, 60)
BARRIER_PURPLE = _c(138, 43, 226, 60)
BARRIER_CYAN = _c(0, 255, 255, 60)
BARRIER_PINK = _c(233, 37, 227, 60)

# 圆形图标（全不透明：半透明会显得颜色很淡，花背景和夜里都看不清）
ICON_RED = _c(255, 0, 0, 255)
ICON_YELLOW = _c(255, 215, 0, 255)
# 纯蓝 (0,0,255) 在夜间全黑背景上几乎看不出来，整体提亮一档
ICON_BLUE = _c(0, 120, 255, 255)
ICON_PURPLE = _c(138, 43, 226, 255)
ICON_PINK = _c(233, 37, 227, 255)

# 「有雷达数据但不判盲区」时的样式：不透明浅色底 + 粗圈。
# 圈色固定落在浅底上，不受摄像头画面影响（比细环/双环稳得多）。
# 盲区实心圆则直接用不透明填充色，不再加黑描边/底衬——实心圆上用不上，也不好看。
RING_WIDTH = 16
RING_BACKDROP = _c(238, 238, 238, 255)

# 线条 / 箭头
LINE_YELLOW = _c(255, 255, 0, 255)
# 同 ICON_BLUE，提亮后夜里才看得见
LINE_BLUE = _c(0, 120, 255, 255)
ARROW_RED = _c(255, 0, 0, 255)
ARROW_YELLOW = _c(255, 215, 0, 255)
DIST_TEXT_YELLOW = _c(255, 255, 0, 255)

# 转向灯图标（样式与 c3-dev 一致：selfdrive/assets/icons_mici/onroad/turn_signal_left.png）
#   右侧用同一张图水平翻转。闪烁节奏照搬 c3-dev：每 0.75s 冲到最亮，
#   其余时间按一阶衰减到 20%（不是硬开关，看起来是"心跳"式闪动）。
TURN_SIGNAL_BLINK_PERIOD = 1 / (80 / 60)   # 0.75s（c3-dev: Mazda 心跳节奏）
TURN_LAMP_TEX = "icons_mici/onroad/turn_signal_left.png"
# c3-dev 里这张贴图按 120x109 显示（外面套 150x150 区域）。C3 屏比 c3-dev 的
# 仪表屏大得多，同样 120px 在这边显得偏小，所以先按 1.1 倍放到 132x120，
# 再按需求加大 50% -> 198x180（贴图内箭头本体约占 84%，即约 166px）。
TURN_LAMP_W = 198               # 显示宽
TURN_LAMP_H = 180               # 高：保持贴图 120:109 的比例
TURN_LAMP_GAP = 14              # 与雷达图标的水平间距
TURN_LAMP_TEXT_RESERVE = 160    # 还要再让出的宽度（避开雷达图标外侧的四角距离文字）
TURN_LAMP_DIM = 0.2             # 闪烁暗态亮度比例（c3-dev: 255*0.2）

# 调试用截屏钩子：文件存在则把当前画面存到 /tmp/shot_<毫秒>.png 并删掉该文件
SHOOT_FLAG = "/tmp/shoot"

# ------------------------------------------------------------------ 布局
CIRCLE_RADIUS = 46
# 三行图标（原车前盲区 / 激光雷达 / 原车后盲区）的纵向间距。
# 行间实际留白 = VERTICAL_SPACING - 2 * CIRCLE_RADIUS：120 时是 28px，
# 按"缩小三分之一"改成 20px（112 - 92）。
VERTICAL_SPACING = 112
HORIZONTAL_OFFSET = 150
# 第一行圆心 = TOP_Y + CIRCLE_RADIUS，顶部留白 50px
TOP_Y = 50
ARROW_HEAD_WIDTH = 46
ARROW_HEAD_LENGTH = 30
ARROW_GAP = 5

DIST_FONT_SIZE = 50
DIST_TEXT_OFFSET_X = 30
DIST_TEXT_OFFSET_Y = 35

# 距离文字的半透明黑底（移植自源端 drawTextWithBg）：
# 相机画面背景太花，纯文字看不清，源端给每个距离值垫了 RGBA(0,0,0,150) 的圆角底
DIST_BG_COLOR = _c(0, 0, 0, 150)
DIST_BG_PADDING_X = 10
DIST_BG_PADDING_Y = 6
DIST_BG_ROUND_PX = 6

SOLID_BAR_WIDTH = 20

# E 徽标（外挂客户端数量）
# 左侧单字符徽标占 dx-55..dx-5，这里紧接其后占 dx+5..dx+55，
# 两块合计 110px，正好是原来 3 字符徽标的位置（与 cpv9-dev 一致）
EXT_BADGE_OFFSET_X = 5
EXT_BADGE_WIDTH = 50
EXT_BADGE_HEIGHT = 48
EXT_BADGE_OK = _c(0, 228, 48, 255)
EXT_BADGE_IDLE = _c(230, 60, 60, 255)

# ------------------------------------------------------------------ 数据
@dataclass
class AmapNaviView:
  """一帧 amapNavi 消息的解包结果（纯数据，便于单测）。"""

  left_blind: int = 0
  right_blind: int = 0
  left_device: int = 0
  right_device: int = 0
  left_line: int = 0
  right_line: int = 0
  line_valid: bool = False
  ext_state: int = 0
  # 外挂转向灯板回传的转向灯状态（0=灭 1=左 2=右），来自 amapNavi.extBlinker
  ext_blinker: int = 0
  # 侧向标志（独立字段，来自 amapNavi.blindLidarL 等，不再依赖位图）
  blind_lidar_l: bool = False
  blind_lidar_lf: bool = False
  blind_lidar_lb: bool = False
  blind_combined_l: bool = False
  blind_car_l: bool = False
  lane_blind_l: bool = False
  blind_lidar_r: bool = False
  blind_lidar_rf: bool = False
  blind_lidar_rb: bool = False
  blind_combined_r: bool = False
  blind_car_r: bool = False
  lane_blind_r: bool = False
  # corner -> (valid, 米)
  distances: dict = field(default_factory=dict)

  def blind(self, side: str) -> int:
    """由独立字段组装成位图（绘制逻辑保持原样；位序与原位图一致）。

    位序：1=激光侧方 2=综合盲区 4=车身盲区 8=实线 16=激光前角 32=激光后角
    """
    bits = 0
    if side == "left":
      if self.blind_lidar_l:
        bits |= BLIND_LIDAR
      if self.blind_combined_l:
        bits |= BLIND_CAMERA
      if self.blind_car_l:
        bits |= BLIND_STOCK_SIDE
      if self.lane_blind_l:
        bits |= BLIND_SOLID_LINE
      if self.blind_lidar_lf:
        bits |= BLIND_FRONT
      if self.blind_lidar_lb:
        bits |= BLIND_REAR
    else:
      if self.blind_lidar_r:
        bits |= BLIND_LIDAR
      if self.blind_combined_r:
        bits |= BLIND_CAMERA
      if self.blind_car_r:
        bits |= BLIND_STOCK_SIDE
      if self.lane_blind_r:
        bits |= BLIND_SOLID_LINE
      if self.blind_lidar_rf:
        bits |= BLIND_FRONT
      if self.blind_lidar_rb:
        bits |= BLIND_REAR
    return bits

  def device(self, side: str) -> int:
    return self.left_device if side == "left" else self.right_device

  @property
  def has_distance(self) -> bool:
    return any(valid for valid, _ in self.distances.values())


def read_amapnavi(sm):
  """从 SubMaster 读一帧 amapNavi；不可用或未在线时返回 None。"""
  try:
    if not (sm.valid['amapNavi'] and sm.alive['amapNavi']):
      return None
    msg = sm['amapNavi']
  except Exception:
    return None

  view = AmapNaviView(
    left_blind=int(msg.leftBlind),
    right_blind=int(msg.rightBlind),
    left_device=int(msg.leftDevice),
    right_device=int(msg.rightDevice),
    left_line=int(msg.leftLine),
    right_line=int(msg.rightLine),
    line_valid=bool(msg.lineValid),
    ext_state=int(getattr(msg, "extState", 0) or 0),
    ext_blinker=int(getattr(msg, "extBlinker", 0) or 0),
    blind_lidar_l=bool(getattr(msg, "blindLidarL", False)),
    blind_lidar_lf=bool(getattr(msg, "blindLidarLf", False)),
    blind_lidar_lb=bool(getattr(msg, "blindLidarLb", False)),
    blind_combined_l=bool(getattr(msg, "blindCombinedL", False)),
    blind_car_l=bool(getattr(msg, "blindCarL", False)),
    lane_blind_l=bool(getattr(msg, "laneBlindL", False)),
    blind_lidar_r=bool(getattr(msg, "blindLidarR", False)),
    blind_lidar_rf=bool(getattr(msg, "blindLidarRf", False)),
    blind_lidar_rb=bool(getattr(msg, "blindLidarRb", False)),
    blind_combined_r=bool(getattr(msg, "blindCombinedR", False)),
    blind_car_r=bool(getattr(msg, "blindCarR", False)),
    lane_blind_r=bool(getattr(msg, "laneBlindR", False)),
  )
  for corner in ("lf", "lb", "rf", "rb"):
    valid = int(getattr(msg, f"{corner}DrelValid", 0) or 0)
    raw = float(getattr(msg, f"{corner}Drel", 0) or 0)
    view.distances[corner] = (valid, raw / 10.0)  # dm -> m
  return view


def _stock_blindspots(sm) -> dict:
  """原车后盲区（carState.leftBlindspot / rightBlindspot）。"""
  out = {"left": False, "right": False}
  try:
    if sm.valid['carState']:
      car_state = sm['carState']
      out["left"] = bool(car_state.leftBlindspot)
      out["right"] = bool(car_state.rightBlindspot)
  except Exception:
    pass
  return out


def _front_blind(sm) -> dict:
  """原车前盲区（原车前雷达判断的侧前方盲区，UI 第一行黄圆图标）。

  源端取 ``modelV2.meta.leftFrontBlind`` / ``rightFrontBlind``，但本 fork 的
  MetaData 没有这两个字段；这里改从 CP 的 ``amapNavi.lFrontBlind`` /
  ``rFrontBlind`` 读（由 ``selfdrive/carrot/amapnavi/stock_front_blind.py``
  依据原车前雷达目标算出）。两者任一为真即为真，取不到则 False。
  """
  out = {"left": False, "right": False}
  try:
    if sm.valid['amapNavi']:
      msg = sm['amapNavi']
      out["left"] = bool(getattr(msg, "lFrontBlind", False))
      out["right"] = bool(getattr(msg, "rFrontBlind", False))
  except Exception:
    pass
  try:
    if sm.valid['modelV2']:
      meta = sm['modelV2'].meta
      out["left"] = out["left"] or bool(getattr(meta, "leftFrontBlind", False))
      out["right"] = out["right"] or bool(getattr(meta, "rightFrontBlind", False))
  except Exception:
    pass
  return out


def _radar_side_leads(sm) -> dict:
  """原车前雷达的侧向目标（``radarState.leadLeft`` / ``leadRight``）距离。

  UI 的 SubMaster 本来就订阅了 ``radarState``（渲染器画原车盲区护栏时在用），
  这里直接取用，单位就是米，无需经 amapNavi 中转。

  :return: ``{"left": (是否有目标, 纵向距离 m), "right": (...)}``
  """
  out = {"left": (False, 0.0), "right": (False, 0.0)}
  try:
    # 注意用 alive 而不是 valid：radarState.valid 反映的是雷达 CAN 错误状态，
    # 经常为 False（本车前雷达走的是另一条链路），但 leadLeft/leadRight 本身
    # 仍然有效——用 valid 当门控会导致距离永远不显示。目标有没有车看 status。
    if sm.alive['radarState'] or sm.valid['radarState']:
      radar = sm['radarState']
      for side, key in (("left", "leadLeft"), ("right", "leadRight")):
        lead = getattr(radar, key, None)
        if lead is not None and bool(getattr(lead, "status", False)):
          out[side] = (True, float(getattr(lead, "dRel", 0.0) or 0.0))
  except Exception:
    pass
  return out


# ------------------------------------------------------------------ 配色决策
def barrier_color(blind: int, front_blind: bool = False):
  """amapnavi 变道护栏的配色（对应源端 ``ui_draw_bsd`` 的颜色分支）。

  原车后盲区（红色）由渲染器自己绘制，这里不覆盖，因此只在 amapnavi 有盲区时返回颜色。

  :return: :class:`rl.Color`，None 表示不需要绘制
  """
  if blind <= 0:
    return None

  lidar = blind & BLIND_LIDAR
  camera = blind & BLIND_CAMERA
  if front_blind and lidar and camera:  # 前盲区 + 雷达 + 摄像头
    return BARRIER_PINK
  if front_blind and lidar:  # 前盲区 + 雷达
    return BARRIER_CYAN
  if camera:  # 摄像头
    return BARRIER_PURPLE
  if lidar:  # 雷达
    return BARRIER_BLUE
  return BARRIER_YELLOW if front_blind else None


def icon_color(blind: int):
  """雷达 / 摄像头盲区圆圈的填充色。"""
  lidar = blind & BLIND_LIDAR
  camera = blind & BLIND_CAMERA
  if lidar and camera:
    return ICON_PINK
  if camera:
    return ICON_PURPLE
  return ICON_BLUE


# ------------------------------------------------------------------ 绘制原语
def _tri(x1, y1, x2, y2, x3, y3, color: rl.Color) -> None:
  """绘制实心三角形。

  raylib 的 ``DrawTriangle`` 受背面剔除影响（源端 nanovg 不受），而本仓库其它
  3D 绘制（例如 cluster 的 ``draw_model_ex``）会把剔除打开，于是绕序相反的三角形
  （向下的箭头、向左的箭头）会被整块剔除掉，表现为"只有部分箭头能显示"。

  这里做两层保护：
  1. 画之前临时关掉背面剔除（``cluster_renderer`` 同款做法）；
  2. 再以相反绕序画一遍同一个三角形——两个三角形完全重合，视觉上无差别，
     但无论当前剔除状态如何，总有一遍能通过，彻底避免"箭头整块消失"。
  """
  try:
    rl.rl_disable_backface_culling()
  except Exception:
    pass

  rl.draw_triangle(
    rl.Vector2(float(x1), float(y1)),
    rl.Vector2(float(x2), float(y2)),
    rl.Vector2(float(x3), float(y3)),
    color,
  )
  rl.draw_triangle(
    rl.Vector2(float(x1), float(y1)),
    rl.Vector2(float(x3), float(y3)),
    rl.Vector2(float(x2), float(y2)),
    color,
  )

  try:
    rl.rl_enable_backface_culling()
  except Exception:
    pass


def _arrow_up(cx, cy, color: rl.Color, gap: float = 0.0) -> None:
  half_w = ARROW_HEAD_WIDTH / 2
  length = ARROW_HEAD_LENGTH
  _tri(cx, cy - length - gap, cx - half_w, cy - gap, cx + half_w, cy - gap, color)


def _arrow_down(cx, cy, color: rl.Color, gap: float = 0.0) -> None:
  half_w = ARROW_HEAD_WIDTH / 2
  length = ARROW_HEAD_LENGTH
  _tri(cx, cy + length + gap, cx - half_w, cy + gap, cx + half_w, cy + gap, color)


def _arrow_center_up(cx, cy, color: rl.Color) -> None:
  """居中向上的小三角（源端第一行「原车前盲区」）。"""
  half_w = ARROW_HEAD_WIDTH / 2
  half_l = ARROW_HEAD_LENGTH / 2
  _tri(cx, cy - half_l, cx - half_w, cy + half_l, cx + half_w, cy + half_l, color)


def _arrow_center_down(cx, cy, color: rl.Color) -> None:
  """居中小向下的三角（源端第三行「原车后盲区」）。"""
  half_w = ARROW_HEAD_WIDTH / 2
  half_l = ARROW_HEAD_LENGTH / 2
  _tri(cx, cy + half_l, cx - half_w, cy - half_l, cx + half_w, cy - half_l, color)


def _arrow_side(cx, cy, to_left: bool, color: rl.Color) -> None:
  half_w = ARROW_HEAD_WIDTH / 2
  half_l = ARROW_HEAD_LENGTH / 2
  if to_left:
    _tri(cx - half_l, cy, cx + half_l, cy - half_w, cx + half_l, cy + half_w, color)
  else:
    _tri(cx + half_l, cy, cx - half_l, cy - half_w, cx - half_l, cy + half_w, color)


# 转向灯闪烁状态（模块级：两侧共用同一节奏，与 c3-dev 的单个 timer 行为一致）
_turn_lamp_timer = 0.0
_turn_lamp_alpha = 0.0


def turn_lamp_alpha() -> int:
  """转向灯图标当前亮度(0~255)：每 0.75s 冲到最亮，其余时间衰减到 20%。"""
  global _turn_lamp_timer, _turn_lamp_alpha
  now = time.monotonic()
  if now - _turn_lamp_timer > TURN_SIGNAL_BLINK_PERIOD:
    _turn_lamp_timer = now
    _turn_lamp_alpha = 255.0
  else:
    _turn_lamp_alpha += (255.0 * TURN_LAMP_DIM - _turn_lamp_alpha) * 0.12
  return int(max(0.0, min(255.0, _turn_lamp_alpha)))


def draw_turn_lamp(cx: float, cy: float, is_left: bool) -> None:
  """在雷达图标外侧画转向灯图标（c3-dev 同款贴图 + 心跳闪烁）。

  位置左右**对称**：左灯在左雷达图标左侧、右灯在右雷达图标右侧。两侧都要
  避开雷达图标外侧的四角距离文字（左侧右对齐往外、右侧左对齐往外），
  所以再让出 ``TURN_LAMP_TEXT_RESERVE``。
  """
  offset = CIRCLE_RADIUS + DIST_TEXT_OFFSET_X + TURN_LAMP_TEXT_RESERVE + TURN_LAMP_GAP
  icon_left = cx - offset - TURN_LAMP_W if is_left else cx + offset
  try:
    tex = gui_app.texture(TURN_LAMP_TEX, TURN_LAMP_W, TURN_LAMP_H, flip_x=not is_left)
  except Exception:
    return                      # 资产缺失时不影响其它绘制
  if tex is None:
    return
  rl.draw_texture_ex(tex, rl.Vector2(float(icon_left), float(cy - TURN_LAMP_H / 2)), 0.0, 1.0,
                     rl.Color(255, 255, 255, turn_lamp_alpha()))


def _draw_text_with_bg(text, x, y, font_size, color, align, font=None) -> None:
  """带半透明黑底的文字（移植自源端 ``drawTextWithBg``）。

  底框紧贴实际文字（随左/右对齐方式变化），四周留白与源端一致：
  左右 10px、上下 6px、圆角 6px，底色 RGBA(0,0,0,150)；文字垂直居中于 ``y``。
  """
  if font is None:
    font = gui_app.font(FontWeight.DISPLAY)

  draw_x, draw_y, size = get_text_draw_pos(font, text, x, y, font_size, align, 0.0)
  w = float(size.x) + DIST_BG_PADDING_X * 2
  h = float(size.y) + DIST_BG_PADDING_Y * 2
  rect = rl.Rectangle(
    float(draw_x - DIST_BG_PADDING_X), float(draw_y - DIST_BG_PADDING_Y), w, h,
  )
  roundness = min(0.5, DIST_BG_ROUND_PX / max(1.0, min(w, h) * 0.5))
  rl.draw_rectangle_rounded(rect, roundness, 8, DIST_BG_COLOR)

  draw_text_ui_style(
    text, x, y, font_size, color,
    font=font, border_width=2.0, shadow_offset=4.0, align=align, y_offset=0.0,
  )


# ------------------------------------------------------------------ 参数缓存
_param_cache: dict = {}


def _cached_int_param(key: str, default: int, ttl: float = 1.0) -> int:
  """带 TTL 的参数读取，避免每帧访问 Params。"""
  now = time.monotonic()
  hit = _param_cache.get(key)
  if hit is not None and now - hit[0] < ttl:
    return hit[1]
  try:
    from openpilot.selfdrive.ui.ui_state import ui_state

    value = int(ui_state.params.get_int(key))
  except Exception:
    value = default
  _param_cache[key] = (now, value)
  return value


def _icon_solid(cx, cy, r, fill) -> None:
  """不透明的实心图标圆（盲区状态用）。

  颜色本身已是全不透明，直接画即可：之前在实心圆上叠的深色描边/底衬
  既看不出效果也不好看，已去掉。
  """
  rl.draw_circle(int(cx), int(cy), int(r), fill)


def _circle_ring(cx, cy, r, color, width: int = RING_WIDTH) -> None:
  """不透明浅色底 + 粗彩色环（非盲区但有雷达目标时用）。

  浅底色固定，圈的对比度不随摄像头画面变化；粗圈比原来的细双环醒目。
  """
  rl.draw_circle(int(cx), int(cy), int(r), RING_BACKDROP)
  center = rl.Vector2(float(cx), float(cy))
  rl.draw_ring(center, max(0.0, r - width), r, 0, 360, 48, color)


# ------------------------------------------------------------------ 顶部图标面板
def draw_bsd_panel(sm, rect: rl.Rectangle, font=None, show_lane_info: int | None = None) -> None:
  """在屏幕顶部中央绘制盲区图标与四角距离（对应源端 ``draw()`` 的圆形部分）。

  三行布局（与源端一致，行数固定以保证图标位置稳定）：

  * 第一行  原车前盲区（黄圆 + 向上红箭头）；若只有原车雷达侧前目标、未判为盲区，
            则画"浅色底 + 粗黄圈"（不带箭头），并在圆圈旁显示该目标的纵向距离
            （格式/颜色/黑底与第二行的四角激光距离一致）
  * 第二行  雷达/摄像头盲区圆 + 箭头 + 在线"浅色底 + 粗蓝圈" + 实线黄条，两侧为四角距离
  * 第三行  原车后盲区（红圆 + 向下黄箭头）
  """
  # 调试用截屏钩子放在最前面：这样即使本次没有数据（面板不绘制）也能拍到画面。
  #   C3 上没有 ffmpeg/scrot 等截图命令，需要看真实画面时：
  #   ssh comma@<ip> "touch /tmp/shoot"  -> UI 把当前画面存成 /tmp/shot_<毫秒>.png
  if os.path.exists(SHOOT_FLAG):
    try:
      os.remove(SHOOT_FLAG)
      rl.take_screenshot(f"/tmp/shot_{int(time.time() * 1000)}.png")
    except Exception:
      pass

  view = read_amapnavi(sm)
  if view is None:
    return

  if show_lane_info is None:
    show_lane_info = _cached_int_param("ShowLaneInfo", 1)

  stock = _stock_blindspots(sm)
  front = _front_blind(sm)
  leads = _radar_side_leads(sm)

  nothing_to_show = not (
    view.blind("left") or view.blind("right") or view.left_device or view.right_device
    or view.has_distance or stock["left"] or stock["right"] or front["left"] or front["right"]
    or leads["left"][0] or leads["right"][0]
  )
  if nothing_to_show:
    return

  center_x = int(rect.x + rect.width / 2)
  r = CIRCLE_RADIUS
  top_y = TOP_Y

  # ---------------- 第一行：原车前盲区 / 原车雷达侧前距离 ----------------
  for side in SIDES:
    is_left = side == "left"
    sign = -1 if is_left else 1
    cx = center_x + sign * HORIZONTAL_OFFSET
    cy = top_y + r
    lead_valid, lead_dist = leads[side]

    if front[side]:
      # 判定为盲区 → 原来的图标（黄圆 + 向上红箭头）
      _icon_solid(cx, cy, r, ICON_YELLOW)
      if show_lane_info >= 1:
        _arrow_center_up(cx, cy, ARROW_RED)
    elif lead_valid:
      # 有原车雷达侧前目标但未判为盲区 → 浅色底 + 粗黄圈（不带箭头）
      _circle_ring(cx, cy, r, LINE_YELLOW)

    if lead_valid:
      # 距离文字：格式/颜色/黑底与第二行四角激光距离保持一致
      text_x = cx + sign * (r + DIST_TEXT_OFFSET_X)
      align = "right_center" if is_left else "left_center"
      _draw_text_with_bg(
        f"{lead_dist:.1f}", text_x, cy, DIST_FONT_SIZE, DIST_TEXT_YELLOW, align, font,
      )
  top_y += VERTICAL_SPACING

  # ---------------- 第二行：距离 + 雷达/摄像头图标 ----------------
  for side in SIDES:
    is_left = side == "left"
    sign = -1 if is_left else 1
    cx = center_x + sign * HORIZONTAL_OFFSET
    cy = top_y + r

    # 四角距离文字（左前/左后 或 右前/右后）
    front_corner, rear_corner = ("lf", "lb") if is_left else ("rf", "rb")
    f_valid, f_dist = view.distances.get(front_corner, (0, 0.0))
    r_valid, r_dist = view.distances.get(rear_corner, (0, 0.0))
    if f_valid or r_valid:
      text_x = cx + sign * (r + DIST_TEXT_OFFSET_X)
      align = "right_center" if is_left else "left_center"
      if f_valid:
        _draw_text_with_bg(
          f"{f_dist:.1f}", text_x, cy - DIST_TEXT_OFFSET_Y, DIST_FONT_SIZE, DIST_TEXT_YELLOW, align, font,
        )
      if r_valid:
        _draw_text_with_bg(
          f"{r_dist:.1f}", text_x, cy + DIST_TEXT_OFFSET_Y, DIST_FONT_SIZE, DIST_TEXT_YELLOW, align, font,
        )

    # 雷达 / 摄像头盲区圆圈
    blind = view.blind(side)
    if blind & ~BLIND_SOLID_LINE:
      _icon_solid(cx, cy, r, icon_color(blind))
      if blind & BLIND_FRONT:
        _arrow_up(cx, cy, ARROW_RED, ARROW_GAP)
      if blind & BLIND_REAR:
        _arrow_down(cx, cy, ARROW_RED, ARROW_GAP)
      if not (blind & (BLIND_FRONT | BLIND_REAR)):
        _arrow_side(cx, cy, is_left, ARROW_RED)
    elif view.device(side) & DEVICE_LIDAR:
      # 设备在线但无盲区：浅色底 + 粗蓝圈（原来是外蓝内黄细双环，细环在花
      # 背景上容易糊；改成浅底粗圈后，圈色永远落在固定底色上）
      _circle_ring(cx, cy, r, LINE_BLUE)

    # 实线：靠近车辆一侧的黄色竖条
    if blind & BLIND_SOLID_LINE:
      bar_x = cx + r + DIST_TEXT_OFFSET_X if is_left else cx - r - DIST_TEXT_OFFSET_X - SOLID_BAR_WIDTH
      rl.draw_rectangle(int(bar_x), int(cy - r), SOLID_BAR_WIDTH, int(r * 2), LINE_YELLOW)

    # 转向灯图标：外挂转向灯板**实际**在打该侧灯时（amapNavi.extBlinker）才显示，
    #   样式/闪烁与 c3-dev 一致（贴图 + 0.75s 心跳），放在对应雷达图标左侧。
    lamp = view.ext_blinker
    lamp_on = (lamp == BLINKER_LEFT) if is_left else (lamp == BLINKER_RIGHT)
    if lamp_on:
      draw_turn_lamp(cx, cy, is_left)
  top_y += VERTICAL_SPACING

  # ---------------- 第三行：原车后盲区 ----------------
  for side in SIDES:
    if not stock[side]:
      continue
    cx = center_x - HORIZONTAL_OFFSET if side == "left" else center_x + HORIZONTAL_OFFSET
    cy = top_y + r
    _icon_solid(cx, cy, r, ICON_RED)
    if show_lane_info >= 1:
      _arrow_center_down(cx, cy, ARROW_YELLOW)


# ------------------------------------------------------------------ E 徽标
def draw_ext_state_badge(count: int, dx: float, dy: float, font=None) -> None:
  """外挂客户端数量徽标（源端在 N/M 徽标右侧的 "E" 方框）。

  有客户端时绿色，无客户端时红色，中间显示数量。
  """
  x = dx + EXT_BADGE_OFFSET_X
  y = dy - 38
  rect = rl.Rectangle(float(x), float(y), float(EXT_BADGE_WIDTH), float(EXT_BADGE_HEIGHT))
  fill = EXT_BADGE_OK if count > 0 else EXT_BADGE_IDLE

  rl.draw_rectangle_rounded(rect, 0.25, 8, fill)
  rl.draw_rectangle_rounded_lines_ex(rect, 0.25, 8, 2, rl.WHITE)
  draw_text_ui_style(
    str(int(count)), x + EXT_BADGE_WIDTH / 2, dy, 40, rl.WHITE,
    font=font, border_width=2.0, shadow_offset=4.0, align="center_bottom",
  )


def ext_state_from(sm) -> int:
  """取当前外挂客户端数量（amapNavi 不可用时返回 0）。"""
  view = read_amapnavi(sm)
  return view.ext_state if view is not None else 0


# ------------------------------------------------------------------ 变道护栏
def pre_lane_change_sides(sm) -> dict:
  """当前处于「变道准备(preLaneChange)」的方向。

  对应源端 ``leftLaneChange`` / ``rightLaneChange``（只认 preLaneChange，不含起步中）。
  """
  out = {"left": False, "right": False}
  try:
    if sm.valid['modelV2']:
      meta = sm['modelV2'].meta
      if meta.laneChangeState == LaneChangeState.preLaneChange:
        direction = str(meta.laneChangeDirection).lower()
        out["left"] = "left" in direction
        out["right"] = "right" in direction
  except Exception:
    pass
  return out


def barrier_visible_sides(sm) -> dict:
  """护栏是否显示（源端 ``if(leftLaneChange || show_lane_info == 2)``）。

  ``ShowLaneInfo >= 2`` 时强制两侧常显；否则只在对应侧处于变道准备时显示。
  """
  if _cached_int_param("ShowLaneInfo", 1) >= 2:
    return {"left": True, "right": True}
  return pre_lane_change_sides(sm)


def draw_barriers_c3(renderer, sm) -> None:
  """c3（大屏）渲染器：补充 amapnavi 的变道护栏着色。

  复用渲染器自身的护栏构建与绘制方法，因此这里只做配色决策。
  """
  view = read_amapnavi(sm)
  if view is None:
    return
  try:
    if not sm.valid['modelV2']:
      return
    front = _front_blind(sm)
    visible = barrier_visible_sides(sm)
    for idx, side in enumerate(SIDES):
      if not visible[side]:
        continue
      color = barrier_color(view.blind(side), front[side])
      if color is None:
        continue
      renderer._update_blind_spot_barriers_carrot(sm, update_left=(idx == 0), update_right=(idx == 1))
      renderer._draw_blind_spot_segments_carrot(renderer._carrot_lane_barrier_vertices[idx], color)
  except Exception:
    # UI 绘制不应因为数据缺失而中断
    return


def draw_barriers_mici(renderer, sm) -> None:
  """mici（小屏）渲染器：补充 amapnavi 的变道护栏着色。"""
  view = read_amapnavi(sm)
  if view is None:
    return
  try:
    if not sm.valid['modelV2']:
      return
    points = renderer._path.raw_points
    if points.shape[0] < 2:
      return
    front = _front_blind(sm)
    visible = barrier_visible_sides(sm)
    max_idx = renderer._get_path_length_idx(points[:, 0], 40.0)
    for side, shift in (("left", -1.7), ("right", 1.7)):
      if not visible[side]:
        continue
      color = barrier_color(view.blind(side), front[side])
      if color is None:
        continue
      polygon = project_blindspot_barrier(
        points[:max_idx + 1], shift, renderer._car_space_transform, renderer._clip_region,
      )
      for quad in blindspot_barrier_quads(polygon):
        draw_polygon(renderer._rect, quad, color)
  except Exception:
    return
