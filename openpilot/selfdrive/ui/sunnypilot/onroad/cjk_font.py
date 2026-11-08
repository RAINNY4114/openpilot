"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
"""
[ROADNAME_CJK 20261027] 动态中文文本（道路名等）用的完整 CJK 字体。

为什么需要单独一套字体
----------------------
UI 的 CJK 字体是**按码点预子集**加载的：

  application.py:709 fallback_font()
      chars = ASCII + EXTRA_FONT_CHARS + app_<lang>.po 的全部字符
      load_font_ex(NotoSansCJK<lang>-Regular.otf, 48, chars, len(chars))

实测 `app_zh-CHS.po` 里有 308 个汉字，而 `assets/fonts/NotoSansCJKsc-Regular.otf`
只有 **459 个字形** —— 这个文件是「刚好够翻译用」的构建期子集，不是完整字体。
（`unifont.otf` 58910 字形、`NotoSansSC-Regular.otf` 30791 字形才是全的。）

道路名来自 mapd/OSM，是**动态文本**，翻译文件里永远不会有「恩」「波」「桥」「路」
⇒ 不在码点表里 ⇒ 没有字形 ⇒ raylib 画成 '?'。
实测 route 8 一条路上出现 7 个道路名，17 个汉字里 14 个缺字形，
只有 大 / 道 / 公 侥幸命中 ⇒ HUD 顶部显示成 `??大道`（实际是 `恩波大道`）。

借鉴 ford 分支的做法
--------------------
ford 分支（`selfdrive/assets/fonts/process.py`）的字体策略是：**字体文件随仓库走**
（`assets/fonts/`），并且用 `unifont.otf` 兜底（`UNIFONT_LANGUAGES` 把 zh-CHS 等
6 种语言的字符全烘进 unifont 图集）。它压根没有 NotoSansCJK 子集。

这条分支（staging-chestnut）的 UI 是运行时 `load_font_ex` 加载 .ttf/.otf，不消费
ford 那套预烘 `.fnt`/`.png`，所以搬的是**策略**不是脚本。落地成一条**四级字体梯子**：

  1. `assets/fonts/NotoSansSC-Regular.otf`（8.0 MB / 30791 字形）—— 跟 ford 一样随
     仓库走。注意它是 **untracked**（设备上没有 git 凭据，没法提交），
     `git clean -fdx` 会把它删掉。
  2. `/usr/share/fonts/NotoSansSC-Regular.otf` —— AGNOS 自带的同一份字体，
     仓库那份被清掉时的兜底。
  3. `assets/fonts/unifont.otf`（5.1 MB / 58910 字形，全 BMP）—— **tracked in git**，
     是唯一能扛住 `git clean` 的候选，所以留作最后一层。
  4. UI 子集字体（459 字形）—— 上面全都不行时由调用方退回，会缺字但绝不崩。

按当前字符串的码点现建字体 + 缓存，一条路线上路名也就十来个。

为什么**不做**运行时字形覆盖检测
--------------------------------
最初用 `rl.get_glyph_index(font, ord(c)) == 0` 判断缺字，**这是错的**：
`LoadFontEx(path, size, codepoints, count)` 会给**每个请求的码点**都建一条 glyph
记录（`glyphs[i].value = codepoints[i]`），缺字时只是把 .notdef 的位图填进去。
于是 `GetGlyphIndex` 永远能命中，检测恒为「不缺」，兜底网永远不触发。

实测证据（2026-10-27）：`䶶`（U+4DB6）只存在于 unifont，NotoSansSC 里没有；
用它建字体后 `get_glyph_index` 仍返回非 0 ⇒ 代码判定"已覆盖"并选了主字体。

所以字形覆盖改成**部署期**用 fontTools 核（见 `66_font_from_ford/check_coverage_v2.py`），
运行期只按「能不能加载」排序 —— 加载成功/失败是运行期唯一能可靠判断的东西。

两个必须注意的坑
----------------
1. **绘制必须绕开被 patch 过的 `rl.draw_text_ex`**。
   `application.py:_patch_text_functions` 把它换成了
   `font_fallback(font)` + `font_size * FONT_SCALE` 的版本，
   而 `font_fallback()` 会把我们传进去的字体**强制换回子集字体** ⇒ 又变回 '?'。
   所以这里走 `rl._orig_draw_text_ex`，并**自己乘 FONT_SCALE**。
2. **`measure_text_cached()` 同样会做 `font_fallback()`**，也不能用。
   测量必须用 `rl.measure_text_ex`（这个函数没被 patch）。
"""
import os

import pyray as rl

from openpilot.common.swaglog import cloudlog

# 本文件在 <root>/selfdrive/ui/sunnypilot/onroad/，往上三级就是 selfdrive/
_HERE = os.path.dirname(os.path.abspath(__file__))
_ASSETS_FONTS = os.path.normpath(os.path.join(_HERE, "..", "..", "..", "assets", "fonts"))

# 完整 CJK 字体候选，按顺序试。第一个是仓库自带的（ford 风格：字体随仓库走），
# 第二个是系统字体 —— 万一 `git clean -fdx` 把仓库里那份冲掉。
CJK_FONT_CANDIDATES = (
  os.path.join(_ASSETS_FONTS, "NotoSansSC-Regular.otf"),
  "/usr/share/fonts/NotoSansSC-Regular.otf",
)

# 兜底网：unifont 全 BMP 覆盖（58910 字形），而且它是 git tracked 的，
# 是唯一能扛住 `git clean` 的候选。
NET_FONT_CANDIDATES = (
  os.path.join(_ASSETS_FONTS, "unifont.otf"),
)

# 栅格化基准字号。实际绘制是 46*1.5=69 px，留一点余量让缩小时更锐利。
CJK_FONT_BASE_SIZE = 128
CJK_FONT_CACHE_MAX = 8

_ASCII = set(range(32, 127))
# key = 码点元组（排序后），保证同一串字无论出现顺序都命中同一份缓存
# value = (font, 用的哪个字体文件)
_fonts: dict[tuple[int, ...], tuple[rl.Font, str]] = {}


def codepoints_for(text: str) -> tuple[int, ...]:
  """text 用到的码点 + 整张 ASCII 表。

  为什么要带整张 ASCII：道路名里可能有数字和标点（"2号路"、"G320"），
  数字必须跟中文一起被栅格化，否则 2 会缺字。
  """
  return tuple(sorted({ord(c) for c in text} | _ASCII))


def cache_size() -> int:
  return len(_fonts)


def clear_cache() -> None:
  for font, _ in _fonts.values():
    try:
      rl.unload_font(font)
    except Exception:
      pass
  _fonts.clear()


def _evict_oldest() -> None:
  if not _fonts:
    return
  old_cps = next(iter(_fonts))
  old, _ = _fonts.pop(old_cps)
  try:
    rl.unload_font(old)
  except Exception:
    pass


def _as_tuple(paths) -> tuple[str, ...]:
  """把候选列表规整成 tuple。

  防的是「单元素元组漏逗号」：写成 `("a.otf")` 就是 str，
  `for p in paths` 会**逐字符**遍历，而 Windows 上 '\\' 恰好是存在的根路径
  ⇒ 会拿根目录当字体文件去 load，行为荒谬且无报错。宁可在这里挡掉。
  """
  if isinstance(paths, str):
    return (paths,)
  return tuple(paths)


def _build(path: str, cps: tuple[int, ...]):
  """按 cps 从 path 建一份字体；失败返回 None。"""
  if not os.path.exists(path):
    return None
  try:
    buf = rl.ffi.new("int[]", list(cps))
    font = rl.load_font_ex(path, CJK_FONT_BASE_SIZE,
                           rl.ffi.cast("int *", buf), len(cps))
  except Exception as e:
    cloudlog.warning(f"[ROADNAME_CJK] load_font_ex failed for {path}: {e!r}")
    return None

  # raylib 载入失败时 texture.id 会是 0，此时画出来是空白
  texture = getattr(font, "texture", None)
  if texture is None or getattr(texture, "id", 0) == 0:
    cloudlog.warning(f"[ROADNAME_CJK] empty texture for {path}")
    return None

  try:
    rl.gen_texture_mipmaps(texture)
    rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)
  except Exception:
    pass
  return font


def _pick(cps: tuple[int, ...]):
  """按梯子顺序取第一份**能加载**的字体，返回 (font, path)；全失败返回 None。

  注意这里只判断「能不能加载」，不判断「覆盖不覆盖」—— 理由见模块 docstring
  的「为什么不做运行时字形覆盖检测」。
  """
  for path in _as_tuple(CJK_FONT_CANDIDATES) + _as_tuple(NET_FONT_CANDIDATES):
    font = _build(path, cps)
    if font is not None:
      return font, path
  return None


def get_font(text: str):
  """取（或构建）text 可用的字体。失败返回 None，调用方自行兜底。"""
  if not text:
    return None

  cps = codepoints_for(text)
  hit = _fonts.get(cps)
  if hit is not None:
    return hit[0]

  picked = _pick(cps)
  if picked is None:
    cloudlog.warning("[ROADNAME_CJK] no usable CJK font found")
    return None

  font, path = picked
  while len(_fonts) >= CJK_FONT_CACHE_MAX:
    _evict_oldest()
  _fonts[cps] = (font, path)
  # 打**完整路径**而不是 basename：两个候选的文件名都叫 NotoSansSC-Regular.otf，
  # 只打文件名就分不清用的是仓库自带那份还是系统那份。
  cloudlog.info(f"[ROADNAME_CJK] built font from {path} "
                f"for {len(cps)} codepoints: {text!r}")
  return font


def measure(font, text: str, px: float) -> rl.Vector2:
  """测量。px 必须是**已乘过 FONT_SCALE 的实际像素大小**。"""
  return rl.measure_text_ex(font, text, px, 0)


def draw(font, text: str, position, px: float, color) -> None:
  """绘制。必须走未 patch 的原始实现，理由见模块 docstring 第 1 条。"""
  impl = getattr(rl, "_orig_draw_text_ex", None) or rl.draw_text_ex
  impl(font, text, position, px, 0, color)


# 自检标志文件。刻意不用 Params：新增 param 键必须重编 libparams_c.so，
# 否则会 UnknownKeyName 卡 comma logo（踩过）。一个文件就够。
SELFCHECK_FLAG = "/data/roadname_font_selfcheck"


def maybe_selfcheck(text: str = "恩波大道") -> bool | None:
  """存在 SELFCHECK_FLAG 时，在真实 GL 上下文里试建一次字体并写日志。

  用途：**不用上路**就能确认这台设备的 raylib 能加载完整 CJK 字体
  （CFF/OTF 解析 + 纹理上传都在真实进程里跑一遍）。

      touch /data/roadname_font_selfcheck && sudo systemctl restart comma
      grep ROADNAME_CJK /data/log/swaglog.*   # 看 selfcheck ok
      rm /data/roadname_font_selfcheck        # 下次开机不再自检

  标志文件里**有内容**时，内容就当作要测的字符串（取第一行、最多 64 字符），
  这样可以拿真实路名或任意串单独跑一遍，不用改代码：

      printf '\\xe4\\xb6\\xb6' > /data/roadname_font_selfcheck   # 测 䶶 U+4DB6

  ⚠️ 自检只能证明「字体加载成功」，**不能**证明「字形覆盖完整」—— 见模块
  docstring 的「为什么不做运行时字形覆盖检测」。覆盖要用 fontTools 离线核。

  返回 None 表示没触发；True/False 是自检结果。
  """
  if not os.path.exists(SELFCHECK_FLAG):
    return None
  try:
    with open(SELFCHECK_FLAG, encoding="utf-8", errors="replace") as f:
      custom = f.readline().strip()
    if custom:
      text = custom[:64]
  except OSError:
    pass
  font = get_font(text)
  if font is None:
    cloudlog.error("[ROADNAME_CJK] selfcheck FAILED: 完整 CJK 字体加载失败，"
                   "道路名会退回子集字体（继续显示 '?'）")
    return False
  glyphs = getattr(font, "glyphCount", None)
  cloudlog.info(f"[ROADNAME_CJK] selfcheck ok: {text!r} 字体构建成功"
                f"（glyphCount={glyphs}）")
  return True
