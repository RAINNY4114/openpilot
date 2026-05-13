from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import (
  simple_item,
  spin_button_item,
)
from openpilot.system.ui.widgets.scroller_tici import Scroller
from dragonpilot.selfdrive.ui.startup_logo import (
  STARTUP_LOGO_FORD_INDEX_KEY,
  STARTUP_LOGO_LINCOLN_INDEX_KEY,
  STARTUP_SPINNER_TRACK_INDEX_KEY,
)

MAX_STARTUP_LOGO_VARIANT = 20


class BrandingLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

    self._ford_logo_item = spin_button_item(
      title=lambda: tr("Ford logo index"),
      description=lambda: tr("0 = spinner_comma.png. >0 = fordX.png. When Ford is set above 0, Lincoln is automatically reset to 0."),
      initial_value=self._get_param_int(STARTUP_LOGO_FORD_INDEX_KEY, 0),
      callback=self._on_ford_logo_index_changed,
      min_val=0,
      max_val=MAX_STARTUP_LOGO_VARIANT,
      step=1,
    )

    self._lincoln_logo_item = spin_button_item(
      title=lambda: tr("Lincoln logo index"),
      description=lambda: tr("0 = spinner_comma.png. >0 = lincolnX.png. When Lincoln is set above 0, Ford is automatically reset to 0."),
      initial_value=self._get_param_int(STARTUP_LOGO_LINCOLN_INDEX_KEY, 0),
      callback=self._on_lincoln_logo_index_changed,
      min_val=0,
      max_val=MAX_STARTUP_LOGO_VARIANT,
      step=1,
    )

    self._scroller = Scroller([
      simple_item(title=lambda: tr("### Startup Logo ###")),
      self._ford_logo_item,
      self._lincoln_logo_item,
      simple_item(title=lambda: tr("### Spinner Animation ###")),
      spin_button_item(
        title=lambda: tr("Spinner animation"),
        description=lambda: tr("0 = spinner_track.png. 1 = 1spinner_track.png. Put custom files in selfdrive/assets/images/."),
        initial_value=self._get_param_int(STARTUP_SPINNER_TRACK_INDEX_KEY, 0),
        callback=lambda val: self._params.put(STARTUP_SPINNER_TRACK_INDEX_KEY, int(val)),
        min_val=0,
        max_val=MAX_STARTUP_LOGO_VARIANT,
        step=1,
      ),
    ], line_separator=True, spacing=0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._sync_logo_controls()
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()

  def _on_ford_logo_index_changed(self, val: int) -> None:
    self._params.put(STARTUP_LOGO_FORD_INDEX_KEY, int(val))
    if int(val) > 0:
      self._params.put(STARTUP_LOGO_LINCOLN_INDEX_KEY, 0)
      self._lincoln_logo_item.action_item.set_value(0)

  def _on_lincoln_logo_index_changed(self, val: int) -> None:
    self._params.put(STARTUP_LOGO_LINCOLN_INDEX_KEY, int(val))
    if int(val) > 0:
      self._params.put(STARTUP_LOGO_FORD_INDEX_KEY, 0)
      self._ford_logo_item.action_item.set_value(0)

  def _sync_logo_controls(self) -> None:
    self._ford_logo_item.action_item.set_value(self._get_param_int(STARTUP_LOGO_FORD_INDEX_KEY, 0))
    self._lincoln_logo_item.action_item.set_value(self._get_param_int(STARTUP_LOGO_LINCOLN_INDEX_KEY, 0))

  def _get_param_int(self, key: str, default: int) -> int:
    val = self._params.get(key)
    if not val:
      return default
    try:
      return int(val)
    except (TypeError, ValueError):
      return default
