from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import simple_item, toggle_item, spin_button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.multilang import tr


class LincolnLayout(Widget):
  """
  Placeholder layout for future Lincoln-specific settings.
  Currently shows a simple message so the panel can be wired up in the UI.
  """

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._scroller = Scroller([
      simple_item(title=lambda: tr("### Lincoln Blindspot Voice Alerts ###")),
      toggle_item(
        title=lambda: tr("Blindspot Voice Alert"),
        description=lambda: tr("Play left/right voice prompts when blindspot sensors detect a vehicle."),
        initial_state=self._params.get_bool("dp_lincoln_bsm_voice_enabled"),
        callback=lambda val: self._params.put_bool("dp_lincoln_bsm_voice_enabled", val),
      ),
      spin_button_item(
        title=lambda: tr("Voice repeat interval"),
        description=lambda: tr("Minimum seconds between consecutive blindspot alerts."),
        initial_value=int(self._params.get("dp_lincoln_bsm_voice_interval_sec") or 3),
        callback=lambda val: self._params.put("dp_lincoln_bsm_voice_interval_sec", int(val)),
        min_val=1,
        max_val=10,
        step=1,
        suffix=tr(" sec"),
      ),
      spin_button_item(
        title=lambda: tr("Voice volume"),
        description=lambda: tr("Playback volume for blindspot alerts (percentage)."),
        initial_value=int(self._params.get("dp_lincoln_bsm_voice_volume_pct") or 100),
        callback=lambda val: self._params.put("dp_lincoln_bsm_voice_volume_pct", int(val)),
        min_val=20,
        max_val=100,
        step=5,
        suffix=tr(" %"),
      ),
    ], line_separator=True, spacing=0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()

  def hide_event(self):
    self._scroller.hide_event()
