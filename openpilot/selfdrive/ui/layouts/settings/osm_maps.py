import datetime
import json
import os
import re
import shutil
import subprocess
import threading
import time
from enum import IntEnum

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.button import ButtonRadio
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog, alert_dialog
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.list_view import (
  button_item,
  multiple_button_item,
  simple_item,
  text_item,
  toggle_item,
)
from openpilot.system.ui.widgets.scroller_tici import Scroller

OSM_BASE_DIR = "/data/media/0/osm"
OSM_OFFLINE_DIR = f"{OSM_BASE_DIR}/offline"


class Page(IntEnum):
  MAIN = 0
  COUNTRIES = 1
  STATES = 2


US_MIDWEST = {
  "IL": "Illinois", "IN": "Indiana", "IA": "Iowa",
  "KS": "Kansas", "MI": "Michigan", "MN": "Minnesota",
  "MO": "Missouri", "NE": "Nebraska", "ND": "North Dakota",
  "OH": "Ohio", "SD": "South Dakota", "WI": "Wisconsin",
}
US_NORTHEAST = {
  "CT": "Connecticut", "ME": "Maine", "MA": "Massachusetts",
  "NH": "New Hampshire", "NJ": "New Jersey", "NY": "New York",
  "PA": "Pennsylvania", "RI": "Rhode Island", "VT": "Vermont",
}
US_SOUTH = {
  "AL": "Alabama", "AR": "Arkansas", "DE": "Delaware",
  "DC": "District of Columbia", "FL": "Florida", "GA": "Georgia",
  "KY": "Kentucky", "LA": "Louisiana", "MD": "Maryland",
  "MS": "Mississippi", "NC": "North Carolina", "OK": "Oklahoma",
  "SC": "South Carolina", "TN": "Tennessee", "TX": "Texas",
  "VA": "Virginia", "WV": "West Virginia",
}
US_WEST = {
  "AK": "Alaska", "AZ": "Arizona", "CA": "California",
  "CO": "Colorado", "HI": "Hawaii", "ID": "Idaho",
  "MT": "Montana", "NV": "Nevada", "NM": "New Mexico",
  "OR": "Oregon", "UT": "Utah", "WA": "Washington",
  "WY": "Wyoming",
}
US_TERRITORIES = {
  "AS": "American Samoa", "GU": "Guam", "MP": "Northern Mariana Islands",
  "PR": "Puerto Rico", "VI": "Virgin Islands",
}

AFRICA = {
  "DZ": "Algeria", "AO": "Angola", "BJ": "Benin",
  "BW": "Botswana", "BF": "Burkina Faso", "BI": "Burundi",
  "CM": "Cameroon", "CF": "Central African Republic", "TD": "Chad",
  "KM": "Comoros", "CG": "Congo (Brazzaville)", "CD": "Congo (Kinshasa)",
  "DJ": "Djibouti", "EG": "Egypt", "GQ": "Equatorial Guinea",
  "ER": "Eritrea", "ET": "Ethiopia", "GA": "Gabon",
  "GM": "Gambia", "GH": "Ghana", "GN": "Guinea",
  "GW": "Guinea-Bissau", "CI": "Ivory Coast", "KE": "Kenya",
  "LS": "Lesotho", "LR": "Liberia", "LY": "Libya",
  "MG": "Madagascar", "MW": "Malawi", "ML": "Mali",
  "MR": "Mauritania", "MA": "Morocco", "MZ": "Mozambique",
  "NA": "Namibia", "NE": "Niger", "NG": "Nigeria",
  "RW": "Rwanda", "SN": "Senegal", "SL": "Sierra Leone",
  "SO": "Somalia", "ZA": "South Africa", "SS": "South Sudan",
  "SD": "Sudan", "SZ": "Swaziland", "TZ": "Tanzania",
  "TG": "Togo", "TN": "Tunisia", "UG": "Uganda",
  "ZM": "Zambia", "ZW": "Zimbabwe",
}
ANTARCTICA = {
  "AQ": "Antarctica",
}
ASIA = {
  "AF": "Afghanistan", "AM": "Armenia", "AZ": "Azerbaijan",
  "BH": "Bahrain", "BD": "Bangladesh", "BT": "Bhutan",
  "BN": "Brunei", "KH": "Cambodia", "CN": "China",
  "CY": "Cyprus", "TL": "East Timor", "HK": "Hong Kong",
  "IN": "India", "ID": "Indonesia", "IR": "Iran",
  "IQ": "Iraq", "IL": "Israel", "JP": "Japan",
  "JO": "Jordan", "KZ": "Kazakhstan", "KW": "Kuwait",
  "KG": "Kyrgyzstan", "LA": "Laos", "LB": "Lebanon",
  "MY": "Malaysia", "MV": "Maldives", "MO": "Macao",
  "MN": "Mongolia", "MM": "Myanmar", "NP": "Nepal",
  "KP": "North Korea", "OM": "Oman", "PK": "Pakistan",
  "PS": "Palestine", "PH": "Philippines", "QA": "Qatar",
  "RU": "Russia", "SA": "Saudi Arabia", "SG": "Singapore",
  "KR": "South Korea", "LK": "Sri Lanka", "SY": "Syria",
  "TW": "Taiwan", "TJ": "Tajikistan", "TH": "Thailand",
  "TR": "Turkey", "TM": "Turkmenistan", "AE": "United Arab Emirates",
  "UZ": "Uzbekistan", "VN": "Vietnam", "YE": "Yemen",
}
EUROPE = {
  "AL": "Albania", "AT": "Austria", "BY": "Belarus",
  "BE": "Belgium", "BA": "Bosnia and Herzegovina", "BG": "Bulgaria",
  "HR": "Croatia", "CZ": "Czech Republic", "DK": "Denmark",
  "EE": "Estonia", "FI": "Finland", "FR": "France",
  "GE": "Georgia", "DE": "Germany", "GR": "Greece",
  "HU": "Hungary", "IS": "Iceland", "IE": "Ireland",
  "IT": "Italy", "KZ": "Kazakhstan", "LV": "Latvia",
  "LT": "Lithuania", "LU": "Luxembourg", "MK": "Macedonia",
  "MD": "Moldova", "ME": "Montenegro", "NL": "Netherlands",
  "NO": "Norway", "PL": "Poland", "PT": "Portugal",
  "RO": "Romania", "RS": "Serbia", "SK": "Slovakia",
  "SI": "Slovenia", "ES": "Spain", "SE": "Sweden",
  "CH": "Switzerland", "TR": "Turkey", "UA": "Ukraine",
  "GB": "United Kingdom",
}
NORTH_AMERICA = {
  "BS": "Bahamas", "BZ": "Belize", "CA": "Canada",
  "CR": "Costa Rica", "CU": "Cuba", "DO": "Dominican Republic",
  "SV": "El Salvador", "GL": "Greenland", "GD": "Grenada",
  "GT": "Guatemala", "HT": "Haiti", "HN": "Honduras",
  "JM": "Jamaica", "MX": "Mexico", "NI": "Nicaragua",
  "PA": "Panama", "TT": "Trinidad and Tobago", "US": "United States",
}
OCEANIA = {
  "AU": "Australia", "FJ": "Fiji", "TF": "French Southern Territories",
  "NC": "New Caledonia", "NZ": "New Zealand", "PG": "Papua New Guinea",
  "SB": "Solomon Islands", "VU": "Vanuatu",
}
SOUTH_AMERICA = {
  "AR": "Argentina", "BO": "Bolivia", "BR": "Brazil",
  "CL": "Chile", "CO": "Colombia", "EC": "Ecuador",
  "FK": "Falkland Islands", "GY": "Guyana", "PY": "Paraguay",
  "PE": "Peru", "SR": "Suriname", "UY": "Uruguay",
  "VE": "Venezuela",
}


def _day_suffix(day: int) -> str:
  if day % 10 == 1 and day != 11:
    return "st"
  if day % 10 == 2 and day != 12:
    return "nd"
  if day % 10 == 3 and day != 13:
    return "rd"
  return "th"


def _format_current_date(now: datetime.datetime | None = None) -> str:
  now = now or datetime.datetime.now(datetime.timezone.utc).astimezone()
  suffix = _day_suffix(now.day)
  return now.strftime(f"%B {now.day}{suffix}, %Y")


def _format_elapsed_time(seconds: int) -> str:
  seconds = max(0, int(seconds))
  hours = seconds // 3600
  minutes = (seconds % 3600) // 60
  secs = seconds % 60

  parts: list[str] = []
  if hours > 0:
    parts.append(f"{hours} " + ("hour" if hours == 1 else "hours"))
  if minutes > 0:
    parts.append(f"{minutes} " + ("minute" if minutes == 1 else "minutes"))
  parts.append(f"{secs} " + ("second" if secs == 1 else "seconds"))
  return " ".join(parts).strip()


class SectionHeader(Widget):
  def __init__(self, title: str):
    super().__init__()
    self._title = title
    self._font = gui_app.font(FontWeight.BOLD)
    self.set_rect(rl.Rectangle(0, 0, 0, 90))

  def set_parent_rect(self, parent_rect: rl.Rectangle) -> None:
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width

  def _render(self, _):
    gui_label(self._rect, self._title, font_size=50, font_weight=FontWeight.BOLD,
              alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
              alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE)


class MapSelectionGrid(Widget):
  COLS = 3
  PADDING = 20
  ROW_SPACING = 20
  COL_SPACING = 20
  BUTTON_HEIGHT = 120

  def __init__(self, entries: dict[str, str], selection_type: str, params: Params):
    super().__init__()
    self._params = params
    self._selection_type = selection_type
    self._entries = dict(entries)
    self._keys = sorted(self._entries.keys())
    self._buttons: list[tuple[str, ButtonRadio]] = []
    self._check_icon = gui_app.texture("icons/checkmark.png", 50, 50)
    self._selected = self._load_selected()

    for code in self._keys:
      label = self._entries[code]
      btn = ButtonRadio(
        label,
        icon=self._check_icon,
        click_callback=None,
        font_size=46,
        text_alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
        border_radius=20,
        text_padding=35,
      )
      btn.selected = code in self._selected
      btn.set_click_callback(lambda c=code, b=btn: self._set_selected(c, not b.selected))
      self._buttons.append((code, btn))

    rows = (len(self._buttons) + self.COLS - 1) // self.COLS
    height = self.PADDING * 2 + rows * self.BUTTON_HEIGHT + max(0, rows - 1) * self.ROW_SPACING
    self.set_rect(rl.Rectangle(0, 0, 0, height))

  def set_touch_valid_callback(self, touch_callback):
    super().set_touch_valid_callback(touch_callback)
    for _, btn in self._buttons:
      btn.set_touch_valid_callback(touch_callback)

  def set_parent_rect(self, parent_rect: rl.Rectangle) -> None:
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width
    for _, btn in self._buttons:
      btn.set_parent_rect(parent_rect)

  def _load_selected(self) -> set[str]:
    selected: set[str] = set()
    raw = self._params.get("MapsSelected")
    if raw:
      try:
        obj = json.loads(raw)
        selected.update(str(x) for x in (obj.get(self._selection_type) or []) if isinstance(x, str))
      except Exception:
        pass
    return selected

  def _set_selected(self, code: str, enabled: bool) -> None:
    selected = set(self._load_selected())
    if enabled:
      selected.add(code)
    else:
      selected.discard(code)

    raw = self._params.get("MapsSelected") or ""
    try:
      obj = json.loads(raw) if raw else {}
    except Exception:
      obj = {}
    obj.setdefault("nations", [])
    obj.setdefault("states", [])
    obj[self._selection_type] = sorted(selected)
    self._params.put_nonblocking("MapsSelected", json.dumps(obj, separators=(",", ":")))

  def _render(self, rect: rl.Rectangle):
    usable_w = rect.width - self.PADDING * 2
    col_w = (usable_w - (self.COLS - 1) * self.COL_SPACING) / self.COLS

    for idx, (_code, btn) in enumerate(self._buttons):
      row = idx // self.COLS
      col = idx % self.COLS
      x = rect.x + self.PADDING + col * (col_w + self.COL_SPACING)
      y = rect.y + self.PADDING + row * (self.BUTTON_HEIGHT + self.ROW_SPACING)
      btn.render(rl.Rectangle(x, y, col_w, self.BUTTON_HEIGHT))


class OSMMapsLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._params_memory = Params("/dev/shm/params")

    self._page = Page.MAIN

    self._size_cache_bytes = 0
    self._size_cache_t = 0.0
    self._download_start_monotonic: float | None = None
    self._download_start_wall: datetime.datetime | None = None
    self._download_start_done = 0

    self._ensure_default_maps_selected()

    self._schedule_item = multiple_button_item(
      title=lambda: tr("Automatically Update Maps"),
      description=lambda: tr("<b>How often maps update</b> from \"OpenStreetMap (OSM)\" with the latest data. Weekly updates run every Sunday; monthly updates run on the 1st."),
      buttons=[lambda: tr("Manually"), lambda: tr("Weekly"), lambda: tr("Monthly")],
      selected_index=self._schedule_index(),
      callback=self._on_schedule_selected,
    )

    self._download_item = button_item(
      title=lambda: tr("Download Maps"),
      button_text=lambda: tr("CANCEL") if self._download_in_progress() else tr("DOWNLOAD"),
      description=lambda: tr("<b>Manually update your selected map sources</b> so map-based features have the latest data."),
      callback=self._on_download_or_cancel_pressed,
      enabled=self._download_button_enabled,
    )

    self._last_updated_item = text_item(
      title=lambda: tr("Last Updated"),
      value=lambda: self._params.get("LastMapsUpdate") or tr("Never"),
    )
    self._last_updated_item.set_visible(lambda: not self._download_in_progress())

    self._map_sources_item = multiple_button_item(
      title=lambda: tr("Map Sources"),
      description=lambda: tr("<b>Select the countries or U.S. states</b> to download for offline map data."),
      buttons=[lambda: tr("COUNTRIES"), lambda: tr("STATES")],
      selected_index=0,
      callback=self._open_sources_page,
    )
    self._map_sources_item.action_item.set_enabled(lambda: not self._download_in_progress())

    self._progress_item = text_item(
      title=lambda: tr("Progress"),
      value=lambda: self._progress_text(),
    )
    self._progress_item.set_visible(lambda: self._download_in_progress())

    self._elapsed_item = text_item(
      title=lambda: tr("Time Elapsed"),
      value=lambda: self._elapsed_text(),
    )
    self._elapsed_item.set_visible(lambda: self._download_in_progress())

    self._eta_item = text_item(
      title=lambda: tr("Time Remaining"),
      value=lambda: self._eta_text(),
    )
    self._eta_item.set_visible(lambda: self._download_in_progress())

    self._remove_maps_item = button_item(
      title=lambda: tr("Remove Maps"),
      button_text=lambda: tr("REMOVE"),
      description=lambda: tr("<b>Delete downloaded map data</b> to free up storage space."),
      callback=self._on_remove_prompt,
      enabled=lambda: (not self._download_in_progress()) and self._offline_maps_dir_exists(),
    )
    self._remove_maps_item.set_visible(lambda: (not self._download_in_progress()) and self._offline_maps_dir_exists())

    self._reset_downloader_item = button_item(
      title=lambda: tr("Reset Downloader"),
      button_text=lambda: tr("RESET"),
      description=lambda: tr("<b>Reset the map downloader.</b> Use this if downloads are stuck or failing. Your device will reboot."),
      callback=self._on_reset_prompt,
      enabled=lambda: not self._download_in_progress(),
    )
    self._reset_downloader_item.set_visible(lambda: not self._download_in_progress())

    self._storage_used_item = text_item(
      title=lambda: tr("Storage Used"),
      value=lambda: self._format_storage_used(self._offline_bytes()),
    )

    self._main_scroller = Scroller([
      simple_item(title=lambda: tr("### Offline Maps ###")),
      self._schedule_item,
      self._download_item,
      self._last_updated_item,
      self._map_sources_item,
      self._progress_item,
      self._elapsed_item,
      self._eta_item,
      self._remove_maps_item,
      self._reset_downloader_item,
      self._storage_used_item,
    ], line_separator=True, spacing=0)

    self._countries_scroller = Scroller(self._build_countries_items(), line_separator=False, spacing=0)
    self._states_scroller = Scroller(self._build_states_items(), line_separator=False, spacing=0)

    self._prev_download_in_progress = self._download_in_progress()

  def _render(self, rect):
    self._update_state()
    if self._page == Page.COUNTRIES:
      self._countries_scroller.render(rect)
    elif self._page == Page.STATES:
      self._states_scroller.render(rect)
    else:
      self._main_scroller.render(rect)

  def show_event(self):
    if self._page == Page.COUNTRIES:
      self._countries_scroller.show_event()
    elif self._page == Page.STATES:
      self._states_scroller.show_event()
    else:
      self._main_scroller.show_event()

  def hide_event(self):
    self._main_scroller.hide_event()
    self._countries_scroller.hide_event()
    self._states_scroller.hide_event()

  @staticmethod
  def _default_maps_selected() -> str:
    return json.dumps({"states": [], "nations": ["CN"]}, separators=(",", ":"))

  def _ensure_default_maps_selected(self) -> None:
    selected = self._params.get("MapsSelected")
    if not selected:
      self._params.put("MapsSelected", self._default_maps_selected())

  def _schedule_index(self) -> int:
    try:
      return max(0, min(2, int(self._params.get("PreferredSchedule") or 0)))
    except Exception:
      return 0

  def _on_schedule_selected(self, idx: int) -> None:
    self._params.put_nonblocking("PreferredSchedule", int(max(0, min(2, idx))))

  def _get_maps_selected(self) -> dict:
    raw = self._params.get("MapsSelected") or ""
    try:
      obj = json.loads(raw) if raw else {}
    except Exception:
      obj = {}
    if not isinstance(obj, dict):
      obj = {}
    obj.setdefault("nations", [])
    obj.setdefault("states", [])
    return obj

  def _has_maps_selected(self) -> bool:
    obj = self._get_maps_selected()
    return bool(obj.get("nations") or obj.get("states"))

  def _download_in_progress(self) -> bool:
    progress = self._params.get("OSMDownloadProgress")
    if progress:
      return True
    return bool(self._params_memory.get("OSMDownloadLocations"))

  def _download_button_enabled(self) -> bool:
    if self._download_in_progress():
      return True
    return self._has_maps_selected() and self._is_online() and self._is_parked()

  @staticmethod
  def _is_parked() -> bool:
    return not ui_state.started

  @staticmethod
  def _is_online() -> bool:
    try:
      from cereal import log
      NetworkType = log.DeviceState.NetworkType
      ds = ui_state.sm["deviceState"]
      return ds.networkType.raw != NetworkType.none
    except Exception:
      # If deviceState isn't available for any reason, don't block UI actions.
      return True

  def _parse_progress(self) -> dict:
    raw = self._params.get("OSMDownloadProgress") or ""
    if not raw:
      return {}
    try:
      return json.loads(raw)
    except Exception:
      m = re.search(r'"total_files"\s*:\s*(\d+).*"downloaded_files"\s*:\s*(\d+)', raw)
      if not m:
        return {}
      return {"total_files": int(m.group(1)), "downloaded_files": int(m.group(2))}

  def _progress_text(self) -> str:
    p = self._parse_progress()
    total = int(p.get("total_files", 0) or 0)
    done = int(p.get("downloaded_files", 0) or 0)
    if total <= 0:
      return tr("Calculating...")
    pct = int((done * 100) / max(1, total))
    return tr("{done} / {total} ({pct}%)").format(done=done, total=total, pct=pct)

  def _elapsed_text(self) -> str:
    if self._download_start_monotonic is None:
      return tr("Calculating...")
    return _format_elapsed_time(int(time.monotonic() - self._download_start_monotonic))

  def _eta_text(self) -> str:
    p = self._parse_progress()
    total = int(p.get("total_files", 0) or 0)
    done = int(p.get("downloaded_files", 0) or 0)
    if total <= 0 or done <= 0 or self._download_start_monotonic is None or self._download_start_wall is None:
      return tr("Calculating...")

    elapsed = max(1.0, time.monotonic() - self._download_start_monotonic)
    est_total = (elapsed * total) / max(1, done)
    remaining = max(0.0, est_total - elapsed)
    finish = self._download_start_wall + datetime.timedelta(seconds=est_total)
    finish_str = finish.strftime("%I:%M %p").lstrip("0")
    return f"{_format_elapsed_time(int(remaining))} ({finish_str})"

  def _offline_maps_dir_exists(self) -> bool:
    return os.path.isdir(OSM_OFFLINE_DIR)

  def _offline_maps_exist(self) -> bool:
    return self._offline_bytes() > 0

  def _offline_bytes(self) -> int:
    now = time.monotonic()
    if now - self._size_cache_t < 1.0:
      return self._size_cache_bytes

    total = 0
    try:
      for root, _, files in os.walk(OSM_OFFLINE_DIR):
        for name in files:
          p = os.path.join(root, name)
          try:
            total += os.path.getsize(p)
          except Exception:
            pass
    except Exception:
      pass

    self._size_cache_bytes = total
    self._size_cache_t = now
    return total

  @staticmethod
  def _format_storage_used(num_bytes: int) -> str:
    mb = 1024.0 * 1024.0
    gb = 1024.0 * mb
    if num_bytes <= 0:
      return tr("0 MB")
    if num_bytes >= gb:
      return f"{num_bytes / gb:.2f} {tr('GB')}"
    return f"{num_bytes / mb:.2f} {tr('MB')}"

  def _open_sources_page(self, idx: int) -> None:
    self._page = Page.COUNTRIES if idx == 0 else Page.STATES
    self.show_event()

  def _go_main(self) -> None:
    self._page = Page.MAIN
    self.show_event()

  def _build_countries_items(self) -> list[Widget]:
    back = button_item(title=lambda: tr("Back"), button_text=lambda: tr("BACK"), callback=self._go_main)
    items: list[Widget] = [
      simple_item(title=lambda: tr("### Countries ###")),
      back,
      SectionHeader(tr("Africa")),
      MapSelectionGrid(AFRICA, "nations", self._params),
      SectionHeader(tr("Antarctica")),
      MapSelectionGrid(ANTARCTICA, "nations", self._params),
      SectionHeader(tr("Asia")),
      MapSelectionGrid(ASIA, "nations", self._params),
      SectionHeader(tr("Europe")),
      MapSelectionGrid(EUROPE, "nations", self._params),
      SectionHeader(tr("North America")),
      MapSelectionGrid(NORTH_AMERICA, "nations", self._params),
      SectionHeader(tr("Oceania")),
      MapSelectionGrid(OCEANIA, "nations", self._params),
      SectionHeader(tr("South America")),
      MapSelectionGrid(SOUTH_AMERICA, "nations", self._params),
    ]
    return items

  def _build_states_items(self) -> list[Widget]:
    back = button_item(title=lambda: tr("Back"), button_text=lambda: tr("BACK"), callback=self._go_main)
    items: list[Widget] = [
      simple_item(title=lambda: tr("### States ###")),
      back,
      SectionHeader(tr("United States - Midwest")),
      MapSelectionGrid(US_MIDWEST, "states", self._params),
      SectionHeader(tr("United States - Northeast")),
      MapSelectionGrid(US_NORTHEAST, "states", self._params),
      SectionHeader(tr("United States - South")),
      MapSelectionGrid(US_SOUTH, "states", self._params),
      SectionHeader(tr("United States - West")),
      MapSelectionGrid(US_WEST, "states", self._params),
      SectionHeader(tr("United States - Territories")),
      MapSelectionGrid(US_TERRITORIES, "states", self._params),
    ]
    return items

  def _start_download(self) -> None:
    if not self._has_maps_selected():
      return

    self._params.remove("OSMDownloadProgress")
    self._params_memory.put("OSMDownloadLocations", json.dumps(self._get_maps_selected(), separators=(",", ":")))

  def _cancel_download(self) -> None:
    self._params.remove("OSMDownloadProgress")
    self._params_memory.remove("OSMDownloadLocations")
    try:
      subprocess.run(["pkill", "mapd"], check=False)
    except Exception:
      pass

  def _on_download_or_cancel_pressed(self) -> None:
    if self._download_in_progress():
      dialog = ConfirmDialog(tr("Cancel the download?"), tr("Yes"), cancel_text=tr("No"))
      gui_app.set_modal_overlay(dialog, callback=self._on_cancel_confirmed)
      return

    if ui_state.started:
      gui_app.set_modal_overlay(alert_dialog(tr("Download maps while driving is not supported.")))
      return

    self._start_download()

  def _on_cancel_confirmed(self, result: int) -> None:
    if result == DialogResult.CONFIRM:
      self._cancel_download()

  def _on_remove_prompt(self):
    dialog = ConfirmDialog(tr("Delete all downloaded maps?"), tr("Remove"))
    gui_app.set_modal_overlay(dialog, callback=self._on_remove_confirmed)

  def _on_remove_confirmed(self, result: int):
    if result != DialogResult.CONFIRM:
      return

    def _do_remove():
      self._cancel_download()
      try:
        shutil.rmtree(OSM_OFFLINE_DIR, ignore_errors=True)
      except Exception:
        pass
      self._size_cache_bytes = 0
      self._size_cache_t = 0.0

    threading.Thread(target=_do_remove, daemon=True).start()

  def _on_reset_prompt(self):
    dialog = ConfirmDialog(tr("Reset the map downloader? Your device will reboot afterward."), tr("Reset"))
    gui_app.set_modal_overlay(dialog, callback=self._on_reset_confirmed)

  def _on_reset_confirmed(self, result: int):
    if result != DialogResult.CONFIRM:
      return

    def _do_reset():
      self._cancel_download()
      try:
        shutil.rmtree(OSM_BASE_DIR, ignore_errors=True)
      except Exception:
        pass
      self._size_cache_bytes = 0
      self._size_cache_t = 0.0
      self._params.put_bool_nonblocking("DoReboot", True)

    threading.Thread(target=_do_reset, daemon=True).start()

  def _update_state(self):
    schedule_idx = self._schedule_index()
    self._schedule_item.action_item.set_selected_button(schedule_idx)

    downloading = self._download_in_progress()
    if downloading:
      # Match FrogPilot behavior: keep the screen awake during long downloads.
      device.reset_interactive_timeout(300)
    if downloading and not self._prev_download_in_progress:
      self._download_start_monotonic = time.monotonic()
      self._download_start_wall = datetime.datetime.now(datetime.timezone.utc).astimezone()
      p = self._parse_progress()
      self._download_start_done = int(p.get("downloaded_files", 0) or 0)

    if not downloading and self._prev_download_in_progress:
      self._download_start_monotonic = None
      self._download_start_wall = None
      self._download_start_done = 0

    # Auto-clear download trigger if mapd doesn't clear it
    if downloading:
      p = self._parse_progress()
      total = int(p.get("total_files", 0) or 0)
      done = int(p.get("downloaded_files", 0) or 0)
      if total > 0 and done >= total and self._offline_maps_exist():
        self._params_memory.remove("OSMDownloadLocations")
        self._params.remove("OSMDownloadProgress")
        self._params.put_nonblocking("LastMapsUpdate", _format_current_date())

    # Button value hints (mimic FrogPilot)
    value = ""
    if not downloading:
      if not self._is_online():
        value = tr("Offline...")
      elif ui_state.started:
        value = tr("Not parked")
    try:
      self._download_item.action_item.set_value(value)
    except Exception:
      pass

    self._prev_download_in_progress = downloading
