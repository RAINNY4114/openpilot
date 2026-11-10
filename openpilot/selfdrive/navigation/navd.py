#!/usr/bin/env python3
import os
import time

import numpy as np
import requests

import openpilot.cereal.messaging as messaging
from openpilot.common.api import API_HOST, Api
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.navigation.helpers import latest_gps_source
from openpilot.selfdrive.navigation.route import along_path, cumulative_distance, curvature_profile, local_meters
from openpilot.selfdrive.navigation.route import speed_cap_from_curvature, speed_limit_series

REROUTE_DISTANCE = 25
MANEUVER_TRANSITION_THRESHOLD = 10
REROUTE_COUNTER_MIN = 3
ROUTE_RESEND_PERIOD = 10  # updates (seconds); a restarted mapsd gets the route within this
RECOMPUTE_HOLD = 10  # updates (seconds) between route requests

NAV_HOST_DEFAULT = 'http://192.168.0.106:8899'
NAV_HOST_FILE = '/data/ford_params/nav_host'

def _resolve_nav_host() -> str:
  host = os.getenv('NAV_HOST', '').strip()
  if not host:
    try:
      with open(NAV_HOST_FILE) as f:
        host = f.read().strip()
    except OSError:
      host = ''
  host = host or NAV_HOST_DEFAULT
  if '://' not in host:
    host = 'http://' + host
  return host.rstrip('/')

NAV_HOST = _resolve_nav_host()

PROFILE_STEP = 5.0
PROFILE_LOOKAHEAD = 300.0
A_LAT_MAX = 1.8
STOP_DECEL = 0.8


def set_banner(instruction, banners, distance_to_maneuver):
  """Fill the banner fields from the banner to show now: the last one still ahead of the ego, else the first."""
  if not banners:
    return
  banner = next((b for b in reversed(banners) if distance_to_maneuver < b['distanceAlongGeometry']), banners[0])
  instruction.showFull = distance_to_maneuver < banner['distanceAlongGeometry']
  primary = banner['primary']
  instruction.maneuverPrimaryText = primary.get('text') or ''
  instruction.maneuverType = primary.get('type') or ''
  instruction.maneuverModifier = primary.get('modifier') or ''
  if banner.get('secondary') is not None:
    instruction.maneuverSecondaryText = banner['secondary']['text']


class RouteEngine:
  def __init__(self, sm, pm, params=None):
    self.sm = sm
    self.pm = pm
    self.params = params if params is not None else Params()

    self.last_position = None  # (longitude, latitude)
    self.last_bearing = None

    self.nav_destination = None
    self.step_idx = None
    self.route = None
    self.route_geometry = None  # an (n, 2) array of (longitude, latitude) per step

    self.route_geom_all = None
    self.route_cum = None
    self.step_start_cum = None
    self.speed_limit_cum = None
    self.speed_limit_mps = None

    self.recompute_hold = 0
    self.reroute_counter = 0
    self.last_gps_mono_time = 0

    self.mapbox_token = os.environ.get("MAPBOX_TOKEN")
    self.api = Api(self.params.get("DongleId")) if self.mapbox_token is None else None
    self.mapbox_host = NAV_HOST or ("https://api.mapbox.com" if self.mapbox_token is not None else API_HOST.rstrip('/') + '/v1/nav')
    cloudlog.warning(f"[NAV_HOST] route source = {self.mapbox_host} (token={'set' if self.mapbox_token else 'from-api'})")

  def update(self):
    self.sm.update(0)

    source = latest_gps_source(self.sm)
    if source is not None:
      gps = self.sm[source]
      self.last_position, self.last_bearing = (gps.longitude, gps.latitude), gps.bearingDeg
      self.last_gps_mono_time = int(self.sm.logMonoTime[source])
    route = self.route
    try:
      self.recompute_route(gps_ok=source is not None)
      self.send_instruction()
    except Exception:
      cloudlog.exception("navd.failed_to_compute")
    # mapsd dedupes repeats, so a periodic resend is how a restarted mapsd gets the route
    if self.route is not route or self.sm.frame % ROUTE_RESEND_PERIOD == 0:
      self.send_route()
    self.send_profile()

  def recompute_route(self, gps_ok: bool):
    if self.last_position is None:
      return

    destination = self.params.get("NavDestination")
    if destination is None:
      self.clear_route()
      self.recompute_hold = 0
      return
    new_destination = (destination['longitude'], destination['latitude'])

    # Don't recompute when GPS drifts in tunnels
    if not gps_ok and self.step_idx is not None:
      return

    self.recompute_hold -= 1
    if (self.should_recompute() or new_destination != self.nav_destination) and self.recompute_hold <= 0:
      self.recompute_hold = RECOMPUTE_HOLD
      self.calculate_route(new_destination)
      self.reroute_counter = 0

  def calculate_route(self, destination):
    assert self.last_position is not None
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")
    self.nav_destination = destination

    lang = self.params.get('LanguageSetting')
    if lang is not None:
      lang = lang.replace('main_', '')

    token = self.mapbox_token
    if token is None:
      token = self.api.get_token()

    params = {
      'access_token': token,
      'geometries': 'geojson',
      'overview': 'full',
      'steps': 'true',
      'banner_instructions': 'true',
      'alternatives': 'false',
      'language': lang,
      'annotations': 'maxspeed',
    }
    if self.last_bearing is not None:
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90;"

    coords_str = ';'.join(f'{lon},{lat}' for lon, lat in (self.last_position, destination))
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str
    try:
      resp = requests.get(url, params=params, timeout=10)
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)
      resp.raise_for_status()

      r = resp.json()
      if len(r['routes']):
        route = r['routes'][0]
        leg = route['legs'][0]
        self.route = leg['steps']
        self.route_geometry = [np.array(step['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2)) for step in self.route]
        self.step_idx = 0
        self._build_profiles(route, leg)
      else:
        cloudlog.warning("Got empty route response")
        self.clear_route()

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")
      self.clear_route()

  def _build_profiles(self, route, leg):
    try:
      if not self.route_geometry or len(self.route_geometry) < 1:
        return
      geom = np.concatenate(self.route_geometry)
      if len(geom) < 2:
        return

      self.route_geom_all = geom
      self.route_cum = cumulative_distance(geom)

      lengths = np.array([len(g) for g in self.route_geometry])
      starts = np.r_[0, np.cumsum(lengths)[:-1]].astype(int)
      starts = np.clip(starts, 0, len(self.route_cum) - 1)
      self.step_start_cum = self.route_cum[starts]

      self.speed_limit_cum = None
      self.speed_limit_mps = None
      annotations = (leg.get('annotation') or {}).get('maxspeed')
      leg_geom = np.array(route['geometry']['coordinates'], dtype=np.float64).reshape((-1, 2))
      if annotations and len(leg_geom) >= 2:
        series = speed_limit_series(leg_geom, cumulative_distance(leg_geom), annotations)
        if series is not None:
          self.speed_limit_cum, self.speed_limit_mps = series
          cloudlog.info(f"nav speed limits: {len(self.speed_limit_mps)} points, "
                        f"{float(np.nanmin(self.speed_limit_mps)) * 3.6:.0f}-"
                        f"{float(np.nanmax(self.speed_limit_mps)) * 3.6:.0f} km/h")
    except Exception:
      cloudlog.exception("navd.build_profiles_failed")
      self.route_geom_all = None
      self.route_cum = None
      self.step_start_cum = None

  def _ego_progress(self) -> float | None:
    if self.step_idx is None or self.route_cum is None or self.last_position is None:
      return None
    if self.step_idx >= len(self.route_geometry) or self.step_start_cum is None:
      return None
    step_geom = self.route_geometry[self.step_idx]
    if len(step_geom) < 2:
      return None
    along, lateral = along_path(step_geom, self.last_position)
    if lateral > REROUTE_DISTANCE:
      return None
    return float(self.step_start_cum[self.step_idx] + along)

  def _current_speed_limit(self) -> float:
    progress = self._ego_progress()
    if progress is None or self.speed_limit_cum is None:
      return -1.0
    return float(np.interp(progress, self.speed_limit_cum, self.speed_limit_mps))

  def send_profile(self):
    msg = messaging.new_message('navProfile', valid=True)
    stamp = self.last_gps_mono_time if self.last_gps_mono_time > 0 else int(time.monotonic() * 1e9)

    progress = self._ego_progress()
    if progress is None or self.route_cum is None or len(self.route_cum) < 2:
      msg.valid = False
      msg.navProfile.valid = False
      msg.navProfile.speedLimit = -1.0
      msg.navProfile.sourceMonoTime = stamp
      self.pm.send('navProfile', msg)
      return

    total = float(self.route_cum[-1])
    end = min(progress + PROFILE_LOOKAHEAD, total)
    progresses = np.arange(progress, end + PROFILE_STEP, PROFILE_STEP, dtype=np.float64)
    progresses = progresses[progresses <= total]
    if len(progresses) == 0 or (total - progresses[-1]) > 0.5:
      progresses = np.r_[progresses, total]

    try:
      curvature = curvature_profile(self.route_geom_all, self.route_cum, progresses)

      caps = speed_cap_from_curvature(curvature, A_LAT_MAX)

      if self.speed_limit_cum is not None:
        caps = np.minimum(caps, np.interp(progresses, self.speed_limit_cum, self.speed_limit_mps))

      caps = np.minimum(caps, np.sqrt(np.maximum(2.0 * STOP_DECEL * (total - progresses), 0.0)))

      caps[~np.isfinite(caps)] = -1.0

      prof = msg.navProfile
      prof.progress = progress
      prof.valid = True
      prof.speedLimit = self._current_speed_limit()
      prof.sourceMonoTime = stamp
      prof.points = [{'progress': float(p), 'curvature': float(k), 'speedCap': float(c)}
                     for p, k, c in zip(progresses, curvature, caps)]
      self.pm.send('navProfile', msg)
    except Exception:
      cloudlog.exception("navd.profile_failed")
      msg.valid = False
      msg.navProfile.valid = False
      msg.navProfile.sourceMonoTime = stamp
      self.pm.send('navProfile', msg)

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)

    if self.step_idx is None:
      msg.valid = False
      self.pm.send('navInstruction', msg)
      return

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    step = self.route[self.step_idx]
    along_geometry, _ = along_path(self.route_geometry[self.step_idx], self.last_position)
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry

    # Banner instructions are for the following maneuver step, don't use empty last step
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]

    # Current instruction
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry
    msg.navInstruction.speedLimit = self._current_speed_limit()
    set_banner(msg.navInstruction, banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)

    # Total remaining distance and time: the rest of this step, then every step after it
    remaining = 1.0 - along_geometry / max(step['distance'], 1)
    future = self.route[self.step_idx + 1:]
    msg.navInstruction.distanceRemaining = step['distance'] * remaining + sum(s['distance'] for s in future)
    msg.navInstruction.timeRemaining = step['duration'] * remaining + sum(s['duration'] for s in future)

    self.pm.send('navInstruction', msg)

    # Transition to next route segment
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.step_idx + 1 < len(self.route):
        self.step_idx += 1
        self.recompute_hold = 0
      else:
        cloudlog.warning("Destination reached")

        # Clear route if driving away from destination
        if np.linalg.norm(local_meters(self.last_position, self.nav_destination)) > REROUTE_DISTANCE:
          self.params.remove("NavDestination")
          self.clear_route()

  def send_route(self):
    coords = np.concatenate(self.route_geometry) if self.route_geometry is not None else np.empty((0, 2))
    msg = messaging.new_message('navRoute', valid=True)
    msg.navRoute.coordinates = [{'longitude': lon, 'latitude': lat} for lon, lat in coords.tolist()]
    self.pm.send('navRoute', msg)

  def clear_route(self):
    self.route = None
    self.route_geometry = None
    self.step_idx = None
    self.nav_destination = None
    self.route_geom_all = None
    self.route_cum = None
    self.step_start_cum = None
    self.speed_limit_cum = None
    self.speed_limit_mps = None

  def should_recompute(self):
    if self.step_idx is None:
      return True

    assert self.route is not None and self.route_geometry is not None and self.last_position is not None
    # Don't recompute in last segment, assume destination is reached
    if self.step_idx == len(self.route) - 1:
      return False

    # TODO: Check for going wrong way in segment
    if along_path(self.route_geometry[self.step_idx], self.last_position)[1] > REROUTE_DISTANCE:
      self.reroute_counter += 1
    else:
      self.reroute_counter = 0
    return self.reroute_counter > REROUTE_COUNTER_MIN


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute', 'navProfile'])
  sm = messaging.SubMaster(['gpsLocation', 'gpsLocationExternal'])

  rk = Ratekeeper(1.0)
  route_engine = RouteEngine(sm, pm)
  while True:
    route_engine.update()
    rk.keep_time()


if __name__ == "__main__":
  main()
