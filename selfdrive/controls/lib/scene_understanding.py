#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
DP 10.2 Scene Understanding
===========================

Lightweight scene-perception and scene-classification layer.

Data sources
------------

modelV2:
    velocity.x
    laneLineProbs
    laneLines
    leadsV3

liveTracks:
    dRel
    yRel
    vRel
    measured
    cnt
    trackId

mr76State:
    auxiliary radar object data

IMPORTANT
---------

This module is READ-ONLY.

It does not:

    - publish radarState
    - create leadOne / leadTwo
    - modify CarState
    - modify CarControl
    - send CAN
    - create threads
    - create timers
    - create Params

The result is only a normalized scene representation.

Architecture
------------

                modelV2
                   |
                   v
             camera perception
                   |
liveTracks ---> object fusion <--- mr76State
                   |
                   v
             scene detection
                   |
                   v
             behavior hint
                   |
                   v
          AutoAvoidanceHelper

OEM radar:
    liveTracks = primary radar source

MR76:
    mr76State = auxiliary confirmation

modelV2:
    camera / road geometry / lead hint
"""


import math


class SceneUnderstanding:

  # ========================================================================
  # Scene types
  # ========================================================================

  SCENE_NORMAL = "normal"

  # Front vehicle
  SCENE_LEAD = "lead_vehicle"
  SCENE_SLOW_VEHICLE = "slow_vehicle"

  # Obstacle
  SCENE_STATIC_OBSTACLE = "static_obstacle"
  SCENE_DYNAMIC_OBSTACLE = "dynamic_obstacle"

  # Opposite traffic
  SCENE_ONCOMING = "oncoming_vehicle"

  # Vulnerable road users
  SCENE_PEDESTRIAN = "pedestrian_area"
  SCENE_BICYCLE = "bicycle_area"
  SCENE_MOTORCYCLE = "motorcycle"

  # Road environment
  SCENE_CONSTRUCTION = "construction_zone"
  SCENE_LANE_BLOCKED = "lane_blocked"
  SCENE_NARROW_ROAD = "narrow_road"

  # Maneuver opportunities
  SCENE_OVERTAKE = "overtake_possible"
  SCENE_PASSING = "passing_obstacle"

  # High-risk
  SCENE_EMERGENCY = "emergency_obstacle"

  # ========================================================================
  # Object types
  # ========================================================================

  OBJECT_UNKNOWN = -1
  OBJECT_CAR = 0
  OBJECT_BIKE = 1
  OBJECT_PEDESTRIAN = 2
  OBJECT_CONE = 3
  OBJECT_MOTORCYCLE = 4

  # ========================================================================
  # Sources
  # ========================================================================

  SOURCE_MODEL = "modelV2"
  SOURCE_LIVE_TRACKS = "liveTracks"
  SOURCE_MR76 = "mr76State"

  # ========================================================================
  # Risk levels
  # ========================================================================

  RISK_NONE = 0
  RISK_LOW = 1
  RISK_MEDIUM = 2
  RISK_HIGH = 3
  RISK_CRITICAL = 4

  # ========================================================================
  # Model thresholds
  # ========================================================================

  MODEL_LEAD_PROB_MIN = 0.30
  MODEL_LANE_PROB_MIN = 0.50

  # ========================================================================
  # OEM radar thresholds
  # ========================================================================

  RADAR_MAX_DISTANCE = 150.0
  RADAR_MIN_DISTANCE = 0.5
  RADAR_MIN_COUNT = 1

  # ========================================================================
  # MR76 thresholds
  # ========================================================================

  MR76_MAX_DISTANCE = 120.0
  MR76_MIN_CONFIDENCE = 0.55

  # ========================================================================
  # Lead
  # ========================================================================

  LEAD_DISTANCE = 80.0
  LEAD_LATERAL = 2.0

  # ========================================================================
  # Oncoming
  # ========================================================================

  ONCOMING_DISTANCE = 80.0
  ONCOMING_LATERAL_MIN = 1.8
  ONCOMING_VREL_MAX = -8.0

  # Critical oncoming
  ONCOMING_CRITICAL_DISTANCE = 30.0
  ONCOMING_CRITICAL_VREL = -10.0

  # ========================================================================
  # Slow vehicle
  # ========================================================================

  SLOW_VEHICLE_DISTANCE = 60.0
  SLOW_VEHICLE_VREL_MAX = -3.0

  # Strong slow vehicle
  VERY_SLOW_DISTANCE = 35.0
  VERY_SLOW_VREL = -5.0

  # ========================================================================
  # Static obstacle
  # ========================================================================

  STATIC_OBSTACLE_DISTANCE = 60.0
  STATIC_VREL_MAX = 1.0

  STATIC_CRITICAL_DISTANCE = 25.0

  # ========================================================================
  # Dynamic obstacle
  # ========================================================================

  DYNAMIC_OBSTACLE_DISTANCE = 50.0
  DYNAMIC_VREL_MAX = -2.0

  # ========================================================================
  # Lane blocking
  # ========================================================================

  LANE_BLOCK_DISTANCE = 40.0

  LANE_LEFT_LIMIT = 2.0
  LANE_RIGHT_LIMIT = -2.0

  # Strong lane block
  LANE_CRITICAL_DISTANCE = 25.0

  # ========================================================================
  # Pedestrian / bicycle / motorcycle
  # ========================================================================

  PEDESTRIAN_DISTANCE = 45.0
  PEDESTRIAN_CRITICAL_DISTANCE = 20.0

  BICYCLE_DISTANCE = 50.0
  MOTORCYCLE_DISTANCE = 50.0

  # ========================================================================
  # Construction
  # ========================================================================

  CONE_SCENE_COUNT = 3
  CONE_CRITICAL_COUNT = 5

  CONE_DISTANCE = 70.0

  # ========================================================================
  # Narrow road
  # ========================================================================

  NARROW_LANE_CONFIDENCE = 0.35

  # ========================================================================
  # Fusion
  # ========================================================================

  FUSION_DISTANCE = 2.5
  FUSION_LATERAL = 1.5

  # ========================================================================
  # Initialization
  # ========================================================================

  def __init__(self):

    self.camera_objects = []
    self.radar_objects = []
    self.mr76_objects = []

    self.objects = []

    self.scene_type = self.SCENE_NORMAL

    self.risk_level = self.RISK_NONE

    # ------------------------------------------------------------
    # Object counters
    # ------------------------------------------------------------

    self.object_counts = {
      "cars": 0,
      "pedestrians": 0,
      "bikes": 0,
      "motorcycles": 0,
      "cones": 0,
    }

    # ------------------------------------------------------------
    # General obstacle
    # ------------------------------------------------------------

    self.obstacle_distance = float("inf")

    self.obstacle_in_path = False

    # ------------------------------------------------------------
    # Vehicle states
    # ------------------------------------------------------------

    self.lead_vehicle = False
    self.slow_vehicle = False
    self.very_slow_vehicle = False

    self.static_obstacle = False
    self.dynamic_obstacle = False

    self.oncoming_vehicle = False

    # ------------------------------------------------------------
    # Vulnerable road users
    # ------------------------------------------------------------

    self.pedestrian_present = False
    self.bicycle_present = False
    self.motorcycle_present = False

    self.pedestrian_in_path = False
    self.bicycle_in_path = False

    # ------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------

    self.construction_zone = False
    self.cone_in_path = False

    # ------------------------------------------------------------
    # Lane state
    # ------------------------------------------------------------

    self.left_blocked = False
    self.right_blocked = False

    self.left_lane_available = False
    self.right_lane_available = False

    self.lane_blocked = False
    self.narrow_road = False

    # ------------------------------------------------------------
    # Behavior
    # ------------------------------------------------------------

    self.avoid_required = False
    self.overtake_available = False
    self.passing_available = False

    # ------------------------------------------------------------
    # Model state
    # ------------------------------------------------------------

    self.model_velocity = 0.0

    self.left_lane_visible = False
    self.right_lane_visible = False
    self.lane_confidence = 0.0

    # ------------------------------------------------------------
    # Model lead
    # ------------------------------------------------------------

    self.model_lead_distance = float("inf")
    self.model_lead_lateral = 0.0
    self.model_lead_velocity = 0.0
    self.model_lead_probability = 0.0
    self.model_lead_valid = False

    # ------------------------------------------------------------
    # Best lead
    # ------------------------------------------------------------

    self.lead_distance = float("inf")
    self.lead_rel_speed = 0.0
    self.lead_lateral = 0.0

  # ========================================================================
  # Safe helpers
  # ========================================================================

  @staticmethod
  def _safe_float(value, default=0.0):
    try:
      value = float(value)

      if math.isfinite(value):
        return value

    except Exception:
      pass

    return default

  @staticmethod
  def _safe_int(value, default=-1):
    try:
      return int(value)
    except Exception:
      return default

  @staticmethod
  def _safe_bool(value, default=False):
    try:
      return bool(value)
    except Exception:
      return default

  @staticmethod
  def _safe_len(value):
    try:
      return len(value)
    except Exception:
      return 0

  @staticmethod
  def _get(obj, name, default=None):
    try:
      return getattr(obj, name)
    except Exception:
      return default

  # ========================================================================
  # Main update
  # ========================================================================

  def update(self, sm):

    self.camera_objects = self._process_model(sm)

    self.radar_objects = self._process_live_tracks(sm)

    self.mr76_objects = self._process_mr76(sm)

    self.objects = self._fuse_objects(
      self.camera_objects,
      self.radar_objects,
      self.mr76_objects,
    )

    # ------------------------------------------------------------
    # Stage 1: statistics
    # ------------------------------------------------------------

    self._update_statistics()

    # ------------------------------------------------------------
    # Stage 2: lead
    # ------------------------------------------------------------

    self._detect_lead()

    # ------------------------------------------------------------
    # Stage 3: road users
    # ------------------------------------------------------------

    self._detect_pedestrians()
    self._detect_bicycles()
    self._detect_motorcycles()

    # ------------------------------------------------------------
    # Stage 4: vehicle scenes
    # ------------------------------------------------------------

    self._detect_oncoming()
    self._detect_slow_vehicle()
    self._detect_static_obstacle()
    self._detect_dynamic_obstacle()

    # ------------------------------------------------------------
    # Stage 5: road environment
    # ------------------------------------------------------------

    self._detect_construction()
    self._check_lane_block()
    self._detect_narrow_road()

    # ------------------------------------------------------------
    # Stage 6: path risk
    # ------------------------------------------------------------

    self._detect_obstacle_in_path()

    # ------------------------------------------------------------
    # Stage 7: maneuver possibility
    # ------------------------------------------------------------

    self._detect_overtake()
    self._detect_passing()

    # ------------------------------------------------------------
    # Stage 8: final classification
    # ------------------------------------------------------------

    self.scene_type = self._classify_scene()

    self.risk_level = self._classify_risk()

    return (
      self.scene_type,
      self.objects,
      self.get_behavior_hint(),
    )

  # ========================================================================
  # ModelV2
  # ========================================================================

  def _process_model(self, sm):

    objects = []

    try:
      model = sm["modelV2"]
    except Exception:
      self._reset_model_state()
      return objects

    self._process_model_velocity(model)

    self._process_lane_lines(model)

    self._process_model_lead(
      model,
      objects,
    )

    return objects

  # ========================================================================
  # Model velocity
  # ========================================================================

  def _process_model_velocity(self, model):

    self.model_velocity = 0.0

    try:

      velocity = self._get(
        model,
        "velocity",
        None,
      )

      if velocity is None:
        return

      velocity_x = self._get(
        velocity,
        "x",
        None,
      )

      if self._safe_len(velocity_x) <= 0:
        return

      self.model_velocity = self._safe_float(
        velocity_x[0],
        0.0,
      )

    except Exception:

      self.model_velocity = 0.0

  # ========================================================================
  # Lane lines
  # ========================================================================

  def _process_lane_lines(self, model):

    self.left_lane_visible = False
    self.right_lane_visible = False
    self.lane_confidence = 0.0

    try:

      probs = self._get(
        model,
        "laneLineProbs",
        None,
      )

      if self._safe_len(probs) < 4:
        return

      left_prob = self._safe_float(
        probs[1],
        0.0,
      )

      right_prob = self._safe_float(
        probs[2],
        0.0,
      )

      left_prob = min(
        max(left_prob, 0.0),
        1.0,
      )

      right_prob = min(
        max(right_prob, 0.0),
        1.0,
      )

      self.left_lane_visible = (
        left_prob >= self.MODEL_LANE_PROB_MIN
      )

      self.right_lane_visible = (
        right_prob >= self.MODEL_LANE_PROB_MIN
      )

      self.lane_confidence = (
        left_prob + right_prob
      ) * 0.5

    except Exception:

      self.left_lane_visible = False
      self.right_lane_visible = False
      self.lane_confidence = 0.0

  # ========================================================================
  # Model lead
  # ========================================================================

  def _process_model_lead(self, model, objects):

    self.model_lead_distance = float("inf")
    self.model_lead_lateral = 0.0
    self.model_lead_velocity = 0.0
    self.model_lead_probability = 0.0
    self.model_lead_valid = False

    try:

      leads = self._get(
        model,
        "leadsV3",
        None,
      )

      if self._safe_len(leads) <= 0:
        return

      lead = leads[0]

      probability = self._safe_float(
        self._get(
          lead,
          "prob",
          0.0,
        ),
        0.0,
      )

      if probability < self.MODEL_LEAD_PROB_MIN:
        return

      distance = self._safe_float(
        self._get(
          lead,
          "x",
          0.0,
        ),
        0.0,
      )

      lateral = self._safe_float(
        self._get(
          lead,
          "y",
          0.0,
        ),
        0.0,
      )

      velocity = self._safe_float(
        self._get(
          lead,
          "v",
          0.0,
        ),
        0.0,
      )

      if distance <= 0.0:
        return

      self.model_lead_distance = distance
      self.model_lead_lateral = lateral
      self.model_lead_velocity = velocity
      self.model_lead_probability = probability
      self.model_lead_valid = True

      objects.append({
        "x": distance,
        "y": lateral,
        "speed": velocity,
        "type": self.OBJECT_CAR,
        "confidence": probability,
        "source": self.SOURCE_MODEL,
        "model_lead": True,
      })

    except Exception:

      self.model_lead_distance = float("inf")
      self.model_lead_lateral = 0.0
      self.model_lead_velocity = 0.0
      self.model_lead_probability = 0.0
      self.model_lead_valid = False

  # ========================================================================
  # Reset model
  # ========================================================================

  def _reset_model_state(self):

    self.model_velocity = 0.0

    self.left_lane_visible = False
    self.right_lane_visible = False
    self.lane_confidence = 0.0

    self.model_lead_distance = float("inf")
    self.model_lead_lateral = 0.0
    self.model_lead_velocity = 0.0
    self.model_lead_probability = 0.0
    self.model_lead_valid = False

  # ========================================================================
  # liveTracks
  # ========================================================================

  def _process_live_tracks(self, sm):

    objects = []

    try:
      live_tracks = sm["liveTracks"]
    except Exception:
      return objects

    try:
      tracks = list(live_tracks)
    except Exception:
      return objects

    for track in tracks:

      try:

        d_rel = self._safe_float(
          self._get(
            track,
            "dRel",
            0.0,
          ),
          0.0,
        )

        if (
          d_rel < self.RADAR_MIN_DISTANCE
          or
          d_rel > self.RADAR_MAX_DISTANCE
        ):
          continue

        y_rel = self._safe_float(
          self._get(
            track,
            "yRel",
            0.0,
          ),
          0.0,
        )

        v_rel = self._safe_float(
          self._get(
            track,
            "vRel",
            0.0,
          ),
          0.0,
        )

        measured = self._safe_bool(
          self._get(
            track,
            "measured",
            False,
          ),
          False,
        )

        count = self._safe_int(
          self._get(
            track,
            "cnt",
            0,
          ),
          0,
        )

        track_id = self._safe_int(
          self._get(
            track,
            "trackId",
            -1,
          ),
          -1,
        )

        if count < self.RADAR_MIN_COUNT:
          continue

        confidence = 0.90

        if not measured:
          confidence = 0.75

        if count >= 3:
          confidence = min(
            confidence + 0.05,
            1.0,
          )

        objects.append({
          "x": d_rel,
          "y": y_rel,
          "speed": v_rel,
          "type": self.OBJECT_CAR,
          "confidence": confidence,
          "source": self.SOURCE_LIVE_TRACKS,
          "measured": measured,
          "cnt": count,
          "track_id": track_id,
          "oem_radar": True,
        })

      except Exception:
        continue

    return objects

  # ========================================================================
  # MR76
  # ========================================================================

  def _process_mr76(self, sm):

    objects = []

    try:
      mr76 = sm["mr76State"]
    except Exception:
      return objects

    raw_objects = self._find_mr76_objects(mr76)

    if raw_objects is None:
      return objects

    try:
      raw_objects = list(raw_objects)
    except Exception:
      return objects

    for raw in raw_objects:

      try:

        distance = self._mr76_distance(raw)

        if (
          distance <= 0.0
          or
          distance > self.MR76_MAX_DISTANCE
        ):
          continue

        lateral = self._mr76_lateral(raw)

        velocity = self._mr76_velocity(raw)

        confidence = self._mr76_confidence(raw)

        if confidence < self.MR76_MIN_CONFIDENCE:
          continue

        obj_type = self._safe_int(
          self._get(
            raw,
            "type",
            self.OBJECT_CAR,
          ),
          self.OBJECT_CAR,
        )

        # Keep unknown radar classification conservative.
        if obj_type not in (
          self.OBJECT_CAR,
          self.OBJECT_BIKE,
          self.OBJECT_PEDESTRIAN,
          self.OBJECT_CONE,
          self.OBJECT_MOTORCYCLE,
        ):
          obj_type = self.OBJECT_UNKNOWN

        objects.append({
          "x": distance,
          "y": lateral,
          "speed": velocity,
          "type": obj_type,
          "confidence": confidence,
          "source": self.SOURCE_MR76,
          "mr76": True,
        })

      except Exception:
        continue

    return objects

  # ========================================================================
  # MR76 helpers
  # ========================================================================

  def _find_mr76_objects(self, mr76):

    for field in (
      "objects",
      "tracks",
      "radarObjects",
    ):

      try:

        value = self._get(
          mr76,
          field,
          None,
        )

        if value is not None:
          return value

      except Exception:
        continue

    return None

  def _mr76_distance(self, obj):

    for field in (
      "dRel",
      "distLong",
      "distance",
      "DistLong",
    ):

      value = self._get(
        obj,
        field,
        None,
      )

      if value is not None:
        return self._safe_float(
          value,
          999.0,
        )

    return 999.0

  def _mr76_lateral(self, obj):

    for field in (
      "yRel",
      "distLat",
      "lateral",
      "DistLat",
    ):

      value = self._get(
        obj,
        field,
        None,
      )

      if value is not None:
        return self._safe_float(
          value,
          0.0,
        )

    return 0.0

  def _mr76_velocity(self, obj):

    for field in (
      "vRel",
      "vRelLong",
      "VRelLong",
    ):

      value = self._get(
        obj,
        field,
        None,
      )

      if value is not None:
        return self._safe_float(
          value,
          0.0,
        )

    return 0.0

  def _mr76_confidence(self, obj):

    for field in (
      "confidence",
      "prob",
    ):

      value = self._get(
        obj,
        field,
        None,
      )

      if value is not None:
        return min(
          max(
            self._safe_float(
              value,
              0.65,
            ),
            0.0,
          ),
          1.0,
        )

    return 0.65

  # ========================================================================
  # Object fusion
  # ========================================================================

  def _fuse_objects(
      self,
      model_objects,
      radar_objects,
      mr76_objects):

    fused = []

    all_objects = (
      list(radar_objects)
      +
      list(mr76_objects)
      +
      list(model_objects)
    )

    for obj in all_objects:

      if not isinstance(obj, dict):
        continue

      ox = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      oy = self._safe_float(
        obj.get("y", 0.0),
        0.0,
      )

      if ox <= 0.0:
        continue

      match = None

      for target in fused:

        tx = self._safe_float(
          target.get("x", 999.0),
          999.0,
        )

        ty = self._safe_float(
          target.get("y", 0.0),
          0.0,
        )

        if (
          abs(ox - tx) <= self.FUSION_DISTANCE
          and
          abs(oy - ty) <= self.FUSION_LATERAL
        ):
          match = target
          break

      if match is None:

        copy = dict(obj)

        copy.setdefault(
          "confidence",
          0.0,
        )

        fused.append(copy)

        continue

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      # ------------------------------------------------------------
      # OEM radar = physical range authority
      # ------------------------------------------------------------

      if source == self.SOURCE_LIVE_TRACKS:

        match["x"] = ox
        match["y"] = oy

        match["speed"] = self._safe_float(
          obj.get(
            "speed",
            match.get(
              "speed",
              0.0,
            ),
          ),
          0.0,
        )

        match["oem_radar"] = True

      # ------------------------------------------------------------
      # MR76 = auxiliary confirmation
      # ------------------------------------------------------------

      elif source == self.SOURCE_MR76:

        match["mr76"] = True

        # If primary object has unknown type,
        # auxiliary radar classification can fill it.
        if (
          match.get("type", self.OBJECT_UNKNOWN)
          == self.OBJECT_UNKNOWN
        ):
          match["type"] = obj.get(
            "type",
            self.OBJECT_UNKNOWN,
          )

      # ------------------------------------------------------------
      # model = visual confirmation
      # ------------------------------------------------------------

      elif source == self.SOURCE_MODEL:

        match["model"] = True

      match["confidence"] = max(
        self._safe_float(
          match.get(
            "confidence",
            0.0,
          ),
          0.0,
        ),
        self._safe_float(
          obj.get(
            "confidence",
            0.0,
          ),
          0.0,
        ),
      )

      old_source = str(
        match.get(
          "source",
          "",
        )
      )

      if source and source not in old_source:

        match["source"] = (
          old_source
          +
          ("+" if old_source else "")
          +
          source
        )

    return fused

  # ========================================================================
  # Statistics
  # ========================================================================

  def _update_statistics(self):

    for key in self.object_counts:
      self.object_counts[key] = 0

    self.obstacle_distance = float("inf")

    for obj in self.objects:

      obj_type = self._safe_int(
        obj.get(
          "type",
          self.OBJECT_UNKNOWN,
        ),
        self.OBJECT_UNKNOWN,
      )

      if obj_type == self.OBJECT_CAR:
        self.object_counts["cars"] += 1

      elif obj_type == self.OBJECT_PEDESTRIAN:
        self.object_counts["pedestrians"] += 1

      elif obj_type == self.OBJECT_BIKE:
        self.object_counts["bikes"] += 1

      elif obj_type == self.OBJECT_MOTORCYCLE:
        self.object_counts["motorcycles"] += 1

      elif obj_type == self.OBJECT_CONE:
        self.object_counts["cones"] += 1

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      if (
        self.SOURCE_LIVE_TRACKS in source
        or
        self.SOURCE_MR76 in source
      ):

        distance = self._safe_float(
          obj.get(
            "x",
            999.0,
          ),
          999.0,
        )

        if (
          0.0 < distance <
          self.obstacle_distance
        ):
          self.obstacle_distance = distance

  # ========================================================================
  # Lead vehicle
  # ========================================================================

  def _detect_lead(self):

    self.lead_vehicle = False

    self.lead_distance = float("inf")
    self.lead_rel_speed = 0.0
    self.lead_lateral = 0.0

    best_distance = float("inf")

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_CAR:
        continue

      distance = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      lateral = self._safe_float(
        obj.get("y", 0.0),
        0.0,
      )

      if distance > self.LEAD_DISTANCE:
        continue

      if abs(lateral) > self.LEAD_LATERAL:
        continue

      if distance >= best_distance:
        continue

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      # Camera lead can participate as a hint.
      if (
        self.SOURCE_MODEL not in source
        and
        self.SOURCE_LIVE_TRACKS not in source
        and
        self.SOURCE_MR76 not in source
      ):
        continue

      best_distance = distance

      self.lead_distance = distance

      self.lead_rel_speed = self._safe_float(
        obj.get(
          "speed",
          0.0,
        ),
        0.0,
      )

      self.lead_lateral = lateral

      self.lead_vehicle = True

      obj["lead_vehicle"] = True

  # ========================================================================
  # Oncoming
  # ========================================================================

  def _detect_oncoming(self):

    self.oncoming_vehicle = False

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_CAR:
        continue

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      if (
        self.SOURCE_LIVE_TRACKS not in source
        and
        self.SOURCE_MR76 not in source
      ):
        continue

      lateral = abs(
        self._safe_float(
          obj.get(
            "y",
            0.0,
          ),
          0.0,
        )
      )

      distance = self._safe_float(
        obj.get(
          "x",
          999.0,
        ),
        999.0,
      )

      velocity = self._safe_float(
        obj.get(
          "speed",
          0.0,
        ),
        0.0,
      )

      if (
        lateral > self.ONCOMING_LATERAL_MIN
        and
        distance < self.ONCOMING_DISTANCE
        and
        velocity < self.ONCOMING_VREL_MAX
      ):

        obj["oncoming"] = True

        if (
          distance < self.ONCOMING_CRITICAL_DISTANCE
          and
          velocity < self.ONCOMING_CRITICAL_VREL
        ):
          obj["risk"] = "critical"
        else:
          obj["risk"] = "high"

        self.oncoming_vehicle = True

  # ========================================================================
  # Slow vehicle
  # ========================================================================

  def _detect_slow_vehicle(self):

    self.slow_vehicle = False
    self.very_slow_vehicle = False

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_CAR:
        continue

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      if (
        self.SOURCE_LIVE_TRACKS not in source
        and
        self.SOURCE_MR76 not in source
      ):
        continue

      distance = self._safe_float(
        obj.get(
          "x",
          999.0,
        ),
        999.0,
      )

      velocity = self._safe_float(
        obj.get(
          "speed",
          0.0,
        ),
        0.0,
      )

      if (
        distance < self.SLOW_VEHICLE_DISTANCE
        and
        velocity < self.SLOW_VEHICLE_VREL_MAX
      ):

        obj["slow_vehicle"] = True

        self.slow_vehicle = True

        if (
          distance < self.VERY_SLOW_DISTANCE
          and
          velocity < self.VERY_SLOW_VREL
        ):
          obj["very_slow"] = True
          self.very_slow_vehicle = True

  # ========================================================================
  # Static obstacle
  # ========================================================================

  def _detect_static_obstacle(self):

    self.static_obstacle = False

    for obj in self.objects:

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      # Conservative:
      # static obstacle requires OEM radar.
      if self.SOURCE_LIVE_TRACKS not in source:
        continue

      distance = self._safe_float(
        obj.get(
          "x",
          999.0,
        ),
        999.0,
      )

      velocity = self._safe_float(
        obj.get(
          "speed",
          0.0,
        ),
        0.0,
      )

      if (
        distance < self.STATIC_OBSTACLE_DISTANCE
        and
        abs(velocity) < self.STATIC_VREL_MAX
      ):

        obj["static_obstacle"] = True

        self.static_obstacle = True

        if distance < self.STATIC_CRITICAL_DISTANCE:
          obj["risk"] = "critical"

  # ========================================================================
  # Dynamic obstacle
  # ========================================================================

  def _detect_dynamic_obstacle(self):

    self.dynamic_obstacle = False

    for obj in self.objects:

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      if (
        self.SOURCE_LIVE_TRACKS not in source
        and
        self.SOURCE_MR76 not in source
      ):
        continue

      distance = self._safe_float(
        obj.get(
          "x",
          999.0,
        ),
        999.0,
      )

      velocity = self._safe_float(
        obj.get(
          "speed",
          0.0,
        ),
        0.0,
      )

      if (
        distance < self.DYNAMIC_OBSTACLE_DISTANCE
        and
        velocity < self.DYNAMIC_OBSTACLE_VREL_MAX
      ):

        obj["dynamic_obstacle"] = True

        self.dynamic_obstacle = True

  # ========================================================================
  # Pedestrians
  # ========================================================================

  def _detect_pedestrians(self):

    self.pedestrian_present = False
    self.pedestrian_in_path = False

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_PEDESTRIAN:
        continue

      distance = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      lateral = abs(
        self._safe_float(
          obj.get("y", 0.0),
          0.0,
        )
      )

      if distance > self.PEDESTRIAN_DISTANCE:
        continue

      self.pedestrian_present = True

      obj["pedestrian"] = True

      # Conservative path region.
      if lateral < 2.5:

        self.pedestrian_in_path = True

        obj["risk"] = (
          "critical"
          if distance < self.PEDESTRIAN_CRITICAL_DISTANCE
          else "high"
        )

  # ========================================================================
  # Bicycles
  # ========================================================================

  def _detect_bicycles(self):

    self.bicycle_present = False
    self.bicycle_in_path = False

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_BIKE:
        continue

      distance = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      lateral = abs(
        self._safe_float(
          obj.get("y", 0.0),
          0.0,
        )
      )

      if distance > self.BICYCLE_DISTANCE:
        continue

      self.bicycle_present = True

      obj["bicycle"] = True

      if lateral < 2.5:
        self.bicycle_in_path = True
        obj["risk"] = "high"

  # ========================================================================
  # Motorcycles
  # ========================================================================

  def _detect_motorcycles(self):

    self.motorcycle_present = False

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_MOTORCYCLE:
        continue

      distance = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      if distance > self.MOTORCYCLE_DISTANCE:
        continue

      self.motorcycle_present = True

      obj["motorcycle"] = True

  # ========================================================================
  # Construction
  # ========================================================================

  def _detect_construction(self):

    self.construction_zone = False
    self.cone_in_path = False

    cone_count = self.object_counts["cones"]

    if cone_count >= self.CONE_SCENE_COUNT:

      self.construction_zone = True

    for obj in self.objects:

      if obj.get("type") != self.OBJECT_CONE:
        continue

      distance = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      lateral = abs(
        self._safe_float(
          obj.get("y", 0.0),
          0.0,
        )
      )

      if distance > self.CONE_DISTANCE:
        continue

      if lateral < 2.5:

        self.cone_in_path = True

        obj["cone_in_path"] = True

        obj["risk"] = "high"

  # ========================================================================
  # Lane block
  # ========================================================================

  def _check_lane_block(self):

    self.left_blocked = False
    self.right_blocked = False

    for obj in self.objects:

      distance = self._safe_float(
        obj.get(
          "x",
          999.0,
        ),
        999.0,
      )

      if distance > self.LANE_BLOCK_DISTANCE:
        continue

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      if (
        self.SOURCE_LIVE_TRACKS not in source
        and
        self.SOURCE_MR76 not in source
      ):
        continue

      lateral = self._safe_float(
        obj.get(
          "y",
          0.0,
        ),
        0.0,
      )

      if lateral > self.LANE_LEFT_LIMIT:
        self.left_blocked = True

      if lateral < self.LANE_RIGHT_LIMIT:
        self.right_blocked = True

    self.lane_blocked = (
      self.left_blocked
      or
      self.right_blocked
    )

  # ========================================================================
  # Narrow road
  # ========================================================================

  def _detect_narrow_road(self):

    self.narrow_road = False

    if self.lane_confidence < self.NARROW_LANE_CONFIDENCE:

      if (
        self.left_lane_visible
        or
        self.right_lane_visible
      ):
        self.narrow_road = True

  # ========================================================================
  # Obstacle in path
  # ========================================================================

  def _detect_obstacle_in_path(self):

    self.obstacle_in_path = False

    for obj in self.objects:

      distance = self._safe_float(
        obj.get("x", 999.0),
        999.0,
      )

      if distance > self.LANE_BLOCK_DISTANCE:
        continue

      lateral = abs(
        self._safe_float(
          obj.get("y", 0.0),
          0.0,
        )
      )

      if lateral > 2.0:
        continue

      source = str(
        obj.get(
          "source",
          "",
        )
      )

      # Model alone does not trigger hard obstacle avoidance.
      if (
        self.SOURCE_LIVE_TRACKS not in source
        and
        self.SOURCE_MR76 not in source
      ):
        continue

      obj["in_path"] = True

      self.obstacle_in_path = True

  # ========================================================================
  # Overtake
  # ========================================================================

  def _detect_overtake(self):

    self.overtake_available = False

    if not self.slow_vehicle:
      return

    if self.oncoming_vehicle:
      return

    if self.left_blocked:
      return

    if self.pedestrian_in_path:
      return

    if self.cone_in_path:
      return

    self.overtake_available = True

  # ========================================================================
  # Passing
  # ========================================================================

  def _detect_passing(self):

    self.passing_available = False

    if not (
      self.static_obstacle
      or
      self.cone_in_path
      or
      self.pedestrian_in_path
      or
      self.bicycle_in_path
    ):
      return

    if self.oncoming_vehicle:
      return

    if self.left_blocked and self.right_blocked:
      return

    self.passing_available = True

  # ========================================================================
  # Scene classification
  # ========================================================================

  def _classify_scene(self):

    # ------------------------------------------------------------
    # Highest priority
    # ------------------------------------------------------------

    if self.oncoming_vehicle:

      return self.SCENE_ONCOMING

    if self.pedestrian_in_path:

      return self.SCENE_PEDESTRIAN

    if self.cone_in_path:

      return self.SCENE_CONSTRUCTION

    if self.static_obstacle:

      return self.SCENE_STATIC_OBSTACLE

    # ------------------------------------------------------------
    # Lane blocked
    # ------------------------------------------------------------

    if self.lane_blocked:

      return self.SCENE_LANE_BLOCKED

    # ------------------------------------------------------------
    # Vulnerable road users
    # ------------------------------------------------------------

    if self.bicycle_in_path:

      return self.SCENE_BICYCLE

    if self.motorcycle_present:

      return self.SCENE_MOTORCYCLE

    # ------------------------------------------------------------
    # Vehicle behavior
    # ------------------------------------------------------------

    if self.very_slow_vehicle:

      return self.SCENE_SLOW_VEHICLE

    if self.slow_vehicle:

      return self.SCENE_SLOW_VEHICLE

    if self.dynamic_obstacle:

      return self.SCENE_DYNAMIC_OBSTACLE

    # ------------------------------------------------------------
    # Maneuver
    # ------------------------------------------------------------

    if self.overtake_available:

      return self.SCENE_OVERTAKE

    if self.passing_available:

      return self.SCENE_PASSING

    # ------------------------------------------------------------
    # Narrow road
    # ------------------------------------------------------------

    if self.narrow_road:

      return self.SCENE_NARROW_ROAD

    # ------------------------------------------------------------
    # Normal
    # ------------------------------------------------------------

    if self.lead_vehicle:

      return self.SCENE_LEAD

    return self.SCENE_NORMAL

  # ========================================================================
  # Risk classification
  # ========================================================================

  def _classify_risk(self):

    # Critical
    if (
      self.oncoming_vehicle
      and
      self._has_critical_object("oncoming")
    ):
      return self.RISK_CRITICAL

    if (
      self.static_obstacle
      and
      self.obstacle_distance <
      self.STATIC_CRITICAL_DISTANCE
    ):
      return self.RISK_CRITICAL

    if self.pedestrian_in_path:

      for obj in self.objects:

        if (
          obj.get("type")
          == self.OBJECT_PEDESTRIAN
          and
          obj.get("risk")
          == "critical"
        ):
          return self.RISK_CRITICAL

    # High
    if (
      self.oncoming_vehicle
      or
      self.pedestrian_in_path
      or
      self.cone_in_path
      or
      self.static_obstacle
    ):
      return self.RISK_HIGH

    # Medium
    if (
      self.very_slow_vehicle
      or
      self.dynamic_obstacle
      or
      self.bicycle_in_path
      or
      self.lane_blocked
    ):
      return self.RISK_MEDIUM

    # Low
    if (
      self.slow_vehicle
      or
      self.motorcycle_present
      or
      self.narrow_road
    ):
      return self.RISK_LOW

    return self.RISK_NONE

  # ========================================================================
  # Critical object helper
  # ========================================================================

  def _has_critical_object(self, field):

    for obj in self.objects:

      if obj.get(field, False):

        if obj.get(
            "risk",
            "",
        ) == "critical":
          return True

    return False

  # ========================================================================
  # Avoidance request
  # ========================================================================

  def get_avoidance_request(self):

    self.avoid_required = (
      self.oncoming_vehicle
      or
      self.static_obstacle
      or
      self.pedestrian_in_path
      or
      self.cone_in_path
      or
      self.bicycle_in_path
      or
      self.obstacle_in_path
    )

    return {
      "avoid": self.avoid_required,

      "obstacle_in_path":
        self.obstacle_in_path,

      "left_blocked":
        self.left_blocked,

      "right_blocked":
        self.right_blocked,

      "oncoming":
        self.oncoming_vehicle,

      "pedestrian":
        self.pedestrian_in_path,

      "bicycle":
        self.bicycle_in_path,

      "cone":
        self.cone_in_path,
    }

  # ========================================================================
  # Overtake request
  # ========================================================================

  def get_overtake_request(self):

    return {
      "overtake":
        self.overtake_available,

      "slow_vehicle":
        self.slow_vehicle,

      "left_blocked":
        self.left_blocked,

      "oncoming":
        self.oncoming_vehicle,
    }

  # ========================================================================
  # Behavior hint
  # ========================================================================

  def get_behavior_hint(self):

    avoidance = self.get_avoidance_request()

    overtake = self.get_overtake_request()

    return {

      "scene":
        self.scene_type,

      "risk_level":
        self.risk_level,

      # --------------------------------------------------------
      # Lead
      # --------------------------------------------------------

      "lead_vehicle":
        self.lead_vehicle,

      "lead_dist":
        self.lead_distance,

      "lead_rel_speed":
        self.lead_rel_speed,

      # --------------------------------------------------------
      # Slow
      # --------------------------------------------------------

      "slow_vehicle":
        self.slow_vehicle,

      "very_slow_vehicle":
        self.very_slow_vehicle,

      # --------------------------------------------------------
      # Obstacle
      # --------------------------------------------------------

      "static_obstacle":
        self.static_obstacle,

      "dynamic_obstacle":
        self.dynamic_obstacle,

      "obstacle_in_path":
        avoidance["obstacle_in_path"],

      "obstacle_distance":
        self.obstacle_distance,

      # --------------------------------------------------------
      # Oncoming
      # --------------------------------------------------------

      "oncoming":
        self.oncoming_vehicle,

      # --------------------------------------------------------
      # Vulnerable road users
      # --------------------------------------------------------

      "pedestrian":
        self.pedestrian_in_path,

      "bicycle":
        self.bicycle_in_path,

      "motorcycle":
        self.motorcycle_present,

      # --------------------------------------------------------
      # Construction
      # --------------------------------------------------------

      "construction":
        self.construction_zone,

      "cone":
        self.cone_in_path,

      # --------------------------------------------------------
      # Lane
      # --------------------------------------------------------

      "left_blocked":
        self.left_blocked,

      "right_blocked":
        self.right_blocked,

      "lane_blocked":
        self.lane_blocked,

      "narrow_road":
        self.narrow_road,

      # --------------------------------------------------------
      # Maneuver
      # --------------------------------------------------------

      "avoid":
        avoidance["avoid"],

      "overtake":
        overtake["overtake"],

      "passing":
        self.passing_available,

      # --------------------------------------------------------
      # Model
      # --------------------------------------------------------

      "model_velocity":
        self.model_velocity,

      "model_lead_distance":
        self.model_lead_distance,

      "model_lead_lateral":
        self.model_lead_lateral,

      "model_lead_velocity":
        self.model_lead_velocity,

      "model_lead_probability":
        self.model_lead_probability,

      "model_lead_valid":
        self.model_lead_valid,

      "left_lane_visible":
        self.left_lane_visible,

      "right_lane_visible":
        self.right_lane_visible,

      "lane_confidence":
        self.lane_confidence,
    }

  # ========================================================================
  # Scene information
  # ========================================================================

  def get_scene_info(self):

    return {

      "scene_type":
        self.scene_type,

      "risk_level":
        self.risk_level,

      "objects":
        list(self.objects),

      "object_counts":
        dict(self.object_counts),

      "obstacle_distance":
        self.obstacle_distance,

      # Vehicle
      "lead_vehicle":
        self.lead_vehicle,

      "lead_distance":
        self.lead_distance,

      "lead_rel_speed":
        self.lead_rel_speed,

      "slow_vehicle":
        self.slow_vehicle,

      "very_slow_vehicle":
        self.very_slow_vehicle,

      # Obstacles
      "static_obstacle":
        self.static_obstacle,

      "dynamic_obstacle":
        self.dynamic_obstacle,

      "obstacle_in_path":
        self.obstacle_in_path,

      # Oncoming
      "oncoming_vehicle":
        self.oncoming_vehicle,

      # Vulnerable road users
      "pedestrian_present":
        self.pedestrian_present,

      "pedestrian_in_path":
        self.pedestrian_in_path,

      "bicycle_present":
        self.bicycle_present,

      "bicycle_in_path":
        self.bicycle_in_path,

      "motorcycle_present":
        self.motorcycle_present,

      # Construction
      "construction_zone":
        self.construction_zone,

      "cone_in_path":
        self.cone_in_path,

      # Lane
      "left_blocked":
        self.left_blocked,

      "right_blocked":
        self.right_blocked,

      "lane_blocked":
        self.lane_blocked,

      "narrow_road":
        self.narrow_road,

      # Behavior
      "avoid_required":
        self.avoid_required,

      "overtake_available":
        self.overtake_available,

      "passing_available":
        self.passing_available,

      # Model
      "model_velocity":
        self.model_velocity,

      "model_lead_distance":
        self.model_lead_distance,

      "model_lead_lateral":
        self.model_lead_lateral,

      "model_lead_velocity":
        self.model_lead_velocity,

      "model_lead_probability":
        self.model_lead_probability,

      "model_lead_valid":
        self.model_lead_valid,

      "left_lane_visible":
        self.left_lane_visible,

      "right_lane_visible":
        self.right_lane_visible,

      "lane_confidence":
        self.lane_confidence,
    }