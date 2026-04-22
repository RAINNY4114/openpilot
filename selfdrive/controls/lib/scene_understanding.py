#!/usr/bin/env python3
"""
SceneUnderstanding（最终稳定版）

核心特性：
1. Scene 只做环境分类
2. Occupancy 决定是否动作
3. 防抖 + 状态机（避免突兀刹车）
4. 分级控制：减速 / 刹车
"""

import numpy as np


class SceneUnderstanding:

  # ===== 场景 =====
  SCENE_NORMAL = "normal"
  SCENE_PEDESTRIAN = "pedestrian_area"
  SCENE_TRAFFIC_JAM = "traffic_jam"
  SCENE_HIGHWAY = "highway"

  # ===== 物体 =====
  OBJECT_TYPE_CAR = 0
  OBJECT_TYPE_BIKE = 1
  OBJECT_TYPE_PEDESTRIAN = 2

  def __init__(self):
    self.objects = []
    self.scene_type = self.SCENE_NORMAL

    self.pedestrian_density = 0.0
    self.traffic_density = 0.0
    self.danger_level = "low"

    # ===== 防抖状态机 =====
    self.brake_counter = 0
    self.slow_counter = 0
    self.brake_active = False
    self.slow_active = False

  # =========================
  def update(self, sm):
    self.objects = self._process_objects(sm)
    self.scene_type = self._classify_scene(sm)
    self._assess_danger()
    return self.scene_type, self.objects, {}

  # =========================
  def _process_objects(self, sm):
    objs = []

    try:
      model = sm['modelV2']

      if hasattr(model, 'objects'):
        for o in model.objects[:10]:
          if o.confidence < 0.5:
            continue

          obj = {
            "x": o.x,
            "y": o.y,
            "type": o.type,
            "confidence": o.confidence
          }

          obj["behavior"] = self._predict_behavior(obj, sm)
          objs.append(obj)

    except:
      pass

    # ===== 密度 =====
    ped = [o for o in objs if o["type"] == self.OBJECT_TYPE_PEDESTRIAN]
    self.pedestrian_density = min(len(ped) / 5.0, 1.0)
    self.traffic_density = min(len(objs) / 10.0, 1.0)

    return objs

  # =========================
  def _predict_behavior(self, obj, sm):
    behavior = {
      "crossing_probability": 0.0,
      "risk_level": "low",
      "time_to_collision": float('inf')
    }

    x = obj["x"]
    y = obj["y"]
    t = obj["type"]

    v_ego = 0.0
    try:
      v_ego = sm['carState'].vEgo
    except:
      pass

    # ===== 行人逻辑 =====
    if t == self.OBJECT_TYPE_PEDESTRIAN:

      if abs(y) < 1.0:
        behavior["crossing_probability"] = 0.7
      elif abs(y) < 2.0:
        behavior["crossing_probability"] = 0.4

      # 远距离衰减（防误刹）
      if x > 40:
        behavior["crossing_probability"] *= 0.5

      if x < 25 and behavior["crossing_probability"] > 0.5:
        behavior["risk_level"] = "high"

    # ===== TTC =====
    if x > 0 and v_ego > 0:
      behavior["time_to_collision"] = x / max(v_ego, 0.1)

    return behavior

  # =========================
  def _classify_scene(self, sm):

    if self.pedestrian_density > 0.5:
      return self.SCENE_PEDESTRIAN

    if self.traffic_density > 0.7:
      return self.SCENE_TRAFFIC_JAM

    try:
      if sm['carState'].vEgo > 30:
        return self.SCENE_HIGHWAY
    except:
      pass

    return self.SCENE_NORMAL

  # =========================
  def _assess_danger(self):
    score = 0.0

    for o in self.objects:
      b = o["behavior"]

      if b["crossing_probability"] > 0.5 and o["x"] < 30:
        score += 2

      if b["time_to_collision"] < 3:
        score += 3

    if score > 6:
      self.danger_level = "high"
    elif score > 3:
      self.danger_level = "medium"
    else:
      self.danger_level = "low"

  # =========================
  # 🚦 稳定刹车
  # =========================
  def should_brake(self):

    close = [o for o in self.objects if 0 < o["x"] < 35]

    if not close:
      self.brake_counter = max(0, self.brake_counter - 1)
      if self.brake_counter == 0:
        self.brake_active = False
      return self.brake_active

    o = min(close, key=lambda x: x["x"])
    b = o["behavior"]

    ttc = b.get("time_to_collision", 999)
    cross = b.get("crossing_probability", 0)

    trigger = False

    # 🚶 行人
    if o["type"] == self.OBJECT_TYPE_PEDESTRIAN:
      if cross > 0.7 and o["x"] < 25:
        trigger = True

    # 🚗 紧急情况
    if ttc < 1.8:
      trigger = True

    # 防抖
    if trigger:
      self.brake_counter += 1
    else:
      self.brake_counter = max(0, self.brake_counter - 1)

    if self.brake_counter >= 3:
      self.brake_active = True

    if self.brake_counter == 0:
      self.brake_active = False

    return self.brake_active

  # =========================
  # 🚗 稳定减速
  # =========================
  def should_slow_down(self):

    trigger = False

    for o in self.objects:
      b = o["behavior"]
      cross = b.get("crossing_probability", 0)

      if o["type"] == self.OBJECT_TYPE_PEDESTRIAN:
        if 0.4 < cross < 0.7 and o["x"] < 35:
          trigger = True

    # 防抖
    if trigger:
      self.slow_counter += 1
    else:
      self.slow_counter = max(0, self.slow_counter - 1)

    if self.slow_counter >= 3:
      self.slow_active = True

    if self.slow_counter == 0:
      self.slow_active = False

    return self.slow_active

  # =========================
  def get_scene_info(self):
    return {
      "scene_type": self.scene_type,
      "pedestrian_density": self.pedestrian_density,
      "traffic_density": self.traffic_density,
      "danger_level": self.danger_level
    }
