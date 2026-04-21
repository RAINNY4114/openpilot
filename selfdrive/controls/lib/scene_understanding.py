#!/usr/bin/env python3
"""
场景理解模块
负责统一环境感知和场景分类，为综合决策提供一致的场景理解
"""

import numpy as np


class SceneUnderstanding:
    """场景理解类"""
    
    # 场景类型定义
    SCENE_NORMAL = "normal"
    SCENE_INTERSECTION = "intersection"
    SCENE_LANE_CHANGE = "lane_change"
    SCENE_CURVE = "curve"
    SCENE_OBSTACLE = "obstacle_avoidance"
    SCENE_TRAFFIC_JAM = "traffic_jam"
    SCENE_EMERGENCY = "emergency"
    # 新增场景类型
    SCENE_PEDESTRIAN = "pedestrian_area"
    SCENE_TWO_WHEELER = "two_wheeler_area"
    SCENE_CONSTRUCTION = "construction_zone"
    SCENE_HIGHWAY = "highway"
    SCENE_URBAN = "urban"
    SCENE_RURAL = "rural"
    SCENE_TRAFFIC_LIGHT = "traffic_light"
    # 扩展场景类型
    SCENE_TUNNEL = "tunnel"
    SCENE_BRIDGE = "bridge"
    SCENE_PARKING_LOT = "parking_lot"
    SCENE_ROUNDABOUT = "roundabout"
    SCENE_SCHOOL_ZONE = "school_zone"
    SCENE_RESIDENTIAL = "residential"
    
    # 物体类型定义
    OBJECT_TYPE_CAR = 0
    OBJECT_TYPE_BIKE = 1
    OBJECT_TYPE_PEDESTRIAN = 2
    OBJECT_TYPE_CONE = 3
    OBJECT_TYPE_MOTORCYCLE = 4
    
    # 危险等级
    DANGER_LEVEL_LOW = "low"
    DANGER_LEVEL_MEDIUM = "medium"
    DANGER_LEVEL_HIGH = "high"
    DANGER_LEVEL_CRITICAL = "critical"
    
    def __init__(self):
        """初始化场景理解模块"""
        self.objects = []
        self.road_conditions = {}
        self.scene_type = self.SCENE_NORMAL
        self.curve_severity = 0.0  # 弯道严重程度
        self.obstacle_distance = float('inf')  # 障碍物距离
        self.traffic_density = 0.0  # 交通密度
        # 物体计数
        self.object_counts = {
            'cars': 0,
            'bikes': 0,
            'pedestrians': 0,
            'cones': 0,
            'motorcycles': 0
        }
        self.danger_level = self.DANGER_LEVEL_LOW
        self.pedestrian_density = 0.0
        self.two_wheeler_density = 0.0
        self.scene_specific_info = {}  # 场景特定信息
    
    def update(self, sm):
        """更新场景理解
        
        Args:
            sm: 传感器数据
            
        Returns:
            scene_type: 场景类型
            objects: 物体列表
            road_conditions: 道路条件
        """
        # 检查sm是否有services属性（SubMaster实例）
        has_modelv2 = hasattr(sm, 'services') and 'modelV2' in sm.services
        has_carstate = hasattr(sm, 'services') and 'carState' in sm.services
        
        # 处理环境信息
        try:
            self.objects = self._process_objects(sm) if has_modelv2 else []
            self.road_conditions = self._analyze_road_conditions(sm) if has_modelv2 else {}
        except Exception:
            self.objects = []
            self.road_conditions = {}
        
        # 分类场景
        self.scene_type = self._classify_scene(sm, has_carstate)
        
        # 评估危险等级
        self._assess_danger_level()
        
        # 更新场景特定信息
        self._update_scene_specific_info()
        
        return self.scene_type, self.objects, self.road_conditions
    
    def _process_objects(self, sm):
        """处理物体信息"""
        objects = []
        
        # 重置物体计数
        for key in self.object_counts:
            self.object_counts[key] = 0
        
        # 从modelV2获取物体信息
        try:
            model_v2 = sm['modelV2']
            if hasattr(model_v2, 'objects'):
                # 限制处理的物体数量，提高性能
                processed_objects = 0
                max_objects = 10  # 最多处理10个物体
                
                for obj in model_v2.objects:
                    if processed_objects >= max_objects:
                        break
                    
                    if obj.confidence > 0.5:
                        obj_info = {
                            'x': obj.x,
                            'y': obj.y,
                            'confidence': obj.confidence,
                            'type': obj.type
                        }
                        # 预测物体行为（简化版）
                        obj_info['behavior'] = self._predict_object_behavior(obj_info, sm)
                        objects.append(obj_info)
                        
                        # 统计物体类型
                        if obj.type == self.OBJECT_TYPE_CAR:
                            self.object_counts['cars'] += 1
                        elif obj.type == self.OBJECT_TYPE_BIKE:
                            self.object_counts['bikes'] += 1
                        elif obj.type == self.OBJECT_TYPE_PEDESTRIAN:
                            self.object_counts['pedestrians'] += 1
                        elif obj.type == self.OBJECT_TYPE_CONE:
                            self.object_counts['cones'] += 1
                        elif obj.type == self.OBJECT_TYPE_MOTORCYCLE:
                            self.object_counts['motorcycles'] += 1
                        
                        processed_objects += 1
        except Exception:
            pass
        
        # 计算障碍物距离
        if objects:
            self.obstacle_distance = min(obj['x'] for obj in objects if obj['x'] > 0)
        else:
            self.obstacle_distance = float('inf')
        
        # 计算交通密度
        self.traffic_density = min(len(objects) / 10.0, 1.0)  # 限制最大密度为1.0
        
        # 计算行人密度和两轮车密度
        self.pedestrian_density = min(self.object_counts['pedestrians'] / 5.0, 1.0)
        self.two_wheeler_density = min((self.object_counts['bikes'] + self.object_counts['motorcycles']) / 5.0, 1.0)
        
        return objects
    
    def _analyze_road_conditions(self, sm):
        """分析道路条件"""
        road_conditions = {
            'lane_width': 3.5,  # 默认车道宽度
            'road_type': 'urban',  # 默认道路类型
            'curvature': 0.0,  # 道路曲率
            'lane_lines_detected': False,  # 车道线检测状态
        }
        
        # 1. 尝试从YOLO车道线输出获取道路信息
        try:
            model_v2 = sm['modelV2']
            if hasattr(model_v2, 'yolo_lane_lines'):
                yolo_lanes = model_v2.yolo_lane_lines
                if yolo_lanes.size > 0:
                    # 使用YOLO车道线计算车道宽度
                    # 假设车道线是左右成对的
                    left_lanes = [lane for lane in yolo_lanes if lane['class_id'] == 2]  # 假设2是左侧车道线
                    right_lanes = [lane for lane in yolo_lanes if lane['class_id'] == 3]  # 假设3是右侧车道线
                    
                    if left_lanes and right_lanes:
                        # 选择最接近中心的左右车道线
                        left_lane = min(left_lanes, key=lambda x: abs(x['y']))
                        right_lane = min(right_lanes, key=lambda x: abs(x['y']))
                        
                        # 计算车道宽度
                        lane_width = abs(right_lane['y'] - left_lane['y'])
                        if 2.5 < lane_width < 5.0:
                            road_conditions['lane_width'] = lane_width
                        
                        # 标记车道线检测状态
                        road_conditions['lane_lines_detected'] = True
                        
                        # 基于YOLO车道线计算曲率
                        # 这里可以根据车道线的位置和角度计算曲率
                        # 暂时使用简化方法
                        if len(yolo_lanes) >= 2:
                            # 计算车道线的平均角度
                            lane_angles = []
                            for lane in yolo_lanes:
                                # 基于车道线的宽度和高度计算角度
                                if lane['width'] > 0 and lane['height'] > 0:
                                    angle = np.arctan(lane['height'] / lane['width'])
                                    lane_angles.append(angle)
                            if lane_angles:
                                avg_angle = np.mean(lane_angles)
                                # 基于角度估算曲率
                                curvature = abs(np.sin(avg_angle)) * 0.01
                                road_conditions['curvature'] = curvature
                                self.curve_severity = min(max(curvature * 1000, 0.0), 1.0)
                                return self.curve_severity > 0.2
        except Exception:
            pass
        
        # 2. 如果YOLO车道线不可用，使用传统方法
        if not road_conditions['lane_lines_detected']:
            try:
                model_v2 = sm['modelV2']
                
                # 分析道路曲率（简化版）
                if hasattr(model_v2, 'position'):
                    pos = model_v2.position
                    if hasattr(pos, 'x') and hasattr(pos, 'y'):
                        xs = np.array(pos.x)
                        ys = np.array(pos.y)
                        if len(xs) >= 3 and len(ys) >= 3:
                            # 计算简化曲率
                            dx = np.gradient(xs)
                            dy = np.gradient(ys)
                            curvature = np.abs(dy / (dx**2 + dy**2)**0.5) if np.any(dx) else 0.0
                            if len(curvature) > 0:
                                road_conditions['curvature'] = float(np.max(curvature))
                                self.curve_severity = road_conditions['curvature']
                
                # 尝试从laneLines获取车道宽度
                if hasattr(model_v2, 'laneLines') and len(model_v2.laneLines) >= 4:
                    left_lane = model_v2.laneLines[1]
                    right_lane = model_v2.laneLines[2]
                    if hasattr(left_lane, 'y') and hasattr(right_lane, 'y'):
                        if len(left_lane.y) > 0 and len(right_lane.y) > 0:
                            # 计算车道宽度
                            lane_width = abs(right_lane.y[0] - left_lane.y[0])
                            if 2.5 < lane_width < 5.0:
                                road_conditions['lane_width'] = lane_width
                            road_conditions['lane_lines_detected'] = True
            except Exception:
                pass
        
        # 3. 根据速度判断道路类型
        try:
            v_ego = sm['carState'].vEgo
            if v_ego > 30:
                road_conditions['road_type'] = 'highway'
            elif v_ego > 15:
                road_conditions['road_type'] = 'rural'
        except Exception:
            pass
        
        return road_conditions
    
    def _classify_scene(self, sm, has_carstate):
        """分类场景"""
        # 检查紧急情况
        if self._is_emergency(sm):
            return self.SCENE_EMERGENCY
        
        # 检查交叉口
        if self._is_intersection():
            return self.SCENE_INTERSECTION
        
        # 检查变道
        if self._is_lane_change(sm, has_carstate):
            return self.SCENE_LANE_CHANGE
        
        # 检查弯道
        if self._is_curve():
            return self.SCENE_CURVE
        
        # 检查障碍物
        if self._is_obstacle():
            return self.SCENE_OBSTACLE

        # 隧道优先级高于拥堵
        if self._is_tunnel():
            return self.SCENE_TUNNEL
        
        # 检查交通拥堵
        if self._is_traffic_jam():
            return self.SCENE_TRAFFIC_JAM
        
        # 检查行人密集区
        if self._is_pedestrian_area():
            return self.SCENE_PEDESTRIAN
        
        # 检查两轮车密集区
        if self._is_two_wheeler_area():
            return self.SCENE_TWO_WHEELER
        
        # 检查施工区域
        if self._is_construction_zone():
            return self.SCENE_CONSTRUCTION
        
        # 检查交通灯场景
        if self._is_traffic_light():
            return self.SCENE_TRAFFIC_LIGHT
        
        # 检查高速场景
        if self._is_highway(sm, has_carstate):
            return self.SCENE_HIGHWAY
        
        # 检查城市场景
        if self._is_urban(sm, has_carstate):
            return self.SCENE_URBAN
        
        # 检查乡村场景
        if self._is_rural(sm, has_carstate):
            return self.SCENE_RURAL
                      
        # 检查桥梁场景
        if self._is_bridge():
            return self.SCENE_BRIDGE
        
        # 检查停车场场景
        if self._is_parking_lot():
            return self.SCENE_PARKING_LOT
        
        # 检查环岛场景
        if self._is_roundabout():
            return self.SCENE_ROUNDABOUT
        
        # 检查学校区域
        if self._is_school_zone():
            return self.SCENE_SCHOOL_ZONE
        
        # 检查 residential 区域
        if self._is_residential(sm, has_carstate):
            return self.SCENE_RESIDENTIAL
        
        # 默认正常场景
        return self.SCENE_NORMAL
    
    def _is_emergency(self, sm):
        """判断是否为紧急情况"""
        # 检查是否有紧急制动请求
        try:
            model_v2 = sm['modelV2']
            if hasattr(model_v2, 'action'):
                action = model_v2.action
                if hasattr(action, 'desiredAcceleration'):
                    if action.desiredAcceleration < -3.0:
                        return True
        except Exception:
            pass
        
        # 检查碰撞预警
        try:
            radar_state = sm['radarState']
            if hasattr(radar_state, 'leadOne'):
                lead = radar_state.leadOne
                if lead.status and lead.dRel < 10.0 and lead.vRel < -5.0:
                    return True
        except Exception:
            pass
        
        return False
    
    def _is_intersection(self):
        """判断是否为交叉口"""
        # 基于物体和道路条件判断交叉路口
        # 1. 检查是否有交通灯
        for obj in self.objects:
            if obj['type'] == 8:  # 交通灯类型
                return True
        
        # 2. 基于道路曲率和物体分布判断
        if len(self.objects) >= 3 and self.traffic_density > 0.3:
            # 检查是否有不同方向的车辆
            lateral_positions = [obj['y'] for obj in self.objects if obj['x'] > 0 and obj['x'] < 50]
            if len(lateral_positions) >= 3:
                # 检查横向分布是否广泛
                if max(lateral_positions) - min(lateral_positions) > 4.0:
                    return True
        
        return False
    
    def _is_lane_change(self, sm, has_carstate):
        """判断是否为变道场景"""
        if not has_carstate:
            return False
        
        # 检查转向灯状态
        try:
            car_state = sm['carState']
            if hasattr(car_state, 'leftBlindspotSignal') and hasattr(car_state, 'rightBlindspotSignal'):
                if car_state.leftBlindspotSignal or car_state.rightBlindspotSignal:
                    return True
        except Exception:
            pass
        
        return False
    
    def _is_curve(self):
        """判断是否为弯道场景"""
        # 基于曲率判断
        # 降低阈值以更好地检测小弯道和入弯前场景
        return self.curve_severity > 0.0015
    
    def is_approaching_curve(self):
        """判断是否为入弯前场景（即将进入弯道）"""
        # 基于曲率变化趋势判断
        # 如果曲率在增加但还未达到弯道阈值，可能是入弯前
        return 0.0005 < self.curve_severity <= 0.0015
    
    def _is_obstacle(self):
        """判断是否为障碍物场景"""
        # 基于障碍物距离判断
        return self.obstacle_distance < 20.0
    
    def _is_traffic_jam(self):
        """判断是否为交通拥堵场景"""
        # 基于交通密度判断
        return self.traffic_density > 0.7
    
    def _is_pedestrian_area(self):
        """判断是否为行人密集区"""
        return self.pedestrian_density > 0.5
    
    def _is_two_wheeler_area(self):
        """判断是否为两轮车密集区"""
        return self.two_wheeler_density > 0.8
    
    def _is_construction_zone(self):
        """判断是否为施工区域"""
        # 基于锥桶数量判断
        return self.object_counts['cones'] >= 3
    
    def _is_highway(self, sm, has_carstate):
        """判断是否为高速场景"""
        if not has_carstate:
            return False
        
        # 基于速度判断
        try:
            v_ego = sm['carState'].vEgo
            if v_ego > 30:  # 速度大于30 m/s (108 km/h)
                return True
        except Exception:
            pass
        
        return False
    
    def _is_urban(self, sm, has_carstate):
        """判断是否为城市场景"""
        if not has_carstate:
            return False
        
        # 基于速度和物体密度判断
        try:
            v_ego = sm['carState'].vEgo
            if v_ego < 20:  # 速度小于20 m/s (72 km/h)
                # 城市场景通常有更多的物体
                total_objects = sum(self.object_counts.values())
                if total_objects >= 3:
                    return True
        except Exception:
            pass
        
        return False
    
    def _is_rural(self, sm, has_carstate):
        """判断是否为乡村场景"""
        if not has_carstate:
            return False
        
        # 基于速度和物体密度判断
        try:
            v_ego = sm['carState'].vEgo
            if 20 <= v_ego <= 30:  # 速度在20-30 m/s之间
                # 乡村场景物体较少
                total_objects = sum(self.object_counts.values())
                if total_objects < 3:
                    return True
        except Exception:
            pass
        
        return False
    
    def _is_traffic_light(self):
        """判断是否为交通灯场景"""
        # 基于物体类型判断
        return any(obj['type'] == 8 for obj in self.objects if obj['confidence'] > 0.5)  # 8是交通灯类型
    
    def _predict_object_behavior(self, obj_info, sm):
        """预测物体行为"""
        behavior = {
            'predicted_movement': 'stationary',
            'risk_level': 'low',
            'time_to_collision': float('inf'),
            'crossing_probability': 0.0,
            'relative_speed': 0.0,
            'relative_acceleration': 0.0,
            'movement_confidence': 0.5
        }
        
        # 获取本车状态
        v_ego = 0.0
        a_ego = 0.0
        try:
            car_state = sm['carState']
            v_ego = car_state.vEgo
            a_ego = car_state.aEgo
        except Exception:
            pass
        
        # 计算物体与本车的相对距离
        obj_distance = obj_info['x']
        obj_lateral = obj_info['y']
        
        # 基于物体类型预测行为
        obj_type = obj_info['type']
        obj_confidence = obj_info.get('confidence', 0.5)
        
        # 估计物体速度和加速度（基于位置变化和类型特性）
        estimated_obj_speed = 0.0
        estimated_obj_accel = 0.0
        
        if obj_type == self.OBJECT_TYPE_PEDESTRIAN:
            # 行人行为预测
            estimated_obj_speed = 1.4  # 行人平均速度
            behavior['crossing_probability'] = 0.3
            
            # 基于横向位置调整穿越概率
            if abs(obj_lateral) < 1.0:
                behavior['crossing_probability'] = 0.7
            elif abs(obj_lateral) < 2.0:
                behavior['crossing_probability'] = 0.5
            
            # 基于距离预测运动
            if obj_distance < 30:
                behavior['predicted_movement'] = 'potential_crossing'
                behavior['risk_level'] = 'medium'
                if behavior['crossing_probability'] > 0.5:
                    behavior['risk_level'] = 'high'
                    behavior['movement_confidence'] = 0.8
            else:
                behavior['predicted_movement'] = 'walking'
                behavior['movement_confidence'] = 0.6
        
        elif obj_type == self.OBJECT_TYPE_BIKE or obj_type == self.OBJECT_TYPE_MOTORCYCLE:
            # 两轮车行为预测
            estimated_obj_speed = 10.0 if obj_type == self.OBJECT_TYPE_BIKE else 15.0
            behavior['predicted_movement'] = 'unpredictable'
            behavior['risk_level'] = 'medium'
            behavior['movement_confidence'] = 0.7
            
            if obj_distance < 20:
                behavior['risk_level'] = 'high'
                behavior['movement_confidence'] = 0.9
            elif obj_distance < 40:
                behavior['risk_level'] = 'medium'
        
        elif obj_type == self.OBJECT_TYPE_CAR:
            # 汽车行为预测 - 添加掉头检测
            estimated_obj_speed = v_ego  # 假设同速
            behavior['predicted_movement'] = 'following_lane'
            behavior['risk_level'] = 'low'
            behavior['movement_confidence'] = 0.8
            
            # 检查是否有掉头标志（如果从雷达数据中获取）
            is_turning_around = obj_info.get('isTurningAround', False)
            turning_around_prob = obj_info.get('turningAroundProb', 0.0)
            lateral_movement_pattern = obj_info.get('lateralMovementPattern', 0)
            
            # 基于横向位置和距离检测可能的掉头
            if (abs(obj_lateral) > 1.5 and obj_distance < 50) or is_turning_around:
                # 可能在掉头
                estimated_obj_speed = 5.0  # 掉头时速度较低
                behavior['predicted_movement'] = 'turning_around'
                behavior['risk_level'] = 'high'
                behavior['movement_confidence'] = 0.7
                behavior['crossing_probability'] = 0.8
                
                if obj_distance < 30:
                    behavior['risk_level'] = 'critical'
                    behavior['movement_confidence'] = 0.9
                elif obj_distance < 40:
                    behavior['risk_level'] = 'high'
            
            # 基于掉头概率调整风险
            if turning_around_prob > 0.7:
                behavior['predicted_movement'] = 'turning_around'
                behavior['risk_level'] = 'critical' if obj_distance < 30 else 'high'
                behavior['crossing_probability'] = 0.9
                behavior['movement_confidence'] = 0.8
            elif turning_around_prob > 0.3:
                behavior['predicted_movement'] = 'potential_turning'
                behavior['risk_level'] = 'high' if obj_distance < 30 else 'medium'
                behavior['crossing_probability'] = 0.6
                behavior['movement_confidence'] = 0.6
            
            # 正常跟随模式的处理
            if behavior['predicted_movement'] == 'following_lane':
                if obj_distance < 15:
                    behavior['risk_level'] = 'medium'
                    behavior['movement_confidence'] = 0.9
                elif obj_distance < 30:
                    behavior['risk_level'] = 'low'
        
        # 计算相对速度和加速度
        behavior['relative_speed'] = estimated_obj_speed - v_ego
        behavior['relative_acceleration'] = estimated_obj_accel - a_ego
        
        # 计算碰撞时间
        if v_ego > estimated_obj_speed and obj_distance > 0:
            time_to_collision = obj_distance / (v_ego - estimated_obj_speed)
            behavior['time_to_collision'] = time_to_collision
            
            # 基于碰撞时间调整风险等级
            if time_to_collision < 2:
                behavior['risk_level'] = 'critical'
            elif time_to_collision < 4:
                behavior['risk_level'] = 'high'
            elif time_to_collision < 7:
                behavior['risk_level'] = 'medium'
        
        # 基于物体置信度调整风险等级
        behavior['risk_level'] = self._adjust_risk_by_confidence(behavior['risk_level'], obj_confidence)
        
        # 考虑复杂交互场景
        behavior = self._consider_complex_interactions(behavior, obj_info, sm)
        
        return behavior
    
    def _adjust_risk_by_confidence(self, risk_level, confidence):
        """基于物体置信度调整风险等级"""
        if confidence < 0.6:
            # 低置信度时降低风险等级
            if risk_level == 'critical':
                return 'high'
            elif risk_level == 'high':
                return 'medium'
            elif risk_level == 'medium':
                return 'low'
        elif confidence > 0.8:
            # 高置信度时提高风险等级
            if risk_level == 'low':
                return 'medium'
            elif risk_level == 'medium':
                return 'high'
        return risk_level
    
    def _consider_complex_interactions(self, behavior, obj_info, sm):
        """考虑复杂的物体交互场景"""
        # 检查是否有其他物体影响当前物体的行为
        obj_x = obj_info['x']
        obj_y = obj_info['y']
        
        # 检查是否有其他物体在附近
        nearby_objects = []
        try:
            model_v2 = sm['modelV2']
            if hasattr(model_v2, 'objects'):
                for other_obj in model_v2.objects:
                    if other_obj.confidence > 0.5:
                        other_x = other_obj.x
                        other_y = other_obj.y
                        distance = ((other_x - obj_x)**2 + (other_y - obj_y)**2)**0.5
                        if distance < 10.0 and distance > 0:
                            nearby_objects.append({
                                'x': other_x,
                                'y': other_y,
                                'type': other_obj.type,
                                'confidence': other_obj.confidence
                            })
        except Exception:
            pass
        
        # 基于附近物体调整行为预测
        if nearby_objects:
            # 有其他物体在附近，增加不可预测性
            behavior['movement_confidence'] *= 0.7
            if behavior['risk_level'] == 'low':
                behavior['risk_level'] = 'medium'
            elif behavior['risk_level'] == 'medium':
                behavior['risk_level'] = 'high'
            
            # 检查是否有对向物体
            for other_obj in nearby_objects:
                if other_obj['x'] < obj_x and abs(other_obj['y'] - obj_y) < 2.0:
                    # 有对向物体，增加碰撞风险
                    behavior['crossing_probability'] = min(behavior['crossing_probability'] + 0.3, 1.0)
                    if behavior['risk_level'] != 'critical':
                        behavior['risk_level'] = 'high'
        
        return behavior
    
    def _update_scene_specific_info(self):
        """更新场景特定信息"""
        self.scene_specific_info = {}
        
        if self.scene_type == self.SCENE_PEDESTRIAN:
            self.scene_specific_info = {
                'pedestrian_count': self.object_counts['pedestrians'],
                'recommended_speed': min(30, 40 - self.pedestrian_density * 10)
            }
        elif self.scene_type == self.SCENE_TWO_WHEELER:
            self.scene_specific_info = {
                'two_wheeler_count': self.object_counts['bikes'] + self.object_counts['motorcycles'],
                'recommended_speed': min(40, 50 - self.two_wheeler_density * 5)
            }
        elif self.scene_type == self.SCENE_CONSTRUCTION:
            self.scene_specific_info = {
                'cone_count': self.object_counts['cones'],
                'recommended_speed': 20
            }
        elif self.scene_type == self.SCENE_CURVE:
            self.scene_specific_info = {
                'curve_severity': self.curve_severity,
                'recommended_speed': max(40, 80 - self.curve_severity * 2000)
            }
        elif self.scene_type == self.SCENE_TRAFFIC_JAM:
            self.scene_specific_info = {
                'traffic_density': self.traffic_density,
                'recommended_speed': max(5, 30 - self.traffic_density * 20)
            }
        # 新增场景特定信息
        elif self.scene_type == self.SCENE_TUNNEL:
            target_speed = 90   # km/h
             
            # 初始化或获取历史速度
            current_speed = self.scene_specific_info.get("recommended_speed", 100)
             
            if current_speed > target_speed:
                speed = 90 + (current_speed - 90) * 0.9
            else:
                speed = current_speed

            self.scene_specific_info = {
                'object_count': len(self.objects),
                'recommended_speed': speed
            }
        elif self.scene_type == self.SCENE_BRIDGE:
            self.scene_specific_info = {
                'lane_width': self.road_conditions.get('lane_width', 3.5),
                'recommended_speed': 50
            }
        elif self.scene_type == self.SCENE_PARKING_LOT:
            self.scene_specific_info = {
                'car_count': self.object_counts['cars'],
                'recommended_speed': 15
            }
        elif self.scene_type == self.SCENE_ROUNDABOUT:
            self.scene_specific_info = {
                'curve_severity': self.curve_severity,
                'object_count': len(self.objects),
                'recommended_speed': 25
            }
        elif self.scene_type == self.SCENE_SCHOOL_ZONE:
            self.scene_specific_info = {
                'pedestrian_count': self.object_counts['pedestrians'],
                'recommended_speed': 20
            }
        elif self.scene_type == self.SCENE_RESIDENTIAL:
            self.scene_specific_info = {
                'object_count': len(self.objects),
                'recommended_speed': 30
            }
    
    def _assess_danger_level(self):
        """评估危险等级"""
        danger_score = 0.0
        
        # 基于障碍物距离
        if self.obstacle_distance < 10:
            danger_score += 4.0
        elif self.obstacle_distance < 30:
            danger_score += 2.0
        elif self.obstacle_distance < 50:
            danger_score += 1.0
        
        # 基于行人密度
        danger_score += self.pedestrian_density * 2.0
        
        # 基于两轮车密度
        danger_score += self.two_wheeler_density * 1.5
        
        # 基于弯道严重程度
        danger_score += self.curve_severity * 100.0
        
        # 基于场景类型
        if self.scene_type == self.SCENE_EMERGENCY:
            danger_score += 5.0
        elif self.scene_type == self.SCENE_PEDESTRIAN:
            danger_score += 3.0
        elif self.scene_type == self.SCENE_TWO_WHEELER:
            danger_score += 2.0
        elif self.scene_type == self.SCENE_INTERSECTION:
            danger_score += 2.0
        
        # 基于物体行为预测（简化版）
        for obj in self.objects[:5]:  # 只处理前5个物体
            behavior = obj.get('behavior', {})
            obj_risk = behavior.get('risk_level', 'low')
            
            # 基于物体风险等级
            if obj_risk == 'critical':
                danger_score += 3.0
            elif obj_risk == 'high':
                danger_score += 2.0
            elif obj_risk == 'medium':
                danger_score += 1.0
        
        # 确定危险等级
        if danger_score >= 10.0:
            self.danger_level = self.DANGER_LEVEL_CRITICAL
        elif danger_score >= 6.0:
            self.danger_level = self.DANGER_LEVEL_HIGH
        elif danger_score >= 3.0:
            self.danger_level = self.DANGER_LEVEL_MEDIUM
        else:
            self.danger_level = self.DANGER_LEVEL_LOW
    
    def get_scene_info(self):
        """获取场景信息"""
        return {
            'scene_type': self.scene_type,
            'curve_severity': self.curve_severity,
            'obstacle_distance': self.obstacle_distance,
            'traffic_density': self.traffic_density,
            'pedestrian_density': self.pedestrian_density,
            'two_wheeler_density': self.two_wheeler_density,
            'object_counts': self.object_counts,
            'danger_level': self.danger_level,
            'scene_specific_info': self.scene_specific_info
        }
    
    def _is_tunnel(self):
        """判断是否为隧道场景"""
        # 基于环境特征判断
        # 隧道内通常光线较暗，物体较少
        # 简化判断：基于物体数量和交通密度
        return (
                len(self.objects) < 1 and
                self.traffic_density < 0.15 and
                self.curve_severity < 0.001
         )
    
    def _is_bridge(self):
        """判断是否为桥梁场景"""
        # 基于道路条件判断
        # 桥梁通常有特定的道路特征
        # 简化判断：基于车道宽度和曲率
        try:
            lane_width = self.road_conditions.get('lane_width', 3.5)
            curvature = self.road_conditions.get('curvature', 0.0)
            # 桥梁通常车道较宽且可能有一定曲率
            return 3.5 < lane_width < 4.5 and curvature > 0.001
        except:
            return False
    
    def _is_parking_lot(self):
        """判断是否为停车场场景"""
        # 基于物体分布和速度判断
        # 停车场通常有较多静止车辆，速度较低
        try:
            # 检查是否有多个静止或低速车辆
            stationary_cars = 0
            for obj in self.objects:
                if obj['type'] == self.OBJECT_TYPE_CAR and obj['x'] < 50:
                    stationary_cars += 1
            # 停车场通常有3个以上的近距离车辆
            return ( 
                    stationary_cars >= 3 and
                    self.traffic_density < 0.3 and
                    self.obstacle_distance < 25 and
                    self.curve_severity < 0.001
            )
        except:
            return False
    
    def _is_roundabout(self):
        """判断是否为环岛场景"""
        # 基于道路曲率和物体分布判断
        # 环岛通常有连续的曲率和特定的物体分布
        return self.curve_severity > 0.002 and len(self.objects) >= 2
    
    def _is_school_zone(self):
        """判断是否为学校区域"""
        # 基于行人密度和速度判断
        # 学校区域通常有较多行人，速度较低
        return self.pedestrian_density > 0.3
    
    def _is_residential(self, sm, has_carstate):
        """判断是否为 residential 区域"""
        # 基于速度和物体密度判断
        # residential 区域通常速度较低，有一定数量的物体
        if not has_carstate:
            return False
        
        try:
            v_ego = sm['carState'].vEgo
            # residential 区域通常速度在10-20 m/s之间，有一定数量的物体
            return 10 <= v_ego <= 20 and 1 <= len(self.objects) <= 5
        except (KeyError, AttributeError):
            return False
