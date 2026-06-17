import json
import math
import os
import time
from my_udp import UDPClient


class Control:
    def __init__(self):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001
        self.server_ip = '192.168.1.100'

        net = "RST6phjHTlYisx4dqWmCnNpkYfTV,127.0.0.1,2076,2077"
        if net != "":
            net = net.split(",")
            self.vehicle_name = net[0]
            self.server_ip = net[1]
            self.udp_port = int(net[2])
            self.udp_send_port = int(net[3])

        print(self.vehicle_name)
        print(self.udp_port)
        print(self.udp_send_port)
        print(self.server_ip)
        self.udp_client = UDPClient(self.server_ip, self.udp_port, self.udp_send_port, self.vehicle_name)

        self.m_v = 0
        self.m_x = 0
        self.m_y = 0
        self.m_yaw = 0
        self.vehpos_initial_index = 0
        self.num_preview = 8
        self.targetPos_Info = [0.0, 0.0]
        self.target_pos_index = 0
        self.Y_points = []
        self.X_points = []
        self.is_closed_path = False
        self.control_rate = 15  # hz
        self.wheel_base = 2.7
        self.min_lookahead = 5.0
        self.max_lookahead = 16.0
        self.base_lookahead = 4.0
        self.speed_lookahead_gain = 0.45
        self.curvature_lookahead_gain = 12.0
        self.lookahead_distance = self.min_lookahead
        self.max_steer = math.radians(30.0)
        self.max_steer_rate = math.radians(120.0)
        self.last_steering_angle = 0.0
        self.min_speed = 4.0
        self.max_lat_acc = 2.5
        self.last_v_cmd = 0.0
        self.last_w_cmd = 0.0
        self.current_v_ref = 0.0
        self.ego_speed = 0.0
        self.frame_count = 0

        # Multi-vehicle safety layer parameters.
        self.behavior_state = "NORMAL_PATH_FOLLOW"
        self.stop_start_time = None
        self.front_block_start_time = None
        self.current_bypass_path = []
        self.current_bypass_index = 0
        self.previous_neighbor_states = {}
        self.last_front_gap = None
        self.last_ttc = None
        self.last_selected_action = "path_follow"
        self.last_candidate_reason = "not_checked"

        self.lane_width = 4.0
        self.conflict_width = self.lane_width * 1.25
        self.min_gap = 5.0
        self.time_headway = 1.5
        self.ttc_threshold = 2.5
        self.front_detect_dist = 25.0
        self.oncoming_ttc_threshold = 3.0
        self.oncoming_detect_dist = 35.0
        self.k_gap = 0.4
        self.low_speed_threshold = 2.0
        self.blocking_time_threshold = 2.0
        self.comfort_decel = 2.0
        self.stop_wait_time = 3.0
        self.lane_change_offset = 4.0
        self.lane_change_length = 25.0
        self.bypass_length = 35.0
        self.vehicle_safe_radius = 4.0
        self.prediction_horizon = 3.0
        self.prediction_dt = 0.3
        self.emergency_ttc = 1.0
        self.emergency_distance = 3.0

        # Boundary JSONs in this project use the same x-right/y-up map
        # coordinate convention as the route JSONs. If a future boundary file
        # comes from an image top-left origin, adjust these transform fields.
        self.map_boundaries = []
        self.map_boundary_path = None
        self.boundary_safe_margin = 2.0
        self.boundary_scale = 1.0
        self.boundary_offset_x = 0.0
        self.boundary_offset_y = 0.0
        self.boundary_flip_y = False
        self.boundary_image_height = 0.0
        self.default_map_boundary_files = [
            "green_edge_boundaries_px_bottom_left.json",
            "exp_routes/green_edge_boundaries_px_bottom_left.json",
            "map/green_edge_boundaries_px_bottom_left.json",
        ]

    def control_node(self):
        start_time = time.time()
        self.load_route('exp_routes/ccw_right_bottom_loop_closed.json')
        self.try_load_default_map_boundaries()
        while True:
            self.run_step()

            elapsed_time = time.time() - start_time
            sleep_time = max((1.0 / self.control_rate) - elapsed_time, 0.0)
            time.sleep(sleep_time)
            start_time = time.time()

    def run_step(self):
        self.frame_count += 1
        vehicle_data = self.udp_client.get_vehicle_state()
        self.m_x = vehicle_data.x
        self.m_y = vehicle_data.y
        self.m_yaw = vehicle_data.yaw / 180 * math.pi
        self.ego_speed = self.get_vehicle_speed(vehicle_data, self.last_v_cmd)
        self.m_v = 10
        self.update_vehpos_index()
        self.search_target_pos()

        v_ref, w_ref = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)
        v_cmd, w_cmd = self.multi_vehicle_safety_layer(v_ref, w_ref)
        self.last_v_cmd = v_cmd
        self.last_w_cmd = w_cmd
        self.udp_client.send_control_command(v_cmd, w_cmd)
        return v_cmd, w_cmd

    def load_route(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            json_track = json.load(file)

        if isinstance(json_track, list):
            self.X_points = [point["x"] for point in json_track]
            self.Y_points = [point["y"] for point in json_track]
        elif isinstance(json_track, dict) and "X" in json_track and "Y" in json_track:
            self.X_points = json_track["X"]
            self.Y_points = json_track["Y"]
        else:
            raise ValueError(
                "Unsupported route format. Expected [{'x': ..., 'y': ...}, ...] "
                "or {'X': [...], 'Y': [...]}."
            )

        if len(self.X_points) != len(self.Y_points) or len(self.X_points) == 0:
            raise ValueError("Route file must contain the same non-zero number of X and Y points.")

        self.X_points = [float(x) for x in self.X_points]
        self.Y_points = [float(y) for y in self.Y_points]
        self.is_closed_path = (
            len(self.X_points) >= 3
            and math.hypot(self.X_points[0] - self.X_points[-1], self.Y_points[0] - self.Y_points[-1]) < 1e-6
        )
        self.vehpos_initial_index = self.normalize_path_index(self.vehpos_initial_index)
        self.target_pos_index = self.vehpos_initial_index

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        if len(self.X_points) < 2 or self.wheel_base <= 1e-6:
            return 0.0, 0.0

        if target_pos is None or len(target_pos) < 2:
            return 0.0, 0.0

        tx = float(target_pos[0])
        ty = float(target_pos[1])
        dx = tx - m_x
        dy = ty - m_y
        target_distance = math.hypot(dx, dy)

        if target_distance < 1e-6:
            steering_angle = self.limit_steering_angle(0.0)
            v = max(0.0, self.m_v)
            w = v * math.tan(steering_angle) / self.wheel_base
            return v, w

        target_yaw = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_yaw - m_yaw)

        local_x = dx * math.cos(m_yaw) + dy * math.sin(m_yaw)
        target_is_behind = local_x < 0.0
        if target_is_behind:
            alpha = self.clamp(alpha, -math.pi / 2.0, math.pi / 2.0)

        lookahead_dist = max(target_distance, 1e-3)
        curvature = 2.0 * math.sin(alpha) / lookahead_dist
        steering_angle = math.atan(self.wheel_base * curvature)
        steering_angle = self.clamp(steering_angle, -self.max_steer, self.max_steer)
        steering_angle = self.limit_steering_angle(steering_angle)

        actual_curvature = abs(math.tan(steering_angle) / self.wheel_base)
        if actual_curvature > 1e-6:
            v_limit = math.sqrt(self.max_lat_acc / actual_curvature)
        else:
            v_limit = self.m_v

        if target_is_behind:
            v_limit = min(v_limit, self.min_speed)

        if self.m_v <= 0.0:
            v = 0.0
        else:
            min_speed = min(self.min_speed, self.m_v)
            v = self.clamp(min(self.m_v, v_limit), min_speed, self.m_v)

        w = v * math.tan(steering_angle) / self.wheel_base
        return v, w

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        if min_value > max_value:
            min_value, max_value = max_value, min_value
        return max(min_value, min(value, max_value))

    def get_time_now(self):
        return time.time()

    def get_vehicle_field(self, vehicle, field_name, default=None):
        if vehicle is None:
            return default
        if isinstance(vehicle, dict):
            return vehicle.get(field_name, default)
        return getattr(vehicle, field_name, default)

    def get_vehicle_speed(self, vehicle, default_speed=0.0):
        raw_speed = self.get_vehicle_field(vehicle, "speed", None)
        if raw_speed is None:
            raw_speed = self.get_vehicle_field(vehicle, "v", None)
        if raw_speed is None:
            raw_speed = self.get_vehicle_field(vehicle, "vx", None)
        try:
            speed = float(raw_speed)
        except (TypeError, ValueError):
            return max(0.0, float(default_speed or 0.0))
        return max(0.0, speed)

    def normalize_yaw_value(self, yaw, default_yaw=0.0):
        try:
            yaw_value = float(yaw)
        except (TypeError, ValueError):
            return default_yaw
        if abs(yaw_value) > 2.0 * math.pi:
            yaw_value = yaw_value / 180.0 * math.pi
        return self.normalize_angle(yaw_value)

    def get_neighbor_vehicles(self):
        raw_neighbors = self.udp_client.get_neighbor_vehicle_state()
        now = self.get_time_now()
        neighbor_vehicles = []

        for index, raw_vehicle in enumerate(raw_neighbors):
            name = self.get_vehicle_field(raw_vehicle, "name", str(index))
            x = self.get_vehicle_field(raw_vehicle, "x", None)
            y = self.get_vehicle_field(raw_vehicle, "y", None)
            if x is None or y is None:
                continue

            try:
                x = float(x)
                y = float(y)
            except (TypeError, ValueError):
                continue

            yaw = self.normalize_yaw_value(self.get_vehicle_field(raw_vehicle, "yaw", 0.0), 0.0)
            raw_speed = self.get_vehicle_field(raw_vehicle, "speed", None)
            speed = self.get_vehicle_speed(raw_vehicle, None)
            previous_state = self.previous_neighbor_states.get(name)
            if raw_speed is None and previous_state is not None:
                dt = max(now - previous_state["time"], 1e-3)
                speed = math.hypot(x - previous_state["x"], y - previous_state["y"]) / dt
            elif raw_speed is None:
                speed = 0.0

            x_rel, y_rel = self.transform_to_ego_frame(x, y)
            distance = math.hypot(x_rel, y_rel)
            heading_diff = abs(self.normalize_angle(yaw - self.m_yaw))
            vehicle = {
                "name": name,
                "x": x,
                "y": y,
                "yaw": yaw,
                "speed": speed,
                "x_rel": x_rel,
                "y_rel": y_rel,
                "distance": distance,
                "heading_diff": heading_diff,
                "raw": raw_vehicle,
                "classification": "IRRELEVANT",
                "ttc": float("inf"),
            }
            vehicle["classification"] = self.classify_neighbor_vehicle(vehicle)
            neighbor_vehicles.append(vehicle)
            self.previous_neighbor_states[name] = {
                "x": x,
                "y": y,
                "time": now,
                "speed": speed,
            }

        active_names = set(vehicle["name"] for vehicle in neighbor_vehicles)
        for old_name in list(self.previous_neighbor_states.keys()):
            if old_name not in active_names and now - self.previous_neighbor_states[old_name]["time"] > 5.0:
                del self.previous_neighbor_states[old_name]

        return neighbor_vehicles

    def transform_to_ego_frame(self, other_x, other_y):
        dx = other_x - self.m_x
        dy = other_y - self.m_y
        x_rel = math.cos(self.m_yaw) * dx + math.sin(self.m_yaw) * dy
        y_rel = -math.sin(self.m_yaw) * dx + math.cos(self.m_yaw) * dy
        return x_rel, y_rel

    def classify_neighbor_vehicle(self, other_vehicle):
        x_rel = other_vehicle["x_rel"]
        y_rel = other_vehicle["y_rel"]
        heading_diff = other_vehicle["heading_diff"]
        distance = other_vehicle["distance"]

        if (
            x_rel > 0.0
            and abs(y_rel) < self.lane_width / 2.0
            and heading_diff < math.radians(45.0)
            and x_rel < self.front_detect_dist
        ):
            return "FRONT_SAME_DIRECTION"

        if (
            x_rel > 0.0
            and abs(y_rel) < self.conflict_width
            and heading_diff > math.radians(120.0)
            and distance < self.oncoming_detect_dist
        ):
            return "ONCOMING_VEHICLE"

        if abs(x_rel) < self.lane_width and abs(y_rel) < self.lane_width * 1.5:
            return "SIDE_VEHICLE"

        if x_rel < 0.0 and abs(y_rel) < self.lane_width / 2.0 and distance < self.front_detect_dist:
            return "REAR_VEHICLE"

        return "IRRELEVANT"

    def compute_ttc(self, distance, closing_speed):
        if closing_speed <= 1e-6:
            return float("inf")
        return max(0.0, distance) / closing_speed

    def get_ego_collision_speed(self):
        return max(
            float(self.ego_speed or 0.0),
            float(self.last_v_cmd or 0.0),
            float(self.current_v_ref or 0.0),
            0.0,
        )

    def has_collision_risk(self, other_vehicle):
        classification = other_vehicle.get("classification", "IRRELEVANT")
        ego_speed = self.get_ego_collision_speed()

        if classification == "FRONT_SAME_DIRECTION":
            gap = max(0.0, other_vehicle["x_rel"])
            relative_speed = ego_speed - other_vehicle.get("speed", 0.0)
            ttc = self.compute_ttc(gap, relative_speed)
            safe_gap = self.min_gap + ego_speed * self.time_headway
            other_vehicle["ttc"] = ttc
            other_vehicle["safe_gap"] = safe_gap
            return gap < safe_gap or ttc < self.ttc_threshold

        if classification == "ONCOMING_VEHICLE":
            distance = max(0.0, other_vehicle["distance"])
            closing_speed = ego_speed + other_vehicle.get("speed", 0.0)
            ttc = self.compute_ttc(distance, closing_speed)
            other_vehicle["ttc"] = ttc
            return ttc < self.oncoming_ttc_threshold and distance < self.oncoming_detect_dist

        if classification == "SIDE_VEHICLE":
            other_vehicle["ttc"] = 0.0
            return other_vehicle["distance"] < self.vehicle_safe_radius

        return False

    def multi_vehicle_safety_layer(self, v_ref, w_ref):
        """
        Apply boundary, vehicle, following and bypass decisions after pure pursuit.
        The base path follower is unchanged; this layer only limits or replaces
        the command when a safety state requires it.
        """
        self.current_v_ref = v_ref
        self.last_selected_action = "path_follow"
        neighbor_vehicles = self.get_neighbor_vehicles()

        emergency_cmd = self.emergency_stop_if_needed(neighbor_vehicles, v_ref, w_ref)
        if emergency_cmd is not None:
            v_cmd, w_cmd = emergency_cmd
            self.print_behavior_debug(v_cmd, w_cmd)
            return v_cmd, w_cmd

        if not self.check_predicted_control_boundary(v_ref, w_ref):
            self.behavior_state = "EMERGENCY_STOP"
            self.last_selected_action = "boundary_stop"
            self.print_behavior_debug(0.0, 0.0)
            return 0.0, 0.0

        if self.current_bypass_path:
            active_cmd = self.execute_current_bypass_path(v_ref, neighbor_vehicles)
            if active_cmd is not None:
                v_cmd, w_cmd = active_cmd
                self.print_behavior_debug(v_cmd, w_cmd)
                return v_cmd, w_cmd

        risky_oncoming = self.find_nearest_risky_vehicle(neighbor_vehicles, "ONCOMING_VEHICLE")
        if risky_oncoming is not None:
            v_cmd, w_cmd = self.handle_oncoming_vehicle(risky_oncoming, v_ref, w_ref)
            self.print_behavior_debug(v_cmd, w_cmd)
            return v_cmd, w_cmd

        risky_front = self.find_nearest_risky_vehicle(neighbor_vehicles, "FRONT_SAME_DIRECTION")
        if risky_front is not None:
            v_cmd, w_cmd = self.handle_front_vehicle(risky_front, v_ref, w_ref)
            self.print_behavior_debug(v_cmd, w_cmd)
            return v_cmd, w_cmd

        risky_side = self.find_nearest_risky_vehicle(neighbor_vehicles, "SIDE_VEHICLE")
        if risky_side is not None:
            self.behavior_state = "FRONT_CAR_FOLLOW"
            self.last_selected_action = "side_vehicle_slow"
            self.print_behavior_debug(0.0, 0.0)
            return 0.0, 0.0

        if self.behavior_state == "RETURN_TO_ROUTE":
            route_distance = self.distance_to_current_route()
            if route_distance < self.lane_width / 2.0:
                self.behavior_state = "NORMAL_PATH_FOLLOW"

        self.stop_start_time = None
        self.front_block_start_time = None
        if self.behavior_state not in ("RETURN_TO_ROUTE", "NORMAL_PATH_FOLLOW"):
            self.behavior_state = "NORMAL_PATH_FOLLOW"
        self.print_behavior_debug(v_ref, w_ref)
        return v_ref, w_ref

    def find_nearest_risky_vehicle(self, neighbor_vehicles, classification):
        risky_vehicles = [
            vehicle for vehicle in neighbor_vehicles
            if vehicle.get("classification") == classification and self.has_collision_risk(vehicle)
        ]
        if not risky_vehicles:
            return None
        return min(risky_vehicles, key=lambda vehicle: vehicle.get("distance", float("inf")))

    def handle_front_vehicle(self, front_vehicle, v_ref, w_ref):
        gap = max(0.0, front_vehicle["x_rel"])
        front_speed = front_vehicle.get("speed", 0.0)
        ego_speed = self.get_ego_collision_speed()
        safe_gap = self.min_gap + ego_speed * self.time_headway
        relative_speed = ego_speed - front_speed
        ttc = self.compute_ttc(gap, relative_speed)
        self.last_front_gap = gap
        self.last_ttc = ttc

        if gap >= safe_gap and ttc >= self.ttc_threshold:
            self.behavior_state = "NORMAL_PATH_FOLLOW"
            self.front_block_start_time = None
            self.last_selected_action = "front_clear"
            return v_ref, w_ref

        self.behavior_state = "FRONT_CAR_FOLLOW"
        gap_error = gap - safe_gap
        v_cmd = front_speed + self.k_gap * gap_error
        v_cmd = self.clamp(v_cmd, 0.0, max(0.0, v_ref))
        w_cmd = w_ref
        self.last_selected_action = "front_follow"

        now = self.get_time_now()
        if front_speed < self.low_speed_threshold and gap < safe_gap + self.lane_width:
            if self.front_block_start_time is None:
                self.front_block_start_time = now
            blocking_time = now - self.front_block_start_time
        else:
            self.front_block_start_time = None
            blocking_time = 0.0

        if blocking_time >= self.blocking_time_threshold:
            candidate_paths = self.generate_lane_change_candidates()
            selected_path = self.select_safe_candidate_path(candidate_paths)
            if selected_path:
                self.current_bypass_path = selected_path
                self.current_bypass_index = 0
                self.behavior_state = "TRY_LANE_CHANGE"
                self.last_selected_action = "lane_change"
                return self.execute_current_bypass_path(v_ref, self.get_neighbor_vehicles()) or (v_cmd, w_cmd)
            self.last_selected_action = "follow_no_safe_lane_change"

        return v_cmd, w_cmd

    def handle_oncoming_vehicle(self, oncoming_vehicle, v_ref, w_ref):
        distance = max(0.0, oncoming_vehicle["distance"])
        closing_speed = self.get_ego_collision_speed() + oncoming_vehicle.get("speed", 0.0)
        ttc = self.compute_ttc(distance, closing_speed)
        self.last_ttc = ttc
        self.last_front_gap = distance

        if ttc >= self.oncoming_ttc_threshold or distance >= self.oncoming_detect_dist:
            self.behavior_state = "NORMAL_PATH_FOLLOW"
            self.stop_start_time = None
            self.last_selected_action = "oncoming_clear"
            return v_ref, w_ref

        now = self.get_time_now()
        stopped_long_enough = False
        current_v = self.last_v_cmd if self.last_v_cmd > 1e-3 else v_ref
        dt = 1.0 / self.control_rate if self.control_rate > 0 else 0.1
        v_cmd = max(0.0, current_v - self.comfort_decel * dt)
        speed_ratio = 0.0 if abs(v_ref) < 1e-6 else self.clamp(v_cmd / max(abs(v_ref), 1e-6), 0.0, 1.0)
        w_cmd = w_ref * speed_ratio

        if v_cmd <= 0.1:
            if self.stop_start_time is None:
                self.stop_start_time = now
            stopped_long_enough = (now - self.stop_start_time) >= self.stop_wait_time
        else:
            self.stop_start_time = None

        if stopped_long_enough:
            self.behavior_state = "TRY_BYPASS"
            candidate_paths = [
                {"name": "bypass_left", "path": self.generate_offset_path(self.lane_change_offset, self.bypass_length)},
                {"name": "bypass_right", "path": self.generate_offset_path(-self.lane_change_offset, self.bypass_length)},
            ]
            selected_path = self.select_safe_candidate_path(candidate_paths)
            if selected_path:
                self.current_bypass_path = selected_path
                self.current_bypass_index = 0
                self.last_selected_action = "bypass_oncoming"
                return self.execute_current_bypass_path(v_ref, self.get_neighbor_vehicles()) or (0.0, 0.0)
            self.last_selected_action = "yield_no_safe_bypass"
            return 0.0, 0.0

        self.behavior_state = "ONCOMING_YIELD"
        self.last_selected_action = "yield_oncoming"
        return v_cmd, w_cmd

    def generate_lane_change_candidates(self):
        return [
            {"name": "lane_change_left", "path": self.generate_offset_path(self.lane_change_offset, self.lane_change_length)},
            {"name": "lane_change_right", "path": self.generate_offset_path(-self.lane_change_offset, self.lane_change_length)},
        ]

    def smooth_step(self, value):
        value = self.clamp(value, 0.0, 1.0)
        return value * value * value * (value * (value * 6.0 - 15.0) + 10.0)

    def smooth_offset_ratio(self, progress):
        progress = self.clamp(progress, 0.0, 1.0)
        if progress <= 0.5:
            return self.smooth_step(progress * 2.0)
        return self.smooth_step((1.0 - progress) * 2.0)

    def generate_offset_path(self, offset, length):
        point_count = self.get_effective_path_point_count()
        if point_count == 0:
            return [(self.m_x, self.m_y)]

        path_points = [(self.m_x, self.m_y)]
        start_index = self.normalize_path_index(self.vehpos_initial_index)
        previous_index = start_index
        traveled = 0.0
        max_steps = point_count if self.is_closed_path else max(0, point_count - start_index - 1)

        for step in range(0, max_steps + 1):
            if self.is_closed_path:
                point_index = self.normalize_path_index(start_index + step)
            else:
                point_index = min(start_index + step, point_count - 1)

            if step > 0:
                traveled += math.hypot(
                    self.X_points[point_index] - self.X_points[previous_index],
                    self.Y_points[point_index] - self.Y_points[previous_index],
                )

            path_yaw = self.get_path_yaw(point_index)
            offset_ratio = self.smooth_offset_ratio(traveled / max(length, 1e-6))
            offset_now = offset * offset_ratio
            normal_x = -math.sin(path_yaw)
            normal_y = math.cos(path_yaw)
            x_new = self.X_points[point_index] + offset_now * normal_x
            y_new = self.Y_points[point_index] + offset_now * normal_y
            path_points.append((x_new, y_new))

            previous_index = point_index
            if traveled >= length or (not self.is_closed_path and point_index == point_count - 1):
                break

        if len(path_points) < 2:
            path_points.append((self.m_x, self.m_y))
        return path_points

    def select_safe_candidate_path(self, candidate_paths, neighbor_vehicles=None):
        if neighbor_vehicles is None:
            neighbor_vehicles = self.get_neighbor_vehicles()

        for candidate in candidate_paths:
            name = candidate.get("name", "candidate") if isinstance(candidate, dict) else "candidate"
            path_points = candidate.get("path", []) if isinstance(candidate, dict) else candidate
            if len(path_points) < 2:
                self.last_candidate_reason = name + ": too_short"
                continue
            if not self.check_boundary_collision(path_points):
                self.last_candidate_reason = name + ": boundary_unsafe"
                continue
            if not self.check_vehicle_collision_for_path(path_points, neighbor_vehicles):
                self.last_candidate_reason = name + ": vehicle_unsafe"
                continue
            self.last_candidate_reason = name + ": safe"
            return path_points

        return None

    def execute_current_bypass_path(self, v_ref, neighbor_vehicles):
        if not self.current_bypass_path:
            return None

        remaining_path = self.current_bypass_path[self.current_bypass_index:]
        if len(remaining_path) >= 2:
            if not self.check_boundary_collision(remaining_path):
                self.behavior_state = "EMERGENCY_STOP"
                self.last_selected_action = "active_path_boundary_stop"
                self.current_bypass_path = []
                self.current_bypass_index = 0
                return 0.0, 0.0
            if not self.check_vehicle_collision_for_path(remaining_path, neighbor_vehicles):
                self.behavior_state = "EMERGENCY_STOP"
                self.last_selected_action = "active_path_vehicle_stop"
                self.current_bypass_path = []
                self.current_bypass_index = 0
                return 0.0, 0.0

        target = self.find_local_path_target(self.current_bypass_path)
        if target is None:
            self.current_bypass_path = []
            self.current_bypass_index = 0
            self.behavior_state = "RETURN_TO_ROUTE"
            self.last_selected_action = "return_to_route"
            return None

        path_v, path_w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, target)
        v_cmd = self.clamp(path_v, 0.0, max(v_ref, path_v))
        if v_ref > 1e-6:
            v_cmd = min(v_cmd, v_ref)
        if path_v > 1e-6 and v_cmd < path_v:
            w_cmd = path_w * (v_cmd / path_v)
        else:
            w_cmd = path_w
        self.last_selected_action = "execute_" + self.behavior_state.lower()
        return v_cmd, w_cmd

    def find_local_path_target(self, path_points):
        if not path_points:
            return None

        start_index = min(max(self.current_bypass_index, 0), len(path_points) - 1)
        nearest_index = start_index
        min_distance = float("inf")
        for index in range(start_index, len(path_points)):
            px, py = path_points[index]
            distance = math.hypot(self.m_x - px, self.m_y - py)
            if distance < min_distance:
                min_distance = distance
                nearest_index = index

        self.current_bypass_index = nearest_index
        if nearest_index >= len(path_points) - 2 and min_distance < max(self.lane_width, self.lookahead_distance):
            return None

        accumulated_distance = 0.0
        target_index = nearest_index
        previous_point = path_points[nearest_index]
        for index in range(nearest_index + 1, len(path_points)):
            current_point = path_points[index]
            accumulated_distance += math.hypot(
                current_point[0] - previous_point[0],
                current_point[1] - previous_point[1],
            )
            target_index = index
            if accumulated_distance >= self.lookahead_distance:
                break
            previous_point = current_point

        return path_points[target_index]

    def predict_vehicle_position(self, vehicle, t):
        speed = vehicle.get("speed", 0.0) if isinstance(vehicle, dict) else self.get_vehicle_speed(vehicle, 0.0)
        yaw = vehicle.get("yaw", 0.0) if isinstance(vehicle, dict) else self.normalize_yaw_value(self.get_vehicle_field(vehicle, "yaw", 0.0), 0.0)
        x = vehicle.get("x", 0.0) if isinstance(vehicle, dict) else float(self.get_vehicle_field(vehicle, "x", 0.0))
        y = vehicle.get("y", 0.0) if isinstance(vehicle, dict) else float(self.get_vehicle_field(vehicle, "y", 0.0))
        return x + speed * math.cos(yaw) * t, y + speed * math.sin(yaw) * t

    def check_vehicle_collision_for_path(self, path_points, neighbor_vehicles):
        if not path_points:
            return True

        speed = max(self.last_v_cmd, self.m_v, 1.0)
        elapsed = 0.0
        previous_point = path_points[0]
        for point in path_points:
            elapsed += math.hypot(point[0] - previous_point[0], point[1] - previous_point[1]) / speed
            if elapsed > self.prediction_horizon:
                break
            for vehicle in neighbor_vehicles:
                if vehicle.get("classification") == "REAR_VEHICLE":
                    continue
                predicted_x, predicted_y = self.predict_vehicle_position(vehicle, elapsed)
                if math.hypot(point[0] - predicted_x, point[1] - predicted_y) < self.vehicle_safe_radius:
                    return False
            previous_point = point

        return True

    def emergency_stop_if_needed(self, neighbor_vehicles, v_ref=None, w_ref=None):
        for vehicle in neighbor_vehicles:
            distance = vehicle.get("distance", float("inf"))
            if distance < self.emergency_distance:
                self.behavior_state = "EMERGENCY_STOP"
                self.last_selected_action = "emergency_distance"
                self.last_ttc = vehicle.get("ttc", 0.0)
                return 0.0, 0.0

            if vehicle.get("classification") in ("FRONT_SAME_DIRECTION", "ONCOMING_VEHICLE"):
                self.has_collision_risk(vehicle)
                if vehicle.get("ttc", float("inf")) < self.emergency_ttc:
                    self.behavior_state = "EMERGENCY_STOP"
                    self.last_selected_action = "emergency_ttc"
                    self.last_ttc = vehicle.get("ttc", 0.0)
                    return 0.0, 0.0

        if v_ref is not None and w_ref is not None and not self.check_predicted_control_boundary(v_ref, w_ref, horizon=0.8):
            self.behavior_state = "EMERGENCY_STOP"
            self.last_selected_action = "emergency_boundary"
            return 0.0, 0.0

        return None

    def predict_ego_path(self, v_cmd, w_cmd, horizon=1.5):
        dt = self.prediction_dt
        steps = max(1, int(horizon / dt))
        x = self.m_x
        y = self.m_y
        yaw = self.m_yaw
        path_points = [(x, y)]
        for _ in range(steps):
            x += v_cmd * math.cos(yaw) * dt
            y += v_cmd * math.sin(yaw) * dt
            yaw = self.normalize_angle(yaw + w_cmd * dt)
            path_points.append((x, y))
        return path_points

    def check_predicted_control_boundary(self, v_cmd, w_cmd, horizon=1.5):
        if not self.map_boundaries:
            return True
        path_points = self.predict_ego_path(v_cmd, w_cmd, horizon)
        return self.check_boundary_collision(path_points)

    def distance_to_current_route(self):
        point_count = self.get_effective_path_point_count()
        if point_count == 0:
            return float("inf")
        index = self.normalize_path_index(self.vehpos_initial_index)
        return math.hypot(self.m_x - self.X_points[index], self.m_y - self.Y_points[index])

    def print_behavior_debug(self, v_cmd, w_cmd):
        if self.frame_count % 20 != 0:
            return
        ttc_text = "inf" if self.last_ttc is None or math.isinf(self.last_ttc) else "%.2f" % self.last_ttc
        gap_text = "None" if self.last_front_gap is None else "%.2f" % self.last_front_gap
        print(
            "[behavior]",
            self.behavior_state,
            "action:", self.last_selected_action,
            "gap:", gap_text,
            "ttc:", ttc_text,
            "candidate:", self.last_candidate_reason,
            "v:", round(v_cmd, 3),
            "w:", round(w_cmd, 3),
        )

    def get_path_yaw(self, index):
        point_count = self.get_effective_path_point_count()
        if point_count < 2:
            return self.m_yaw

        index = self.normalize_path_index(index)
        previous_index = self.normalize_path_index(index - 1)
        next_index = self.normalize_path_index(index + 1)

        if not self.is_closed_path:
            previous_index = max(0, index - 1)
            next_index = min(point_count - 1, index + 1)
            if previous_index == next_index:
                return self.m_yaw

        dx = self.X_points[next_index] - self.X_points[previous_index]
        dy = self.Y_points[next_index] - self.Y_points[previous_index]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return self.m_yaw
        return math.atan2(dy, dx)

    def try_load_default_map_boundaries(self):
        for json_path in self.default_map_boundary_files:
            if os.path.exists(json_path):
                return self.load_map_boundaries(json_path)
        return False

    def load_map_boundaries(self, json_path):
        if not os.path.exists(json_path):
            print("[boundary] file not found:", json_path)
            return False

        with open(json_path, "r", encoding="utf-8") as file:
            boundary_data = json.load(file)

        parsed_boundaries = []
        for raw_boundary in self.extract_boundary_lists(boundary_data):
            boundary = []
            for raw_point in raw_boundary:
                point = self.parse_json_point(raw_point)
                if point is None:
                    continue
                boundary.append(self.boundary_point_to_world(point))
            if len(boundary) >= 3:
                parsed_boundaries.append(boundary)

        self.map_boundaries = parsed_boundaries
        self.map_boundary_path = json_path
        print("[boundary] loaded", len(self.map_boundaries), "boundaries from", json_path)
        return True

    def extract_boundary_lists(self, data):
        if self.is_point_like(data):
            return [[data]]

        if isinstance(data, list):
            if all(self.is_point_like(item) for item in data):
                return [data]
            boundaries = []
            for item in data:
                boundaries.extend(self.extract_boundary_lists(item))
            return boundaries

        if isinstance(data, dict):
            for key in ("boundaries", "boundary", "polygons", "polygon", "regions", "points", "vertices"):
                if key in data:
                    return self.extract_boundary_lists(data[key])

        return []

    def is_point_like(self, value):
        if isinstance(value, dict):
            return "x" in value and "y" in value
        if isinstance(value, (list, tuple)):
            return len(value) >= 2 and isinstance(value[0], (int, float)) and isinstance(value[1], (int, float))
        return False

    def parse_json_point(self, raw_point):
        if isinstance(raw_point, dict):
            try:
                return float(raw_point["x"]), float(raw_point["y"])
            except (KeyError, TypeError, ValueError):
                return None
        if isinstance(raw_point, (list, tuple)) and len(raw_point) >= 2:
            try:
                return float(raw_point[0]), float(raw_point[1])
            except (TypeError, ValueError):
                return None
        return None

    def boundary_point_to_world(self, point):
        # Current route JSONs and the named px_bottom_left boundary JSON both use
        # x-right/y-up map coordinates, so the default transform is identity.
        x, y = point
        if self.boundary_flip_y:
            y = self.boundary_image_height - y
        x = x * self.boundary_scale + self.boundary_offset_x
        y = y * self.boundary_scale + self.boundary_offset_y
        return x, y

    def point_in_polygon(self, point, polygon):
        if len(polygon) < 3:
            return False

        x, y = point
        inside = False
        previous = polygon[-1]
        for current in polygon:
            xi, yi = current
            xj, yj = previous
            if self.point_on_segment(point, previous, current):
                return True
            intersects = ((yi > y) != (yj > y))
            if intersects:
                x_intersection = (xj - xi) * (y - yi) / (yj - yi) + xi
                if x < x_intersection:
                    inside = not inside
            previous = current
        return inside

    def point_on_segment(self, p, a, b, eps=1e-9):
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cross) > eps:
            return False
        return (
            min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
            and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
        )

    def segment_intersects_segment(self, a, b, c, d):
        def orientation(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

        eps = 1e-9
        o1 = orientation(a, b, c)
        o2 = orientation(a, b, d)
        o3 = orientation(c, d, a)
        o4 = orientation(c, d, b)

        if o1 * o2 < -eps and o3 * o4 < -eps:
            return True

        if abs(o1) <= eps and self.point_on_segment(c, a, b):
            return True
        if abs(o2) <= eps and self.point_on_segment(d, a, b):
            return True
        if abs(o3) <= eps and self.point_on_segment(a, c, d):
            return True
        if abs(o4) <= eps and self.point_on_segment(b, c, d):
            return True

        return False

    def iter_polygon_segments(self, polygon):
        if len(polygon) < 2:
            return
        for index in range(len(polygon)):
            yield polygon[index], polygon[(index + 1) % len(polygon)]

    def segment_intersects_polygon(self, p1, p2, polygon):
        for a, b in self.iter_polygon_segments(polygon):
            if self.segment_intersects_segment(p1, p2, a, b):
                return True
        return False

    def distance_point_to_segment(self, p, a, b):
        ax, ay = a
        bx, by = b
        px, py = p
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return math.hypot(px - ax, py - ay)
        ratio = ((px - ax) * dx + (py - ay) * dy) / length_sq
        ratio = self.clamp(ratio, 0.0, 1.0)
        closest_x = ax + ratio * dx
        closest_y = ay + ratio * dy
        return math.hypot(px - closest_x, py - closest_y)

    def check_boundary_collision(self, path_points):
        # Returns True when the path is boundary-safe, False when it enters,
        # crosses, or comes too close to any forbidden closed boundary.
        if not self.map_boundaries or not path_points:
            return True

        normalized_path = []
        for point in path_points:
            parsed_point = self.parse_json_point(point)
            if parsed_point is not None:
                normalized_path.append(parsed_point)

        if not normalized_path:
            return True

        for polygon in self.map_boundaries:
            for point in normalized_path:
                if self.point_in_polygon(point, polygon):
                    return False
                for a, b in self.iter_polygon_segments(polygon):
                    if self.distance_point_to_segment(point, a, b) < self.boundary_safe_margin:
                        return False

            for index in range(len(normalized_path) - 1):
                if self.segment_intersects_polygon(normalized_path[index], normalized_path[index + 1], polygon):
                    return False

        return True

    def get_effective_path_point_count(self):
        point_count = len(self.X_points)
        if self.is_closed_path and point_count > 1:
            return point_count - 1
        return point_count

    def normalize_path_index(self, index):
        point_count = self.get_effective_path_point_count()
        if point_count <= 0:
            return 0
        index = int(index)
        if self.is_closed_path:
            return index % point_count
        return int(self.clamp(index, 0, point_count - 1))

    def calc_lookahead_distance(self):
        curvature = self.estimate_path_curvature(self.vehpos_initial_index)
        speed = max(self.m_v, 0.0)
        lookahead = self.base_lookahead + self.speed_lookahead_gain * speed
        lookahead = lookahead / (1.0 + self.curvature_lookahead_gain * curvature)
        return self.clamp(lookahead, self.min_lookahead, self.max_lookahead)

    def estimate_path_curvature(self, index, step=5):
        point_count = self.get_effective_path_point_count()
        if point_count < 3:
            return 0.0

        step = max(1, int(step))
        step = min(step, max(1, (point_count - 1) // 2))
        center_index = self.normalize_path_index(index)

        if self.is_closed_path:
            p0_index = self.normalize_path_index(center_index)
            p1_index = self.normalize_path_index(center_index + step)
            p2_index = self.normalize_path_index(center_index + 2 * step)
        else:
            p0_index = self.normalize_path_index(center_index)
            p1_index = self.normalize_path_index(center_index + step)
            p2_index = self.normalize_path_index(center_index + 2 * step)
            if p0_index == p1_index or p1_index == p2_index:
                p0_index = self.normalize_path_index(center_index - step)
                p1_index = center_index
                p2_index = self.normalize_path_index(center_index + step)

        if p0_index == p1_index or p1_index == p2_index:
            return 0.0

        x0, y0 = self.X_points[p0_index], self.Y_points[p0_index]
        x1, y1 = self.X_points[p1_index], self.Y_points[p1_index]
        x2, y2 = self.X_points[p2_index], self.Y_points[p2_index]

        segment_1 = math.hypot(x1 - x0, y1 - y0)
        segment_2 = math.hypot(x2 - x1, y2 - y1)
        if segment_1 < 1e-6 or segment_2 < 1e-6:
            return 0.0

        yaw_1 = math.atan2(y1 - y0, x1 - x0)
        yaw_2 = math.atan2(y2 - y1, x2 - x1)
        yaw_delta = self.normalize_angle(yaw_2 - yaw_1)
        average_length = 0.5 * (segment_1 + segment_2)
        return abs(yaw_delta) / max(average_length, 1e-6)

    def limit_steering_angle(self, steering_angle):
        steering_angle = self.clamp(steering_angle, -self.max_steer, self.max_steer)
        dt = 1.0 / self.control_rate if self.control_rate > 0 else 0.1
        max_delta = self.max_steer_rate * dt
        lower = self.last_steering_angle - max_delta
        upper = self.last_steering_angle + max_delta
        steering_angle = self.clamp(steering_angle, lower, upper)
        steering_angle = self.clamp(steering_angle, -self.max_steer, self.max_steer)
        self.last_steering_angle = steering_angle
        return steering_angle

    def search_vehicle_initial_index(self):
        point_count = self.get_effective_path_point_count()
        if point_count == 0:
            self.vehpos_initial_index = 0
            return

        min_distance = float('inf')
        nearest_index = 0

        for i in range(point_count):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        self.vehpos_initial_index = nearest_index


    def find_nearest_point_index(self, target_x, target_y):
        point_count = self.get_effective_path_point_count()
        if point_count == 0:
            return -1

        min_distance = float('inf')
        nearest_index = -1

        for i in range(point_count):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((target_x - this_point_x) ** 2 + (target_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index

    def update_vehpos_index(self):
        point_count = self.get_effective_path_point_count()
        if point_count == 0:
            self.vehpos_initial_index = 0
            return

        if point_count == 1:
            self.vehpos_initial_index = 0
            return

        min_distance = float('inf')
        nearest_index = self.normalize_path_index(self.vehpos_initial_index)
        start_index = nearest_index
        search_count = min(point_count, 40)

        for i in range(search_count):
            if self.is_closed_path:
                find_index = (start_index + i) % point_count
            else:
                find_index = start_index + i
                if find_index >= point_count:
                    break

            this_point_x = self.X_points[find_index]
            this_point_y = self.Y_points[find_index]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = find_index
        if min_distance > 25:
            self.search_vehicle_initial_index()
        else:
            self.vehpos_initial_index = nearest_index

    def search_target_pos(self):
        point_count = self.get_effective_path_point_count()
        if point_count == 0:
            self.targetPos_Info[0] = self.m_x
            self.targetPos_Info[1] = self.m_y
            self.target_pos_index = 0
            return

        if point_count == 1:
            self.targetPos_Info[0] = self.X_points[0]
            self.targetPos_Info[1] = self.Y_points[0]
            self.target_pos_index = 0
            return

        self.lookahead_distance = self.calc_lookahead_distance()
        start_index = self.normalize_path_index(self.vehpos_initial_index)
        target_pos_index = start_index
        previous_index = start_index
        accumulated_distance = 0.0
        max_steps = point_count if self.is_closed_path else max(0, point_count - start_index - 1)

        for step in range(1, max_steps + 1):
            if self.is_closed_path:
                current_index = (start_index + step) % point_count
            else:
                current_index = start_index + step
                if current_index >= point_count:
                    current_index = point_count - 1

            segment_length = math.hypot(
                self.X_points[current_index] - self.X_points[previous_index],
                self.Y_points[current_index] - self.Y_points[previous_index],
            )
            accumulated_distance += segment_length
            target_pos_index = current_index

            if accumulated_distance >= self.lookahead_distance:
                break

            previous_index = current_index
            if not self.is_closed_path and current_index == point_count - 1:
                break

        self.targetPos_Info[0] = self.X_points[target_pos_index]
        self.targetPos_Info[1] = self.Y_points[target_pos_index]
        self.target_pos_index = target_pos_index

if __name__ == '__main__':
    control = Control()
    control.udp_client.start()
    control.control_node()
