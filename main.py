import json
import math
import time
from my_udp import UDPClient


class Control:
    def __init__(self):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001 
        self.server_ip = '192.168.1.100'

        net = "z2JkjyynJGQ6oCucSM9fhrEDYLsf,192.168.1.109,8700,8701"
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
        self.control_rate = 10  # hz
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

    def control_node(self):
        start_time = time.time()
        self.load_route('exp_routes/leftInside.json')
        while True:
            self.run_step()

            elapsed_time = time.time() - start_time
            sleep_time = max((1.0 / self.control_rate) - elapsed_time, 0.0)
            time.sleep(sleep_time)
            start_time = time.time()

    def run_step(self):
        vehicle_data = self.udp_client.get_vehicle_state()
        self.m_x = vehicle_data.x
        self.m_y = vehicle_data.y
        self.m_yaw = vehicle_data.yaw / 180 * math.pi
        self.m_v = 10
        self.update_vehpos_index()
        self.search_target_pos()

        v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)
        self.udp_client.send_control_command(v, w)
        return v, w

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
