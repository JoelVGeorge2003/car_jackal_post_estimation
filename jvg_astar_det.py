#!/usr/bin/env python3

import math
import re
import subprocess
import threading
import time
import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
WORLD = 'jackal_lab'
JACKAL_MODEL = 'j100_0000/robot'
CAR_MODEL = 'prius_01'
CMD_TOPIC = '/j100_0000/platform/cmd_vel'
CAR_LENGTH = 4.5
CAR_WIDTH = 1.8
REAR_CLEARANCE = 1.0
CAR_FRONT_AXIS_DEG = -90.0
MAX_LINEAR = 0.6
MAX_ANGULAR = 0.8
DIST_TOL = 0.15
YAW_TOL = 0.06
NAV_TIMEOUT = 300.0
GRID_RESOLUTION = 0.5
MAP_MARGIN = 5.0
INFLATION = 0.4

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(value, limit):
    return max(-limit, min(limit, value))

def get_model_pose(model_name, attempts=5):
    pattern = 'Pose \\[ XYZ \\(m\\) \\] \\[ RPY \\(rad\\) \\]:\\s*\\[([^\\]]+)\\]\\s*\\[([^\\]]+)\\]'
    last_error = ''
    for attempt in range(1, attempts + 1):
        try:
            result = subprocess.run(['gz', 'model', '-m', model_name, '-p'], capture_output=True, text=True, timeout=8)
            match = re.search(pattern, result.stdout)
            if result.returncode == 0 and match:
                xyz = [float(v) for v in match.group(1).split()]
                rpy = [float(v) for v in match.group(2).split()]
                return (xyz[0], xyz[1], rpy[2])
            last_error = f'returncode={result.returncode}, stdout={result.stdout[:250]!r}, stderr={result.stderr[:250]!r}'
        except subprocess.TimeoutExpired:
            last_error = 'Gazebo pose request timed out'
        print(f'[POSE RETRY {attempt}/{attempts}] {model_name}: {last_error}')
        time.sleep(0.4)
    raise RuntimeError(f'Cannot retrieve pose for {model_name} after {attempts} attempts. {last_error}')

def calculate_rear_target(car_pose):
    cx, cy, car_yaw = car_pose
    front_yaw = normalize_angle(car_yaw + math.radians(CAR_FRONT_AXIS_DEG))
    distance = CAR_LENGTH / 2.0 + REAR_CLEARANCE
    goal_x = cx - distance * math.cos(front_yaw)
    goal_y = cy - distance * math.sin(front_yaw)
    goal_yaw = front_yaw
    bumper_x = cx - CAR_LENGTH / 2.0 * math.cos(front_yaw)
    bumper_y = cy - CAR_LENGTH / 2.0 * math.sin(front_yaw)
    return ((goal_x, goal_y, goal_yaw), (bumper_x, bumper_y))

def inside_inflated_car(x, y, car_pose):
    cx, cy, yaw = car_pose
    heading = normalize_angle(yaw + math.radians(CAR_FRONT_AXIS_DEG))
    dx = x - cx
    dy = y - cy
    local_x = math.cos(heading) * dx + math.sin(heading) * dy
    local_y = -math.sin(heading) * dx + math.cos(heading) * dy
    return abs(local_x) < CAR_LENGTH / 2.0 + INFLATION and abs(local_y) < CAR_WIDTH / 2.0 + INFLATION

def straight_path_clear(start, goal, car_pose):
    sx, sy = start
    gx, gy = goal
    distance = math.hypot(gx - sx, gy - sy)
    steps = max(1, math.ceil(distance / 0.1))
    for i in range(steps + 1):
        t = i / steps
        x = sx + t * (gx - sx)
        y = sy + t * (gy - sy)
        if inside_inflated_car(x, y, car_pose):
            return False
    return True

def plan_astar(start, goal, car_pose):
    resolution = GRID_RESOLUTION
    margin = MAP_MARGIN
    sx, sy = start
    gx, gy = goal
    cx, cy, _ = car_pose
    xmin = min(sx, gx, cx) - margin
    ymin = min(sy, gy, cy) - margin
    xmax = max(sx, gx, cx) + margin
    ymax = max(sy, gy, cy) + margin

    def to_grid(x, y):
        return (round((x - xmin) / resolution), round((y - ymin) / resolution))

    def to_world(cell):
        return (xmin + cell[0] * resolution, ymin + cell[1] * resolution)

    def blocked(cell):
        x, y = to_world(cell)
        return x < xmin or x > xmax or y < ymin or (y > ymax) or inside_inflated_car(x, y, car_pose)
    start_node = to_grid(sx, sy)
    goal_node = to_grid(gx, gy)
    if blocked(start_node):
        raise RuntimeError('A* start is inside an obstacle')
    if blocked(goal_node):
        raise RuntimeError('A* goal is inside an obstacle')

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    queue = [(0, start_node)]
    costs = {start_node: 0.0}
    parents = {}
    visited = set()
    while queue:
        _, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        if current == goal_node:
            break
        for dx, dy in moves:
            nxt = (current[0] + dx, current[1] + dy)
            if blocked(nxt):
                continue
            if dx != 0 and dy != 0:
                if blocked((current[0] + dx, current[1])) or blocked((current[0], current[1] + dy)):
                    continue
            new_cost = costs[current] + math.hypot(dx, dy)
            if new_cost < costs.get(nxt, float('inf')):
                costs[nxt] = new_cost
                parents[nxt] = current
                priority = new_cost + heuristic(nxt, goal_node)
                heapq.heappush(queue, (priority, nxt))
    if goal_node not in costs:
        raise RuntimeError('A* could not find a path')
    cells = []
    node = goal_node
    while node != start_node:
        cells.append(node)
        node = parents[node]
    cells.reverse()
    raw_path = [start] + [to_world(c) for c in cells] + [goal]
    path = []
    current = 0
    while current < len(raw_path) - 1:
        furthest = current + 1
        for j in range(len(raw_path) - 1, current, -1):
            if straight_path_clear(raw_path[current], raw_path[j], car_pose):
                furthest = j
                break
        path.append(raw_path[furthest])
        current = furthest
    return path

class JackalController(Node):

    def __init__(self):
        super().__init__('jackal_rear_navigator')
        self.publisher = self.create_publisher(TwistStamped, CMD_TOPIC, 10)
        self.lock = threading.Lock()
        self.linear = 0.0
        self.angular = 0.0
        self.last_update = time.monotonic()
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def set_velocity(self, linear, angular):
        with self.lock:
            self.linear = float(linear)
            self.angular = float(angular)
            self.last_update = time.monotonic()

    def publish_velocity(self):
        with self.lock:
            linear = self.linear
            angular = self.angular
            if time.monotonic() - self.last_update > 2.0:
                linear = 0.0
                angular = 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.publisher.publish(msg)

    def stop(self):
        self.set_velocity(0.0, 0.0)

def navigate(controller, path, final_yaw):
    waypoint_index = 0
    state = 'TURN'
    start_time = time.monotonic()
    last_log = 0.0
    last_pose = None
    last_motion_time = time.monotonic()
    print('Waypoints:', path)
    while rclpy.ok():
        now = time.monotonic()
        if now - start_time > NAV_TIMEOUT:
            raise RuntimeError('Navigation timed out')
        x, y, yaw = get_model_pose(JACKAL_MODEL)
        now = time.monotonic()
        if last_pose is None:
            last_pose = (x, y, yaw)
        translation = math.hypot(x - last_pose[0], y - last_pose[1])
        rotation = abs(normalize_angle(yaw - last_pose[2]))
        if translation > 0.03 or rotation > 0.04:
            last_pose = (x, y, yaw)
            last_motion_time = now
        if now - last_motion_time > 20.0:
            raise RuntimeError('No Jackal movement detected for 20 seconds')
        if waypoint_index < len(path):
            tx, ty = path[waypoint_index]
            dx = tx - x
            dy = ty - y
            distance = math.hypot(dx, dy)
            desired_yaw = math.atan2(dy, dx)
            yaw_error = normalize_angle(desired_yaw - yaw)
            tolerance = DIST_TOL if waypoint_index == len(path) - 1 else 0.25
            if distance < tolerance:
                controller.stop()
                print(f'Waypoint {waypoint_index + 1}/{len(path)} reached')
                waypoint_index += 1
                state = 'TURN'
                last_motion_time = now
                continue
            if state == 'TURN':
                if abs(yaw_error) < 0.12:
                    controller.stop()
                    state = 'DRIVE'
                    continue
                linear = 0.0
                angular = clamp(1.8 * yaw_error, MAX_ANGULAR)
            else:
                if abs(yaw_error) > 0.45:
                    state = 'TURN'
                    controller.stop()
                    continue
                linear = min(MAX_LINEAR, 0.8 * distance)
                linear *= max(0.2, math.cos(yaw_error))
                angular = clamp(1.5 * yaw_error, MAX_ANGULAR)
            controller.set_velocity(linear, angular)
        else:
            state = 'FINAL_ALIGN'
            yaw_error = normalize_angle(final_yaw - yaw)
            if abs(yaw_error) < YAW_TOL:
                controller.stop()
                print(f'Final X: {x:.3f}')
                print(f'Final Y: {y:.3f}')
                print(f'Final yaw: {yaw:.3f}')
                print('Jackal is facing the Prius rear (assuming the car axis is configured correctly).')
                return
            controller.set_velocity(0.0, clamp(1.5 * yaw_error, MAX_ANGULAR))
        if now - last_log > 1.5:
            print(f'[{state}] Jackal=({x:.2f},{y:.2f}), yaw={yaw:.2f}, waypoint={min(waypoint_index + 1, len(path))}/{len(path)}')
            last_log = now
        time.sleep(0.1)

def main():
    rclpy.init()
    controller = JackalController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()
    try:
        print('\nReading Gazebo model poses...')
        car_pose = get_model_pose(CAR_MODEL)
        jackal_pose = get_model_pose(JACKAL_MODEL)
        goal, rear_bumper = calculate_rear_target(car_pose)
        print('Prius:', car_pose)
        print('Jackal:', jackal_pose)
        print('Rear bumper:', rear_bumper)
        print('Rear approach target:', goal)
        path = plan_astar(jackal_pose[:2], goal[:2], car_pose)
        print('\nA* planned route:')
        for i, point in enumerate(path):
            print(f'  {i + 1}: {point}')
        print('\nWaiting for velocity controller...')
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and controller.publisher.get_subscription_count() == 0 and (time.monotonic() < deadline):
            time.sleep(0.2)
        if controller.publisher.get_subscription_count() == 0:
            raise RuntimeError(f'No subscriber detected on {CMD_TOPIC}. Check that platform_velocity_controller is active.')
        print('Velocity controller connected.')
        print('Starting rear approach...')
        navigate(controller, path, goal[2])
    except KeyboardInterrupt:
        print('\nNavigation interrupted.')
    except Exception as error:
        print('\nERROR:', error)
    finally:
        controller.stop()
        time.sleep(0.5)
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        controller.destroy_node()
if __name__ == '__main__':
    main()
