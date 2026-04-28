#!/usr/bin/env python3
"""Frontier-based autonomous exploration for production-quality SLAM maps."""

import heapq
import math
from collections import deque
from dataclasses import dataclass
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class Mode(Enum):
    STARTUP_SPIN = "startup_spin"
    PLAN = "plan"
    NAVIGATE = "navigate"
    RECOVER = "recover"
    FINISHED = "finished"


@dataclass
class FrontierCluster:
    cells: list[tuple[int, int]]
    target: tuple[int, int]
    score: float


@dataclass
class ExplorationPlan:
    target: tuple[int, int]
    path: list[tuple[int, int]]
    frontier_size: int
    score: float


class AutonomousMapper(Node):
    """Map-driven frontier explorer with A* global planning and LiDAR safety."""

    def __init__(self):
        super().__init__("autonomous_mapper")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("linear_speed", 0.26)
        self.declare_parameter("slow_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.90)
        self.declare_parameter("startup_turn_speed", 0.45)
        self.declare_parameter("startup_spin_duration", 12.5)
        self.declare_parameter("recover_turn_duration", 1.8)
        self.declare_parameter("front_clearance", 0.90)
        self.declare_parameter("emergency_clearance", 0.42)
        self.declare_parameter("side_clearance", 0.55)
        self.declare_parameter("slow_clearance", 1.35)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("waypoint_lookahead", 0.70)
        self.declare_parameter("replan_interval", 2.0)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("scan_timeout", 1.0)
        self.declare_parameter("map_timeout", 3.0)
        self.declare_parameter("obstacle_threshold", 55)
        self.declare_parameter("free_threshold", 25)
        self.declare_parameter("obstacle_inflation_radius", 0.32)
        self.declare_parameter("min_frontier_size", 14)
        self.declare_parameter("frontier_gain", 0.10)
        self.declare_parameter("max_frontier_plans", 12)
        self.declare_parameter("max_astar_expansions", 140000)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.slow_linear_speed = float(self.get_parameter("slow_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.startup_turn_speed = float(self.get_parameter("startup_turn_speed").value)
        self.startup_spin_duration = float(self.get_parameter("startup_spin_duration").value)
        self.recover_turn_duration = float(self.get_parameter("recover_turn_duration").value)
        self.front_clearance = float(self.get_parameter("front_clearance").value)
        self.emergency_clearance = float(self.get_parameter("emergency_clearance").value)
        self.side_clearance = float(self.get_parameter("side_clearance").value)
        self.slow_clearance = float(self.get_parameter("slow_clearance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.waypoint_lookahead = float(self.get_parameter("waypoint_lookahead").value)
        self.replan_interval = float(self.get_parameter("replan_interval").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.map_timeout = float(self.get_parameter("map_timeout").value)
        self.obstacle_threshold = int(self.get_parameter("obstacle_threshold").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.obstacle_inflation_radius = float(
            self.get_parameter("obstacle_inflation_radius").value
        )
        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.frontier_gain = float(self.get_parameter("frontier_gain").value)
        self.max_frontier_plans = int(self.get_parameter("max_frontier_plans").value)
        self.max_astar_expansions = int(self.get_parameter("max_astar_expansions").value)

        control_rate = max(1.0, float(self.get_parameter("control_rate").value))

        self.scan: LaserScan | None = None
        self.map: OccupancyGrid | None = None
        self.blocked: bytearray | None = None
        self.last_scan_time = None
        self.last_map_time = None
        self.mode = Mode.STARTUP_SPIN
        self.mode_started = self.get_clock().now()
        self.last_plan_time = self.get_clock().now()
        self.current_plan: ExplorationPlan | None = None
        self.path_cursor = 0
        self.recover_direction = 1.0
        self.waiting_message = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "exploration_path", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "exploration_goal", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "exploration_markers", 10)

        self.create_subscription(LaserScan, "scan", self.on_scan, 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_timer(1.0 / control_rate, self.on_timer)

        self.get_logger().info(
            "Frontier explorer ready: SLAM map -> frontier target -> inflated-grid A* -> LiDAR-safe drive."
        )

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.last_scan_time = self.get_clock().now()

    def on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.last_map_time = self.get_clock().now()
        self.blocked = self.build_blocked_grid(msg)

    def on_timer(self):
        if not self.inputs_ready():
            self.publish_stop()
            return

        pose = self.robot_pose()
        if pose is None:
            self.publish_stop()
            return

        now = self.get_clock().now()

        if self.mode == Mode.STARTUP_SPIN:
            if self.seconds_since(self.mode_started, now) < self.startup_spin_duration:
                self.publish_cmd(0.0, self.startup_turn_speed)
                return
            self.set_mode(Mode.PLAN)

        if self.mode == Mode.RECOVER:
            if self.seconds_since(self.mode_started, now) < self.recover_turn_duration:
                self.publish_cmd(0.0, self.recover_direction * self.max_angular_speed)
                return
            self.set_mode(Mode.PLAN)

        if self.mode in (Mode.PLAN, Mode.FINISHED) or self.should_replan(now):
            plan = self.make_plan(pose)
            self.last_plan_time = now
            if plan is None:
                self.current_plan = None
                self.path_cursor = 0
                self.publish_empty_path()
                self.set_mode(Mode.FINISHED)
                self.publish_stop()
                return
            self.current_plan = plan
            self.path_cursor = 0
            self.publish_plan(plan)
            self.set_mode(Mode.NAVIGATE)

        if self.current_plan is None:
            self.publish_stop()
            return

        if self.local_obstacle_requires_recovery():
            self.set_recovery_from_scan()
            self.set_mode(Mode.RECOVER)
            self.publish_cmd(0.0, self.recover_direction * self.max_angular_speed)
            return

        if self.reached_goal(pose, self.current_plan.target):
            self.set_mode(Mode.PLAN)
            self.publish_stop()
            return

        self.follow_path(pose)

    def inputs_ready(self) -> bool:
        if self.scan is None or self.scan_is_stale():
            self.log_wait_once("Waiting for /hikbot/scan before autonomous exploration.")
            return False
        if self.map is None or self.map_is_stale() or self.blocked is None:
            self.log_wait_once("Waiting for /map from slam_toolbox.")
            return False
        self.waiting_message = None
        return True

    def log_wait_once(self, message: str):
        if self.waiting_message != message:
            self.get_logger().warn(message)
            self.waiting_message = message

    def scan_is_stale(self) -> bool:
        if self.last_scan_time is None:
            return True
        return self.seconds_since(self.last_scan_time, self.get_clock().now()) > self.scan_timeout

    def map_is_stale(self) -> bool:
        if self.last_map_time is None:
            return True
        return self.seconds_since(self.last_map_time, self.get_clock().now()) > self.map_timeout

    def robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.log_wait_once(f"Waiting for TF {self.map_frame}->{self.base_frame}: {exc}")
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return t.x, t.y, yaw

    def make_plan(self, pose: tuple[float, float, float]) -> ExplorationPlan | None:
        if self.map is None or self.blocked is None:
            return None

        start = self.world_to_grid(pose[0], pose[1])
        if start is None:
            return None

        start = self.nearest_safe_cell(start, max_radius=20)
        if start is None:
            self.get_logger().warn("Robot is not on a safe free map cell; cannot plan.")
            return None

        clusters = self.find_frontier_clusters(start)
        if not clusters:
            self.get_logger().info("Exploration complete: no reachable frontier candidates.")
            return None

        best_plan = None
        for cluster in clusters[: self.max_frontier_plans]:
            path = self.astar(start, cluster.target)
            if not path:
                continue

            path_length = len(path) * self.map.info.resolution
            score = cluster.score - path_length
            if best_plan is None or score > best_plan.score:
                best_plan = ExplorationPlan(
                    target=cluster.target,
                    path=path,
                    frontier_size=len(cluster.cells),
                    score=score,
                )

        if best_plan is None:
            self.get_logger().warn("Frontiers exist, but none are reachable on the inflated grid.")
            return None

        target_world = self.grid_to_world(*best_plan.target)
        self.get_logger().info(
            "New exploration target: "
            f"x={target_world[0]:.2f}, y={target_world[1]:.2f}, "
            f"frontier_cells={best_plan.frontier_size}, path_cells={len(best_plan.path)}"
        )
        return best_plan

    def find_frontier_clusters(self, start: tuple[int, int]) -> list[FrontierCluster]:
        if self.map is None or self.blocked is None:
            return []

        width = self.map.info.width
        height = self.map.info.height
        data = self.map.data
        frontier = bytearray(width * height)

        for y in range(height):
            row = y * width
            for x in range(width):
                index = row + x
                if self.blocked[index] or not self.is_free_value(data[index]):
                    continue
                if self.has_unknown_neighbor(x, y):
                    frontier[index] = 1

        visited = bytearray(width * height)
        clusters: list[FrontierCluster] = []
        robot_world = self.grid_to_world(*start)

        for index, is_frontier in enumerate(frontier):
            if not is_frontier or visited[index]:
                continue

            cells = []
            queue = deque([index])
            visited[index] = 1

            while queue:
                current = queue.popleft()
                x = current % width
                y = current // width
                cells.append((x, y))
                for nx, ny in self.neighbors8(x, y):
                    ni = ny * width + nx
                    if frontier[ni] and not visited[ni]:
                        visited[ni] = 1
                        queue.append(ni)

            if len(cells) < self.min_frontier_size:
                continue

            target = self.cluster_target(cells)
            target_world = self.grid_to_world(*target)
            distance = math.hypot(target_world[0] - robot_world[0], target_world[1] - robot_world[1])
            score = len(cells) * self.frontier_gain - distance
            clusters.append(FrontierCluster(cells=cells, target=target, score=score))

        clusters.sort(key=lambda cluster: cluster.score, reverse=True)
        return clusters

    def cluster_target(self, cells: list[tuple[int, int]]) -> tuple[int, int]:
        cx = sum(cell[0] for cell in cells) / len(cells)
        cy = sum(cell[1] for cell in cells) / len(cells)
        return min(cells, key=lambda cell: (cell[0] - cx) ** 2 + (cell[1] - cy) ** 2)

    def astar(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]] | None:
        if self.map is None or self.blocked is None:
            return None

        width = self.map.info.width
        start_index = start[1] * width + start[0]
        goal_index = goal[1] * width + goal[0]

        open_heap = [(0.0, start_index)]
        came_from: dict[int, int] = {}
        cost_so_far = {start_index: 0.0}
        expanded = 0

        while open_heap and expanded < self.max_astar_expansions:
            _, current = heapq.heappop(open_heap)
            expanded += 1

            if current == goal_index:
                return self.reconstruct_path(came_from, current, width)

            cx = current % width
            cy = current // width
            for nx, ny, step_cost in self.planning_neighbors(cx, cy):
                neighbor = ny * width + nx
                new_cost = cost_so_far[current] + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.grid_distance((nx, ny), goal)
                    heapq.heappush(open_heap, (priority, neighbor))
                    came_from[neighbor] = current

        return None

    def reconstruct_path(
        self, came_from: dict[int, int], current: int, width: int
    ) -> list[tuple[int, int]]:
        path = []
        while current in came_from:
            path.append((current % width, current // width))
            current = came_from[current]
        path.append((current % width, current // width))
        path.reverse()
        return path

    def planning_neighbors(self, x: int, y: int):
        if self.map is None or self.blocked is None:
            return

        width = self.map.info.width
        height = self.map.info.height
        for dx, dy, cost in (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.4142),
            (1, -1, 1.4142),
            (-1, 1, 1.4142),
            (1, 1, 1.4142),
        ):
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if self.blocked[ny * width + nx]:
                continue
            if dx != 0 and dy != 0:
                if self.blocked[y * width + nx] or self.blocked[ny * width + x]:
                    continue
            yield nx, ny, cost

    def build_blocked_grid(self, msg: OccupancyGrid) -> bytearray:
        width = msg.info.width
        height = msg.info.height
        blocked = bytearray(width * height)
        occupied = []

        for index, value in enumerate(msg.data):
            if value < 0:
                blocked[index] = 1
            elif value >= self.obstacle_threshold:
                blocked[index] = 1
                occupied.append(index)

        inflation_cells = max(1, int(math.ceil(self.obstacle_inflation_radius / msg.info.resolution)))
        radius_squared = inflation_cells * inflation_cells

        for index in occupied:
            ox = index % width
            oy = index // width
            for dy in range(-inflation_cells, inflation_cells + 1):
                ny = oy + dy
                if ny < 0 or ny >= height:
                    continue
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx * dx + dy * dy > radius_squared:
                        continue
                    nx = ox + dx
                    if 0 <= nx < width:
                        blocked[ny * width + nx] = 1

        return blocked

    def nearest_safe_cell(
        self, start: tuple[int, int], max_radius: int
    ) -> tuple[int, int] | None:
        if self.map is None or self.blocked is None:
            return None

        width = self.map.info.width
        height = self.map.info.height
        sx, sy = start
        if not (0 <= sx < width and 0 <= sy < height):
            return None
        if not self.blocked[sy * width + sx] and self.is_free_cell(sx, sy):
            return start

        seen = {(sx, sy)}
        queue = deque([(sx, sy, 0)])
        while queue:
            x, y, radius = queue.popleft()
            if radius >= max_radius:
                continue
            for nx, ny in self.neighbors8(x, y):
                if (nx, ny) in seen:
                    continue
                seen.add((nx, ny))
                if not self.blocked[ny * width + nx] and self.is_free_cell(nx, ny):
                    return nx, ny
                queue.append((nx, ny, radius + 1))
        return None

    def follow_path(self, pose: tuple[float, float, float]):
        if self.current_plan is None:
            self.publish_stop()
            return

        path = self.current_plan.path
        if not path:
            self.set_mode(Mode.PLAN)
            return

        self.path_cursor = self.closest_path_index(pose, path)
        target = self.lookahead_target(pose, path, self.path_cursor)
        target_world = self.grid_to_world(*target)

        angle_to_target = math.atan2(target_world[1] - pose[1], target_world[0] - pose[0])
        angle_error = self.normalize_angle(angle_to_target - pose[2])
        front = self.sector_distance(-28.0, 28.0)
        left = self.sector_distance(35.0, 105.0)
        right = self.sector_distance(-105.0, -35.0)

        angular = self.clamp(1.8 * angle_error, -self.max_angular_speed, self.max_angular_speed)

        if abs(angle_error) > 0.65:
            linear = 0.0
        else:
            heading_scale = max(0.25, 1.0 - abs(angle_error) / 0.90)
            linear = self.linear_speed * heading_scale

        if front < self.slow_clearance:
            linear = min(linear, self.slow_linear_speed)
            angular += self.clamp((left - right) * 0.20, -0.35, 0.35)

        if left < self.side_clearance:
            angular -= 0.25
        elif right < self.side_clearance:
            angular += 0.25

        self.publish_cmd(linear, self.clamp(angular, -self.max_angular_speed, self.max_angular_speed))

    def local_obstacle_requires_recovery(self) -> bool:
        front = self.sector_distance(-30.0, 30.0)
        return front < self.emergency_clearance

    def set_recovery_from_scan(self):
        left = self.sector_distance(45.0, 130.0)
        right = self.sector_distance(-130.0, -45.0)
        self.recover_direction = 1.0 if left >= right else -1.0

    def should_replan(self, now) -> bool:
        if self.mode != Mode.NAVIGATE or self.current_plan is None:
            return True
        if self.seconds_since(self.last_plan_time, now) < self.replan_interval:
            return False
        if self.blocked is not None and self.map is not None:
            width = self.map.info.width
            target_index = self.current_plan.target[1] * width + self.current_plan.target[0]
            if self.blocked[target_index]:
                return True
            for cell in self.current_plan.path[self.path_cursor :: 5]:
                if self.blocked[cell[1] * width + cell[0]]:
                    return True
            return False
        return True

    def reached_goal(self, pose: tuple[float, float, float], target: tuple[int, int]) -> bool:
        target_world = self.grid_to_world(*target)
        return math.hypot(target_world[0] - pose[0], target_world[1] - pose[1]) < self.goal_tolerance

    def closest_path_index(
        self, pose: tuple[float, float, float], path: list[tuple[int, int]]
    ) -> int:
        start = max(0, self.path_cursor - 5)
        end = min(len(path), self.path_cursor + 60)
        best_index = start
        best_distance = float("inf")
        for index in range(start, end):
            wx, wy = self.grid_to_world(*path[index])
            distance = (wx - pose[0]) ** 2 + (wy - pose[1]) ** 2
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def lookahead_target(
        self, pose: tuple[float, float, float], path: list[tuple[int, int]], start_index: int
    ) -> tuple[int, int]:
        for index in range(start_index, len(path)):
            wx, wy = self.grid_to_world(*path[index])
            if math.hypot(wx - pose[0], wy - pose[1]) >= self.waypoint_lookahead:
                return path[index]
        return path[-1]

    def publish_plan(self, plan: ExplorationPlan):
        if self.map is None:
            return

        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = stamp

        stride = max(1, len(plan.path) // 150)
        sampled_path = plan.path[::stride]
        if sampled_path[-1] != plan.path[-1]:
            sampled_path.append(plan.path[-1])

        for cell in sampled_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(*cell)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = stamp
        goal.pose.position.x, goal.pose.position.y = self.grid_to_world(*plan.target)
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.publish_markers(plan)

    def publish_empty_path(self):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(msg)
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_markers(self, plan: ExplorationPlan):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        goal_marker = Marker()
        goal_marker.header.frame_id = self.map_frame
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "exploration_goal"
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x, goal_marker.pose.position.y = self.grid_to_world(*plan.target)
        goal_marker.pose.position.z = 0.15
        goal_marker.scale.x = 0.35
        goal_marker.scale.y = 0.35
        goal_marker.scale.z = 0.35
        goal_marker.color.r = 0.0
        goal_marker.color.g = 0.8
        goal_marker.color.b = 1.0
        goal_marker.color.a = 0.95
        marker_array.markers.append(goal_marker)
        self.marker_pub.publish(marker_array)

    def has_unknown_neighbor(self, x: int, y: int) -> bool:
        if self.map is None:
            return False
        width = self.map.info.width
        for nx, ny in self.neighbors8(x, y):
            if self.map.data[ny * width + nx] < 0:
                return True
        return False

    def is_free_cell(self, x: int, y: int) -> bool:
        if self.map is None:
            return False
        return self.is_free_value(self.map.data[y * self.map.info.width + x])

    def is_free_value(self, value: int) -> bool:
        return 0 <= value <= self.free_threshold

    def neighbors8(self, x: int, y: int):
        if self.map is None:
            return
        width = self.map.info.width
        height = self.map.info.height
        for dy in (-1, 0, 1):
            ny = y + dy
            if ny < 0 or ny >= height:
                continue
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                if 0 <= nx < width:
                    yield nx, ny

    def world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        if self.map is None:
            return None
        info = self.map.info
        yaw = self.quaternion_yaw(info.origin.orientation)
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        gx = int(math.floor(local_x / info.resolution))
        gy = int(math.floor(local_y / info.resolution))
        if 0 <= gx < info.width and 0 <= gy < info.height:
            return gx, gy
        return None

    def grid_to_world(self, x: int, y: int) -> tuple[float, float]:
        if self.map is None:
            return 0.0, 0.0
        info = self.map.info
        yaw = self.quaternion_yaw(info.origin.orientation)
        local_x = (x + 0.5) * info.resolution
        local_y = (y + 0.5) * info.resolution
        world_x = info.origin.position.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        world_y = info.origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        return world_x, world_y

    def sector_distance(self, min_deg: float, max_deg: float) -> float:
        msg = self.scan
        if msg is None or not msg.ranges:
            return float("inf")

        distances = []
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            angle = self.normalize_angle(msg.angle_min + index * msg.angle_increment)
            if self.angle_in_sector(angle, min_rad, max_rad):
                distances.append(distance)

        if not distances:
            return msg.range_max if math.isfinite(msg.range_max) else 12.0

        distances.sort()
        percentile = min(len(distances) - 1, max(0, int(len(distances) * 0.20)))
        return distances[percentile]

    def set_mode(self, mode: Mode):
        if mode != self.mode:
            self.mode = mode
            self.mode_started = self.get_clock().now()
            self.get_logger().info(f"Explorer mode: {mode.value}")

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    @staticmethod
    def angle_in_sector(angle: float, min_angle: float, max_angle: float) -> bool:
        min_angle = AutonomousMapper.normalize_angle(min_angle)
        max_angle = AutonomousMapper.normalize_angle(max_angle)
        if min_angle <= max_angle:
            return min_angle <= angle <= max_angle
        return angle >= min_angle or angle <= max_angle

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def quaternion_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def grid_distance(a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def seconds_since(start, end) -> float:
        return (end - start).nanoseconds / 1_000_000_000.0

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return min(high, max(low, value))


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
