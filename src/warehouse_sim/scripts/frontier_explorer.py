#!/usr/bin/env python3
"""Autonomous frontier exploration for hikbot.

Subscribes to /map, finds frontier cells (free adjacent to unknown),
clusters them, and drives to the largest non-blacklisted cluster.
Stops when no frontiers remain or too many total failures occur.
"""

import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class FrontierExplorer(BasicNavigator):
    MIN_FRONTIER_CELLS = 10   # ignore tiny frontiers (noise)
    MAX_TOTAL_FAILURES = 10   # stop exploration after this many total failures
    BLACKLIST_RADIUS = 1.2    # m — don't retry near a previously failed goal

    def __init__(self):
        super().__init__('frontier_explorer')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_data: OccupancyGrid | None = None
        self._blacklist: list[tuple[float, float]] = []
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_data = msg

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------

    def _find_frontiers(self) -> list[tuple[float, float, int]]:
        """Return (world_x, world_y, cell_count) for each frontier cluster."""
        msg = self.map_data
        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
        free = grid == 0
        unknown = grid == -1

        # Free cells adjacent to unknown = frontier
        up = np.pad(unknown, ((1, 0), (0, 0)), constant_values=False)[:-1, :]
        dn = np.pad(unknown, ((0, 1), (0, 0)), constant_values=False)[1:, :]
        lt = np.pad(unknown, ((0, 0), (1, 0)), constant_values=False)[:, :-1]
        rt = np.pad(unknown, ((0, 0), (0, 1)), constant_values=False)[:, 1:]
        frontier_mask = free & (up | dn | lt | rt)

        if not frontier_mask.any():
            return []

        # Flood-fill connected frontier regions
        labeled = np.zeros((h, w), dtype=np.int32)
        label_id = 0
        for r, c in zip(*np.where(frontier_mask)):
            r, c = int(r), int(c)
            if labeled[r, c] != 0:
                continue
            label_id += 1
            stack = [(r, c)]
            while stack:
                cr, cc = stack.pop()
                if labeled[cr, cc] != 0 or not frontier_mask[cr, cc]:
                    continue
                labeled[cr, cc] = label_id
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < h and 0 <= nc < w and labeled[nr, nc] == 0:
                        stack.append((nr, nc))

        centroids = []
        for lid in range(1, label_id + 1):
            rs, cs = np.where(labeled == lid)
            if len(rs) < self.MIN_FRONTIER_CELLS:
                continue
            wx = float(np.mean(cs)) * res + ox
            wy = float(np.mean(rs)) * res + oy
            centroids.append((wx, wy, int(len(rs))))

        return centroids

    def _is_blacklisted(self, x: float, y: float) -> bool:
        r2 = self.BLACKLIST_RADIUS ** 2
        return any((x - bx) ** 2 + (y - by) ** 2 < r2
                   for bx, by in self._blacklist)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def explore(self) -> None:
        self.info('Waiting for Nav2 to become active...')
        self.waitUntilNav2Active(localizer='slam_toolbox')
        self.info('Nav2 active. Exploration starting.')

        total_failures = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)

            if self.map_data is None:
                self.info('Waiting for /map...')
                continue

            frontiers = self._find_frontiers()

            # Filter blacklisted frontiers
            available = [(x, y, s) for x, y, s in frontiers
                         if not self._is_blacklisted(x, y)]

            if not available:
                if frontiers:
                    # All frontiers blacklisted — clear half the blacklist and retry
                    self.warn('All frontiers blacklisted. Clearing blacklist.')
                    self._blacklist = self._blacklist[len(self._blacklist) // 2:]
                    continue
                self.info('No frontiers left — exploration complete!')
                break

            # Pick largest available frontier
            available.sort(key=lambda f: f[2], reverse=True)
            tx, ty, tsize = available[0]

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = tx
            goal.pose.position.y = ty
            goal.pose.orientation.w = 1.0

            self.info(
                f'Frontier ({tx:.2f}, {ty:.2f}) size={tsize} '
                f'available={len(available)} blacklisted={len(self._blacklist)}'
            )
            self.goToPose(goal)

            while not self.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            result = self.getResult()
            if result == TaskResult.SUCCEEDED:
                self.info('Reached frontier.')
                total_failures = max(0, total_failures - 1)  # reward success
            else:
                total_failures += 1
                self._blacklist.append((tx, ty))
                self.warn(
                    f'Failed ({result}). Blacklisted ({tx:.2f},{ty:.2f}). '
                    f'Total failures: {total_failures}/{self.MAX_TOTAL_FAILURES}'
                )
                if total_failures >= self.MAX_TOTAL_FAILURES:
                    self.error('Too many failures. Stopping.')
                    break

        self.info('Frontier exploration finished.')


def main() -> None:
    rclpy.init()
    explorer = FrontierExplorer()
    try:
        explorer.explore()
    except KeyboardInterrupt:
        pass
    finally:
        explorer.lifecycleShutdown()
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
