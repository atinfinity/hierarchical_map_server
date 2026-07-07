#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""Integration test that drives to a single goal and records the tile window re-centering during the drive.

Run this with tb3_tile_groundtruth_sim.launch.py (or, if a stable nav2_amcl is
available, tb3_tile_nav_sim.launch.py) already running.

  ros2 run tile_map_server drive_across_boundary.py <goal_x> <goal_y>

Pass criteria: navigation succeeds AND two or more distinct /map windows
(re-centering due to crossing a tile boundary) are published during the drive.

Note: place the goal inside the currently loaded tile window (window_size x tiles).
A goal outside the window results in ABORTED because the global planner cannot plan
a path (a known limitation; for long distances, split into intermediate waypoints
inside the window using NavigateThroughPoses / the waypoint follower).
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class DriveAcrossBoundary(Node):

    def __init__(self, gx, gy):
        super().__init__('drive_across_boundary')
        self.gx, self.gy = gx, gy
        q = QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.origins = []
        self.create_subscription(OccupancyGrid, 'map', self._on_map, q)
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_map(self, m):
        o = (round(m.info.origin.position.x, 2), round(m.info.origin.position.y, 2))
        if not self.origins or self.origins[-1] != o:
            self.origins.append(o)
            self.get_logger().info(f'  tile window recenter -> origin {o} '
                                   f'({m.info.width}x{m.info.height})')

    def run(self):
        if not self.cli.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose server not available')
            return False
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = self.gx
        g.pose.pose.position.y = self.gy
        g.pose.pose.orientation.w = 1.0
        self.get_logger().info(f'sending goal ({self.gx}, {self.gy})')
        f = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if not h or not h.accepted:
            self.get_logger().error('goal rejected')
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=180.0)
        if not rf.result():
            self.get_logger().error('navigation timed out')
            return False
        st = rf.result().status
        self.get_logger().info(f'navigation status={st} (4=SUCCEEDED, 6=ABORTED)')
        return st == 4


def main():
    if len(sys.argv) < 3:
        print('usage: drive_across_boundary.py <goal_x> <goal_y>')
        sys.exit(2)
    rclpy.init()
    node = DriveAcrossBoundary(float(sys.argv[1]), float(sys.argv[2]))
    ok = node.run()
    node.get_logger().info('=' * 48)
    node.get_logger().info(f'navigation success: {ok}')
    node.get_logger().info(f'distinct tile windows during drive: {len(node.origins)}')
    for o in node.origins:
        node.get_logger().info(f'   origin {o}')
    passed = ok and len(node.origins) >= 2
    node.get_logger().info(f'RESULT: {"PASS" if passed else "FAIL"}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
