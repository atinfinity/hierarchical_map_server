#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""tile_map_server integration drive test.

Drives via NavigateToPose to a goal that crosses multiple tile boundaries, and during
the drive verifies that:
  - The /map origin transitions as the tile window re-centers
  - The AMCL covariance (/amcl_pose) does not diverge (localization continuity across map swaps)
  - Navigation finishes with success
Run this with tb3_tile_nav_sim.launch.py already running.
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class BoundaryNavTester(Node):

    def __init__(self):
        super().__init__('nav_across_boundary_test')
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_origins = []
        self.max_cov = 0.0
        self.amcl_samples = 0
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def set_initial_pose(self, x, y, yaw):
        """Publish an initial pose to make AMCL converge."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.068  # yaw
        # Not transient_local, so publish several times to avoid drops
        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info(f'published initial pose ({x}, {y})')

    def _on_map(self, msg):
        origin = (round(msg.info.origin.position.x, 2),
                  round(msg.info.origin.position.y, 2))
        if not self.map_origins or self.map_origins[-1] != origin:
            self.map_origins.append(origin)
            self.get_logger().info(
                f'map window origin -> {origin} '
                f'({msg.info.width}x{msg.info.height})')

    def _on_amcl(self, msg):
        cov = msg.pose.covariance
        # Track the maximum of the x, y, yaw variances
        var = max(cov[0], cov[7], cov[35])
        self.max_cov = max(self.max_cov, var)
        self.amcl_samples += 1

    def send_goal(self, x, y, yaw):
        self.get_logger().info(f'waiting for navigate_to_pose action server ...')
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'sending goal ({x}, {y}) ...')
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        if result_future.result() is None:
            self.get_logger().error('navigation timed out')
            return False
        status = result_future.result().status
        self.get_logger().info(f'navigation finished, status={status}')
        return status == 4  # STATUS_SUCCEEDED


def main():
    rclpy.init()
    node = BoundaryNavTester()

    # Converge AMCL before starting the drive
    node.set_initial_pose(-2.0, -0.5, 0.0)
    # Spin for a few seconds until AMCL/costmap stabilize
    end = node.get_clock().now() + Duration(seconds=5.0)
    while node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    # From spawn (-2, -0.5, tile 3/4 boundary) to (1.5, 1.5, tile5):
    # crosses x=0 (tile4->5) and y=0 (tile4->5)
    ok = node.send_goal(1.5, 1.5, 0.0)

    unique_origins = len(node.map_origins)
    node.get_logger().info('=' * 50)
    node.get_logger().info(f'navigation success : {ok}')
    node.get_logger().info(f'distinct map windows published : {unique_origins}')
    node.get_logger().info(f'  origins: {node.map_origins}')
    node.get_logger().info(f'amcl pose samples : {node.amcl_samples}')
    node.get_logger().info(f'max amcl position variance : {node.max_cov:.4f} m^2')
    node.get_logger().info('=' * 50)

    # Pass criteria: drive succeeds + window transitions two or more times (boundary crossing) + covariance has not diverged (<1.0 m^2)
    passed = ok and unique_origins >= 2 and node.max_cov < 1.0
    node.get_logger().info(f'RESULT: {"PASS" if passed else "FAIL"}')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
