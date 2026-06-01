#!/usr/bin/env python3
"""
query_and_go
============
End-to-end CLI test: take a text query, resolve it via /query_location,
and drive there with Nav2 (NavigateToPose).

Usage:
  ros2 run semantic_nav query_and_go "where is the toilet?"
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from semantic_msgs.srv import QueryLocation


class QueryAndGo(Node):
    def __init__(self, query):
        super().__init__("query_and_go")
        self.query = query
        self.cli = self.create_client(QueryLocation, "/query_location")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def run(self):
        self.get_logger().info("waiting for /query_location ...")
        self.cli.wait_for_service()
        req = QueryLocation.Request()
        req.query = self.query
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()

        if not res.found:
            self.get_logger().error(
                f'no place matched "{self.query}" (score {res.score:.2f})')
            return

        self.get_logger().info(
            f'matched "{res.matched_label}" ({res.score:.2f}) -> navigating')

        self.nav.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose = res.pose
        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal")
            return
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        self.get_logger().info("navigation finished")


def main():
    query = " ".join(sys.argv[1:]) or "where is the toilet?"
    rclpy.init()
    node = QueryAndGo(query)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
