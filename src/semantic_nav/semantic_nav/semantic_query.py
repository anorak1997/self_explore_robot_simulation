#!/usr/bin/env python3
"""
semantic_query
==============
Exposes the open-vocabulary query API as a ROS 2 service.

Service : /query_location  (semantic_msgs/QueryLocation)
          request.query (free text)  ->  response.pose (map frame)

Subscribes : /semantic/places (String JSON) to keep an up-to-date copy
of the robot's semantic memory.

This node only RESOLVES a query to a pose. Driving there is done by the
caller (the query_and_go helper, or the web backend) so the API stays a
clean "text in, pose out" contract.

CLI test:
  ros2 service call /query_location semantic_msgs/srv/QueryLocation \
      "{query: 'where is the toilet?'}"
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math

from semantic_msgs.srv import QueryLocation
from semantic_nav.store import SemanticStore
from semantic_nav.embedding import get_embedder


class SemanticQuery(Node):
    def __init__(self):
        super().__init__("semantic_query")

        self.declare_parameter("match_threshold", 0.35)
        self.declare_parameter("map_frame", "map")
        self.threshold = self.get_parameter("match_threshold").value
        self.map_frame = self.get_parameter("map_frame").value

        self.store = SemanticStore()
        self.embedder = get_embedder()
        self.get_logger().info(f"Embedding backend: {self.embedder.name}")

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, "/semantic/places", self._places_cb, latched)

        self.srv = self.create_service(
            QueryLocation, "/query_location", self._on_query)
        self.get_logger().info("/query_location service ready")

    def _places_cb(self, msg):
        self.store.load_json(msg.data)

    def _on_query(self, request, response):
        query = request.query
        qvec = self.embedder.embed(query)
        place, score = self.store.query(qvec)

        if place is None or score < self.threshold:
            response.found = False
            response.score = float(max(score, 0.0))
            self.get_logger().info(
                f'query "{query}" -> no match (best score {score:.2f})')
            return response

        response.found = True
        response.matched_label = place["label"]
        response.score = float(score)

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = place["x"]
        pose.pose.position.y = place["y"]
        theta = place["theta"]
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        response.pose = pose

        self.get_logger().info(
            f'query "{query}" -> {place["label"]} '
            f'({score:.2f}) @ ({place["x"]:.2f}, {place["y"]:.2f})')
        return response


def main():
    rclpy.init()
    node = SemanticQuery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
