#!/usr/bin/env python3
"""
semantic_map
============
The robot's semantic memory.

Subscribes : /semantic/observations (String JSON from the tagger)
Publishes  : /semantic/places       (String JSON, latched - the place list)
             /semantic/markers       (visualization_msgs/MarkerArray for RViz)
Persists   : <persist_path> JSON on every update

Each incoming observation is embedded (so query-time matching uses the
same vector space) and merged into a nearby place of the same label, or
becomes a new place.
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from semantic_nav.store import SemanticStore
from semantic_nav.embedding import get_embedder


class SemanticMap(Node):
    def __init__(self):
        super().__init__("semantic_map")

        self.declare_parameter("persist_path", "/tmp/semantic_map.json")
        self.declare_parameter("merge_distance", 1.0)
        self.declare_parameter("map_frame", "map")

        self.persist_path = self.get_parameter("persist_path").value
        self.map_frame = self.get_parameter("map_frame").value
        merge = self.get_parameter("merge_distance").value

        self.store = SemanticStore(merge_distance=merge)

        # Load semantic map from disk if it exists (to preserve tags across
        # localization sessions). The map is only cleared at the web_backend
        # level when Explore is started (via /api/clear_semantic).
        try:
            self.store.load(self.persist_path)
            self.get_logger().info(
                f"Loaded {len(self.store.list_places())} places from "
                f"{self.persist_path}")
        except FileNotFoundError:
            self.get_logger().info(
                f"No prior semantic map at {self.persist_path} — starting fresh")
        except Exception as e:
            self.get_logger().warn(f"Could not load semantic map: {e}")

        self.embedder = get_embedder()
        self.get_logger().info(f"Embedding backend: {self.embedder.name}")

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.places_pub = self.create_publisher(
            String, "/semantic/places", latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/semantic/markers", latched)

        self.create_subscription(
            String, "/semantic/observations", self._obs_cb, 10)

        self._publish_all()

    def _obs_cb(self, msg):
        obs = json.loads(msg.data)
        emb = self.embedder.embed(obs["caption"])
        self.store.add_observation(
            obs["label"], obs["caption"],
            obs["x"], obs["y"], obs["theta"],
            emb, obs["confidence"], image=obs.get("image", ""))
        try:
            self.store.save(self.persist_path)
        except Exception as e:
            self.get_logger().warn(f"persist failed: {e}")
        self._publish_all()

    def _publish_all(self):
        places = self.store.list_places()
        self.places_pub.publish(String(data=json.dumps({"places": places})))
        self.marker_pub.publish(self._markers(places))

    def _markers(self, places) -> MarkerArray:
        arr = MarkerArray()
        for p in places:
            dot = Marker()
            dot.header.frame_id = self.map_frame
            dot.ns = "semantic_dot"
            dot.id = p["id"]
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = p["x"]
            dot.pose.position.y = p["y"]
            dot.pose.position.z = 0.1
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.25
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = \
                0.12, 0.55, 0.96, 0.9
            arr.markers.append(dot)

            txt = Marker()
            txt.header.frame_id = self.map_frame
            txt.ns = "semantic_label"
            txt.id = p["id"]
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = p["x"]
            txt.pose.position.y = p["y"]
            txt.pose.position.z = 0.5
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.3
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = p["label"]
            arr.markers.append(txt)
        return arr


def main():
    rclpy.init()
    node = SemanticMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
