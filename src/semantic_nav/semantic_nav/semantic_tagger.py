#!/usr/bin/env python3
"""
semantic_tagger
===============
Automatically tags rooms as the robot explores, and publishes semantic
observations on /semantic/observations (std_msgs/String JSON).

Backends (param `backend`):
  clip  - REAL perception: classify the live camera frame with CLIP
          (open-vocabulary, labels from config/labels.yaml). Optional depth
          (depth image or pointcloud) places the tag at the observed surface
          and rejects empty views. THIS IS FULLY AUTOMATIC - no hand marking.
  mock  - pose-based fallback for a camera-less robot: caption from
          config/regions.yaml (requires hand-marked boxes).
  auto  - try clip; if transformers/camera unavailable, fall back to mock.

Pose comes from TF (map -> robot_frame), so an active `map` frame is
required (SLAM running, or AMCL+map_server on a saved map).
"""

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2

import tf2_ros
from tf2_ros import TransformException

import yaml


def yaw_from_quat(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class SemanticTagger(Node):
    def __init__(self):
        super().__init__("semantic_tagger")

        self.declare_parameter("backend", "auto")          # auto|clip|mock
        self.declare_parameter("labels_path", "")
        self.declare_parameter("regions_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("use_depth", False)
        self.declare_parameter("depth_type", "image")       # image|cloud|none
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_max_range", 4.0)
        self.declare_parameter("min_confidence", 0.30)
        self.declare_parameter("tag_period", 2.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.use_depth = self.get_parameter("use_depth").value
        self.depth_type = self.get_parameter("depth_type").value
        self.depth_max = self.get_parameter("depth_max_range").value
        period = self.get_parameter("tag_period").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(String, "/semantic/observations", 10)

        self._last_image = None
        self._last_depth = None
        self._warned_no_pose = False
        self._warned_no_image = False
        # Cooldown: don't re-emit the same label within this many seconds
        # (prevents flooding the store while the robot stands still).
        self._last_tag: dict = {}   # label -> timestamp (float)
        self._tag_cooldown = 15.0   # seconds

        self.mode = self._select_backend()
        self._setup_subscriptions()

        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"semantic_tagger up | mode={self.mode} | period={period}s | "
            f"use_depth={self.use_depth}")

    # ---- backend selection ----------------------------------------------
    def _select_backend(self):
        want = self.get_parameter("backend").value
        if want in ("auto", "clip"):
            try:
                labels = self._load_labels()
                from semantic_nav.clip_vlm import ClipVLM
                self.clip = ClipVLM(
                    labels,
                    min_confidence=self.get_parameter("min_confidence").value)
                self.get_logger().info(
                    f"CLIP backend ready ({len(labels)} candidate labels)")
                return "clip"
            except Exception as e:
                if want == "clip":
                    self.get_logger().error(
                        f"CLIP backend requested but failed: "
                        f"{type(e).__name__}: {e}")
                else:
                    self.get_logger().warn(
                        f"CLIP unavailable ({type(e).__name__}: {e}); "
                        f"falling back to mock.")
        # mock
        from semantic_nav.mock_vlm import MockVLM
        self.mock = MockVLM(self.get_parameter("regions_path").value)
        if not self.mock.regions:
            self.get_logger().warn(
                "mock backend has no regions - nothing will be tagged. "
                "Set regions_path or use the clip backend with a camera.")
        return "mock"

    def _load_labels(self):
        path = self.get_parameter("labels_path").value
        if not path:
            raise FileNotFoundError("labels_path not set")
        with open(path) as fh:
            data = yaml.safe_load(fh) or {}
        labels = data.get("labels", [])
        if not labels:
            raise ValueError("labels.yaml has no labels")
        return labels

    def _setup_subscriptions(self):
        if self.mode == "clip":
            self.create_subscription(
                Image, self.get_parameter("image_topic").value,
                self._image_cb, 1)
            if self.use_depth and self.depth_type == "image":
                self.create_subscription(
                    Image, self.get_parameter("depth_topic").value,
                    self._depth_cb, 1)
            elif self.use_depth and self.depth_type == "cloud":
                self.create_subscription(
                    PointCloud2, self.get_parameter("depth_topic").value,
                    self._depth_cb, 1)

    # ---- callbacks -------------------------------------------------------
    def _image_cb(self, msg):
        self._last_image = msg

    def _depth_cb(self, msg):
        self._last_depth = msg

    def _pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return (tf.transform.translation.x, tf.transform.translation.y,
                yaw_from_quat(q.x, q.y, q.z, q.w))

    # ---- main loop -------------------------------------------------------
    def _tick(self):
        pose = self._pose()
        if pose is None:
            if not self._warned_no_pose:
                self.get_logger().warn(
                    "no map->%s transform yet; is SLAM/localization running?"
                    % self.robot_frame)
                self._warned_no_pose = True
            return
        self._warned_no_pose = False
        x, y, theta = pose

        if self.mode == "clip":
            self._tick_clip(x, y, theta)
        else:
            self._tick_mock(x, y, theta)

    def _tick_clip(self, x, y, theta):
        if self._last_image is None:
            if not self._warned_no_image:
                self.get_logger().warn(
                    "clip mode but no image on %s yet"
                    % self.get_parameter("image_topic").value)
                self._warned_no_image = True
            return
        self._warned_no_image = False

        from semantic_nav.clip_vlm import decode_image_msg
        rgb = decode_image_msg(self._last_image)
        if rgb is None:
            self.get_logger().warn(
                f"unsupported image encoding: {self._last_image.encoding}")
            return

        result = self.clip.classify(rgb)
        if result is None:
            return  # nothing confident in view

        tag_x, tag_y = x, y
        if self.use_depth and self._last_depth is not None:
            from semantic_nav.depth_utils import (
                center_depth_from_image, center_depth_from_cloud,
                project_forward)
            if self.depth_type == "image":
                d = center_depth_from_image(self._last_depth)
            elif self.depth_type == "cloud":
                d = center_depth_from_cloud(self._last_depth, self.depth_max)
            else:
                d = None
            if d is None or d > self.depth_max:
                return  # nothing solid close enough -> don't tag empty space
            tag_x, tag_y = project_forward(x, y, theta, d)

        self._publish(result, tag_x, tag_y, theta)

    def _tick_mock(self, x, y, theta):
        result = self.mock.caption_image(x, y)
        if result is None:
            return
        # Use the region centre (if provided) so duplicates merge cleanly.
        tx = result.get("x", x)
        ty = result.get("y", y)
        self._publish(result, tx, ty, theta)

    def _publish(self, result, x, y, theta):
        import time as _time
        label = result["label"]
        now = _time.monotonic()
        if now - self._last_tag.get(label, 0.0) < self._tag_cooldown:
            return  # same room still cooling down — skip
        self._last_tag[label] = now

        obs = {
            "label": result["label"],
            "caption": result["caption"],
            "confidence": result["confidence"],
            "image": result.get("image", ""),
            "x": x, "y": y, "theta": theta,
            "stamp": self.get_clock().now().nanoseconds,
        }
        self.pub.publish(String(data=json.dumps(obs)))
        self.get_logger().info(
            f'tagged "{result["label"]}" ({result["confidence"]:.2f}) '
            f'@ ({x:.2f}, {y:.2f})')


def main():
    rclpy.init()
    node = SemanticTagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
