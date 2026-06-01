#!/usr/bin/env python3
"""
rrt_planner_node
================
Custom RRT path planner.

Inputs
  /map          nav_msgs/OccupancyGrid          (SLAM or map_server)
  /rrt_goal     geometry_msgs/PoseStamped       (dedicated goal - avoids Nav2 bt_navigator conflict)
  start pose    TF map->base_footprint, or /initialpose, or param

Outputs
  /rrt_path     nav_msgs/Path                   (the planned path)
  /rrt_tree     visualization_msgs/MarkerArray  (the search tree + endpoints)

Publish to /rrt_goal to trigger planning. Do NOT use /goal_pose - Nav2
bt_navigator also subscribes to that topic and would drive the robot.
The path appears on /rrt_path. The robot does not move.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import tf2_ros
from tf2_ros import TransformException

from rrt_planner.rrt import RRTPlanner


class RRTPlannerNode(Node):
    def __init__(self):
        super().__init__("rrt_planner")

        self.declare_parameter("use_sim_time", False)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("inflation_radius", 0.20)
        self.declare_parameter("step_size", 0.30)
        self.declare_parameter("goal_bias", 0.10)
        self.declare_parameter("max_iters", 5000)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("rrt_star", True)
        self.declare_parameter("rewire_radius", 0.60)
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value

        self.grid = None
        self.last_initial = None

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, "/map", self._map_cb, latched)
        # Use /rrt_goal (NOT /goal_pose) so Nav2 bt_navigator — which also
        # subscribes to /goal_pose — does not receive this message and drive the robot.
        self.create_subscription(PoseStamped, "/rrt_goal", self._goal_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._init_cb, 10)

        self.path_pub = self.create_publisher(Path, "/rrt_path", latched)
        self.tree_pub = self.create_publisher(MarkerArray, "/rrt_tree", latched)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            "rrt_planner ready - publish to /rrt_goal to plan a path (robot will NOT move)")

    # ---- inputs ----------------------------------------------------------
    def _map_cb(self, msg):
        self.grid = msg
        self.get_logger().info(
            f"map: {msg.info.width}x{msg.info.height} "
            f"@ {msg.info.resolution:.3f} m/cell")

    def _init_cb(self, msg):
        self.last_initial = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"start set via /initialpose: {self.last_initial}")

    def _start_pose(self):
        # 1) live robot pose from TF
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            pass
        # 2) last RViz initialpose
        if self.last_initial is not None:
            return self.last_initial
        # 3) parameter fallback
        return (self.get_parameter("start_x").value,
                self.get_parameter("start_y").value)

    # ---- planning --------------------------------------------------------
    def _goal_cb(self, msg):
        if self.grid is None:
            self.get_logger().warn("no map yet - cannot plan")
            return

        info = self.grid.info
        occ = np.array(self.grid.data, dtype=int).reshape(
            info.height, info.width)

        planner = RRTPlanner(
            occ, info.resolution,
            info.origin.position.x, info.origin.position.y,
            inflation_radius=self.get_parameter("inflation_radius").value,
            step_size=self.get_parameter("step_size").value,
            goal_bias=self.get_parameter("goal_bias").value,
            max_iters=int(self.get_parameter("max_iters").value),
            goal_tolerance=self.get_parameter("goal_tolerance").value,
            rrt_star=self.get_parameter("rrt_star").value,
            rewire_radius=self.get_parameter("rewire_radius").value,
            treat_unknown_as_obstacle=self.get_parameter(
                "treat_unknown_as_obstacle").value)

        start = self._start_pose()
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"planning {start} -> {goal} ...")

        result = planner.plan(start, goal)
        self._publish_tree(result["edges"], start, goal)

        if not result["found"]:
            self.get_logger().warn(
                f"no path ({result.get('reason')}, {result['iters']} iters)")
            self.path_pub.publish(Path(header=self._hdr()))
            return

        self.get_logger().info(
            f"path found: {len(result['path'])} pts, "
            f"{result['length']:.2f} m, {result['iters']} iters")
        self._publish_path(result["path"])

    # ---- outputs ---------------------------------------------------------
    def _hdr(self):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        return path.header

    def _publish_path(self, pts):
        path = Path()
        path.header = self._hdr()
        for (x, y) in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _publish_tree(self, edges, start, goal):
        arr = MarkerArray()

        line = Marker()
        line.header = self._hdr()
        line.ns = "rrt_tree"
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.01
        line.color = ColorRGBA(r=0.4, g=0.5, b=0.6, a=0.6)
        line.pose.orientation.w = 1.0
        for a, b in edges:
            line.points.append(Point(x=a[0], y=a[1], z=0.0))
            line.points.append(Point(x=b[0], y=b[1], z=0.0))
        arr.markers.append(line)

        for i, (pt, col) in enumerate([(start, (0.2, 0.8, 0.4)),
                                       (goal, (0.9, 0.6, 0.2))]):
            m = Marker()
            m.header = self._hdr()
            m.ns = "rrt_endpoints"
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = float(pt[0]), float(pt[1])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=1.0)
            arr.markers.append(m)

        self.tree_pub.publish(arr)


def main():
    rclpy.init()
    node = RRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
