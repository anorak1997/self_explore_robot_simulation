#!/usr/bin/env python3
"""
Frontier-based autonomous exploration node for TurtleBot3.

How it works:
  1. Subscribes to /map (the occupancy grid from SLAM Toolbox).
  2. Finds "frontier" cells: free cells (value 0) adjacent to unknown cells (value -1).
  3. Clusters frontier cells into groups and picks the best one as a goal.
  4. Sends the goal to Nav2 via the NavigateToPose action.
  5. Repeats until no frontiers remain -> map is complete.
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header

import tf2_ros
from tf2_ros import TransformException


# Occupancy grid cell values
FREE = 0
UNKNOWN = -1
OCCUPIED = 100


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ---- Parameters ----
        self.declare_parameter('min_frontier_size', 8)        # min cluster size (cells)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('exploration_timeout', 60.0)   # sec per goal
        self.declare_parameter('replan_period', 5.0)          # sec between replans
        self.declare_parameter('safety_radius', 0.25)         # meters from obstacles
        self.declare_parameter('goal_reached_tolerance', 0.35)
        self.declare_parameter('min_goal_distance', 1.0)      # reject goals closer than this
        self.declare_parameter('initial_spin', True)          # spin once at startup
        self.declare_parameter('initial_spin_duration', 12.0) # ~full 360° at 0.5 rad/s

        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.replan_period = self.get_parameter('replan_period').value
        self.safety_radius = self.get_parameter('safety_radius').value
        self.goal_tolerance = self.get_parameter('goal_reached_tolerance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.do_initial_spin = self.get_parameter('initial_spin').value
        self.initial_spin_duration = self.get_parameter('initial_spin_duration').value

        # ---- State ----
        self.map_data = None        # latest OccupancyGrid
        self.current_goal = None    # (x, y) in world coords
        self.goal_start_time = None
        self.nav_in_progress = False
        self.goal_handle = None
        self.blacklist = []         # failed goal positions to avoid
        self.no_frontier_count = 0  # consecutive cycles with no frontier
        self.initial_spin_done = not self.do_initial_spin
        self.initial_spin_start = None

        # ---- QoS for map topic (Transient Local so we receive latched maps) ----
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # ---- Subscribers / Publishers / Action client ----
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ---- TF2 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Timers ----
        self.exploration_timer = self.create_timer(self.replan_period, self.explore_step)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Frontier Explorer started')
        self.get_logger().info(f'  min_frontier_size = {self.min_frontier_size}')
        self.get_logger().info(f'  replan_period     = {self.replan_period}s')
        self.get_logger().info(f'  safety_radius     = {self.safety_radius}m')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for /map and Nav2 to come up...')

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    # ----------------------------------------------------------------------
    # Robot pose lookup
    # ----------------------------------------------------------------------
    def get_robot_pose(self):
        """Return (x, y) of the robot in map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return None

    # ----------------------------------------------------------------------
    # World <-> grid index conversions
    # ----------------------------------------------------------------------
    def world_to_map(self, x, y):
        info = self.map_data.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None

    def map_to_world(self, mx, my):
        info = self.map_data.info
        x = info.origin.position.x + (mx + 0.5) * info.resolution
        y = info.origin.position.y + (my + 0.5) * info.resolution
        return x, y

    # ----------------------------------------------------------------------
    # Frontier detection
    # ----------------------------------------------------------------------
    def find_frontiers(self, grid_2d):
        """
        A frontier cell is FREE and has at least one UNKNOWN neighbor.
        Returns clusters of frontier cells.
        """
        h, w = grid_2d.shape
        is_free = (grid_2d == FREE)
        is_unknown = (grid_2d == UNKNOWN)

        # Shift unknown mask in 4 directions and OR them — gives "has unknown neighbor"
        unknown_neighbor = np.zeros_like(is_unknown)
        unknown_neighbor[1:, :]  |= is_unknown[:-1, :]
        unknown_neighbor[:-1, :] |= is_unknown[1:, :]
        unknown_neighbor[:, 1:]  |= is_unknown[:, :-1]
        unknown_neighbor[:, :-1] |= is_unknown[:, 1:]

        frontier_mask = is_free & unknown_neighbor

        # Cluster connected frontier cells via flood fill
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []
        ys, xs = np.where(frontier_mask)

        for start_y, start_x in zip(ys, xs):
            if visited[start_y, start_x]:
                continue
            cluster = []
            q = deque([(start_y, start_x)])
            visited[start_y, start_x] = True
            while q:
                cy, cx = q.popleft()
                cluster.append((cx, cy))
                # 8-connectivity
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w:
                            if frontier_mask[ny, nx] and not visited[ny, nx]:
                                visited[ny, nx] = True
                                q.append((ny, nx))
            if len(cluster) >= self.min_frontier_size:
                clusters.append(cluster)

        return clusters

    def cluster_centroid(self, cluster):
        xs = [c[0] for c in cluster]
        ys = [c[1] for c in cluster]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    # ----------------------------------------------------------------------
    # Safety: ensure the goal cell is not too close to obstacles
    # ----------------------------------------------------------------------
    def is_safe_cell(self, grid_2d, mx, my):
        info = self.map_data.info
        r_cells = int(math.ceil(self.safety_radius / info.resolution))
        h, w = grid_2d.shape
        x0 = max(0, mx - r_cells)
        x1 = min(w, mx + r_cells + 1)
        y0 = max(0, my - r_cells)
        y1 = min(h, my + r_cells + 1)
        patch = grid_2d[y0:y1, x0:x1]
        return not np.any(patch >= 65)  # treat >=65 as occupied/inflated

    # ----------------------------------------------------------------------
    # Pick best frontier
    #   - For each cluster, pick the SAFE cell that's farthest from the robot
    #     (centroids fall right next to the robot at the start).
    #   - Reject anything closer than min_goal_distance.
    # ----------------------------------------------------------------------
    def select_goal(self, clusters, grid_2d, robot_xy):
        rx, ry = robot_xy
        best = None
        best_score = float('inf')

        for cluster in clusters:
            # Find the SAFE cell in this cluster farthest from the robot.
            chosen_world = None
            chosen_dist = -1.0
            for (mx, my) in cluster:
                if not self.is_safe_cell(grid_2d, mx, my):
                    continue
                wx, wy = self.map_to_world(mx, my)
                d = math.hypot(wx - rx, wy - ry)
                if d > chosen_dist:
                    chosen_dist = d
                    chosen_world = (wx, wy)

            if chosen_world is None:
                continue

            wx, wy = chosen_world

            # Reject goals that are too close — Nav2 reports them as already-reached.
            if chosen_dist < self.min_goal_distance:
                continue

            # Skip blacklisted goals
            if any(math.hypot(wx - bx, wy - by) < 0.5 for bx, by in self.blacklist):
                continue

            # Score: prefer closer + larger clusters (but past the min distance gate)
            score = chosen_dist - 0.05 * len(cluster)
            if score < best_score:
                best_score = score
                best = (wx, wy)

        return best

    # ----------------------------------------------------------------------
    # Send goal to Nav2
    # ----------------------------------------------------------------------
    def send_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available yet.')
            return False

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0  # face +x (orientation isn't critical for exploration)
        goal_msg.pose = pose

        self.get_logger().info(f'-> Sending goal to ({x:.2f}, {y:.2f})')
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

        self.current_goal = (x, y)
        self.goal_start_time = time.time()
        self.nav_in_progress = True
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.nav_in_progress = False
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        # 4 = SUCCEEDED in action_msgs/GoalStatus
        if status == 4:
            self.get_logger().info('✓ Goal reached.')
        else:
            self.get_logger().warn(f'✗ Goal ended with status {status}, blacklisting.')
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)
        self.nav_in_progress = False
        self.goal_handle = None

    def cancel_current_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling stuck goal...')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.nav_in_progress = False

    # ----------------------------------------------------------------------
    # Main exploration loop
    # ----------------------------------------------------------------------
    def explore_step(self):
        if self.map_data is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        robot_xy = self.get_robot_pose()
        if robot_xy is None:
            self.get_logger().info('Waiting for robot TF...', throttle_duration_sec=5.0)
            return

        # ---- Initial 360° spin so SLAM gets a full view before exploring ----
        if not self.initial_spin_done:
            if self.initial_spin_start is None:
                self.initial_spin_start = time.time()
                self.get_logger().info(
                    f'Initial 360° scan: spinning for {self.initial_spin_duration:.1f}s...'
                )
            elapsed = time.time() - self.initial_spin_start
            if elapsed < self.initial_spin_duration:
                self.spin_in_place()
                return
            self.stop_robot()
            self.initial_spin_done = True
            self.get_logger().info('Initial scan complete. Starting exploration.')
            return

        # Check timeout on current goal
        if self.nav_in_progress and self.goal_start_time is not None:
            elapsed = time.time() - self.goal_start_time
            if elapsed > self.exploration_timeout:
                self.get_logger().warn(f'Goal timeout ({elapsed:.1f}s), cancelling.')
                if self.current_goal is not None:
                    self.blacklist.append(self.current_goal)
                self.cancel_current_goal()
            # Also: if we got close enough AND we've been moving a while, replan
            elif self.current_goal is not None and elapsed > 3.0:
                d = math.hypot(robot_xy[0] - self.current_goal[0],
                               robot_xy[1] - self.current_goal[1])
                if d < self.goal_tolerance:
                    self.get_logger().info('Within tolerance, replanning to next frontier.')
                    self.cancel_current_goal()
            if self.nav_in_progress:
                return  # still working on current goal

        # Build 2D grid
        info = self.map_data.info
        grid_2d = np.array(self.map_data.data, dtype=np.int8).reshape(
            info.height, info.width
        )

        clusters = self.find_frontiers(grid_2d)
        self.get_logger().info(f'Found {len(clusters)} frontier clusters.')

        if not clusters:
            self.no_frontier_count += 1
            if self.no_frontier_count >= 3:
                self.get_logger().info('=' * 60)
                self.get_logger().info('🎉 Exploration complete — no more frontiers!')
                self.get_logger().info('Save your map with:')
                self.get_logger().info('  ros2 run nav2_map_server map_saver_cli -f ~/house_map')
                self.get_logger().info('=' * 60)
                self.stop_robot()
                # keep spinning so the user can save the map; just stop replanning
                self.exploration_timer.cancel()
            else:
                # rotate in place to look around
                self.spin_in_place()
            return
        else:
            self.no_frontier_count = 0

        goal = self.select_goal(clusters, grid_2d, robot_xy)
        if goal is None:
            self.get_logger().warn('All frontiers blacklisted or unsafe. Clearing blacklist.')
            self.blacklist.clear()
            return

        self.send_goal(goal[0], goal[1])

    def spin_in_place(self):
        """Rotate slowly to gather more LIDAR data when stuck."""
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
