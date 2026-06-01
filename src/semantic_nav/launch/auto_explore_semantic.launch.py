"""
ONE COMMAND, fully automatic:
  Gazebo + SLAM + Nav2 + autonomous frontier exploration   (Section 1)
  + real CLIP camera tagging + semantic map/query + dashboard (Section 2)

  ros2 launch semantic_nav auto_explore_semantic.launch.py

The robot explores on its own; as it sees rooms, CLIP tags them
automatically (no hand-marking); query them in RViz or the dashboard at
http://localhost:8080. Use TURTLEBOT3_MODEL=waffle_pi (has a camera).

Args:
  use_depth:=true depth_type:=cloud depth_topic:=/intel_realsense_r200_depth/points
      -> fuse a depth pointcloud (needs the RealSense waffle model)
  backend:=mock
      -> camera-less fallback using config/regions.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("semantic_nav")
    labels = os.path.join(pkg, "config", "labels.yaml")
    regions = os.path.join(pkg, "config", "regions.yaml")
    persist = os.path.join(os.path.expanduser("~"), "semantic_map.json")

    explorer_pkg = get_package_share_directory("turtlebot3_explorer")
    explore_launch = os.path.join(explorer_pkg, "launch", "explore.launch.py")

    args = [
        DeclareLaunchArgument("backend", default_value="auto"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("use_depth", default_value="false"),
        DeclareLaunchArgument("depth_type", default_value="image"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/depth/image_raw"),
    ]
    LC = LaunchConfiguration
    common = {"use_sim_time": True}

    # Section 1: sim + SLAM + Nav2 + exploration
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(explore_launch))

    # Section 2: give Section 1 a head start so the map frame + camera exist
    tagger = Node(
        package="semantic_nav", executable="semantic_tagger",
        name="semantic_tagger", output="screen",
        parameters=[{**common, "backend": LC("backend"),
                     "labels_path": labels, "regions_path": regions,
                     "image_topic": LC("image_topic"),
                     "use_depth": LC("use_depth"),
                     "depth_type": LC("depth_type"),
                     "depth_topic": LC("depth_topic")}])
    smap = Node(package="semantic_nav", executable="semantic_map",
                name="semantic_map", output="screen",
                parameters=[{**common, "persist_path": persist}])
    query = Node(package="semantic_nav", executable="semantic_query",
                 name="semantic_query", output="screen",
                 parameters=[{**common}])
    web = Node(package="semantic_nav", executable="web_backend",
               name="web_backend", output="screen",
               parameters=[{**common}])

    rrt = Node(
        package="rrt_planner", executable="rrt_planner",
        name="rrt_planner", output="screen",
        parameters=[{**common,
                     "rrt_star": True,
                     "inflation_radius": 0.20,
                     "goal_bias": 0.10,
                     "step_size": 0.30}])

    delayed = TimerAction(period=8.0, actions=[tagger, smap, query, web, rrt])

    return LaunchDescription(args + [explore, delayed])
