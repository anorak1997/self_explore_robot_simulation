"""
Semantic layer + web dashboard, with proper launch arguments.

  ros2 launch semantic_nav semantic_web.launch.py            # auto backend
  ros2 launch semantic_nav semantic_web.launch.py backend:=clip use_depth:=true
  ros2 launch semantic_nav semantic_web.launch.py backend:=mock

Run ALONGSIDE your Section 1 stack (which provides the map frame + camera),
or use auto_explore_semantic.launch.py to bring up everything at once.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("semantic_nav")
    labels = os.path.join(pkg, "config", "labels.yaml")
    regions = os.path.join(pkg, "config", "regions.yaml")
    persist = os.path.join(os.path.expanduser("~"), "semantic_map.json")

    args = [
        DeclareLaunchArgument("backend", default_value="auto",
                              description="auto | clip | mock"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("use_depth", default_value="false"),
        DeclareLaunchArgument("depth_type", default_value="image",
                              description="image | cloud | none"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("min_confidence", default_value="0.30"),
        DeclareLaunchArgument("tag_period", default_value="2.0"),
        DeclareLaunchArgument("match_threshold", default_value="0.35"),
    ]
    LC = LaunchConfiguration
    common = {"use_sim_time": True}

    tagger = Node(
        package="semantic_nav", executable="semantic_tagger",
        name="semantic_tagger", output="screen",
        parameters=[{**common,
                     "backend": LC("backend"),
                     "labels_path": labels,
                     "regions_path": regions,
                     "image_topic": LC("image_topic"),
                     "use_depth": LC("use_depth"),
                     "depth_type": LC("depth_type"),
                     "depth_topic": LC("depth_topic"),
                     "min_confidence": LC("min_confidence"),
                     "tag_period": LC("tag_period"),
                     "robot_frame": "base_footprint"}])

    smap = Node(
        package="semantic_nav", executable="semantic_map",
        name="semantic_map", output="screen",
        parameters=[{**common, "persist_path": persist,
                     "merge_distance": 1.0}])

    query = Node(
        package="semantic_nav", executable="semantic_query",
        name="semantic_query", output="screen",
        parameters=[{**common, "match_threshold": LC("match_threshold")}])

    web = Node(
        package="semantic_nav", executable="web_backend",
        name="web_backend", output="screen",
        parameters=[{**common, "match_threshold": LC("match_threshold")}])

    rrt = Node(
        package="rrt_planner", executable="rrt_planner",
        name="rrt_planner", output="screen",
        parameters=[{**common,
                     "rrt_star": True,
                     "inflation_radius": 0.20,
                     "goal_bias": 0.10,
                     "step_size": 0.30}])

    return LaunchDescription(args + [tagger, smap, query, web, rrt])
