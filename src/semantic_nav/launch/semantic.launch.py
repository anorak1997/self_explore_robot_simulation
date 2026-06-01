"""
Semantic layer: tagger + semantic map + query service.

Run alongside the sim:
  ros2 launch semantic_nav semantic.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg     = get_package_share_directory("semantic_nav")
    regions = os.path.join(pkg, "config", "regions.yaml")
    labels  = os.path.join(pkg, "config", "labels.yaml")
    persist = os.path.join(os.path.expanduser("~"), "semantic_map.json")

    common = {"use_sim_time": True}

    tagger = Node(
        package="semantic_nav", executable="semantic_tagger",
        name="semantic_tagger", output="screen",
        parameters=[{
            **common,
            # "mock" uses regions.yaml (correct positions, camera-free).
            # Change to "clip" if transformers + camera are available.
            "backend":      "mock",
            "regions_path": regions,
            "labels_path":  labels,
            "tag_period":   3.0,        # emit every 3 s (less spam)
            "robot_frame":  "base_footprint",
            "map_frame":    "map",
        }])

    smap = Node(
        package="semantic_nav", executable="semantic_map",
        name="semantic_map", output="screen",
        parameters=[{
            **common,
            "persist_path":   persist,
            # 2.5 m: large enough that one room = one place even if the
            # robot wanders, but small enough that adjacent rooms differ.
            "merge_distance": 2.5,
        }])

    query = Node(
        package="semantic_nav", executable="semantic_query",
        name="semantic_query", output="screen",
        parameters=[{**common, "match_threshold": 0.30}])

    return LaunchDescription([tagger, smap, query])
