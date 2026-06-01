"""
RRT planner demo on a SAVED static map.

  ros2 launch rrt_planner rrt_demo.launch.py map:=/path/to/your_map.yaml

Then in RViz use the "2D Goal Pose" tool to set a goal; the planned path
appears on /rrt_path and the search tree on /rrt_tree. Use "2D Pose
Estimate" (/initialpose) to set the start if there is no live robot TF.

To plan on the LIVE SLAM map instead, just run the node while your
Section 1 stack is up:
  ros2 run rrt_planner rrt_planner
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("rrt_planner")
    rviz_cfg = os.path.join(pkg, "rviz", "rrt.rviz")

    map_arg = DeclareLaunchArgument(
        "map", description="Path to a map YAML (from Section 1 or Nav2 samples)")
    map_yaml = LaunchConfiguration("map")

    map_server = Node(
        package="nav2_map_server", executable="map_server",
        name="map_server", output="screen",
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}])

    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True,
                     "node_names": ["map_server"]}])

    planner = Node(
        package="rrt_planner", executable="rrt_planner",
        name="rrt_planner", output="screen",
        parameters=[{"use_sim_time": True,
                     "rrt_star": True,
                     "inflation_radius": 0.20,
                     "goal_bias": 0.10,
                     "step_size": 0.30}])

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg], output="screen",
        parameters=[{"use_sim_time": True}])

    return LaunchDescription([map_arg, map_server, lifecycle, planner, rviz])
