"""
Use this if the simulation (Gazebo + SLAM + Nav2) is ALREADY running
in another terminal. This launches only the explorer node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    explorer = Node(
        package='turtlebot3_explorer',
        executable='explorer_node',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'min_frontier_size': 8,
            'robot_frame': 'base_footprint',
            'map_frame': 'map',
            'exploration_timeout': 60.0,
            'replan_period': 15.0,
            'safety_radius': 0.25,
            'goal_reached_tolerance': 0.35,
            'min_goal_distance': 1.0,
            'initial_spin': True,
            'initial_spin_duration': 12.0,
        }]
    )
    return LaunchDescription([explorer])
