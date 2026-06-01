"""
Launch the existing TurtleBot3 house sim (Gazebo + SLAM + Nav2)
and then start the autonomous frontier explorer after Nav2 is up.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_house_sim = get_package_share_directory('turtlebot3_house_sim')

    # Bring up the full simulation stack (Gazebo + SLAM + Nav2)
    house_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_house_sim, 'launch', 'house_sim.launch.py')
            # ^ rename if your launch file has a different name
        )
    )

    # Frontier explorer — start AFTER Nav2 is fully up
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
            'replan_period': 20.0,
            'safety_radius': 0.25,
            'goal_reached_tolerance': 0.35,
            'min_goal_distance': 1.0,
            'initial_spin': True,
            'initial_spin_duration': 12.0,
        }]
    )

    # Nav2 starts ~40s into the existing house launch; give it another 20s buffer
    delayed_explorer = TimerAction(period=60.0, actions=[explorer])

    return LaunchDescription([
        house_sim,
        delayed_explorer,
    ])
