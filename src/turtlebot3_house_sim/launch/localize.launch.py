"""
Phase 2: localize on a saved map using AMCL + Nav2 (no slam_toolbox).

Usage:
  ros2 launch turtlebot3_house_sim localize.launch.py map:=$HOME/house_map.yaml

After launch, set the initial pose in RViz with "2D Pose Estimate" near
x=5.0, y=0.0 (the robot's spawn point).
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                             IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_this         = get_package_share_directory('turtlebot3_house_sim')

    world       = os.path.join(pkg_this, 'worlds', 'house.world')
    nav2_params = os.path.join(pkg_this, 'config', 'nav2_params.yaml')
    default_map = os.path.join(os.path.expanduser('~'), 'house_map.yaml')

    map_arg  = DeclareLaunchArgument('map', default_value=default_map,
                                     description='Full path to map yaml')
    map_yaml = LaunchConfiguration('map')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'verbose': 'false'}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch',
                         'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch',
                         'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '5.0', 'y_pose': '0.0'}.items()
    )

    # Remap /scan -> /scan/out for AMCL and Nav2
    remap_scan = SetRemap(src='/scan', dst='/scan/out')

    # Localization (map_server + amcl) + Navigation
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch',
                         'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam':         'False',
            'map':          map_yaml,
            'params_file':  nav2_params,
        }.items()
    )

    delayed_nav2 = TimerAction(period=30.0, actions=[nav2_bringup])

    return LaunchDescription([
        map_arg,
        gazebo,
        robot_state_publisher,
        spawn_turtlebot,
        remap_scan,          # <-- remap active before amcl/nav2 launch
        delayed_nav2,
    ])
