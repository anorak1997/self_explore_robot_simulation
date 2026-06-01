"""
TurtleBot3 house sim: Gazebo + SLAM (slam_toolbox) + Nav2.

/scan is remapped to /scan/out by placing SetRemap as a direct action
in the LaunchDescription, which guarantees it's active when slam_toolbox
and nav2 nodes subscribe. This avoids ROS 2 version-dependent bugs with
GroupAction remap propagation.
"""

import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_gazebo_ros   = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo   = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_this         = get_package_share_directory('turtlebot3_house_sim')

    world       = os.path.join(pkg_this, 'worlds',  'house.world')
    slam_params = os.path.join(pkg_this, 'config',  'slam.yaml')
    nav2_params = os.path.join(pkg_this, 'config',  'nav2_params.yaml')

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

    # Remap /scan -> /scan/out globally (before slam_toolbox launches).
    # This is a direct action, not inside a GroupAction, so it reliably
    # propagates to all nodes that come after it.
    remap_scan = SetRemap(src='/scan', dst='/scan/out')

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch',
                         'online_async_launch.py')),
        launch_arguments={
            'use_sim_time':    'true',
            'slam_params_file': slam_params,
        }.items()
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch',
                         'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file':  nav2_params,
        }.items()
    )

    delayed_slam = TimerAction(period=25.0, actions=[slam_toolbox])
    delayed_nav2 = TimerAction(period=40.0, actions=[nav2])

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_turtlebot,
        remap_scan,          # <-- remap active before SLAM launches
        delayed_slam,
        delayed_nav2,
    ])
