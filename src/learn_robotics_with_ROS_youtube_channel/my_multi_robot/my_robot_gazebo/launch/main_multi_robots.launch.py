import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch multiple robots in Gazebo simulation."""
    
    # Package paths
    gazebo_pkg = FindPackageShare('my_robot_gazebo')
    
    # Gazebo world launch
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'gazebo_world.launch.py'
            ])
        )
    )
    
    # Robot 1 spawn
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'spawn_robot.launch.py'
            ])
        ),
        launch_arguments={
            'robot_name': 'robot1',
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.5'
        }.items()
    )
    
    # Robot 2 spawn
    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'spawn_robot.launch.py'
            ])
        ),
        launch_arguments={
            'robot_name': 'robot2',
            'x_pose': '2.0',
            'y_pose': '0.0',
            'z_pose': '0.0'
        }.items()
    )
    
    # Build launch description
    return LaunchDescription([
        gazebo_world,
        # how to write without TimerAction
        robot1,
        # TimerAction(period=5.0, actions=[robot1]),
        # TimerAction(period=12.0, actions=[robot2]),
    ])
