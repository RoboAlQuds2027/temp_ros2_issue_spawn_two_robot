import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    
    # Get package paths
    gazebo_pkg = FindPackageShare('my_robot_gazebo')
    
    # Launch Gazebo world
    gazebo_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'gazebo_world.launch.py'
            ])
        ])
    )
    
    # Launch Robot
    robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gazebo_pkg,
                'launch',
                'spawn_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.0'
        }.items()
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    ld.add_action(gazebo_world_cmd)
    ld.add_action(robot_cmd)
    
    return ld
