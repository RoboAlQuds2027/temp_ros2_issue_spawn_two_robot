import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get ros_gz_sim package path
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    # Launch Gazebo Fortress with empty world
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
            # 'on_exit_shutdown': 'true'
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo_cmd)
    
    return ld
