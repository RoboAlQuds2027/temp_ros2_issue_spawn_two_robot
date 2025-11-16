import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Package name
    package_name = 'demo_control'

    # Get package directory
    pkg_path = get_package_share_directory(package_name)

    # File paths
    xacro_file = os.path.join(pkg_path, 'model', 'model.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'config', 'config.rviz')
    controller_config_file = os.path.join(pkg_path, 'config', 'robot_controller.yaml')

    # Process xacro file
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    # Robot description parameter
    params = {'robot_description': robot_description}

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )
    
    # Spawn entity node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                  '-name', 'simple_robot'],
        output='screen'
    )
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_file],
        output='screen'
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Position controller spawner
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', 
                  '--param-file', controller_config_file],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        # rviz,
        # controller_manager,
        joint_state_broadcaster_spawner,
        position_controller_spawner
    ])