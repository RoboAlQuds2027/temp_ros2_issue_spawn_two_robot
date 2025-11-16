import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    
    # Declare launch arguments
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')


    # Package name
    pkg_name = 'my_robot_description'

    # Get package directory
    pkg_dir = get_package_share_directory(pkg_name)

    # URDF file path
    robot_urdf = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf.xacro')

    # controller config file path
    controller_config_file = os.path.join(pkg_dir, 'config', 'robot_controller.yaml')


    # Build URDF command - pass robot_name to XACRO
    robot_description = Command([
        'xacro ',
        robot_urdf,
        ' robot_name:=',
        robot_name
    ])

    # Gazebo spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-name', robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        namespace=robot_name,
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str)
        }],
        output='screen'
    )
    
    # Joint state publisher GUI (for testing)
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     namespace=robot_name,
    #     output='screen'
    # )


    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/robot1/controller_manager'
        ],
        output='screen'
    )
    
    # Position controller spawner
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '--param-file', controller_config_file,
            '-c', '/robot1/controller_manager'
        ],
        output='screen'
    )

    # RViz node (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'my_robot_display.rviz')],
        output='screen'
    )    
    
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('robot_name', default_value='robot1', description='Name of the robot to spawn'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.0'))
    
    # Add nodes
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(position_controller_spawner)
    # ld.add_action(joint_state_publisher_node)
    
    
    return ld
