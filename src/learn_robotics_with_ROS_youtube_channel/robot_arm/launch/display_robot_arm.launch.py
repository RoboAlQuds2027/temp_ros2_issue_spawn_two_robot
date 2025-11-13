import os
import xacro
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    # Package name and base path
    package_name = 'robot_arm'
    pkg_path = get_package_share_directory(package_name)

    # Process the xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Process cube URDF
    cube_xacro = os.path.join(pkg_path, 'urdf', 'cube_pick_place.urdf.xacro')
    cube_config = xacro.process_file(cube_xacro)
    cube_description = cube_config.toxml()

    # RViz config
    rviz_config_file = os.path.join(pkg_path, 'config', 'rviz2_config.rviz')

    controller_config_file = os.path.join(pkg_path, 'config', 'controller.yaml')

    params = {'robot_description': robot_description}

    # Declare launch argument for GUI
    declare_use_gui_cmd = DeclareLaunchArgument(
        name='use_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )
    use_gui = LaunchConfiguration('use_gui')

    # Launch Gazebo (replaces empty_world.launch)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf',  # -r for run immediately
            'on_exit_shutdown': 'true'
        }.items()) # When set to 'true', this tells ROS2 to automatically 
                                                                                            # shutdown all other nodes in your launch file when Gazebo exits. 
   

    # Spawn robot (NEW - 'create' instead of 'spawn_entity.py')
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Spawn cube in Gazebo
    # spawn_cube = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-topic', 'cube_description',
    #         '-name', 'grasp_cube',
    #         '-x', '1.0',
    #         '-y', '1.0',
    #         '-z', '0.025'  # Half of 0.05m so bottom touches ground
    #     ],
    #     output='screen'
    # )    

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot_arm',   # Namespace added here
        output='screen',
        parameters=[params]
    )

    # Publish cube description
    cube_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='cube',    # Namespace added here
        parameters=[{'robot_description': cube_description}],
        output='screen'
    )    

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # controller manager
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[params, controller_config_file],
    #     output='screen'
    # )

    # controllers spawner
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=["joint_state_broadcaster"],
            output='screen'
        )]
    )

    joint_trajectory_controller = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=["joint_trajectory_controller"],
            output='screen'
        )]
    )

    # rqt_gui
    rqt_gui = Node(
    package='rqt_gui',
    executable='rqt_gui',
    output='screen'
    )


    return LaunchDescription([
        declare_use_gui_cmd,
        gazebo,
        # spawn_entity,
        # spawn_cube,
        robot_state_publisher,
        cube_publisher,
        rviz2,
        joint_state_broadcaster,
        joint_trajectory_controller,
        rqt_gui,

        
    ])
