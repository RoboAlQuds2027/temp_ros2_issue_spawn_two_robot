import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import LaunchConfigurationEquals
import xacro



def generate_launch_description():

    # Get package directory
    pkg_dir = get_package_share_directory('multiple_robot')
    urdf_dir = os.path.join(pkg_dir, 'urdf')

    # Declare arguments
    use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Launch Gazebo simulator'
    )

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )

    # ========== ROBOT 1 SETUP ==========

    # Robot 1 URDF
    robot1_urdf = os.path.join(urdf_dir, 'robot1.urdf')
    with open(robot1_urdf, 'r') as infp:
        robot1_description = infp.read()
    # doc1 = xacro.process_file(robot1_urdf)
    # robot1_description = doc1.toxml()

    # Robot 1 State Publisher (NO namespace argument here - handled by PushRosNamespace)
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot1_description,
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        output='screen'
    )

    # Robot 1 Joint State Publisher
    robot1_joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        output='screen'
    )

    # Group Robot 1 nodes with PushRosNamespace
    robot1_group = GroupAction([
        PushRosNamespace('robot1'),
        robot1_state_pub,
        robot1_joint_state_pub,
    ])

    # ========== ROBOT 2 SETUP ==========

    # Robot 2 URDF
    robot2_urdf = os.path.join(urdf_dir, 'robot2.urdf')
    robot2_description = Command(['xacro ', robot2_urdf])

    # Robot 2 State Publisher (NO namespace argument - handled by PushRosNamespace)
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot2_description,
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        output='screen'
    )

    # Robot 2 Joint State Publisher GUI
    robot2_joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        output='screen'
    )

    # Group Robot 2 nodes with PushRosNamespace
    robot2_group = GroupAction([
        PushRosNamespace('robot2'),
        robot2_state_pub,
        robot2_joint_state_pub,
    ])

    # ========== VISUALIZATION ==========

    # RViz2 node (not namespaced - needs to see both robots)
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'multi_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=LaunchConfigurationEquals('use_rviz', 'true')
    )

    # ========== GAZEBO ==========

    # Launch Gazebo (if requested)
    gazebo_pkg_dir = get_package_share_directory('ros_gz_sim')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gz_sim.launch.py')
        ),
        condition=LaunchConfigurationEquals('use_gazebo', 'true'),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn Robot 1 in Gazebo
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot1/robot_description',
            '-name', 'robot1',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen',
        condition=LaunchConfigurationEquals('use_gazebo', 'true')
    )

    # Spawn Robot 2 in Gazebo
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot2/robot_description',
            '-name', 'robot2',
            '-x', '2',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen',
        condition=LaunchConfigurationEquals('use_gazebo', 'true')
    )

    # ========== ROS-GAZEBO BRIDGE ==========

    # Bridge for Robot 1
    bridge_robot1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_robot1',
        output='screen',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[('/joint_states', '/robot1/joint_states')],
        condition=LaunchConfigurationEquals('use_gazebo', 'true')
    )

    # Bridge for Robot 2
    bridge_robot2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_robot2',
        output='screen',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[('/joint_states', '/robot2/joint_states')],
        condition=LaunchConfigurationEquals('use_gazebo', 'true')
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========

    ld = LaunchDescription([
        # Arguments
        use_gazebo,
        use_rviz,

        # Gazebo
        gazebo_launch,

        # Robot Groups (using PushRosNamespace)
        robot1_group,
        robot2_group,

        # RViz
        rviz_node,

        # Spawn robots in Gazebo
        spawn_robot1,
        spawn_robot2,

        # Bridges
        bridge_robot1,
        bridge_robot2,
    ])

    return ld
