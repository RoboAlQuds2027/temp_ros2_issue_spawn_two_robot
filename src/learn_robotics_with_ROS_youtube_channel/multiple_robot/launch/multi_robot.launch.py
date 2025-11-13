import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
    doc = xacro.process_file(robot1_urdf)
    robot1_description = doc.toxml()
    # robot1_description = Command(['xacro ', robot1_urdf])

    # Robot 1 State Publisher (in robot1 namespace)
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': robot1_description,
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        # remappings=[  # ← ADD THIS ENTIRE BLOCK
        # ('/tf', '/robot1/tf'),
        # ('/tf_static', '/robot1/tf_static')
        # ],
        output='screen'
    )

    # Robot 1 Joint State Publisher
    robot1_joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='robot1',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        output='screen'
    )

    # ========== ROBOT 2 SETUP ==========

    # Robot 2 URDF
    robot2_urdf = os.path.join(urdf_dir, 'robot2.urdf')
    robot2_description = Command(['xacro ', robot2_urdf])

    # Robot 2 State Publisher (in robot2 namespace)
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': robot2_description,
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        # remappings=[  # ← ADD THIS ENTIRE BLOCK
        # ('/tf', '/tf'),
        # ('/tf_static', '/tf_static')
        # ],
        output='screen'
    )

    # Robot 2 Joint State Publisher
    robot2_joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='robot2',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_gazebo'),
        }],
        output='screen'
    )

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

    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot1/robot_description',
                   '-name', 'robot1',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.1'],
        output='screen',
        condition=LaunchConfigurationEquals('use_gazebo', 'true')
    )

    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot2/robot_description',
                   '-name', 'robot2',
                   '-x', '2',
                   '-y', '0',
                   '-z', '0.1'],
        output='screen',
        condition=LaunchConfigurationEquals('use_gazebo', 'true')
    )

    bridge= Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_ign_bridge',
        output='screen',
        arguments=[
            # Example bridges (add your robot’s topics)
            # '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
      
            # "/model/my_robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[('/joint_states', '/robot1/joint_states')]
    )


    bridge_2= Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_ign_bridge_2',
        output='screen',
        arguments=[
            # Example bridges (add your robot’s topics)
            # '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
      
            # "/model/my_robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[('/joint_states', '/robot2/joint_states')]
    )


    # Build launch description
    ld = LaunchDescription([
        use_gazebo,
        use_rviz,

        # Gazebo
        gazebo_launch,

        # Robot 1
        robot1_state_pub,
        robot1_joint_state_pub,

        # Robot 2
        robot2_state_pub,
        robot2_joint_state_pub,

        # RViz
        rviz_node,

        # Spawn robots in Gazebo
        spawn_robot1,
        spawn_robot2,

        bridge
    ])

    return ld