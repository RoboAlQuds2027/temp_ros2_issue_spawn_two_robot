# my_robot_gazebo — Multi-robot launch notes

This package contains launch files to spawn multiple instances of `my_robot` into Ignition/Gazebo.

Quick build & run

```sh
# from workspace root
colcon build --packages-select my_robot_description my_robot_gazebo
source install/setup.sh
ros2 launch my_robot_gazebo main_multi_robots.launch.py
```

Notes about namespaces

- Each robot instance is spawned with a `robot_name` (for example `robot1`, `robot2`).
- The URDF/gazebo plugin was updated to remap the ros2_control plugin topics so the
  plugin picks up the namespaced `robot_description` and `controller_manager` for
  each robot. This is necessary when running multiple robots in the same simulation.

Troubleshooting: `gz_ros2_control: robot_state_publisher service not available`

1. Confirm nodes and services are present (run in another terminal):

```sh
ros2 node list
ros2 service list | grep robot1
ros2 service list | grep controller_manager
ros2 topic list | grep robot1
```

2. If controllers are not starting, verify the controller spawners target the correct controller manager namespace:

```py
# in spawn_robot.launch.py the spawner uses:
# '--controller-manager', ['/', robot_name, '/controller_manager']
```

3. Rebuild and re-source the workspace after changes to URDF/xacro files:

```sh
colcon build --packages-select my_robot_description
source install/setup.sh
```

If things still fail, paste the `ros2 node list` and the launch console output here and I'll help interpret it.

---
Created to document namespace & ros2_control interaction for multi-robot spawns.
