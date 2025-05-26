# Reddog ROS2 control
hardware_unitree_mujoco ref: [legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control/tree/main/hardwares/hardware_unitree_mujoco)
## Quick start
```
nano src/reddog_description/config/robot_control.yaml
```
change the model path in the robot_control.yaml
```
policy_path: /your/absolute/path/to/reddog_description/config/legged_gym/policy_him2.pt
config_path: /your/absolute/path/to/reddog_description/config/legged_gym/reddog_him.yaml
```

And build/source/launch
```
source rebuild.sh
ros2 launch rl_quadruped_controller bringup.launch.py pkg_description:=reddog_description
```

```
ros2 topic pub /mode std_msgs/msg/String "data: 'sit'"
ros2 topic pub /mode std_msgs/msg/String "data: 'stand'"
ros2 topic pub /mode std_msgs/msg/String "data: 'move'"
```
