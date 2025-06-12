colcon build --packages-select hardware_unitree_mujoco
colcon build --packages-select reddog_description
colcon build --packages-select rl_quadruped_controller
source install/setup.bash
