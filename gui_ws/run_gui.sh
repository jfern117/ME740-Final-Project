#!/usr/bin/env bash

colcon build --packages-select swarm_gui tb_sim_custom
source install/setup.bash && ros2 launch swarm_gui gazebo_sim_launch.py