#!/usr/bin/env bash

colcon build --packages-select swarm_gui
source install/setup.bash && ros2 launch swarm_gui gazebo_sim_launch.py