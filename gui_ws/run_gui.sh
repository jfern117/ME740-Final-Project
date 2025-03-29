#!/usr/bin/env bash

colcon build
source install/setup.bash && ros2 launch swarm_gui sim_launch.py