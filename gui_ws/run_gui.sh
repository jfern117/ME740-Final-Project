#!/usr/bin/env bash

colcon build
source install/setup.bash && ros2 run swarm_gui listener