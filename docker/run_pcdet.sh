#!/bin/bash

#  Change to the ros user
su - ros

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 launch pcdet_ros2 pcdet.launch.py