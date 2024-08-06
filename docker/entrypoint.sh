#!/bin/bash
set -e

# Creates User for Shared Memory Transport
id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} ros

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
# echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc


exec "$@"
