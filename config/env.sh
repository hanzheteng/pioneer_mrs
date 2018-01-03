#!/bin/sh

export ROS_MASTER_URI=http://hzteng:11311
source /opt/ros/kinetic/setup.bash
source /home/hao/thz_ws/devel/setup.bash
exec "$@"
