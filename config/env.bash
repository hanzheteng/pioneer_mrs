#!/usr/bin/env bash
. /opt/ros/kinetic/setup.bash
. ~/thz_ws/devel/setup.bash
export ROS_MASTER_URI=http://hzteng:11311
exec "$@" #replace the current parent process with the new child process
