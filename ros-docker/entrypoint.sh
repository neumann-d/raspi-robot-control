#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash

cd /root
exec "$@"

