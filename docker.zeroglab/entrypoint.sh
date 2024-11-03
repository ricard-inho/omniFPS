#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
export FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/zeroGLab_src/fastdds.xml"
cd /workspace
exec "$@"