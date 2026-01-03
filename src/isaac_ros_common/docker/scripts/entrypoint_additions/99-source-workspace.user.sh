#!/bin/bash
# Auto-source Isaac ROS workspace on container startup
# This ensures built packages are available immediately

if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
    echo "✓ Auto-sourced Isaac ROS workspace"
fi

