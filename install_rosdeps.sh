#!/bin/bash
# Auto-install rosdep dependencies for Isaac ROS workspace
# Source this script or add to .bashrc to auto-install on container startup

set -e

echo "=== Checking rosdep dependencies ==="
MISSING_DEPS=$(rosdep check --from-paths src --ignore-src 2>&1 | grep -E "System dependencies have not been satisfied" || true)

if [ -z "$MISSING_DEPS" ]; then
    echo "✓ All rosdep dependencies are satisfied"
else
    echo "Installing missing rosdep dependencies..."
    rosdep install --from-paths src --ignore-src -y \
        --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python3-opencv ros-humble-magic-enum" || true
    echo "✓ Rosdep dependencies installation complete"
fi

