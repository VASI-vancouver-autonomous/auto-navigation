#!/bin/bash

# Launch Foxglove Bridge with optimized settings for fast camera streaming
# This script configures the bridge for low-latency, high-throughput streaming

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

echo "=========================================="
echo "Foxglove Fast Streaming Setup"
echo "=========================================="
echo ""

# Check if we're in the workspace
if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    echo "Error: Workspace not found. Expected: ${WORKSPACE_DIR}"
    echo "Please run this script from the isaac_ros-dev workspace"
    exit 1
fi

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

# Default values
PORT=${1:-8765}
SEND_BUFFER_LIMIT=${2:-200000000}  # 200MB (increased for point clouds)
MAX_QOS_DEPTH=${3:-1}
USE_COMPRESSION=${4:-true}

echo "Configuration:"
echo "  Port: ${PORT}"
echo "  Send Buffer Limit: ${SEND_BUFFER_LIMIT} bytes ($((${SEND_BUFFER_LIMIT} / 1024 / 1024))MB)"
echo "  Max QoS Depth: ${MAX_QOS_DEPTH}"
echo "  Use Compression: ${USE_COMPRESSION}"
echo ""

# Check if foxglove_bridge is installed
if ! ros2 pkg list | grep -q "^foxglove_bridge$"; then
    echo "Warning: foxglove_bridge not found. Installing..."
    sudo apt-get update
    sudo apt-get install -y ros-humble-foxglove-bridge
fi

echo "Launching Foxglove Bridge with optimized settings..."
echo ""
echo "Connect from Foxglove Studio using:"
echo "  Connection Type: Foxglove WebSocket"
echo "  URL: ws://$(hostname -I | awk '{print $1}'):${PORT}"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch with optimized parameters
# Using basic launch with only buffer limit to avoid topic discovery issues
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=${PORT} \
    send_buffer_limit:=${SEND_BUFFER_LIMIT}

