#!/bin/bash

# Complete setup script for running YOLOv8 with DDS buffer tuning
# This ensures buffer issues are prevented before starting the pipeline

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DDS_TUNING_SCRIPT="${SCRIPT_DIR}/tune_dds_buffers.sh"

echo "=========================================="
echo "YOLOv8 Setup with DDS Tuning"
echo "=========================================="
echo ""

# Check if DDS tuning has been applied
check_dds_tuning() {
    local ipfrag_time=$(sysctl -n net.ipv4.ipfrag_time 2>/dev/null || echo "30")
    local rmem_max=$(sysctl -n net.core.rmem_max 2>/dev/null || echo "0")
    
    if [ "$ipfrag_time" -eq 3 ] && [ "$rmem_max" -gt 1000000000 ]; then
        return 0  # Tuning is applied
    else
        return 1  # Tuning not applied
    fi
}

# Step 1: Apply DDS tuning (if not already done)
echo "Step 1: Checking DDS buffer tuning..."
if check_dds_tuning; then
    echo "  ✓ DDS tuning already applied"
else
    echo "  ⚠ DDS tuning not applied"
    echo ""
    echo "  Applying DDS buffer tuning (requires sudo)..."
    if [ -f "$DDS_TUNING_SCRIPT" ]; then
        sudo "$DDS_TUNING_SCRIPT"
    else
        echo "  Error: DDS tuning script not found at $DDS_TUNING_SCRIPT"
        exit 1
    fi
fi

echo ""
echo "Step 2: Source ROS 2 environment"
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo "  ✓ ROS 2 environment sourced"
else
    echo "  Error: Workspace not found. Expected: ${WORKSPACE_DIR}"
    exit 1
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "1. Start Foxglove Bridge (in a separate terminal):"
echo "   ${SCRIPT_DIR}/launch_foxglove_fast.sh"
echo ""
echo "2. Start your YOLOv8 pipeline:"
echo "   ros2 launch isaac_ros_yolov8 <your_launch_file>"
echo ""
echo "3. In Foxglove Studio:"
echo "   - Connect to: ws://$(hostname -I | awk '{print $1}'):8765"
echo "   - Subscribe to: /yolov8_processed_image/compressed"
echo ""
echo "=========================================="
echo "Current DDS Settings:"
echo "=========================================="
echo "net.ipv4.ipfrag_time: $(sysctl -n net.ipv4.ipfrag_time)s"
echo "net.ipv4.ipfrag_high_thresh: $(($(sysctl -n net.ipv4.ipfrag_high_thresh) / 1024 / 1024))MB"
echo "net.core.rmem_max: $(($(sysctl -n net.core.rmem_max) / 1024 / 1024))MB"
echo ""

