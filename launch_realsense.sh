#!/bin/bash
# Launch RealSense D435i with proper USB permissions

echo "Setting up USB permissions for RealSense..."

# Set permissions on all USB devices
for dev in /dev/bus/usb/*/*; do
    sudo chmod 666 "$dev" 2>/dev/null
done

# Source ROS 2
source /opt/ros/humble/setup.bash
if [ -f "/workspaces/isaac_ros-dev/install/setup.bash" ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
fi

echo "Launching RealSense camera..."
echo "If you still get USB access errors, you may need to:"
echo "  1. Add udev rules on the host system"
echo "  2. Or run: sudo chmod 666 /dev/bus/usb/*/*/* on the host"
echo ""

# Launch with color enabled for YOLOv8
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_color:=true \
    enable_depth:=false \
    enable_imu:=true \
    depth_module.emitter_enabled:=0 \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30

