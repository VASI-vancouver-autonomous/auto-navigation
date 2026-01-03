# RealSense D435i Camera Test Documentation

This document describes the setup, testing, and verification of the Intel RealSense D435i depth camera.

## Date
December 7, 2024

---

## Hardware Specifications

**Model**: Intel RealSense D435i  
**Serial Number**: 347622076476  
**Firmware Version**: 5.15.1.55  
**USB Type**: 3.2  
**Physical Port**: 2-1.4-3

### Camera Capabilities
- **Stereo Infrared Cameras**: Infra1 and Infra2 (for depth/stereo vision)
- **RGB Camera**: Optional color imaging
- **Depth Sensor**: Time-of-flight depth sensing
- **IMU**: 6-axis (gyroscope + accelerometer)
- **Resolution**: Up to 848x480 @ 30fps (infrared), 1280x720 @ 30fps (RGB)
- **Field of View**: 87° × 58° (depth), 69° × 42° (RGB)

---

## Prerequisites

### 1. Hardware Requirements
- RealSense D435i camera
- USB 3.0+ port (required for full functionality)
- USB 3.0 cable

### 2. Software Requirements
- ROS 2 Humble
- `realsense2_camera` package (installed in Docker container)
- Docker container with USB device access

### 3. Docker Configuration

The Docker container must have USB device access. This is configured in `~/.isaac_ros_dev-dockerargs`:

```
-v $HOME/.ngc:/home/admin/.ngc:ro
-v /dev/bus/usb:/dev/bus/usb
-v /sys/devices/platform:/sys/devices/platform
-v /dev/snd:/dev/snd
--group-add audio
```

---

## Installation Steps

### Step 1: Verify USB Device Access

Check if USB devices are accessible in the container:

```bash
ls -la /dev/bus/usb/*/*
```

### Step 2: Set USB Permissions

Set permissions on USB devices (run inside container):

```bash
for dev in /dev/bus/usb/*/*; do
    sudo chmod 666 "$dev" 2>/dev/null
done
```

**Note**: For persistent permissions, add udev rules on the host system:

Create `/etc/udev/rules.d/99-realsense.rules` on the host:
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"
```

Then reload udev:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Step 3: Verify RealSense Packages

Check if RealSense packages are installed:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep realsense
```

Expected output:
```
realsense2_camera
realsense2_camera_msgs
realsense2_description
```

---

## Testing Procedure

### Method 1: Using Launch Script

Use the provided launch script:

```bash
cd /workspaces/isaac_ros-dev
./launch_realsense.sh
```

### Method 2: Manual Launch

Launch RealSense camera node manually:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_color:=false \
    enable_depth:=false \
    enable_imu:=true \
    depth_module.emitter_enabled:=0
```

### Launch Parameters Explained

- `enable_infra1:=true` - Enable left infrared camera (required for stereo)
- `enable_infra2:=true` - Enable right infrared camera (required for stereo)
- `enable_color:=false` - Disable RGB camera (optional)
- `enable_depth:=false` - Disable depth stream (optional, can enable if needed)
- `enable_imu:=true` - Enable IMU (gyroscope + accelerometer)
- `depth_module.emitter_enabled:=0` - Disable IR projector (for stereo vision)

---

## Verification Steps

### Step 1: Check Node Status

When the camera launches successfully, you should see:

```
[INFO] [camera.camera]: Device with name Intel RealSense D435I was found.
[INFO] [camera.camera]: Device Serial No: 347622076476
[INFO] [camera.camera]: RealSense Node Is Up!
```

### Step 2: List Available Topics

In another terminal, check published topics:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 topic list | grep camera
```

**Expected Topics**:
- `/camera/infra1/image_rect_raw` - Left infrared camera images
- `/camera/infra2/image_rect_raw` - Right infrared camera images
- `/camera/infra1/camera_info` - Left camera calibration info
- `/camera/infra2/camera_info` - Right camera calibration info
- `/camera/imu` - IMU data (gyroscope + accelerometer)
- `/camera/extrinsics/depth_to_infra1` - Transform data
- `/camera/extrinsics/depth_to_infra2` - Transform data

### Step 3: Verify Image Streaming

Check image topic data:

```bash
ros2 topic echo /camera/infra1/image_rect_raw --no-arr
```

**Expected Output**:
```
header:
  stamp:
    sec: <timestamp>
    nanosec: <timestamp>
  frame_id: camera_infra1_optical_frame
height: 480
width: 848
encoding: mono8
is_bigendian: 0
step: 848
data: '<sequence type: uint8, length: 407040>'
```

### Step 4: Check Frame Rate

Verify camera is streaming at expected rate:

```bash
ros2 topic hz /camera/infra1/image_rect_raw
```

**Expected Output**:
```
average rate: 29.985-30.000
        min: 0.032s max: 0.035s std dev: 0.0005s
```

### Step 5: Verify IMU Data

Check IMU topic:

```bash
ros2 topic echo /camera/imu
```

---

## Test Results

### Successful Test Output

**Device Detection**:
- ✅ Device detected: Intel RealSense D435I
- ✅ Serial number: 347622076476
- ✅ Firmware: 5.15.1.55
- ✅ USB type: 3.2

**Stream Configuration**:
- ✅ Infra1 stream: 848x480 @ 30fps, Y8 format
- ✅ Infra2 stream: 848x480 @ 30fps, Y8 format
- ✅ IMU: Active (gyro @ 200Hz, accel @ 100Hz)

**Performance**:
- ✅ Frame rate: ~30 FPS (stable)
- ✅ Latency: 0.032-0.035 seconds per frame
- ✅ Jitter: Low (std dev ~0.0005s)
- ✅ Image data: 407040 bytes per frame (848 × 480)

---

## Common Issues and Solutions

### Issue 1: USB Access Denied

**Error**:
```
RS2_USB_STATUS_ACCESS
failed to open usb interface: 0, error: RS2_USB_STATUS_ACCESS
```

**Solution**:
1. Set USB device permissions in container:
   ```bash
   sudo chmod 666 /dev/bus/usb/*/*
   ```

2. Or fix permissions on host:
   ```bash
   sudo chmod 666 /dev/bus/usb/*/*/*
   ```

3. Add udev rules (see Step 2 in Installation)

### Issue 2: Device Not Found

**Error**:
```
The requested device with  is NOT found. Will Try again.
```

**Solutions**:
- Ensure camera is connected to USB 3.0 port
- Check USB cable is properly connected
- Verify USB devices are mounted in Docker: `ls -la /dev/bus/usb/`
- Try unplugging and reconnecting the camera

### Issue 3: Low Frame Rate

**Symptoms**: Frame rate below 30 FPS

**Solutions**:
- Ensure USB 3.0 connection (not USB 2.0)
- Check USB bandwidth: `lsusb -t` (on host)
- Reduce resolution if needed
- Close other applications using USB bandwidth

### Issue 4: Permission Errors

**Error**: Cannot access `/dev/bus/usb/`

**Solution**:
- Verify Docker args include: `-v /dev/bus/usb:/dev/bus/usb`
- Check container has proper permissions
- Restart container after adding USB mounts

---

## Integration with Isaac ROS

### Visual SLAM

The RealSense D435i is compatible with `isaac_ros_visual_slam`:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Requirements**:
- Infra1 and Infra2 streams (stereo)
- IMU data
- Camera info topics

### 3D Mapping (NVBlox)

Compatible with `isaac_ros_nvblox` for 3D mapping:

```bash
ros2 launch isaac_ros_nvblox isaac_ros_nvblox.launch.py
```

**Requirements**:
- Depth stream (enable `enable_depth:=true`)
- Camera info
- Pose estimates (from Visual SLAM)

---

## Launch Script

The `launch_realsense.sh` script automates the setup:

```bash
#!/bin/bash
# Launch RealSense D435i with proper USB permissions

# Set USB permissions
for dev in /dev/bus/usb/*/*; do
    sudo chmod 666 "$dev" 2>/dev/null
done

# Source ROS 2
source /opt/ros/humble/setup.bash
if [ -f "/workspaces/isaac_ros-dev/install/setup.bash" ]; then
    source /workspaces/isaac_ros-dev/install/setup.bash
fi

# Launch camera
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_color:=false \
    enable_depth:=false \
    enable_imu:=true \
    depth_module.emitter_enabled:=0
```

---

## Next Steps

After successful camera testing:

1. **Calibrate cameras** (if needed for high-precision applications)
2. **Test Visual SLAM integration** with `isaac_ros_visual_slam`
3. **Test 3D mapping** with `isaac_ros_nvblox`
4. **Integrate with navigation stack** for autonomous navigation
5. **Combine with motor controller** for complete robot system

---

## References

- [Intel RealSense D435i Documentation](https://www.intelrealsense.com/depth-camera-d435i/)
- [RealSense ROS 2 Package](https://github.com/IntelRealSense/realsense-ros)
- [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
- [Isaac ROS NVBlox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)

