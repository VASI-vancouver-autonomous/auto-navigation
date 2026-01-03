# Building Isaac ROS Packages on Jetson

## ✅ Build Status: COMPLETE

**All 79 packages have been successfully built!**

- **Total packages in workspace:** 79
- **Packages built:** 79
- **Build completion:** 100%

All build artifacts are persisted on the host at `$ISAAC_ROS_WS/` (or `~/workspaces/isaac_ros-dev/`) and will remain even after exiting the container.

---

## Problem: Jetson Crashes During Full Build

Building all packages at once with `colcon build` can overwhelm the Jetson's resources, causing system crashes and reboots, resulting in lost build progress.

## Solution: Build Packages One at a Time

### Basic Command

Build a single package:
```bash
colcon build --packages-select <package_name> --symlink-install --parallel-workers 1
```

**Flags explained:**
- `--packages-select <package_name>`: Build only the specified package
- `--symlink-install`: Use symlinks instead of copying files (faster for development)
- `--parallel-workers 1`: Limit to one package at a time (safer for Jetson)

### Build with Dependencies

If a package has dependencies, use `--packages-up-to` to automatically build them first:
```bash
colcon build --packages-up-to <package_name> --symlink-install --parallel-workers 1
```

This will:
1. Build all dependencies first
2. Then build the target package

---

## Container Persistence and Data Safety

### ✅ Your Builds Are Safe!

**All build artifacts persist on the host filesystem:**
- The workspace is mounted as a bind mount: `$ISAAC_ROS_WS` (host) → `/workspaces/isaac_ros-dev` (container)
- All built packages in `install/` directory persist
- All build artifacts in `build/` directory persist
- You can safely exit the container - everything remains!

**Container Configuration:**
- The `--rm` flag has been removed from `run_dev.sh` - container persists after exit
- Container automatically reuses existing stopped containers
- System dependencies (rosdep packages) are auto-installed via entrypoint script

**To use your built packages:**
```bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

---

## Build Order Reference (All Packages Built)

### Batch 1: GXF Core Packages
```bash
colcon build --packages-select gxf_isaac_message_compositor --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_optimizer --allow-overriding gxf_isaac_optimizer --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_sight --allow-overriding gxf_isaac_sight --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_atlas --allow-overriding gxf_isaac_atlas --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_cuda --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_utils --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_argus --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_ros_cuda --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_ros_messages --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_messages_throttler --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_timestamp_correlator --symlink-install --parallel-workers 1
```

### Batch 2: GXF Specialized Packages
```bash
colcon build --packages-select gxf_isaac_point_cloud --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_flatscan_localization --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_hesai --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_localization --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_range_scan_processing --symlink-install --parallel-workers 1
colcon build --packages-select gxf_isaac_segway --symlink-install --parallel-workers 1
```

### Batch 3: NITROS Type Packages

**Note:** Three packages were fixed to find `magic_enum`:
- `isaac_ros_nitros_occupancy_grid_type` - FIXED: Added `find_package(magic_enum REQUIRED)`
- `isaac_ros_nitros_pose_array_type` - FIXED: Added `find_package(magic_enum REQUIRED)`
- `isaac_ros_nitros_detection2_d_array_type` - FIXED: Added `find_package(magic_enum REQUIRED)`

```bash
colcon build --packages-select isaac_ros_nitros_image_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_point_cloud_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_compressed_image_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_camera_info_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_tensor_list_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_std_msg_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_imu_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_odometry_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_twist_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_battery_state_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_correlated_timestamp_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_disparity_image_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_encoder_ticks_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_flat_scan_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_occupancy_grid_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_pose_array_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_pose_cov_stamped_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_detection2_d_array_type --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_detection3_d_array_type --symlink-install --parallel-workers 1
```

### Batch 4: NITROS Bridge and Tools

**Note:** Two packages were fixed to find `magic_enum`:
- `isaac_ros_nitros_bridge_ros2` - FIXED: Added `find_package(magic_enum REQUIRED)`
- `isaac_ros_nitros_topic_tools` - FIXED: Added `find_package(magic_enum REQUIRED)`

```bash
colcon build --packages-select isaac_ros_nitros_bridge_ros2 --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nitros_topic_tools --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_pynitros --symlink-install --parallel-workers 1
```

### Batch 5: Custom NITROS Packages

**Note:** Four packages were fixed to find `magic_enum`:
- `custom_nitros_image` - FIXED: Added `find_package(magic_enum REQUIRED)`
- `custom_nitros_string` - FIXED: Added `find_package(magic_enum REQUIRED)`
- `custom_nitros_dnn_image_encoder` - FIXED: Added `find_package(magic_enum REQUIRED)`
- `custom_nitros_message_filter` - FIXED: Added `find_package(magic_enum REQUIRED)`

**Build Order:** `custom_nitros_message_filter_interfaces` must be built before `custom_nitros_message_filter`.

```bash
colcon build --packages-select custom_nitros_image --symlink-install --parallel-workers 1
colcon build --packages-select custom_nitros_string --symlink-install --parallel-workers 1
colcon build --packages-select custom_nitros_dnn_image_encoder --symlink-install --parallel-workers 1
colcon build --packages-select custom_nitros_message_filter_interfaces --symlink-install --parallel-workers 1
colcon build --packages-select custom_nitros_message_filter --symlink-install --parallel-workers 1
```

### Batch 6: Utility Packages

**Note:** `isaac_ros_rosbag_utils` depends on `isaac_ros_r2b_galileo` from Batch 7. Build `isaac_ros_r2b_galileo` first.

```bash
colcon build --packages-select isaac_ros_launch_utils --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_test_cmake --symlink-install --parallel-workers 1
colcon build --packages-select semantic_label_conversion --symlink-install --parallel-workers 1
# Build isaac_ros_r2b_galileo from Batch 7 first, then:
colcon build --packages-select isaac_ros_rosbag_utils --symlink-install --parallel-workers 1
```

### Batch 7: Main Application Packages

**Status:** ✅ All packages BUILT

```bash
colcon build --packages-select isaac_ros_r2b_galileo --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_visual_slam --symlink-install --parallel-workers 1
colcon build --packages-select isaac_ros_nvblox --symlink-install --parallel-workers 1
```

**Note:** `isaac_ros_nvblox` requires `nvblox_image_padding` and `nvblox_examples_bringup` from Batch 8 to be built first.

### Batch 8: NVBlox Remaining Packages

**Status:** ✅ All packages BUILT

**Build Order:**
- `nvblox_image_padding` must be built before `nvblox_examples_bringup` and `isaac_ros_nvblox`
- `nvblox_test_data` must be built before `nvblox_test`

```bash
colcon build --packages-select nvblox_image_padding --symlink-install --parallel-workers 1
colcon build --packages-select nvblox_examples_bringup --symlink-install --parallel-workers 1
colcon build --packages-select nvblox_test_data --symlink-install --parallel-workers 1
colcon build --packages-select nvblox_test --symlink-install --parallel-workers 1
```

---

## Troubleshooting Build Issues

### Permission Errors

If you see `Permission denied` errors when building packages:
```bash
# Fix ownership (if needed)
sudo chown -R admin:admin install/ build/
```

### Magic Enum Errors

If you see `magic_enum::magic_enum target not found` errors:

**Solution:** The following packages have been fixed by adding `find_package(magic_enum REQUIRED)` to their `CMakeLists.txt`:
- `isaac_ros_nitros_occupancy_grid_type`
- `isaac_ros_nitros_pose_array_type`
- `isaac_ros_nitros_detection2_d_array_type`
- `isaac_ros_nitros_bridge_ros2`
- `isaac_ros_nitros_topic_tools`
- `custom_nitros_image`
- `custom_nitros_string`
- `custom_nitros_dnn_image_encoder`
- `custom_nitros_message_filter`

**Magic Enum Location:**
- Headers: `/opt/ros/humble/include/magic_enum.hpp`
- CMake config: `/opt/ros/humble/lib/cmake/magic_enum/magic_enumConfig.cmake`
- Target: `magic_enum::magic_enum`

### Missing Dependencies

If you see an error like:
```
ERROR: Failed to find the following files:
- /workspaces/isaac_ros-dev/install/<package>/share/<package>/package.sh
Check that the following packages have been built:
- <package>
```

**Solution:** Build the missing dependency first, or use:
```bash
colcon build --packages-up-to <package_name> --symlink-install --parallel-workers 1
```

### Packages Already Installed Warnings

If you see warnings about packages being used from `/opt/ros/humble`:
```
WARNING: The following packages are in the workspace but haven't been built:
- gxf_isaac_message_compositor
They are being used from the following locations instead:
- /opt/ros/humble
```

**This is normal!** These packages were installed via `rosdep install` and are being used from the system installation. You can:
- Ignore the warning (recommended)
- Or build them from source if you need to modify them using `--allow-overriding <package_name>`

---

## Quick Reference

```bash
# Build single package
colcon build --packages-select <package_name> --symlink-install --parallel-workers 1

# Build package with dependencies
colcon build --packages-up-to <package_name> --symlink-install --parallel-workers 1

# Source workspace
source install/setup.bash

# List all packages
colcon list --names-only

# Filter packages by repository
colcon list --names-only | grep <repo_name>

# Check package dependencies
cat src/<repo_name>/<package_name>/package.xml

# Verify package is installed
ros2 pkg list | grep <package_name>

# Check build status
ls -1 install/ | grep -v -E '^(COLCON_IGNORE|\.colcon|local_setup|setup|_local_setup)' | wc -l
```

---

## Best Practices

1. **Always use `--parallel-workers 1`** on Jetson to prevent crashes
2. **Use `--symlink-install`** for faster development cycles
3. **Build dependencies first** or use `--packages-up-to` for automatic dependency resolution
4. **Source after each build** to make packages available: `source install/setup.bash`
5. **Build incrementally** - only rebuild what you've changed
6. **Respect build order** - some packages have dependencies that must be built first

---

## Comparison: Full Build vs One-by-One

| Aspect | `colcon build` (All) | One-by-One |
|--------|---------------------|------------|
| **Parallelization** | Multiple packages + multiple jobs | One package, limited jobs |
| **Memory Usage** | Very High | Low |
| **CPU Usage** | Maximum (all cores) | Minimal (1-2 cores) |
| **Build Time** | Fastest (if resources allow) | Slower (sequential) |
| **Jetson Safety** | Can crash/reboot | Safe |
| **Error Tracking** | Hard to isolate | Easy to identify |
| **Progress** | All-or-nothing | Incremental |

---

## Container Persistence Setup

### Auto-Install Rosdep Dependencies

An entrypoint script automatically installs rosdep dependencies on container startup:
- Location: `/usr/local/bin/scripts/entrypoint_additions/00-install-rosdeps.sh`
- Runs automatically when container starts
- Ensures dependencies are available even after container recreation

### Auto-Source Workspace

An entrypoint script automatically sources the workspace for the user:
- Location: `/usr/local/bin/scripts/entrypoint_additions/99-source-workspace.user.sh`
- Runs automatically when you log into the container
- Makes all built packages immediately available

---

## Summary of Fixes Applied

### Magic Enum Fixes (9 packages)
Added `find_package(magic_enum REQUIRED)` to CMakeLists.txt for:
- Batch 3: `isaac_ros_nitros_occupancy_grid_type`, `isaac_ros_nitros_pose_array_type`, `isaac_ros_nitros_detection2_d_array_type`
- Batch 4: `isaac_ros_nitros_bridge_ros2`, `isaac_ros_nitros_topic_tools`
- Batch 5: `custom_nitros_image`, `custom_nitros_string`, `custom_nitros_dnn_image_encoder`, `custom_nitros_message_filter`

### Container Persistence
- Removed `--rm` flag from `run_dev.sh` - container persists after exit
- Updated container reuse logic to start stopped containers
- Added auto-installation of rosdep dependencies via entrypoint script
- Added auto-sourcing of workspace via entrypoint script

### Build Order Fixes
- Documented dependency order for Batch 6 (`isaac_ros_rosbag_utils` → `isaac_ros_r2b_galileo`)
- Documented dependency order for Batch 7 (`isaac_ros_nvblox` → `nvblox_image_padding`, `nvblox_examples_bringup`)
- Documented dependency order for Batch 8 (`nvblox_test` → `nvblox_test_data`)

---

## Next Steps: Hardware Setup

### 1. Intel RealSense D435i Camera Setup

The RealSense D435i is a depth camera with RGB, depth, and IMU sensors.

#### Prerequisites
- RealSense D435i camera connected via USB 3.0
- USB 3.0 port (required for full functionality)

#### Installation Steps

**Step 1: Install RealSense SDK**
```bash
# Inside the Docker container
sudo apt update
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

**Step 2: Verify Camera Detection**
```bash
# Check if camera is detected
lsusb | grep -i intel

# Test with realsense-viewer (if available)
realsense-viewer
```

**Step 3: Test ROS 2 Integration**
```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

# Launch RealSense camera node
ros2 launch realsense2_camera rs_launch.py
```

**Step 4: Verify Topics**
```bash
# In another terminal, check available topics
ros2 topic list | grep realsense

# View camera data
ros2 topic echo /camera/color/image_raw
ros2 topic echo /camera/depth/image_rect_raw
```

#### Common Issues

**Camera not detected:**
- Ensure USB 3.0 connection (blue USB port)
- Check USB permissions: `sudo chmod 666 /dev/video*`
- Verify udev rules: `ls -la /etc/udev/rules.d/ | grep realsense`

**Permission errors:**
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Log out and back in for changes to take effect
```

**Low FPS or poor performance:**
- Use USB 3.0 port (not USB 2.0)
- Reduce resolution/frame rate in launch file
- Check USB bandwidth: `lsusb -t`

#### Integration with Isaac ROS

The RealSense camera can be integrated with Isaac ROS NITROS for accelerated processing:
```bash
# Example: Launch with NITROS bridge
ros2 launch isaac_ros_nitros isaac_ros_nitros_realsense.launch.py
```

---

### 2. RoboClaw 2x7A Motor Controller Setup

The RoboClaw 2x7A is a dual-channel motor controller for brushed DC motors.

#### Prerequisites
- RoboClaw 2x7A motor controller
- USB-to-Serial connection (USB cable or USB-to-TTL adapter)
- Motors connected to M1 and M2 terminals
- Power supply connected (7-34V DC)

#### Installation Steps

**Step 1: Install RoboClaw Python Library**
```bash
# Inside the Docker container
pip3 install pyserial
# Or if using system Python
sudo apt install -y python3-serial
```

**Step 2: Install RoboClaw ROS 2 Package**
```bash
# Check if roboclaw_ros2 package exists in workspace
# If not, you may need to add it as a dependency or create a custom package

# For now, we'll set up basic serial communication
sudo apt install -y python3-pip
pip3 install roboclaw-python  # If available, or use pyserial directly
```

**Step 3: Configure Serial Port**
```bash
# Find the serial device
ls -la /dev/ttyACM* /dev/ttyUSB*

# Set permissions
sudo chmod 666 /dev/ttyACM0  # Replace with your device
sudo usermod -a -G dialout $USER
```

**Step 4: Test Serial Communication**
```bash
# Test connection (replace /dev/ttyACM0 with your device)
python3 -c "
import serial
ser = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
# Send test command (check RoboClaw documentation for command format)
ser.write(b'?')  # Example: get version
response = ser.read(100)
print('Response:', response)
ser.close()
"
```

**Step 5: Create ROS 2 Node (if needed)**

If a RoboClaw ROS 2 package doesn't exist, you may need to create a custom node:

```python
# Example structure for roboclaw_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class RoboClawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', 38400, timeout=1)
        self.subscription = self.create_subscription(
            Float32,
            'motor1_speed',
            self.motor1_callback,
            10)
        # Add more subscriptions for motor2, etc.
    
    def motor1_callback(self, msg):
        # Convert speed to RoboClaw command
        # Send command via serial
        pass
```

**Step 6: Configure RoboClaw Settings**

Use RoboClaw Motion Studio (Windows) or command-line tools to configure:
- Motor limits (current, velocity)
- PID parameters
- Encoder settings
- Communication settings (baud rate, address)

**Step 7: Launch ROS 2 Node**
```bash
# Source workspace
source /workspaces/isaac_ros-dev/install/setup.bash

# Launch your RoboClaw node
ros2 run your_package roboclaw_node
```

#### Common Issues

**Serial port not found:**
- Check USB connection
- Verify device: `dmesg | tail` after plugging in
- Check permissions: `ls -la /dev/ttyACM*`

**Permission denied:**
```bash
sudo chmod 666 /dev/ttyACM0
sudo usermod -a -G dialout $USER
# Log out and back in
```

**No response from RoboClaw:**
- Verify power supply is connected
- Check baud rate matches (default: 38400)
- Verify wiring connections
- Check RoboClaw address (default: 128)

**Motor not moving:**
- Check motor connections (M1/M2 terminals)
- Verify power supply voltage (7-34V DC)
- Check motor limits in RoboClaw settings
- Verify command format matches RoboClaw protocol

#### Integration with Isaac ROS

For integration with Isaac ROS navigation or control systems:
```bash
# Example: Publish motor commands from Isaac ROS
ros2 topic pub /motor1_speed std_msgs/msg/Float32 "{data: 0.5}"
```

---

### 3. Combined System Setup

Once both devices are set up, you can integrate them:

**Example Launch File:**
```python
# combined_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RealSense camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera'
        ),
        # RoboClaw motor controller
        Node(
            package='your_roboclaw_package',
            executable='roboclaw_node',
            name='roboclaw_controller'
        ),
        # Your application node
        Node(
            package='your_application',
            executable='main_node',
            name='main_application'
        ),
    ])
```

**Launch combined system:**
```bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch your_package combined_system.launch.py
```

---

### Verification Checklist

- [ ] RealSense D435i detected: `lsusb | grep Intel`
- [ ] RealSense topics available: `ros2 topic list | grep realsense`
- [ ] RealSense camera streaming: `ros2 topic echo /camera/color/image_raw`
- [ ] RoboClaw serial port detected: `ls -la /dev/ttyACM*`
- [ ] RoboClaw communication working: Test serial commands
- [ ] RoboClaw ROS 2 node running: `ros2 node list | grep roboclaw`
- [ ] Motors responding to commands: Test motor movement
- [ ] Combined system operational: Launch all nodes together

---

### Additional Resources

**RealSense D435i:**
- Official documentation: https://www.intelrealsense.com/depth-camera-d435i/
- ROS 2 package: https://github.com/IntelRealSense/realsense-ros
- Isaac ROS integration: Check `isaac_ros_nitros` packages

**RoboClaw 2x7A:**
- Product manual: https://www.basicmicro.com/roboclaw-2x7a-7a-7a-30v-peak-dual-channel-motor-controller-p-6.html
- Python library: https://github.com/basicmicro/roboclaw-python
- ROS 2 examples: Search for `roboclaw_ros2` or create custom node

---

### Next Steps After Hardware Setup

1. **Calibrate sensors** - Camera calibration, IMU calibration
2. **Configure motor parameters** - PID tuning, velocity/current limits
3. **Create launch files** - Combine all nodes into convenient launch files
4. **Test integration** - Verify data flow between sensors and actuators
5. **Develop application** - Build your specific robotics application
