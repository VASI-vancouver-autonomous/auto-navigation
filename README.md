# Vancouver Autonomous Systems Initiative (VASI)

Open-source autonomous robotics built on Isaac ROS 2. This repository contains the codebase for autonomous mobile robots with perception, mapping, and navigation capabilities.

## Quick Start

```bash
# Install ROS dependencies
./install_rosdeps.sh

# Build packages
./build_packages_one_by_one.sh

# Download YOLOv8 models
./download_yolo8.sh

# Launch RealSense camera
./launch_realsense.sh

# Launch YOLOv8 detection
./launch_yolov8_visualizer.sh
```

## Repository Structure

### `src/` - ROS 2 Workspace Source Code

**`isaac_ros_common/`** - Common utilities and interfaces for Isaac ROS packages
- Message definitions, launch utilities, testing frameworks
- See: `src/isaac_ros_common/README.md`

**`isaac_ros_nitros/`** - NVIDIA TensorRT ROS bridge for accelerated message passing
- Type adaptation and negotiation for optimized communication
- See: `src/isaac_ros_nitros/README.md`

**`isaac_ros_object_detection/`** - Object detection packages
- **`isaac_ros_yolov8/`** - YOLOv8 real-time object detection (TensorRT optimized)
  - Publishes: `/detections_output` (Detection2DArray)
  - Subscribes: `/image_rect` (Image)
  - See: `src/isaac_ros_object_detection/README.md`
- **`isaac_ros_detectnet/`** - DetectNet object detection
- **`isaac_ros_rtdetr/`** - RT-DETR object detection

**`isaac_ros_visual_slam/`** - Visual SLAM for mapping and localization
- Stereo visual-inertial odometry (VIO)
- Loop closure and map building
- Publishes: `/visual_slam/odometry`, `/visual_slam/pose`
- See: `src/isaac_ros_visual_slam/README.md`

**`isaac_ros_nvblox/`** - GPU-accelerated 3D mapping and reconstruction
- TSDF (Truncated Signed Distance Function) mapping
- Costmap generation for Nav2
- Publishes: `/nvblox_node/mesh`, `/nvblox_node/esdf_pointcloud`
- See: `src/isaac_ros_nvblox/README.md`

**`ros2_roboclaw_driver/`** - Motor controller driver for RoboClaw 2x7A
- Differential drive control
- Subscribes: `/cmd_vel` (Twist)
- Publishes: `/odom` (Odometry), `/joint_states` (JointState)
- See: `src/ros2_roboclaw_driver/README.md`

### Root Scripts

**`launch_realsense.sh`** - Launch RealSense D435i camera node
- Publishes: `/camera/color/image_rect`, `/camera/imu`
- See: `hardware_tests/REALSENSE_D435I_TEST.md`

**`launch_yolov8_visualizer.sh`** - Launch YOLOv8 detection with visualization
- Runs object detection pipeline and visualizer

**`ps5_button_teleop.py`** - PS5 controller teleoperation node
- Subscribes: `/joy` (Joy)
- Publishes: `/cmd_vel` (Twist)
- Launch: `ros2 launch ps5_button_teleop.launch.py`

**`yolov8_visualizer_lightweight.py`** - Lightweight YOLOv8 visualizer
- Subscribes: `/image_rect`, `/detections_output`
- Publishes: `/yolov8_processed_image` (with bounding boxes)

**`export_yolov8_to_onnx.py`** - Export YOLOv8 model to ONNX format
**`export_yolov8n_to_onnx.py`** - Export YOLOv8n (nano) to ONNX

### `hardware_tests/` - Hardware Documentation

- **`REALSENSE_D435I_TEST.md`** - RealSense camera setup and testing
- **`MOTOR_CONTROLLERS_AND_FUNCTIONALITY.md`** - Motor controller specs and integration
- **`AUDIO_VOICE_COMMAND_TEST.md`** - Audio/voice command setup
- **`BUILD_SETUP.md`** - Build configuration and Isaac ROS setup
- **`README.md`** - Hardware tests overview

### `isaac_ros_assets/` - Model Configurations

- **`isaac_ros_yolov8/`** - YOLOv8 interface specs for different cameras
- **`isaac_ros_visual_slam/`** - SLAM configurations
- **`models/`** - Pre-trained model configs (e.g., PeopleNet)

### `FOXGLOVE_SETUP.md` - Visualization Setup

Instructions for connecting Foxglove Studio to visualize ROS topics remotely via WebSocket.

## Usage Examples

### Object Detection Pipeline

```bash
# Terminal 1: Launch camera
./launch_realsense.sh

# Terminal 2: Launch YOLOv8 detection
./launch_yolov8_visualizer.sh

# View detections
ros2 topic echo /detections_output
ros2 topic echo /yolov8_processed_image
```

### Visual SLAM

```bash
# Launch Visual SLAM with RealSense
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# View odometry
ros2 topic echo /visual_slam/odometry
```

### Motor Control

```bash
# Launch RoboClaw driver
ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### PS5 Teleoperation

```bash
# Launch PS5 controller node
ros2 launch ps5_button_teleop.launch.py

# Connect PS5 controller via USB or Bluetooth
```

## Key ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_rect` | `sensor_msgs/Image` | RealSense RGB image |
| `/detections_output` | `vision_msgs/Detection2DArray` | YOLOv8 detections |
| `/yolov8_processed_image` | `sensor_msgs/Image` | Image with bounding boxes |
| `/visual_slam/odometry` | `nav_msgs/Odometry` | Visual SLAM odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/joy` | `sensor_msgs/Joy` | Joystick input |

## Prerequisites

- **Hardware**: NVIDIA Jetson (Orin/Xavier) or x86_64 with GPU
- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Isaac ROS**: v3.2-11 (see `hardware_tests/BUILD_SETUP.md`)
- **NGC Account**: For accessing Isaac ROS models

## Documentation

- **Hardware Setup**: `hardware_tests/README.md`
- **Build Configuration**: `hardware_tests/BUILD_SETUP.md`
- **Foxglove Visualization**: `FOXGLOVE_SETUP.md`
- **Isaac ROS Docs**: https://nvidia-isaac-ros.github.io/

## Contact

**VASI Initiative**  
Email: vasinitiative@yahoo.com  
Primary Technical Stewardship: Lawrence Okolo

---

*All work is released as open source to support practical learning and system-level understanding of autonomous robotics.*
