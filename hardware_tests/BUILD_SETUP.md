# Isaac ROS Build and Setup Documentation

This document records the build and configuration steps performed to set up Isaac ROS with RealSense D435i camera support on a Jetson device.

## Date
December 5, 2024

## Repository Version
- **Version**: `v3.2-11`
- **Set via**: `git checkout v3.2-11`

## System Information
- **Platform**: Jetson (ARM64/aarch64)
- **OS**: Linux 5.15.148-tegra
- **ROS 2**: Humble
- **Camera**: Intel RealSense D435i

---

## 1. Repository Version Setup

### Set repository to v3.2-11
```bash
cd $ISAAC_ROS_WS/src
git checkout v3.2-11
```

**Verification**:
```bash
git describe --tags --always
# Output: v3.2-11
```

---

## 2. NGC Authentication Configuration

### Problem
The `isaac_ros_r2b_galileo` package failed to build due to authentication errors when downloading MCAP files from NVIDIA NGC (NVIDIA GPU Cloud).

**Error**: `HTTP/2 stream 0 was not closed cleanly: INTERNAL_ERROR (err 2)`

### Solution

#### 2.1 Install NGC CLI
1. Downloaded NGC CLI for ARM64 architecture:
   ```bash
   wget https://ngc.nvidia.com/downloads/ngccli_arm64.zip
   unzip ngccli_arm64.zip -d ngccli
   ```

2. Configured NGC CLI:
   ```bash
   ./ngccli/ngc-cli/ngc config set
   # Entered API key when prompted
   ```

#### 2.2 Mount NGC Config into Docker Container
Created `~/.isaac_ros_dev-dockerargs` with:
```
-v $HOME/.ngc:/home/admin/.ngc:ro
```

This mounts the host's NGC configuration directory into the container as read-only.

#### 2.3 Modified CMakeLists.txt for NGC Authentication
**File**: `isaac_ros_common/isaac_ros_r2b_galileo/CMakeLists.txt`

**Changes**:
- Added logic to read NGC API key from `~/.ngc/config` or `/home/admin/.ngc/config`
- Modified `FetchContent_Declare` calls to include:
  - `HTTPHEADER` with `Authorization: Bearer` and `X-NGC-API-KEY`
  - `api_key` query parameter in URLs

**Key Code Addition**:
```cmake
# Try to read NGC API key from config file
set(NGC_API_KEY "")
if(EXISTS "$ENV{HOME}/.ngc/config")
  file(READ "$ENV{HOME}/.ngc/config" NGC_CONFIG_CONTENT)
  string(REGEX MATCH "apikey = ([^\n]+)" _ "${NGC_CONFIG_CONTENT}")
  if(CMAKE_MATCH_1)
    set(NGC_API_KEY "${CMAKE_MATCH_1}")
  endif()
endif()

# If not found in HOME, try /home/admin/.ngc/config (container default)
if("${NGC_API_KEY}" STREQUAL "")
  if(EXISTS "/home/admin/.ngc/config")
    file(READ "/home/admin/.ngc/config" NGC_CONFIG_CONTENT)
    string(REGEX MATCH "apikey = ([^\n]+)" _ "${NGC_CONFIG_CONTENT}")
    if(CMAKE_MATCH_1)
      set(NGC_API_KEY "${CMAKE_MATCH_1}")
    endif()
  endif()
endif()

# Set HTTP headers for authentication if API key is found
set(HTTP_HEADERS "")
if(NOT "${NGC_API_KEY}" STREQUAL "")
  list(APPEND HTTP_HEADERS "Authorization: Bearer ${NGC_API_KEY}")
  list(APPEND HTTP_HEADERS "X-NGC-API-KEY: ${NGC_API_KEY}")
endif()
```

---

## 3. APT Repository Issues

### Problem
During `rosdep install`, encountered repository metadata changes and file size mismatches.

**Errors**:
- `E: Repository 'https://isaac.download.nvidia.com/isaac-ros/release-3 jammy InRelease' changed its 'Origin' value`
- `E: Failed to fetch ... File has unexpected size (actual != expected). Mirror sync in progress?`

### Solution
```bash
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*
sudo apt-get update --allow-releaseinfo-change
```

This clears the local apt cache and forces a fresh update, accepting repository metadata changes.

---

## 4. RealSense D435i Camera Configuration

### 4.1 Enable RealSense Support in Docker Image

**File Created**: `isaac_ros_common/scripts/.isaac_ros_common-config`

**Content**:
```
CONFIG_IMAGE_KEY=ros2_humble.realsense
```

This configures `run_dev.sh` to build the Docker image with the RealSense layer, which includes:
- `librealsense2` SDK
- `realsense-ros` ROS 2 packages

### 4.2 Rebuild Docker Container
```bash
# Stop and remove existing container to force rebuild
docker stop isaac_ros_dev-aarch64-container
docker rm isaac_ros_dev-aarch64-container

# Rebuild with RealSense support
cd $ISAAC_ROS_WS/src/isaac_ros_common/scripts
./run_dev.sh
```

**Note**: Rebuilding takes several minutes as it includes the RealSense layer.

### 4.3 USB Device Access Configuration

**Problem**: RealSense camera node failed with `RS2_USB_STATUS_ACCESS` error.

**Solution**: Updated `~/.isaac_ros_dev-dockerargs` to include USB bus mount:

```
-v $HOME/.ngc:/home/admin/.ngc:ro
-v /dev/bus/usb:/dev/bus/usb
-v /sys/devices/platform:/sys/devices/platform
```

**Final `.isaac_ros_dev-dockerargs` content**:
```
-v $HOME/.ngc:/home/admin/.ngc:ro
-v /dev/bus/usb:/dev/bus/usb
-v /sys/devices/platform:/sys/devices/platform
```

### 4.4 USB Device Permissions Fix

Inside the container, manually set permissions on the USB device:
```bash
sudo chmod 666 /dev/bus/usb/002/003
```

**Note**: The device path (`/dev/bus/usb/002/003`) may vary. Check with:
```bash
lsusb | grep -i realsense
ls -la /dev/bus/usb/*/003  # Adjust based on lsusb output
```

### 4.5 Verify RealSense Camera

**Launch camera node**:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_color:=false \
  enable_depth:=false \
  depth_module.emitter_enabled:=0
```

**Check topics**:
```bash
ros2 topic list | grep camera
```

**Expected topics**:
- `/camera/infra1/image_rect_raw`
- `/camera/infra2/image_rect_raw`
- `/camera/imu`
- `/camera/extrinsics/depth_to_infra1`
- `/camera/extrinsics/depth_to_infra2`
- Camera info and metadata topics

**Verify data streaming**:
```bash
ros2 topic echo /camera/infra1/image_rect_raw --no-arr
```

---

## 5. GPIO Access Configuration

### Mount Platform Devices
Added to `~/.isaac_ros_dev-dockerargs`:
```
-v /sys/devices/platform:/sys/devices/platform
```

This allows the container to access GPIO pins and other platform devices.

---

## 6. Package Verification

### Verify Installed Packages
```bash
ros2 pkg list | grep -E "(isaac|realsense)"
```

**Expected packages**:
- `isaac_ros_common`
- `isaac_ros_nitros`
- `isaac_ros_visual_slam`
- `isaac_ros_nvblox`
- `realsense2_camera`
- Other Isaac ROS packages

---

## 7. Key Files Modified/Created

### Created Files
1. `~/.isaac_ros_dev-dockerargs`
   - Docker bind mount configurations
   - NGC config mount
   - USB device access
   - Platform device access

2. `isaac_ros_common/scripts/.isaac_ros_common-config`
   - RealSense Docker image configuration

### Modified Files
1. `isaac_ros_common/isaac_ros_r2b_galileo/CMakeLists.txt`
   - Added NGC API key reading logic
   - Added HTTP headers for authenticated downloads
   - Added API key query parameter to URLs

---

## 8. Troubleshooting Notes

### NGC Download Issues
- Ensure NGC CLI is installed and configured on the host
- Verify `.ngc/config` file exists and contains valid API key
- Check that the config is mounted into the container

### USB Access Issues
- Verify user is in `plugdev` group on host: `groups | grep plugdev`
- Check USB device permissions: `ls -la /dev/bus/usb/*/003`
- Manually set permissions if needed: `sudo chmod 666 /dev/bus/usb/002/003`
- Ensure `/dev/bus/usb` is mounted in container

### Container Rebuild Issues
- `run_dev.sh` reuses existing containers
- To force rebuild: `docker stop <container> && docker rm <container>`
- Then run `./run_dev.sh` again

### APT Repository Issues
- If repository metadata changes: use `--allow-releaseinfo-change` flag
- If file size mismatches: clear cache and update: `sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* && sudo apt-get update`

---

## 9. Current Status

✅ **Repository version**: v3.2-11  
✅ **NGC authentication**: Configured and working  
✅ **RealSense D435i**: Detected and streaming  
✅ **Camera topics**: All publishing correctly  
✅ **USB access**: Configured  
✅ **GPIO access**: Configured  
✅ **All packages**: Built and installed  

---

## 10. Next Steps

### Ready to Use
- Launch Visual SLAM with RealSense:
  ```bash
  ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
  ```

- Use other Isaac ROS packages with RealSense camera data

- Access GPIO pins from within the container

---

## Notes

- The NGC API key is stored in `~/.ngc/config` on the host and mounted read-only into the container
- USB device permissions may need to be reset if the device is unplugged/replugged
- The RealSense Docker image layer adds significant build time but provides all necessary dependencies
- All configurations persist across container restarts via the `.isaac_ros_dev-dockerargs` file



