# Hardware Tests Documentation

This folder contains documentation and test procedures for all hardware components integrated with the Isaac ROS system.

## Contents

### 1. [RealSense D435i Camera Test](./REALSENSE_D435I_TEST.md)
- Setup and configuration
- Testing procedures
- Verification steps
- Integration with Isaac ROS Visual SLAM and NVBlox

### 2. [Audio and Voice Command Test](./AUDIO_VOICE_COMMAND_TEST.md)
- Audio device setup
- Speech recognition configuration
- Voice command testing
- Integration with ROS 2

### 3. [Motor Controller Documentation](../MOTOR_CONTROLLERS_AND_FUNCTIONALITY.md)
- RoboClaw 2x7A setup
- Motor control configuration
- ROS 2 integration
- Complete robot functionality

---

## Quick Start

### Test RealSense D435i
```bash
cd /workspaces/isaac_ros-dev
./launch_realsense.sh
```

### Test Audio/Voice Commands
```bash
cd /workspaces/isaac_ros-dev
python3 test_audio.py
python3 test_speech_api.py
```

### Test Motor Controller
See [MOTOR_CONTROLLERS_AND_FUNCTIONALITY.md](../MOTOR_CONTROLLERS_AND_FUNCTIONALITY.md)

---

## Hardware Status

| Component | Status | Test Date |
|-----------|--------|-----------|
| RealSense D435i | ✅ Working | Dec 7, 2024 |
| Audio/Microphone | ✅ Working | Dec 7, 2024 |
| RoboClaw 2x7A | 📋 Pending | - |

---

## Docker Configuration

All hardware requires proper Docker device mounts. Configured in `~/.isaac_ros_dev-dockerargs`:

```
-v $HOME/.ngc:/home/admin/.ngc:ro
-v /dev/bus/usb:/dev/bus/usb
-v /sys/devices/platform:/sys/devices/platform
-v /dev/snd:/dev/snd
--group-add audio
```

---

## Test Scripts Location

All test scripts are in the root workspace directory:
- `/workspaces/isaac_ros-dev/test_audio.py`
- `/workspaces/isaac_ros-dev/test_speech_api.py`
- `/workspaces/isaac_ros-dev/test_speech_recognition.py`
- `/workspaces/isaac_ros-dev/test_realsense.sh`
- `/workspaces/isaac_ros-dev/launch_realsense.sh`

---

## Notes

- All tests should be run inside the Docker container
- Ensure hardware is connected before running tests
- Some tests require internet connection (Google Speech API)
- USB devices may need permission fixes on first use

