# DDS Buffer Tuning Guide

This guide addresses buffer overflow issues when using DDS implementations (Fast RTPS, Cyclone DDS, RTI Connext) with ROS 2, particularly on lossy network connections (WiFi) or when sending large messages.

## Problem Description

When using DDS over lossy connections (especially WiFi), several issues can occur:

1. **IP Fragment Buffer Overflow**: When UDP packets are fragmented and some fragments are dropped, the remaining fragments fill up the kernel buffer. The default 30-second timeout can cause the connection to "hang" for extended periods.

2. **Large Message Delivery Issues**: Large messages may not be delivered reliably despite using reliable QoS settings, especially over wired networks.

3. **Network Flooding**: Fast RTPS can flood the network with large or fast-published data when operating over WiFi.

## Solutions

### 1. Kernel Parameter Tuning (Recommended First Step)

Apply kernel-level tuning using the provided script:

```bash
sudo $ISAAC_ROS_WS/src/isaac_ros_common/scripts/tune_dds_buffers.sh
```

This script applies the following changes:

- **`net.ipv4.ipfrag_time`**: Reduced from 30s to 3s
  - Reduces the window where IP fragments can block the buffer
  - Prevents long blocking periods when fragments are lost

- **`net.ipv4.ipfrag_high_thresh`**: Increased from 256KB to 128MB
  - Prevents the IP fragment buffer from becoming completely full
  - Important for lossy connections (WiFi)

- **`net.core.rmem_max`**: Increased to 2GB (maximum)
  - Allows larger socket receive buffers for large message delivery
  - Critical for Fast RTPS and other DDS implementations

- **`net.core.rmem_default`**: Increased to 8MB
  - Sets a higher default receive buffer size

**Note**: These changes are persistent across reboots. The configuration is saved to `/etc/sysctl.d/99-dds-buffer-tuning.conf`.

### 2. Fast RTPS Profile Configuration

Two Fast RTPS profiles are available:

#### Standard Profile: `rtps_udp_profile.xml`
- Basic UDP transport configuration
- Suitable for most use cases

#### Tuned Profile: `rtps_udp_profile_tuned.xml`
- Enhanced buffer sizes (2MB send/receive buffers)
- Increased preallocated buffers
- Better handling of large/fast messages
- Includes commented options for best-effort QoS (useful for lossy connections)

To use the tuned profile, set the environment variable:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/rtps_udp_profile_tuned.xml
```

Or in your launch file:

```python
import os
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/path/to/rtps_udp_profile_tuned.xml'
```

### 3. QoS Settings (Optional)

For lossy connections, consider using **best-effort** QoS instead of reliable:

- **Reliable QoS**: Requires acknowledgements and retransmissions, increasing network overhead
- **Best-effort QoS**: Reduces network traffic and overhead, but messages may be lost

To use best-effort QoS, uncomment the reliability settings in `rtps_udp_profile_tuned.xml` or configure it in your ROS 2 nodes:

```python
from rclcpp.qos import QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)
```

## Usage with Isaac ROS

### Option 1: Apply kernel tuning before launching

```bash
# Apply kernel parameters (run once, persists across reboots)
sudo $ISAAC_ROS_WS/src/isaac_ros_common/scripts/tune_dds_buffers.sh

# Launch your ROS 2 nodes normally
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Option 2: Use tuned Fast RTPS profile

```bash
# Set the tuned profile
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/isaac_ros_common/docker/middleware_profiles/rtps_udp_profile_tuned.xml

# Launch your ROS 2 nodes
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Option 3: Both (Recommended for severe buffer issues)

```bash
# Apply kernel tuning
sudo $ISAAC_ROS_WS/src/isaac_ros_common/scripts/tune_dds_buffers.sh

# Use tuned profile
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/isaac_ros_common/docker/middleware_profiles/rtps_udp_profile_tuned.xml

# Launch nodes
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

## Verifying Settings

Check current kernel parameters:

```bash
sysctl net.ipv4.ipfrag_time
sysctl net.ipv4.ipfrag_high_thresh
sysctl net.core.rmem_max
sysctl net.core.rmem_default
```

Check if Fast RTPS profile is loaded:

```bash
echo $FASTRTPS_DEFAULT_PROFILES_FILE
```

## Reverting Changes

To revert kernel parameter changes:

```bash
sudo rm /etc/sysctl.d/99-dds-buffer-tuning.conf
sudo sysctl --system
```

To revert to default Fast RTPS profile:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

## Additional Notes

- **Resource Impact**: Increasing buffer sizes uses more memory. Monitor system resources if you have memory constraints.

- **Network Topology**: Tuning may need adjustment based on your specific network setup, message sizes, and data rates.

- **Testing**: After applying changes, test your system to ensure the buffer issues are resolved. You may need to adjust values based on your specific use case.

- **Other DDS Implementations**: While this guide focuses on Fast RTPS, the kernel parameter tuning applies to all DDS implementations (Cyclone DDS, RTI Connext, etc.).

## References

Based on ROS 2 DDS tuning recommendations:
- [ROS 2 DDS Tuning Guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)
- Fast RTPS documentation
- Linux kernel networking parameters

