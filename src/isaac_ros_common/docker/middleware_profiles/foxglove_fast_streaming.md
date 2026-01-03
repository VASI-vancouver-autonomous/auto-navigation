# Fast Camera Streaming with Foxglove (NITROS Transport)

This guide optimizes Foxglove Studio for fast camera streaming from your RealSense camera when using **NITROS transport**.

## Current Setup

Your RealSense camera is configured at:
- Resolution: 640x360
- Frame rate: 90 FPS
- Topics: `/camera/infra1/image_rect_raw` and `/camera/infra2/image_rect_raw`
- **Transport**: NITROS (shared memory, GPU-accelerated)

## Understanding NITROS + Foxglove

**NITROS Transport:**
- Uses **shared memory (IPC)** for inter-process communication
- Images stay in **GPU memory** - no CPU copies
- Extremely fast for local processes

**Foxglove Connection:**
- Connects via **WebSocket** to ROS 2 topics
- Data must go through **DDS** (not NITROS shared memory)
- Network bandwidth becomes the bottleneck

**Key Insight:** NITROS is fast locally, but Foxglove still uses DDS over the network. The optimizations focus on the DDS layer.

## Optimization Strategies

### 1. Apply DDS Buffer Tuning (CRITICAL)

Even with NITROS, data going to Foxglove uses DDS. The buffer tuning prevents overflow:

```bash
sudo $ISAAC_ROS_WS/src/isaac_ros_common/scripts/tune_dds_buffers.sh
```

This addresses:
- IP fragment buffer overflow (especially on WiFi)
- Large message delivery issues
- Network flooding with fast-published data

### 2. Optimize Foxglove Bridge Settings

Use these optimized settings for fast streaming:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    send_buffer_limit:=50000000 \
    max_qos_depth:=1 \
    use_compression:=true
```

### 3. Use Compressed Image Topics (Optional)

If you have compressed image topics available, use them in Foxglove:
- `/yolov8_processed_image/compressed` (from your visualizer)

**Note:** The `image_compressor_node.py` is **NOT needed** with NITROS - NITROS already handles GPU memory efficiently. Compression should happen at the application level (like your visualizer) if needed.

## Quick Setup Script

Run this to set up fast streaming:

```bash
# 1. Apply DDS tuning (CRITICAL - prevents buffer overflow)
sudo $ISAAC_ROS_WS/src/isaac_ros_common/scripts/tune_dds_buffers.sh

# 2. Launch optimized Foxglove bridge
cd /workspaces/isaac_ros-dev
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    send_buffer_limit:=50000000 \
    max_qos_depth:=1 \
    use_compression:=true
```

Or use the convenience script:
```bash
$ISAAC_ROS_WS/src/isaac_ros_common/scripts/launch_foxglove_fast.sh
```

## Expected Performance

**With NITROS + Optimizations:**
- **Local processing**: Near-zero latency (shared memory)
- **Foxglove streaming**: 50-200ms latency (network dependent)
- **Frame rate**: 30-60 FPS in Foxglove (limited by network, not NITROS)
- **Bandwidth**: 60+ MB/s per camera stream (uncompressed over network)

**Without DDS tuning:**
- **Buffer overflows**: Connection hangs, dropped frames
- **Latency**: 500ms-2s+ (buffer blocking)
- **Frame rate**: 5-15 FPS (buffer issues)

**Key Point:** NITROS is already optimized for local processing. The DDS buffer tuning ensures data flows smoothly to Foxglove over the network.

## Troubleshooting Slow Streaming

1. **Check if using compressed topics**: Look for `/compressed` suffix
2. **Monitor bandwidth**: `sudo iftop -i <interface>` or `nethogs`
3. **Reduce resolution**: Lower camera resolution if needed
4. **Use topic whitelist**: Only subscribe to topics you need
5. **Check network**: Use wired connection instead of WiFi if possible

