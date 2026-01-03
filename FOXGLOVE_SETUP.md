# Foxglove Studio Setup for YOLOv8 Visualization

This guide documents how to set up Foxglove Studio to visualize YOLOv8 object detection results from Isaac ROS.

## Prerequisites

- Isaac ROS YOLOv8 pipeline running (camera + detection)
- `isaac_ros_yolov8_visualizer.py` script running
- Network access between your local machine and the Jetson

## Step 1: Fix the Visualizer Script

The default visualizer uses `TimeSynchronizer` which requires exact timestamp matches. We modified it to use `ApproximateTimeSynchronizer` for more flexible matching.

**File:** `src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/isaac_ros_yolov8_visualizer.py`

**Change:**
```python
# OLD (line 136-138):
self.time_synchronizer = message_filters.TimeSynchronizer(
    [self._detections_subscription, self._image_subscription],
    self.QUEUE_SIZE)

# NEW:
# Use ApproximateTimeSynchronizer for more flexible timestamp matching
# Allow 0.1 second (100ms) tolerance for timestamp differences
self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
    [self._detections_subscription, self._image_subscription],
    self.QUEUE_SIZE,
    slop=0.1)
```

**Note:** After making this change, you can either:
- Run the script directly from source: `python3 src/isaac_ros_object_detection/isaac_ros_yolov8/scripts/isaac_ros_yolov8_visualizer.py`
- Or rebuild the package to install the updated version

## Step 2: Install Foxglove Bridge

The `foxglove_bridge` package should already be installed. If not:

```bash
sudo apt-get install -y ros-humble-foxglove-bridge
```

## Step 3: Launch Foxglove Bridge

In your ROS 2 workspace:

```bash
cd /workspaces/isaac_ros-dev
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

The bridge will start and listen on port 8765. You should see:
```
[INFO] [foxglove_bridge]: [WS] WebSocket server listening at ws://0.0.0.0:8765
```

## Step 4: Configure Firewall

Allow port 8765 through the firewall:

```bash
sudo ufw allow 8765/tcp
sudo ufw status  # Verify it's allowed
```

## Step 5: Find Your Jetson IP Address

```bash
hostname -I | awk '{print $1}'
```

Example output: `10.0.0.109`

## Step 6: Connect from Foxglove Studio

### Option A: Direct Connection (Same Network)

1. Open Foxglove Studio (web or desktop app)
2. Click "Open connection" or the "+" button
3. Select **"Foxglove WebSocket"** (important - not "Rosbridge" or "Foxglove Studio")
4. Enter the connection URL: `ws://<jetson-ip>:8765`
   - Example: `ws://10.0.0.109:8765`
5. Click "Open"

### Option B: SSH Port Forwarding (Different Network)

If you're on a different network or having connection issues:

**On your local machine:**
```bash
ssh -L 8765:localhost:8765 admin@<jetson-ip>
```

**In Foxglove Studio:**
- Use: `ws://localhost:8765`

## Step 7: Visualize Detections

Once connected, you should see ROS topics in the left panel.

### To View Processed Images with Detections:

1. Click "+" or "Add panel"
2. Select "Image" panel
3. In the topic dropdown, select `/yolov8_processed_image`
4. You should see the camera feed with YOLOv8 bounding boxes and labels overlaid

### Other Useful Topics:

- `/image_rect` - Original camera image (no detections)
- `/detections_output` - Raw detection data (use "Raw Messages" panel)
- `/yolov8_encoder/resize/image` - Resized image before detection

## Troubleshooting

### Connection Fails

1. **Check if bridge is running:**
   ```bash
   ps aux | grep foxglove_bridge | grep -v grep
   ```

2. **Verify port is listening:**
   ```bash
   netstat -tlnp | grep 8765
   # or
   ss -tlnp | grep 8765
   ```

3. **Test connectivity from your local machine:**
   ```bash
   curl -v http://<jetson-ip>:8765
   ```
   Should return "426 Upgrade Required" (this is correct for WebSocket)

4. **Check firewall:**
   ```bash
   sudo ufw status
   ```

### "Send buffer limit reached" Warnings

These warnings in the bridge logs are **normal** and not critical. They indicate:
- Connection is working ✓
- Data is flowing ✓
- Bridge is sending data faster than client can consume (common with many topics)

If warnings become excessive, you can:
- Reduce the number of topics being published
- Lower publish rates for some topics
- The warnings don't affect functionality

### Web Version Issues

If using Foxglove Studio web version:
- Try a different browser (Chrome/Edge often work better than Safari)
- Check browser console (F12) for errors
- Consider using the desktop app instead (more reliable for WebSocket connections)

### Visualizer Not Publishing

If `/yolov8_processed_image` topic doesn't appear:

1. **Check if visualizer is running:**
   ```bash
   ps aux | grep isaac_ros_yolov8_visualizer | grep -v grep
   ```

2. **Verify topics exist:**
   ```bash
   ros2 topic list | grep -E "(detections_output|yolov8_encoder/resize/image)"
   ```

3. **Check topic rates:**
   ```bash
   ros2 topic hz /detections_output
   ros2 topic hz /yolov8_encoder/resize/image
   ros2 topic hz /yolov8_processed_image
   ```

4. **Restart visualizer:**
   ```bash
   pkill -f isaac_ros_yolov8_visualizer
   source install/setup.bash
   ros2 run isaac_ros_yolov8 isaac_ros_yolov8_visualizer.py
   ```

## Summary

**What we set up:**
1. ✅ Modified visualizer to use `ApproximateTimeSynchronizer` for better timestamp matching
2. ✅ Launched `foxglove_bridge` on port 8765
3. ✅ Configured firewall to allow port 8765
4. ✅ Connected Foxglove Studio to visualize YOLOv8 detections

**Key Topics:**
- `/yolov8_processed_image` - Images with bounding boxes and labels (main visualization)
- `/detections_output` - Raw detection data
- `/image_rect` - Original camera feed

**Connection URL:** `ws://<jetson-ip>:8765` (use "Foxglove WebSocket" connection type)



