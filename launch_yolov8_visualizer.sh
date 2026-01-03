#!/bin/bash
# Launch the lightweight YOLOv8 visualizer

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$SCRIPT_DIR"

source install/setup.bash

echo "Starting YOLOv8 Visualizer (lightweight)..."
echo "Publishing to: /yolov8_processed_image and /yolov8_processed_image/compressed"
echo ""
echo "In Foxglove Studio, add an Image panel and subscribe to:"
echo "  - /yolov8_processed_image/compressed  (recommended - lower bandwidth)"
echo "  - /yolov8_processed_image           (uncompressed)"
echo ""

python3 "$SCRIPT_DIR/yolov8_visualizer_lightweight.py"

