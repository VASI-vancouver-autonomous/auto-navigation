#!/usr/bin/env python3
"""
Export YOLOv8n (nano) model from PyTorch (.pt) to ONNX format
This is the smallest YOLOv8 model, best for Jetson devices with limited GPU memory
"""

from ultralytics import YOLO
import os

# Path to the YOLOv8n model file (will auto-download if not present)
model_path = 'yolov8n.pt'

print(f"Loading YOLOv8n (nano) model...")
print("Note: If yolov8n.pt doesn't exist, it will be automatically downloaded.")

# Load model (will auto-download if not found)
model = YOLO(model_path)

print("Exporting model to ONNX format...")
try:
    # Export to ONNX
    model.export(format='onnx')
    print("✓ Successfully exported to ONNX format!")
    print(f"Output file: {model_path.replace('.pt', '.onnx')}")
    print("\nThis nano model is much smaller and should work better on Jetson!")
except Exception as e:
    print(f"Error during export: {e}")
    exit(1)

