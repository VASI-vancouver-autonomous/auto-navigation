#!/usr/bin/env python3
"""
Export YOLOv8 model from PyTorch (.pt) to ONNX format
"""

from ultralytics import YOLO
import os

# Path to the YOLOv8 model file
model_path = 'yolov8n.pt'

# Check if model file exists
if not os.path.exists(model_path):
    print(f"Error: Model file '{model_path}' not found!")
    print(f"Current directory: {os.getcwd()}")
    print("Please make sure yolov8n.pt is in the current directory or update the path.")
    exit(1)

print(f"Loading model from: {model_path}")
model = YOLO(model_path)

print("Exporting model to ONNX format...")
try:
    # Export to ONNX
    model.export(format='onnx')
    print("✓ Successfully exported to ONNX format!")
    print(f"Output file: {model_path.replace('.pt', '.onnx')}")
except Exception as e:
    print(f"Error during export: {e}")
    exit(1)

