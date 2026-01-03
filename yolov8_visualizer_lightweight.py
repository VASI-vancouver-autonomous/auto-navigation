#!/usr/bin/env python3

# Lightweight YOLOv8 Visualizer - Uses /image_rect instead of /yolov8_encoder/resize/image
# to avoid hanging issues. Publishes compressed images for low-latency viewing in Foxglove.

import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray

# COCO class names
names = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus',
    6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant',
    11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat',
    16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear',
    22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag',
    27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard',
    32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove',
    36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle',
    40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl',
    46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli',
    51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair',
    57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet',
    62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard',
    67: 'cell phone', 68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink',
    72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors',
    77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush',
}


class Yolov8VisualizerLightweight(Node):
    QUEUE_SIZE = 5  # Smaller queue for lower latency
    color = (0, 255, 0)  # Green bounding boxes
    bbox_thickness = 2

    def __init__(self):
        super().__init__('yolov8_visualizer_lightweight')
        self._bridge = cv_bridge.CvBridge()
        
        # Publish both uncompressed (for compatibility) and compressed (for Foxglove)
        self._processed_image_pub = self.create_publisher(
            Image, 'yolov8_processed_image', self.QUEUE_SIZE)
        self._processed_image_compressed_pub = self.create_publisher(
            CompressedImage, 'yolov8_processed_image/compressed', self.QUEUE_SIZE)

        # Subscribe to detections and image_rect (which doesn't hang)
        self._detections_subscription = message_filters.Subscriber(
            self, Detection2DArray, 'detections_output')
        self._image_subscription = message_filters.Subscriber(
            self, Image, '/image_rect')  # Use /image_rect instead of resize/image

        # Use ApproximateTimeSynchronizer with 0.1s tolerance
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            self.QUEUE_SIZE,
            slop=0.1)

        self.time_synchronizer.registerCallback(self.detections_callback)
        self.get_logger().info('YOLOv8 Visualizer started - subscribing to /image_rect and /detections_output')

    def detections_callback(self, detections_msg, img_msg):
        try:
            txt_color = (255, 0, 255)  # Magenta text
            cv2_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            
            for detection in detections_msg.detections:
                center_x = detection.bbox.center.position.x
                center_y = detection.bbox.center.position.y
                width = detection.bbox.size_x
                height = detection.bbox.size_y

                # Get class name and confidence
                class_id = int(detection.results[0].hypothesis.class_id)
                conf_score = detection.results[0].hypothesis.score
                label = names.get(class_id, f'class_{class_id}')
                label = f'{label} {conf_score:.2f}'

                # Calculate bounding box corners
                min_pt = (round(center_x - (width / 2.0)),
                          round(center_y - (height / 2.0)))
                max_pt = (round(center_x + (width / 2.0)),
                          round(center_y + (height / 2.0)))

                # Calculate text size
                lw = max(round((img_msg.height + img_msg.width) / 2 * 0.003), 2)
                tf = max(lw - 1, 1)
                w, h = cv2.getTextSize(label, 0, fontScale=lw / 3, thickness=tf)[0]
                outside = min_pt[1] - h >= 3

                # Draw bounding box
                cv2.rectangle(cv2_img, min_pt, max_pt, self.color, self.bbox_thickness)
                
                # Draw label background and text
                cv2.putText(cv2_img, label,
                          (min_pt[0], min_pt[1]-2 if outside else min_pt[1]+h+2),
                          0, lw / 3, txt_color, thickness=tf, lineType=cv2.LINE_AA)

            # Publish uncompressed image
            processed_img = self._bridge.cv2_to_imgmsg(cv2_img, encoding='bgr8')
            processed_img.header = img_msg.header
            self._processed_image_pub.publish(processed_img)
            
            # Publish compressed image (better for Foxglove over SSH)
            compressed_msg = CompressedImage()
            compressed_msg.header = img_msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = cv2.imencode('.jpg', cv2_img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
            self._processed_image_compressed_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in detections_callback: {str(e)}')


def main():
    rclpy.init()
    node = Yolov8VisualizerLightweight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


