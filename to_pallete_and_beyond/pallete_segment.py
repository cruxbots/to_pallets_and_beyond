#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# import torch
from ultralytics import YOLO
import numpy as np

class PalletDetectorNode(Node):
    def __init__(self):
        super().__init__('pallete_segment')

        # Set up image subscriber and mask publisher
        self.image_subscriber = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10)
        
        self.pallete_mask_publisher = self.create_publisher(Image, 'pallete_mask_topic', 10)
        self.ground_mask_publisher = self.create_publisher(Image, 'ground_mask_topic', 10)
        
        # Set up YOLO model with specified weights
        model_weights_path = '/home/rahul/to_pallet_and_beyond/ros2_ws/src/to_pallete_and_beyond/weights/runs/segment/train/weights/best.pt'  # Update this path
        self.model = YOLO(model_weights_path)
        self.bridge = CvBridge()

        self.get_logger().info("Pallet Detector Node has been started")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Perform segmentation using YOLO model
        results = self.model(cv_image)
        
        if results[0].masks is None:
            self.get_logger().warning("No masks detected in the image")
            return

        # Initialize mask image (same size as input, single channel)
        pallete_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        ground_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        for res in results[0]:

            if res.names[res.boxes.cls.tolist().pop()] == 'ground':
                contour = res.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                _ = cv2.drawContours(ground_mask, [contour], -1, (255, 255, 255), cv2.FILLED)

            elif res.names[res.boxes.cls.tolist().pop()] == 'pallets':
                contour = res.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
                _ = cv2.drawContours(pallete_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
        
        

        # Convert the mask to a ROS Image message and publish
        
        self.pallete_mask_publisher.publish(self.bridge.cv2_to_imgmsg(pallete_mask, encoding="mono8"))
        self.ground_mask_publisher.publish(self.bridge.cv2_to_imgmsg(ground_mask, encoding="mono8"))
        self.get_logger().info("Published mask image")

def main(args=None):
    rclpy.init(args=args)
    pallet_detector_node = PalletDetectorNode()
    rclpy.spin(pallet_detector_node)
    pallet_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
