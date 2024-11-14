#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob

class ImagePublisherNode(Node):
    def __init__(self, image_directory, publish_rate=1.0):
        super().__init__('simulated_camera')
        self.image_publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)
        self.bridge = CvBridge()

        # Get list of images in the specified directory
        self.image_files = sorted(glob.glob(os.path.join(image_directory, '*.jpg')))
        self.index = 0  # Start with the first image

        if not self.image_files:
            self.get_logger().error(f"No images found in directory: {image_directory}")
            raise FileNotFoundError(f"No images found in directory: {image_directory}")
        else:
            self.get_logger().info(f"Found {len(self.image_files)} images in {image_directory}")

    def publish_image(self):
        # Read and publish the current image
        image_path = self.image_files[self.index]
        image = cv2.imread(image_path)
        
        if image is None:
            self.get_logger().warning(f"Unable to read image: {image_path}")
            return

        # Convert OpenCV image (BGR) to ROS Image message
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_publisher.publish(image_message)
        self.get_logger().info(f"Published image: {image_path}")

        # Move to the next image in the directory (loop back to the start if at the end)
        self.index = (self.index + 1) % len(self.image_files)

def main(args=None):
    rclpy.init(args=args)
    image_directory = '/home/rahul/to_pallet_and_beyond/ros2_ws/src/to_pallete_and_beyond/dataset/Pallets'  # Replace with the path to your images
    publish_rate = 1.0  # Rate in Hz (e.g., 1 image per second)

    image_publisher_node = ImagePublisherNode(image_directory=image_directory, publish_rate=publish_rate)
    rclpy.spin(image_publisher_node)

    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
