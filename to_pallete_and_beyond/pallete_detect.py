#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from ultralytics import YOLO
from geometry_msgs.msg import PolygonStamped, Point32

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class PalletDetectorNode(Node):
    def __init__(self):
        super().__init__('pallete_detect')

        # Set up image subscriber and mask publisher
        self.image_subscriber = self.create_subscription(
            Image,
            '/robot1/zed2i/left/image_rect_color',
            self.image_callback,
            10)
        
        self.pallete_bbox = self.create_publisher(PolygonStamped, 'pallete_bbox_topic', 10)
        self.trouble_shoot_img = self.create_publisher(Image, "bbox_img",10)
        
        package_dir = Path(
            get_package_share_directory('to_pallete_and_beyond')
            ).parent.parent.parent.parent
        # Set up YOLO model with specified weights
        model_weights_path = Path(package_dir / "src" /"to_pallete_and_beyond" / "yolo_weights" / "pallete_detect_weights.pt")  # Update this path
        self.model = YOLO(model_weights_path.absolute())
        self.bridge = CvBridge()

        self.get_logger().info("Pallet Detector Node has been started")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_height, img_width = cv_image.shape[:2]
        # Perform segmentation using YOLO model
        results = self.model.predict(cv_image)
        

        for detection in results[0].boxes:

            if len(detection) >= 1:

                x_min = float(detection.xyxyn.tolist()[0][0] * img_width)
                y_min = float(detection.xyxyn.tolist()[0][1] * img_height)
                x_max = float(detection.xyxyn.tolist()[0][2] * img_width)
                y_max = float(detection.xyxyn.tolist()[0][3] * img_height)

                print(x_min, y_min, x_max, y_max)

                polygon = PolygonStamped()
                polygon.header.stamp = self.get_clock().now().to_msg()
                polygon.header.frame_id = "camera_frame"

                polygon.polygon.points = [
                        Point32(x=x_min, y=y_min, z=0.0),
                        Point32(x=x_max, y=y_min, z=0.0),
                        Point32(x=x_max, y=y_max, z=0.0),
                        Point32(x=x_min, y=y_max, z=0.0)
                    ]
                
                self.pallete_bbox.publish(polygon)
                self.trouble_shoot_img.publish(self.bridge.cv2_to_imgmsg(results[0].plot()))
                self.get_logger().info("Published bbox")
            else:

                self.get_logger().warning("No palletes")
                pass


def main(args=None):
    rclpy.init(args=args)
    pallet_detector_node = PalletDetectorNode()
    rclpy.spin(pallet_detector_node)
    pallet_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
