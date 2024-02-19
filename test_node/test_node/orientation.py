import cv2 as cv
from math import atan2, cos, sin, sqrt, pi
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OrientationNode(Node):
    def __init__(self):
        super().__init__('orientation_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info('Failed to convert ROS Image to OpenCV image: {}'.format(e))
            return

        # Process the image and compute the orientation
        self.process_image(cv_image)

    def process_image(self, img):
        # Was the image there?
        if img is None:
            self.get_logger().info('Error: Image not found')
            return

        # Convert the image to HSV
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Threshold the HSV image to get only red colors
        red_mask = cv.inRange(hsv, (0, 100, 100), (10, 255, 255))

        # Threshold the HSV image to get only green colors
        green_mask = cv.inRange(hsv, (40, 40, 40), (70, 255, 255))

        # Threshold the HSV image to get only blue colors
        blue_mask = cv.inRange(hsv, (110, 50, 50), (130, 255, 255))

        # Find bounding box of each color
        red_bbox = cv.boundingRect(red_mask)
        green_bbox = cv.boundingRect(green_mask)
        blue_bbox = cv.boundingRect(blue_mask)

        # Draw the bounding boxes in black
        cv.rectangle(img, (red_bbox[0], red_bbox[1]), (red_bbox[0]+red_bbox[2], red_bbox[1]+red_bbox[3]), (0, 0, 0), 2)
        cv.rectangle(img, (green_bbox[0], green_bbox[1]), (green_bbox[0]+green_bbox[2], green_bbox[1]+green_bbox[3]), (0, 0, 0), 2)
        cv.rectangle(img, (blue_bbox[0], blue_bbox[1]), (blue_bbox[0]+blue_bbox[2], blue_bbox[1]+blue_bbox[3]), (0, 0, 0), 2)

        # Show the image
        cv.imshow('Image', img)
        cv.waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)
    orientation_node = OrientationNode()
    rclpy.spin(orientation_node)
    orientation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
