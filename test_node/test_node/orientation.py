import cv2 as cv
from math import atan2, cos, sin, sqrt, pi, tan
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

# Object surface Z distance from the camera
OBJECT_DISTANCE = 0.57

# Camera extrinsic parameters
CAMERA_POSITION = np.array([0.5, 0.0, 1.35])
CAMERA_ORIENTATION = Rotation.from_euler('XYZ', [-180, 0.0, 90.0], degrees=True)

# Camera intrinsic parameters
K_MATRIX = np.array([[476.7030836014194, 0.0, 400.5], [0.0, 476.7030836014194, 200.5], [0.0, 0.0, 1.0]])

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
        red_mask = cv.inRange(hsv, (0, 100, 240), (10, 255, 255))

        # Threshold the HSV image to get only green colors
        green_mask = cv.inRange(hsv, (40, 40, 240), (70, 255, 255))

        # Threshold the HSV image to get only blue colors
        blue_mask = cv.inRange(hsv, (110, 50, 240), (130, 255, 255))

        # Find contours in the masks
        red_contours, _ = cv.findContours(red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv.findContours(blue_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Find position of the objects
        if len(red_contours) > 0:
            red_contour = max(red_contours, key=cv.contourArea)
            red_position = self.find_position(red_contour, img)

        if len(green_contours) > 0:
            green_contour = max(green_contours, key=cv.contourArea)
            green_position = self.find_position(green_contour, img)


        if len(blue_contours) > 0:
            blue_contour = max(blue_contours, key=cv.contourArea)
            blue_position = self.find_position(blue_contour, img)

        # Show the image
        cv.imshow('Image', img)
        cv.waitKey(1)

    def find_position(self, contour, img):
        # Compute the center of the contour
        M = cv.moments(contour)
        if M["m00"] == 0:
            return None
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # Compute the distance from the camera to the object
        z = OBJECT_DISTANCE

        # Compute the position of the object in the camera frame
        x = (cX - K_MATRIX[0, 2]) * z / K_MATRIX[0, 0]
        y = (cY - K_MATRIX[1, 2]) * z / K_MATRIX[1, 1]

        # Compute the position of the object in the world frame
        camera_position = CAMERA_POSITION
        camera_orientation = CAMERA_ORIENTATION.as_matrix()
        object_position = np.array([x, y, z])
        object_position_world = camera_position + np.dot(camera_orientation, object_position)

        # Write the position on the image above the object
        font = cv.FONT_HERSHEY_SIMPLEX
        cv.putText(img, 'X: {:.2f} m'.format(object_position_world[0]), (cX, cY-20), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        cv.putText(img, 'Y: {:.2f} m'.format(object_position_world[1]), (cX, cY), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)

        return object_position_world
        
        



def main(args=None):
    rclpy.init(args=args)
    orientation_node = OrientationNode()
    rclpy.spin(orientation_node)
    orientation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
