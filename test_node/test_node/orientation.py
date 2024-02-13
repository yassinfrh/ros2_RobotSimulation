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

        # Convert image to grayscale
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Convert image to binary
        _, bw = cv.threshold(gray, 20, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

        # Find all the contours in the thresholded image
        contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

        for i, c in enumerate(contours):
            # Calculate the area of each contour
            area = cv.contourArea(c)

            # Ignore contours that are too small or too large
            if area < 50 or 10000 < area:
                continue

            # Draw each contour only for visualization purposes
            cv.drawContours(img, contours, i, (0, 0, 0), 2)

            # Find the orientation of each shape
            self.get_orientation(c, img)

        # Display the output image
        cv.imshow('Output Image', img)
        cv.waitKey(1)

        # Save the output image to the current directory
        cv.imwrite("output_img.jpg", img)

    def draw_axis(self, img, p_, q_, color, scale):
        p = list(p_)
        q = list(q_)

        angle = atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
        hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

        q[0] = p[0] - scale * hypotenuse * cos(angle)
        q[1] = p[1] - scale * hypotenuse * sin(angle)
        cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

        p[0] = q[0] + 9 * cos(angle + pi / 4)
        p[1] = q[1] + 9 * sin(angle + pi / 4)
        cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

        p[0] = q[0] + 9 * cos(angle - pi / 4)
        p[1] = q[1] + 9 * sin(angle - pi / 4)
        cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

    def get_orientation(self, pts, img):
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i, 0] = pts[i, 0, 0]
            data_pts[i, 1] = pts[i, 0, 1]

        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)

        cntr = (int(mean[0, 0]), int(mean[0, 1]))

        cv.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0],
              cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0],
              cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
        self.draw_axis(img, cntr, p1, (255, 255, 0), 10)
        self.draw_axis(img, cntr, p2, (0, 0, 255), 20)

        angle = atan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

        return angle

def main(args=None):
    rclpy.init(args=args)
    orientation_node = OrientationNode()
    rclpy.spin(orientation_node)
    orientation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
