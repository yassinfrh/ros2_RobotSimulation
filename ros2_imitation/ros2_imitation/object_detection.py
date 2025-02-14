import cv2 as cv
from math import atan2, cos, sin, sqrt, pi, tan
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

from pick_place_interface.msg import DetectedObject, ListDetected

# Object surface Z distance from the camera
OBJECT_DISTANCE = 0.61

# Camera extrinsic parameters
CAMERA_POSITION = np.array([0.5, 0.0, 1.35])
CAMERA_ORIENTATION = Rotation.from_euler('XYZ', [-180, 0.0, 90.0], degrees=True)

# Camera intrinsic parameters
K_MATRIX = np.array([[476.7030836014194, 0.0, 400.5], [0.0, 476.7030836014194, 200.5], [0.0, 0.0, 1.0]])

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        # Publisher for the detected objects positions 
        self.publisher = self.create_publisher(ListDetected, '/detected_objects', 10)

        # Message for the detected objects positions
        self.detected_objects_msg = ListDetected()
                     
        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info('Failed to convert ROS Image to OpenCV image: {}'.format(e))
            return

        # Process the image and compute the detected objects positions
        self.process_image(cv_image)

    def process_image(self, img):

        # Reset the detected objects array
        self.detected_objects_msg.list.clear()

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
            # Create a DetectedObject message
            red_object = DetectedObject()
            red_object.name = 'red_box'
            # Check if find_position returns a valid position
            position = self.find_position(red_contour, img)
            if position is not None:
                red_object.x, red_object.y, red_object.z = position
                # Append the message to the list of detected objects
                self.detected_objects_msg.list.append(red_object)
                angle = self.get_orientation(red_contour, img)
            
            


        if len(green_contours) > 0:
            green_contour = max(green_contours, key=cv.contourArea)
            # Create a DetectedObject message
            green_object = DetectedObject()
            green_object.name = 'green_box'
            # Check if find_position returns a valid position
            position = self.find_position(green_contour, img)
            if position is not None:
                green_object.x, green_object.y, green_object.z = position
                # Append the message to the list of detected objects
                self.detected_objects_msg.list.append(green_object)
                angle = self.get_orientation(green_contour, img)


        if len(blue_contours) > 0:
            blue_contour = max(blue_contours, key=cv.contourArea)
            # Create a DetectedObject message
            blue_object = DetectedObject()
            blue_object.name = 'blue_box'
            # Check if find_position returns a valid position
            position = self.find_position(blue_contour, img)
            if position is not None:
                blue_object.x, blue_object.y, blue_object.z = position
                # Append the message to the list of detected objects
                self.detected_objects_msg.list.append(blue_object)
                angle = self.get_orientation(blue_contour, img)
            

        # Show the image
        cv.imshow('Image', img)
        cv.waitKey(1)

        # Publish the detected objects positions
        self.publisher.publish(self.detected_objects_msg)

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

        # Round the position to 3 decimal places
        object_position_world = np.round(object_position_world, 3)

        # Write the position on the image above the object
        font = cv.FONT_HERSHEY_SIMPLEX
        cv.putText(img, 'X: {:.2f} m'.format(object_position_world[0]), (cX, cY-20), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        cv.putText(img, 'Y: {:.2f} m'.format(object_position_world[1]), (cX, cY), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)

        # Diplay contour
        cv.drawContours(img, [contour], -1, (0, 0, 0), 2)

        return object_position_world
    
    def get_orientation(self, contour, img):
        # Take the corners of the contour
        rect = cv.minAreaRect(contour)

        # Compute the angle of the object
        angle = rect[2]

        # Compute the center of the contour
        x, y = rect[0]

        # Keep the angle between -45 and 45 degrees
        if angle < -45:
            angle += 90
        elif angle > 45:
            angle -= 90

        angle = -angle

        # Draw the text on the image
        font = cv.FONT_HERSHEY_SIMPLEX
        cv.putText(img, 'Angle: {:.2f} deg'.format(angle), (int(x), int(y) + 20), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)

        return angle
        
        
        



def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
