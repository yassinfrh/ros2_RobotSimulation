import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from pick_place_interface.msg import DetectedObject, ListDetected
from ultralytics import YOLO

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
        #self.publisher = self.create_publisher(ListDetected, '/detected_objects', 10)

        # Message for the detected objects positions
        #self.detected_objects_msg = ListDetected()
                     
        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/yassin/ros_ws/src/ros2_RobotSimulation/ros2_imitation/model/best.pt')  # Specify the path to your YOLO model file

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
        #self.detected_objects_msg.list.clear()

        # Was the image there?
        if img is None:
            self.get_logger().info('Error: Image not found')
            return

        # Perform YOLO object detection
        results = self.model(img, device='0')

        labels = results[0].names

        # Process each detected object
        for detection in results[0].boxes.data.tolist():
            bbox, confidence, class_id = detection[:4], detection[4], int(detection[5])

            # Get the label of the detected object
            label = labels[class_id]

            # Find the center of the bounding box
            cX = (bbox[0] + bbox[2]) / 2
            cY = (bbox[1] + bbox[3]) / 2

            # Find the position of the object
            position = self.find_position(cX, cY, img)
            if position is not None:
                # Append the message to the list of detected objects
                #self.detected_objects_msg.list.append(detected_object)

                # Draw the bounding box and label on the image
                cv.rectangle(img, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
                cv.putText(img, f'{label} ({confidence:.2f})', (int(bbox[0]), int(bbox[1]) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the image
        cv.imshow('Image', img)
        cv.waitKey(1)

        # Publish the detected objects positions
        #self.publisher.publish(self.detected_objects_msg)

    def find_position(self, cX, cY, img):
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
        cv.putText(img, 'X: {:.2f} m'.format(object_position_world[0]), (int(cX), int(cY) - 20), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)
        cv.putText(img, 'Y: {:.2f} m'.format(object_position_world[1]), (int(cX), int(cY)), font, 0.5, (0, 0, 0), 1, cv.LINE_AA)

        return object_position_world

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
