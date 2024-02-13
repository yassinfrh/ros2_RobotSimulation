# Write node to publish images from webcam

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamNode(Node):
    
        def __init__(self):
            super().__init__('webcam_node')
            self._pub = self.create_publisher(Image, 'webcam/image_raw', 10)
            self._timer = self.create_timer(0.1, self.timer_callback)
            self.cv_bridge = CvBridge()
            self.cap = cv2.VideoCapture(0)
    
        def timer_callback(self):
            ret, frame = self.cap.read()
            if ret:
                msg = self.cv_bridge.cv2_to_imgmsg(frame)
                self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    rclpy.spin(node)
    rclpy.shutdown()