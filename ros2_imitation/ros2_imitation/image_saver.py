import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity
from std_srvs.srv import Empty
import cv2 as cv
import os, os.path
import time
import threading

RES = "null"

# Service to spawn objects and save the image of the scene
class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Current image
        self.current_image = None

        # Path to save the images
        self.path = '/home/yassin/ros_ws/src/ros2_RobotSimulation/ros2_imitation/images'

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info('Failed to convert ROS Image to OpenCV image: {}'.format(e))
            return
        
    # Function to save the current image
    def save_image(self):
        images = [name for name in os.listdir(self.path) if os.path.isfile(os.path.join(self.path, name))]
        n_images = len(images)
        # Save the image
        cv.imwrite(os.path.join(self.path, 'image_{}.png'.format(n_images)), self.current_image)

class SpawnObjectClient(Node):
    def __init__(self):
        super().__init__('spawn_objects_client')
        self.client = self.create_client(Empty, '/spawn_objects')

    def send_request(self):
        request = Empty.Request()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self._spawn_objects_response_callback)

    def _spawn_objects_response_callback(self, future):
        global RES
        response = future.result()
        if response is not None:
            self.get_logger().info('Objects spawned')
            RES = "done"
        else:
            self.get_logger().info('Service call failed')

class DeleteEntityClient(Node):
    def __init__(self):
        super().__init__('delete_entity_client')
        self.client = self.create_client(DeleteEntity, 'delete_entity')

    def send_request(self, entity_name):
        request = DeleteEntity.Request()
        request.name = entity_name
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self._delete_entity_response_callback)

    def _delete_entity_response_callback(self, future):
        global RES
        response = future.result()
        if response is not None:
            self.get_logger().info('Entity deleted')
            RES = "done"
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    global RES
    rclpy.init(args=args)

    image_saver = ImageSaver()
    spawn_object_client = SpawnObjectClient()
    delete_entity_client = DeleteEntityClient()

    # Create an executor to spin the nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(image_saver)
    executor.add_node(spawn_object_client)
    executor.add_node(delete_entity_client)

    # Spin the nodes in a separate thread
    thread = threading.Thread(target=executor.spin)
    thread.start()

    while rclpy.ok():
        if image_saver.current_image is not None:
            # Spawn objects
            spawn_object_client.send_request()
            while RES != "done":
                rclpy.spin_once(spawn_object_client)
            RES = "null"

            # Wait 2 seconds for the objects to be spawned
            time.sleep(2)

            # Save the image
            image_saver.save_image()

            # Delete the objects
            delete_entity_client.send_request("red_box")
            # Wait for the service to finish
            while rclpy.ok() and RES == "null":
                rclpy.spin_once(delete_entity_client)
            RES = "null"

            delete_entity_client.send_request("blue_box")
            # Wait for the service to finish
            while rclpy.ok() and RES == "null":
                rclpy.spin_once(delete_entity_client)
            RES = "null"

            delete_entity_client.send_request("green_box")
            # Wait for the service to finish
            while rclpy.ok() and RES == "null":
                rclpy.spin_once(delete_entity_client)
            RES = "null"

            # Wait 2 seconds for the objects to be deleted
            time.sleep(1)

    image_saver.destroy_node()
    spawn_object_client.destroy_node()
    delete_entity_client.destroy_node()
    rclpy.shutdown()

    # Wait for the image_saver thread to finish
    thread.join()

if __name__ == '__main__':
    main()