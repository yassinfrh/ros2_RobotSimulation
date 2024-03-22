import os
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from std_srvs.srv import Empty
import rclpy
import random
from scipy.spatial.transform import Rotation

# Limits of the table
TABLE_X_MIN = 0.3
TABLE_X_MAX = 0.65
TABLE_Y_MIN = -0.4
TABLE_Y_MAX = -0.05

# Class to spawn objects into the Gazebo world
class SpawnObject:
    def __init__(self):
        # Start node:
        rclpy.init()
        self.node = rclpy.create_node('entity_spawner')

        self.node.get_logger().info(
            'Creating Service client to connect to `/spawn_entity`')
        self.client = self.node.create_client(SpawnEntity, '/spawn_entity')

        self.node.get_logger().info('Connecting to `/spawn_entity` service...')
        if not self.client.service_is_ready():
            self.client.wait_for_service()
            self.node.get_logger().info('...connected!')

        self.node.get_logger().info('Creating Service server for `/spawn_objects`')
        self.server = self.node.create_service(Empty, '/spawn_objects', self.spawn_objects_callback)

        self.red_request = SpawnEntity.Request()
        self.red_request.name = 'red_box'
        self.red_request.xml = self.get_urdf('red_box.urdf')
        self.red_request.initial_pose.position.z = 0.73

        self.green_request = SpawnEntity.Request()
        self.green_request.name = 'green_box'
        self.green_request.xml = self.get_urdf('green_box.urdf')
        self.green_request.initial_pose.position.z = 0.73

        self.blue_request = SpawnEntity.Request()
        self.blue_request.name = 'blue_box'
        self.blue_request.xml = self.get_urdf('blue_box.urdf')
        self.blue_request.initial_pose.position.z = 0.73

    def get_urdf(self, urdf_file):
        urdf_file_path = os.path.join(get_package_share_directory('ros2_grasping'), 'urdf', urdf_file)
        xacro_file = xacro.process_file(urdf_file_path)
        return xacro_file.toxml()
    
    '''
    def random_orientation(self):
        # Compute random angle for the object
        angle = random.uniform(0, 2*3.1415)

        # Create rotation around the z-axis
        rotation = Rotation.from_euler('z', angle)

        # Convert the rotation to quaternion
        return rotation.as_quat()
    '''
    

    def spawn_objects_callback(self, request, response):
        # Compute random position for the object
        self.red_request.initial_pose.position.x = random.uniform(TABLE_X_MIN, TABLE_X_MAX)
        self.red_request.initial_pose.position.y = random.uniform(TABLE_Y_MIN, TABLE_Y_MAX)
        # Compute random orientation for the object
        '''orientation = self.random_orientation()
        self.red_request.initial_pose.orientation.z = orientation[2]
        self.red_request.initial_pose.orientation.w = orientation[3]'''

        # Compute random position for the object (avoiding collisions)
        while True:
            self.green_request.initial_pose.position.x = random.uniform(TABLE_X_MIN, TABLE_X_MAX)
            self.green_request.initial_pose.position.y = random.uniform(TABLE_Y_MIN, TABLE_Y_MAX)
            if (self.green_request.initial_pose.position.x - self.red_request.initial_pose.position.x)**2 + (self.green_request.initial_pose.position.y - self.red_request.initial_pose.position.y)**2 > 0.05:
                break

        # Compute random orientation for the object
        '''orientation = self.random_orientation()
        self.green_request.initial_pose.orientation.z = orientation[2]
        self.green_request.initial_pose.orientation.w = orientation[3]'''

        # Compute random position for the object (avoiding collisions)
        while True:
            self.blue_request.initial_pose.position.x = random.uniform(TABLE_X_MIN, TABLE_X_MAX)
            self.blue_request.initial_pose.position.y = random.uniform(TABLE_Y_MIN, TABLE_Y_MAX)
            if (self.blue_request.initial_pose.position.x - self.red_request.initial_pose.position.x)**2 + (self.blue_request.initial_pose.position.y - self.red_request.initial_pose.position.y)**2 > 0.05 and (self.blue_request.initial_pose.position.x - self.green_request.initial_pose.position.x)**2 + (self.blue_request.initial_pose.position.y - self.green_request.initial_pose.position.y)**2 > 0.05:
                break
        
        # Compute random orientation for the object
        '''orientation = self.random_orientation()
        self.blue_request.initial_pose.orientation.z = orientation[2]
        self.blue_request.initial_pose.orientation.w = orientation[3]'''

        # Spawn the objects
        self.node.get_logger().info('Spawning red box...')
        self.client.call_async(self.red_request)
        self.node.get_logger().info('Spawning green box...')
        self.client.call_async(self.green_request)
        self.node.get_logger().info('Spawning blue box...')
        self.client.call_async(self.blue_request)
        return response



def main(args=None):
    spawn_object = SpawnObject()
    rclpy.spin(spawn_object.node)

    spawn_object.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        