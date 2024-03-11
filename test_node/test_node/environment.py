import gymnasium
from gymnasium import spaces
import numpy as np

import rclpy
from rclpy.action import ActionClient
from pick_place_interface.action import PickPlace
from pick_place_interface.msg import DetectedObject, ListDetected
from gazebo_msgs import DeleteEntity
from std_srvs.srv import Empty

RES = "null"

# Final positions of the objects
RED_BOX_POSITION = [0.4, -0.3, 0.78]
BLUE_BOX_POSITION = [0.5, 0.3, 0.78]
GREEN_BOX_POSITION = [0.6, -0.3, 0.78]

# Threshold for the done condition
THRESHOLD = 0.01

# Action client for the pick_place action
class PickPlaceActionClient:
    def __init__(self):
        self.node = rclpy.create_node('pick_place_action_client')
        self.action_client = ActionClient(self.node, PickPlace, 'pick_place')

    def send_goal(self, object_name, start_position, goal_position):
        goal_msg = PickPlace.Goal()
        goal_msg.object_name = object_name
        goal_msg.start_position.x = start_position[0]
        goal_msg.start_position.y = start_position[1]
        goal_msg.start_position.z = start_position[2]
        goal_msg.end_position.x = goal_position[0]
        goal_msg.end_position.y = goal_position[1]
        goal_msg.end_position.z = goal_position[2]
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return
        self.node.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        global RES
        result = future.result().result
        RES = result
        self.node.get_logger().info('Result: {0}'.format(result))

# Service client to spawn the objects
class SpawnObjectClient:
    def __init__(self):
        self.node = rclpy.create_node('entity_spawner')
        self.client = self.node.create_client(Empty, '/spawn_objects')

    def send_request(self):
        request = Empty.Request()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self._spawn_objects_response_callback)

    def _spawn_objects_response_callback(self, future):
        global RES
        response = future.result()
        if response is not None:
            self.node.get_logger().info('Objects spawned')
            RES = "done"
        else:
            self.node.get_logger().info('Service call failed')

# Service client to delete the objects
class DeleteEntityClient:
    def __init__(self):
        self.node = rclpy.create_node('delete_entity_client')
        self.client = self.node.create_client(DeleteEntity, 'delete_entity')

    def send_request(self, entity_name):
        request = DeleteEntity.Request()
        request.name = entity_name
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self._delete_entity_response_callback)

    def _delete_entity_response_callback(self, future):
        global RES
        response = future.result()
        if response is not None:
            self.node.get_logger().info('Entity deleted')
            RES = "done"
        else:
            self.node.get_logger().info('Service call failed')

# Enivonment class
class PickPlaceEnv(gymnasium.Env):
    def __init__(self):
        # Action space: object name and 3D goal position
        self.action_space = spaces.Tuple((
            spaces.Discrete(3),  # 3 objects: "red_box", "blue_box", "green_box"
            spaces.Box(low=-1, high=1, shape=(3,))  # 3D goal position
        ))

        # Observation space: 3D position and color for each of the three objects
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,))

        # ROS2 node
        rclpy.init()
        self.node = rclpy.create_node('pick_place_env')
        # Action client
        self.action_client = PickPlaceActionClient()

        # Subscriber to the detected objects
        self.detected_objects = None
        self.detected_objects_sub = self.node.create_subscription(ListDetected, 'detected_objects', self.detected_objects_callback)

        # Service client to spawn the objects
        self.spawn_object_client = SpawnObjectClient()

        # Service client to delete the objects
        self.delete_entity_client = DeleteEntityClient()

    def step(self, action):
        global RES

        # Send action to the action server
        object_name = ["red_box", "blue_box", "green_box"][action[0]]
        goal_position = action[1]
        start_position = self.get_start_position(object_name)
        self.action_client.send_goal(object_name, start_position, goal_position)
        # Wait for the action server to finish
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self.node)
        RES = "null"
        
        # Update the observation
        observation = self.get_observation()

        # Compute the reward
        reward = self.get_reward()

        # Check if the done condition is met
        done = self.is_done()

        return observation, reward, done, {}

    def reset(self):
        # Delete the objects
        self.delete_entity_client.send_request("red_box")
        self.delete_entity_client.send_request("blue_box")
        self.delete_entity_client.send_request("green_box")
        # Wait for the service to finish
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self.node)
        RES = "null"

        # Clear the detected objects
        self.detected_objects = None
        
        # Spawn the objects in random positions
        self.spawn_object_client.send_request()
        # Wait for the service to finish
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self.node)
        RES = "null"

        return self.get_observation()

    def render(self, mode='human'):
        pass

    def close(self):
        # Destroy all the nodes
        self.action_client.node.destroy_node()
        self.delete_entity_client.node.destroy_node()
        self.spawn_object_client.node.destroy_node()
        self.node.destroy_node()


    def detected_objects_callback(self, msg):
        self.detected_objects = msg.list

    # Function to get the start position of an object
    def get_start_position(self, object_name):
        # For each detected object
        for obj in self.detected_objects:
            # If the object name is the one we are looking for
            if obj.name == object_name:
                # Return its position
                return [obj.position.x, obj.position.y, obj.position.z]
        return None
    
    # Function to get the observation
    def get_observation(self):
        observation = np.zeros(12)
        for i, obj in enumerate(self.detected_objects):
            observation[i*4] = obj.name
            observation[i*4+1: i*4+4] = [obj.x, obj.y, obj.z]
        return observation
    
    # Function to get the reward
    def get_reward(self):
        reward = 0
        for obj in self.detected_objects:
            if obj.name == "red_box":
                reward += np.linalg.norm([obj.x, obj.y, obj.z] - RED_BOX_POSITION)
            elif obj.name == "blue_box":
                reward += np.linalg.norm([obj.x, obj.y, obj.z] - BLUE_BOX_POSITION)
            elif obj.name == "green_box":
                reward += np.linalg.norm([obj.x, obj.y, obj.z] - GREEN_BOX_POSITION)
        return -reward
    
    # Function to compute the done condition
    def is_done(self):
        for obj in self.detected_objects:
            if obj.name == "red_box":
                if np.linalg.norm([obj.x, obj.y, obj.z] - RED_BOX_POSITION) > THRESHOLD:
                    return False
            elif obj.name == "blue_box":
                if np.linalg.norm([obj.x, obj.y, obj.z] - BLUE_BOX_POSITION) > THRESHOLD:
                    return False
            elif obj.name == "green_box":
                if np.linalg.norm([obj.x, obj.y, obj.z] - GREEN_BOX_POSITION) > THRESHOLD:
                    return False
        return True