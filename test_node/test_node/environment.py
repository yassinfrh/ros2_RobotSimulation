import gymnasium
from gymnasium import spaces
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from pick_place_interface.action import PickPlace
from pick_place_interface.msg import DetectedObject, ListDetected
from gazebo_msgs.srv import DeleteEntity
from std_srvs.srv import Empty

from gymnasium.wrappers import TimeLimit
from imitation.data import rollout
from imitation.data.wrappers import RolloutInfoWrapper
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
from stable_baselines3.ppo import MlpPolicy
from stable_baselines3.common.evaluation import evaluate_policy

RES = "null"

# Final positions of the objects
RED_BOX_POSITION = [0.4, -0.3, 0.78]
BLUE_BOX_POSITION = [0.5, 0.3, 0.78]
GREEN_BOX_POSITION = [0.6, -0.3, 0.78]

# Threshold for the done condition
THRESHOLD = 0.01

# Action client for the pick_place action
class PickPlaceActionClient(Node):
    def __init__(self):
        super().__init__('pick_place_action_client')
        self.action_client = ActionClient(self, PickPlace, 'pick_place')

    def send_goal(self, object_name, start_position, goal_position):
        goal_msg = PickPlace.Goal()
        goal_msg.object_name = object_name
        goal_msg.start_position_x = start_position[0]
        goal_msg.start_position_y = start_position[1]
        goal_msg.start_position_z = start_position[2]
        goal_msg.end_position_x = goal_position[0]
        goal_msg.end_position_y = goal_position[1]
        goal_msg.end_position_z = goal_position[2]
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        global RES
        result = future.result().result
        RES = result
        self.get_logger().info('Result: {0}'.format(result))

# Service client to spawn the objects
class SpawnObjectClient(Node):
    def __init__(self):
        super().__init__('spawn_object_client')
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

# Service client to delete the objects
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

# Enivonment class
class PickPlaceEnv(gymnasium.Env, Node):

    def __init__(self):
        super().__init__('pick_place_env')

        # Define the lower and upper bounds for the action space
        low = np.array([0, 0, 0, 0.3, -0.45, 0.78])
        high = np.array([1, 1, 1, 0.7, 0.45, 0.8])
        # Action space: object name and 3D goal position
        self.action_space = spaces.Box(low=low, high=high, shape=(6,), dtype=np.float32)

        # Observation space: 3D position and color for each of the three objects
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,))

        # Action client
        self.action_client = PickPlaceActionClient()

        # Subscriber to the detected objects
        self.detected_objects = None
        self.detected_objects_sub = self.create_subscription(ListDetected, 'detected_objects', self.detected_objects_callback, 10)

        # Service client to spawn the objects
        self.spawn_object_client = SpawnObjectClient()

        # Service client to delete the objects
        self.delete_entity_client = DeleteEntityClient()

    def step(self, action):
        global RES

        self.detected_objects = None

        # Spin until the detected objects are at least 3
        while self.detected_objects is None or len(self.detected_objects) < 3:
            rclpy.spin_once(self)

        # Send action to the action server
        object_index = np.argmax(action[:3])
        object_name = ["red_box", "blue_box", "green_box"][object_index]
        goal_position = np.round(np.array(action[3:]).astype(float), 3)
        start_position = np.array(self.get_start_position(object_name)).astype(float)
        print(f"Moving {object_name} from {start_position} to {goal_position}")
        self.action_client.send_goal(object_name, start_position, goal_position)
        # Wait for the action server to finish
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self.action_client)
        RES = "null"
        
        # Update the observation
        observation = self.get_observation()

        # Update the info
        info = self.get_info()

        # Compute the reward
        reward = self.get_reward(info)

        # Check if the done condition is met
        done = self.is_done(info)

        return observation, reward, done, False, info

    def reset(self, seed=None):
        global RES

        # Delete the objects
        self.delete_entity_client.send_request("red_box")
        self.delete_entity_client.send_request("blue_box")
        self.delete_entity_client.send_request("green_box")
        # Wait for the service to finish
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self.delete_entity_client)
        RES = "null"

        # Clear the detected objects
        self.detected_objects = None
        
        # Spawn the objects in random positions
        self.spawn_object_client.send_request()
        # Wait for the service to finish
        while rclpy.ok() and RES == "null":
            rclpy.spin_once(self.spawn_object_client)
        RES = "null"

        # Wait for the objects to be detected (3 elements in the list of detected objects)
        while self.detected_objects is None or len(self.detected_objects) < 3:
            rclpy.spin_once(self)

        return self.get_observation(), self.get_info()

    def render(self, mode='human'):
        pass

    def close(self):
        # Destroy all the nodes
        self.action_client.destroy_node()
        self.delete_entity_client.destroy_node()
        self.spawn_object_client.destroy_node()
        self.destroy_node()


    def detected_objects_callback(self, msg):
        self.detected_objects = msg.list

    # Function to get the start position of an object
    def get_start_position(self, object_name):
        # For each detected object
        for obj in self.detected_objects:
            # If the object name is the one we are looking for
            if obj.name == object_name:
                # Return its position
                return [obj.x, obj.y, obj.z]
        return None
    
    # Function to get the observation
    def get_observation(self):
        observation = np.zeros(18)
        for obj in self.detected_objects:
            if obj.name == "red_box":
                observation[:3] = [1, 0, 0]
                observation[3:6] = [obj.x, obj.y, obj.z]
            elif obj.name == "blue_box":
                observation[6:9] = [0, 0, 1]
                observation[9:12] = [obj.x, obj.y, obj.z]
            elif obj.name == "green_box":
                observation[12:15] = [0, 1, 0]
                observation[15:] = [obj.x, obj.y, obj.z]
        return observation
            
    
    # Function to get the reward
    def get_reward(self, info):
        # Reward is the negative sum of the distances to the goal positions
        reward = info["red_box_distance"] + info["blue_box_distance"] + info["green_box_distance"]
        return -reward
    
    # Function to compute the done condition
    def is_done(self, info):
        # Done if all the distances are below the threshold
        return info["red_box_distance"] < THRESHOLD and info["blue_box_distance"] < THRESHOLD and info["green_box_distance"] < THRESHOLD
    
    # Function to get the info
    def get_info(self):
        info = {}
        for obj in self.detected_objects:
            if obj.name == "red_box":
                info["red_box_distance"] = np.linalg.norm(np.array([obj.x, obj.y, obj.z]) - np.array(RED_BOX_POSITION))
            elif obj.name == "blue_box":
                info["blue_box_distance"] = np.linalg.norm(np.array([obj.x, obj.y, obj.z]) - np.array(BLUE_BOX_POSITION))
            elif obj.name == "green_box":
                info["green_box_distance"] = np.linalg.norm(np.array([obj.x, obj.y, obj.z]) - np.array(GREEN_BOX_POSITION))
        return info
    
# Main function
def main():
    rclpy.init()

    # Create the environment
    env = PickPlaceEnv()
    env = TimeLimit(env, max_episode_steps=50)

    # Create the expert
    expert = PPO(
        policy=MlpPolicy,
        env=env,
        seed=0,
        batch_size=64,
        ent_coef=0.0,
        learning_rate=3e-4,
        n_epochs=10,
        n_steps=64,
    )

    # Reward before training
    reward, _ = evaluate_policy(expert, env, 10)
    print(f"Reward before training: {reward}")

    # Train the expert
    expert.learn(20)

    # Reward after training
    reward, _ = evaluate_policy(expert, expert.get_env(), 10)
    print(f"Reward after training: {reward}")

    # Close the environment
    env.close()


if __name__ == "__main__":
    main()