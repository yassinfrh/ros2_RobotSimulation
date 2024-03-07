import gymnasium
from gymnasium import spaces
import numpy as np

import rclpy
from rclpy.action import ActionClient
from pick_place_interface.action import PickPlace


# Enivonment class
class PickPlaceEnv(gymnasium.Env):
    def __init__(self):
        # Action space: object name and 2D goal position
        self.action_space = spaces.Tuple((
            spaces.Discrete(3),  # 3 objects: "red_box", "blue_box", "green_box"
            spaces.Box(low=-1, high=1, shape=(2,))  # 2D goal position
        ))

        # Observation space: 2D position and color for each of the three objects
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,))

        # ROS2 node
        rclpy.init()
        self.node = rclpy.create_node('pick_place_env')
        # Action client
        self.action_client = ActionClient(self.node, PickPlace, 'pick_place')

    def step(self, action):
        # Send action to the action server
        goal_msg = PickPlace.Goal()
        goal_msg.object_name = ["red_box", "blue_box", "green_box"][action[0]]
        goal_msg.goal_position.x = action[1][0]
        goal_msg.goal_position.y = action[1][1]
        self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, self.action_client.goal_future)

        # TODO: update observation, reward, done
        observation = np.zeros(9)
        reward = 0
        done = False

        return observation, reward, done, {}

    def reset(self):
        pass

    def render(self, mode='human'):
        pass

    def close(self):
        pass