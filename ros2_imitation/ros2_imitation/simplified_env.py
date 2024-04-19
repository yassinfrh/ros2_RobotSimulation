import gymnasium
from gymnasium import spaces
import numpy as np
import shutil

from stable_baselines3.common.vec_env import DummyVecEnv
from imitation.data.wrappers import RolloutInfoWrapper
from stable_baselines3.common.evaluation import evaluate_policy

from imitation.algorithms import bc
from imitation.algorithms.dagger import DAggerTrainer, ExponentialBetaSchedule, LinearBetaSchedule
from imitation.data import rollout
from gymnasium.utils.env_checker import check_env
from gymnasium.wrappers import TimeLimit

# Final positions of the objects
RED_BOX_POSITION = np.array([0.4, 0.3, 0.74])
GREEN_BOX_POSITION = np.array([0.5, 0.3, 0.74])
BLUE_BOX_POSITION = np.array([0.6, 0.3, 0.74])

# Limits of the initial positions of the objects
TABLE_X_MIN = 0.3
TABLE_X_MAX = 0.6
TABLE_Y_MIN = -0.4
TABLE_Y_MAX = -0.05

# Threshold for the done condition
THRESHOLD = 0.02


# Simplified Gym environment for training and testing
class SimplifiedEnv(gymnasium.Env):
    def __init__(self):
        # Define the lower and upper bounds for the action space
        low = np.array([0, 0, 0, 0.3, -0.4, 0.74])
        high = np.array([1, 1, 1, 0.6, 0.4, 0.74])
        # Action space: object name and 3D goal position
        self.action_space = spaces.Box(low=low, high=high, shape=(6,), dtype=np.float32)

        # Observation space: 3D position and color for each of the three objects
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,))
        
        # Initial state
        self._state = np.zeros(18, dtype=np.float32)
        # One-hot encoding for the color of the objects
        self._state[0] = 1.0
        self._state[7] = 1.0
        self._state[14] = 1.0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Spawn objects in random positions avoiding collisions
        self._state[3:6] = self.random_position()
        while True:
            self._state[9:12] = self.random_position()
            if np.linalg.norm(self._state[3:6] - self._state[9:12]) > 0.05:
                break
        while True:
            self._state[15:] = self.random_position()
            if np.linalg.norm(self._state[3:6] - self._state[15:]) > 0.05 and np.linalg.norm(self._state[9:12] - self._state[15:]) > 0.05:
                break

        return self._state, self.get_info()

    def step(self, action):
        # Check which object is being moved converting it to one-hot encoding
        moving_object = np.eye(3)[np.argmax(action[:3])]

        # Check the goal position
        goal_position = action[3:]

        # Update the state
        if np.array_equal(moving_object, np.array([1, 0, 0])):
            self._state[3:6] = goal_position
        elif np.array_equal(moving_object, np.array([0, 1, 0])):
            self._state[9:12] = goal_position
        elif np.array_equal(moving_object, np.array([0, 0, 1])):
            self._state[15:] = goal_position

        # Generate random number between 0 and 1
        random_number = np.random.uniform(0, 1)

        '''# If the random number is below 0.1, slightly move one of the objects to simulate collision
        if random_number < 0.1:
            random_object = np.random.randint(0, 3)
            if random_object == 0:
                self._state[3:5] += np.random.uniform(-0.05, 0.05, 2)
            elif random_object == 1:
                self._state[9:11] += np.random.uniform(-0.05, 0.05, 2)
            elif random_object == 2:
                self._state[15:17] += np.random.uniform(-0.05, 0.05, 2)'''

        # Get the info
        info = self.get_info()

        # Compute the reward
        reward = self.compute_reward(info)

        # Check if the done condition is met
        done = self.is_done(info)

        return self._state, reward, done, False, info


    def render(self, mode='human'):
        pass

    def close(self):
        pass

    # Function to generate the info
    def get_info(self):
        info = {}
        # Compute the distance between the final and current positions of the objects
        red_distance = np.linalg.norm(self._state[3:6] - RED_BOX_POSITION)
        green_distance = np.linalg.norm(self._state[9:12] - GREEN_BOX_POSITION)
        blue_distance = np.linalg.norm(self._state[15:] - BLUE_BOX_POSITION)

        # Store the info
        info['red_box_distance'] = red_distance
        info['green_box_distance'] = green_distance
        info['blue_box_distance'] = blue_distance

        return info
    
    # Function to compute the reward
    def compute_reward(self, info):
        # Check that all the distances are in the info
        if "red_box_distance" not in info or "green_box_distance" not in info or "blue_box_distance" not in info:
            return -100
        # Reward is the negative sum of the distances to the goal positions
        reward = info["red_box_distance"] + info["blue_box_distance"] + info["green_box_distance"]
        return -reward**2
    
    # Function to compute the done condition
    def is_done(self, info):
        # Check that all the distances are in the info
        if "red_box_distance" not in info or "green_box_distance" not in info or "blue_box_distance" not in info:
            return False
        # Done if all the distances are below the threshold
        return info["red_box_distance"] < THRESHOLD and info["blue_box_distance"] < THRESHOLD and info["green_box_distance"] < THRESHOLD


    # Function to randmoly generate the position of the objects
    def random_position(self):
        position = np.zeros(3)
        position[0] = np.random.uniform(TABLE_X_MIN, TABLE_X_MAX)
        position[1] = np.random.uniform(TABLE_Y_MIN, TABLE_Y_MAX)
        position[2] = 0.74
        return position
    

# Callable class for policy with observation as input and action as output
class Policy:
    def __call__(self, obs, state, dones):

        actions = []
        
        # For each observation
        for ob in obs:
            # If red object is not in the correct position
            if np.linalg.norm(ob[3:6] - RED_BOX_POSITION) > THRESHOLD:
                # Move red object to the correct position
                object_one_hot = np.array([1, 0, 0])
                goal_position = RED_BOX_POSITION
            # If green object is not in the correct position
            elif np.linalg.norm(ob[9:12] - GREEN_BOX_POSITION) > THRESHOLD:
                # Move green object to the correct position
                object_one_hot = np.array([0, 1, 0])
                goal_position = GREEN_BOX_POSITION
            # If blue object is not in the correct position
            elif np.linalg.norm(ob[15:] - BLUE_BOX_POSITION) > THRESHOLD:
                # Move blue object to the correct position
                object_one_hot = np.array([0, 0, 1])
                goal_position = BLUE_BOX_POSITION

            # Create the action
            action = np.concatenate([object_one_hot, goal_position])

            # Append the action to the list of actions
            actions.append(action)

        actions = np.array(actions)

        # If the size of action is 6, reshape it to (1, 6)
        if actions.size == 6:
            actions = actions.reshape(1, 6)
        
        return actions, state
    
# Function to create the environment
def _make_env():
    _env = SimplifiedEnv()
    _env = TimeLimit(_env, max_episode_steps=500)
    _env = RolloutInfoWrapper(_env)
    return _env

# Main function
def main():
    # Create the environment
    env = DummyVecEnv([_make_env for _ in range(4)])

    rng = np.random.default_rng(0)

    # Delete scratch directory if it exists
    shutil.rmtree("/home/yassin/ros_ws/src/ros2_RobotSimulation/ros2_imitation/scratch", ignore_errors=True)

    # Create BC trainer
    bc_trainer = bc.BC(
        observation_space=env.observation_space,
        action_space=env.action_space,
        rng=rng,
        device="cuda:0"
    )

    # Create the DAgger trainer
    dagger_trainer = DAggerTrainer(
        venv=env,
        scratch_dir="/home/yassin/ros_ws/src/ros2_RobotSimulation/ros2_imitation/scratch",
        rng=rng,
        bc_trainer=bc_trainer,
        beta_schedule=LinearBetaSchedule(8)
    )


    total_timesteps = 800
    total_timestep_count = 0
    rollout_round_min_timesteps = 200
    rollout_round_min_episodes = 3

    while total_timestep_count < total_timesteps:
        collector = dagger_trainer.create_trajectory_collector()

        sample_until = rollout.make_sample_until(
            min_timesteps=max(rollout_round_min_timesteps, dagger_trainer.batch_size),
            min_episodes=rollout_round_min_episodes,
        )

        trajectories = rollout.generate_trajectories(
            policy=Policy(),
            venv=collector,
            sample_until=sample_until,
            rng=collector.rng,
        )

        for traj in trajectories:
            total_timestep_count += len(traj)

        print("Total timesteps: ", total_timestep_count)

        # `logger.dump` is called inside BC.train within the following fn call:
        dagger_trainer.extend_and_update()

    # Save the policy
    dagger_trainer.save_trainer()

    # Evaluate the policy
    mean_reward, _ = evaluate_policy(dagger_trainer.policy, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward}")

if __name__ == "__main__":
    main()
    