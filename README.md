# ROS2 Robot Simulation

This repository contains a Gazebo and ROS2 Humble simulation of a Franka Panda robotic arm performing pick-and-place tasks. The robotic arm is trained using the Imitation Learning (DAGGER algorithm) and utilizes computer vision to detect and arrange objects on a table.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Project Structure](#project-structure)
- [Acknowledgements](#acknowledgements)


## Installation

1. **Clone the repository:**
    ```sh
    git clone https://github.com/yassinfrh/ros2_RobotSimulation.git
    cd ros2_RobotSimulation
    ```

2. **Install dependencies:**
    Follow the instructions for installing ROS2 Humble and other dependencies as described in [ros2_RobotSimulation](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/humble).

3. **Build the packages:**
    ```sh
    colcon build
    ```

## Usage

1. **Source the ROS2 environment:**
    ```sh
    source /opt/ros/humble/setup.bash
    ```

2. **Launch the simulation:**
    ```sh
    ros2 launch panda_ros2_moveit2 panda_interface.launch.py
    ```

3. **Running the trained policy:**
    ```sh
    ros2 launch ros2_imitation policy_test.launch.py
    ```

## Features

- **Gazebo Integration:** Seamless integration with Gazebo for robot simulation.
- **MoveIt!2 Support:** Advanced motion planning and manipulation capabilities using MoveIt!2.
- **Multiple Robots:** Simulation packages for different industrial and collaborative robots.

## Project Structure

- **IFRA_LinkAttacher:** Scripts and configurations for attaching links in simulations.
- **PandaRobot:** Simulation files specific to the Panda robot.
- **include:** Common headers and utilities.
- **pick_place_interface:** Interface scripts for pick and place operations.
- **ros2_actions:** ROS2 action server and client implementations.
- **ros2_data:** Data handling scripts and configurations.
- **ros2_imitation:** Imitation learning setups and scripts.

## Acknowledgements

This project utilizes code and concepts from the following repositories and libraries:
- [ros2_RobotSimulation by IFRA-Cranfield](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/humble)
- [IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)
- [Imitation Learning Documentation](https://imitation.readthedocs.io/en/latest/#)
