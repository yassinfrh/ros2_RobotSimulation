import rclpy
import xacro
import os
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from ros2_data.action import MoveXYZ
from ros2_data.action import MoveG
from ros2_data.action import MoveL
from ros2_data.action import MoveXYZW
from ros2_grasping.action import Attacher
from gazebo_msgs.srv import SpawnEntity

RES = "null"
HAND_TO_EE = 0.17047
EE_TO_BOX = 0.15

R_tool_to_EE = Rotation.from_euler('ZYX', [45, 0, 0], degrees=True)


# Class to spawn an object in the Gazebo world:
class SpawnObject(Node):
    def __init__(self):
        super().__init__('spawn_object')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def spawn_object(self, package, urdf, name, x, y, z):
        self.req.name = name
        urdf_file_path = os.path.join(get_package_share_directory(package), 'urdf', urdf)
        xacro_file = xacro.process_file(urdf_file_path)
        self.req.xml = xacro_file.toxml()
        self.req.initial_pose.position.x = float(x)
        self.req.initial_pose.position.y = float(y)
        self.req.initial_pose.position.z = float(z)
        self.future = self.client.call_async(self.req)

# Class to move the robot to a specific position:
class MoveXYZclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveXYZ_client')
        self._action_client = ActionClient(self, MoveXYZ, '/MoveXYZ')
        # 2. Wait for MoveXYZ server to be available:
        print ("Waiting for MoveXYZ action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveXYZ ACTION SERVER detected.")
    
    def send_goal(self, GoalXYZx, GoalXYZy, GoalXYZz, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveXYZ.Goal()
        goal_msg.positionx = GoalXYZx
        goal_msg.positiony = GoalXYZy
        goal_msg.positionz = GoalXYZz
        goal_msg.speed = JointSPEED        
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveXYZ ACTION CALL finished.")    

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveXYZ ACTION CALL.

# Class to move the robot to a specific position with specific orientation:
class MoveXYZWclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveXYZW_client')
        self._action_client = ActionClient(self, MoveXYZW, 'MoveXYZW')
        # 2. Wait for MoveXYZW server to be available:
        print ("Waiting for MoveXYZW action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveXYZW ACTION SERVER detected.")
    
    def send_goal(self, GoalXYZWx, GoalXYZWy, GoalXYZWz, GoalXYZWyaw, GoalXYZWpitch, GoalXYZWroll, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveXYZW.Goal()
        goal_msg.positionx = GoalXYZWx
        goal_msg.positiony = GoalXYZWy
        goal_msg.positionz = GoalXYZWz
        goal_msg.yaw = GoalXYZWyaw
        goal_msg.pitch = GoalXYZWpitch
        goal_msg.roll = GoalXYZWroll
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveXYZW ACTION CALL finished.")     

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveXYZW ACTION CALL.

# Class to move the robot linearly:
class MoveLclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveL_client')
        self._action_client = ActionClient(self, MoveL, 'MoveL')
        # 2. Wait for MoveL server to be available:
        print ("Waiting for MoveL action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveL ACTION SERVER detected.")
    
    def send_goal(self, GoalLx, GoalLy, GoalLz, JointSPEED):
        # 1. Assign variables:
        goal_msg = MoveL.Goal()
        goal_msg.movex = GoalLx
        goal_msg.movey = GoalLy
        goal_msg.movez = GoalLz
        goal_msg.speed = JointSPEED
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveL ACTION CALL finished.")       

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveL ACTION CALL.

# Class to open and close the gripper:
class MoveGclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('MoveG_client')
        self._action_client = ActionClient(self, MoveG, 'MoveG')
        # 2. Wait for MoveG server to be available:
        print ("Waiting for MoveG action server to be available...")
        self._action_client.wait_for_server()
        print ("MoveG ACTION SERVER detected.")
    
    def send_goal(self, GP):
        # 1. Assign variables:
        goal_msg = MoveG.Goal()
        goal_msg.goal = GP
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        global RES
        # 1. Assign RESULT variable:
        result = future.result().result
        RES = result.result
        # 2. Print RESULT:
        print ("MoveG ACTION CALL finished.")

    def feedback_callback(self, feedback_msg):
        # 1. Assign FEEDBACK variable:
        feedback = feedback_msg.feedback
        # NO FEEDBACK NEEDED IN MoveG ACTION CALL.

# Class to attach the object to the robot's gripper:
class ATTACHERclient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('Attacher_client')
        self._action_client = ActionClient(self, Attacher, 'Attacher')
        # 2. Wait for ATTACHER server to be available:
        print ("Waiting for ATTACHER action server to be available...")
        self._action_client.wait_for_server()
        print ("Attacher ACTION SERVER detected.")
    
    def send_goal(self, object, endeffector):
        # 1. Assign variables:
        goal_msg = Attacher.Goal()
        goal_msg.object = object
        goal_msg.endeffector = endeffector
        # 2. ACTION CALL:
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

# Class to detach the object from the robot's gripper:
class DetacherPUB(Node):
    
    def __init__(self):
        # Declare NODE:
        super().__init__("ros2_PUBLISHER")
        # Declare PUBLISHER:
        self.publisher_ = self.create_publisher(String, "ros2_Detach", 5) #(msgType, TopicName, QueueSize)

# Function to move object from one position to another:
def move_object(x1, y1, z1, x2, y2, z2, object, move_robot, move_robotL, move_robotW, gripper_control, attacher_control, detach_publisher):
    global RES

    # Move the robot above the box
    R_base_to_EE = Rotation.from_euler('ZYX', [90, 0, 90], degrees=True)
    R_base_to_tool = R_base_to_EE * R_tool_to_EE.inv()
    yaw, pitch, roll = R_base_to_tool.as_euler('ZYX', degrees=True)
    move_robotW.send_goal(x1 - HAND_TO_EE, y1, z1 + EE_TO_BOX, yaw, pitch, roll, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(move_robotW)
                if (RES != "null"):
                    break
    RES = "null"
    
    # Open the gripper
    gripper_control.send_goal(0.0)
    # Wait for the gripper to open
    while rclpy.ok():
                rclpy.spin_once(gripper_control)
                if (RES != "null"):
                    break
    RES = "null"
    # Move the robot down to the box
    move_robotL.send_goal(0.0, 0.0, -EE_TO_BOX, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(move_robotL)
                if (RES != "null"):
                    break
    print(RES)
    RES = "null"
    # Close the gripper
    gripper_control.send_goal(0.009)
    # Wait for the gripper to close
    while rclpy.ok():
                rclpy.spin_once(gripper_control)
                if (RES != "null"):
                    break
    RES = "null"
    # Attach the box to the robot's gripper
    attacher_control.send_goal(object, 'EE_egp64')
    # Wait for the box to be attached
    rclpy.spin_once(attacher_control)
    # Move the robot up
    move_robotL.send_goal(0.0, 0.0, EE_TO_BOX + 0.2, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(move_robotL)
                if (RES != "null"):
                    break
    RES = "null"
    # Move the robot to the new position
    move_robotL.send_goal(x2-x1, y2-y1, z2-z1, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(move_robotL)
                if (RES != "null"):
                    break
    RES = "null"
    # Move the robot down
    move_robotL.send_goal(0.0, 0.0, -EE_TO_BOX - 0.2, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(move_robotL)
                if (RES != "null"):
                    break
    RES = "null"
    # Detach the box from the robot's gripper
    msg = String()
    msg.data = "True"
    t_end = time.time() + 1
    while time.time() < t_end:
        detach_publisher.publisher_.publish(msg)
    # Open the gripper
    gripper_control.send_goal(0.0)
    # Wait for the gripper to open
    while rclpy.ok():
                rclpy.spin_once(gripper_control)
                if (RES != "null"):
                    break
    RES = "null"
    # Move the robot up
    move_robotL.send_goal(0.0, 0.0, EE_TO_BOX, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(move_robotL)
                if (RES != "null"):
                    break
    RES = "null"



# Main function:
def main(args=None):
    global RES

    rclpy.init(args=args)

    spawn_object = SpawnObject()
    move_robot = MoveXYZclient()
    move_robotL = MoveLclient()
    move_robotW = MoveXYZWclient()
    gripper_control = MoveGclient()
    attacher_control = ATTACHERclient()
    detach_publisher = DetacherPUB()


    # Spawn the box
    spawn_object.spawn_object('ros2_grasping', 'box.urdf', 'box', 0.5, -0.3, 0.75)
    # Wait for the object to spawn
    rclpy.spin_once(spawn_object)
    
    # Move the object from one position to another
    move_object(0.5, -0.3, 0.8, 0.5, 0.3, 0.8, 'box', move_robot, move_robotL, move_robotW, gripper_control, attacher_control, detach_publisher)

    
    rclpy.shutdown()
