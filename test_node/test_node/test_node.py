import rclpy
import xacro
import os
import time
from scipy.spatial.transform import Rotation

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient
from ros2_data.action import MoveXYZ
from ros2_data.action import MoveL
from ros2_data.action import MoveXYZW
from gazebo_msgs.srv import SpawnEntity
from linkattacher_msgs.srv import AttachLink, DetachLink

# Global variables:
RES = "null"
HAND_TO_EE = 0.17047
EE_TO_BOX = 0.15
R_tool_to_EE = Rotation.from_euler('ZYX', [45, 0, 0], degrees=True)
robot_model = "irb120"
ee_link = "EE_egp64"

# Nodes
spawn_object = None
XYZclient = None
Lclient = None
XYZWclient = None
gripper_control = None
attacher_control = None
detacher_control = None



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

# Class to attach the object to the robot's gripper:
class AttacherClient(Node):
    
    def __init__(self):
        # 1. Initialise node:
        super().__init__('Attacher_client')
        # 2. Declare SERVICE CLIENT:
        self.client = self.create_client(AttachLink, 'ATTACHLINK')
        # 3. Wait for ATTACHLINK service to be available:
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AttachLink.Request()

    def call(self, model1, link1, model2, link2):
        # 1. Assign variables:
        self.req.model1_name = model1
        self.req.link1_name = link1
        self.req.model2_name = model2
        self.req.link2_name = link2
        # 2. SERVICE CALL:
        self.future = self.client.call_async(self.req)    

# Class to detach the object from the robot's gripper:
class DetacherClient(Node):
    
    def __init__(self):
        # Declare NODE:
        super().__init__("Detacher_client")
        # Declare SERVICE CLIENT:
        self.client = self.create_client(DetachLink, 'DETACHLINK')
        # Wait for DETACHLINK service to be available:
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DetachLink.Request()

    def call(self, model1, link1, model2, link2):
        # 1. Assign variables:
        self.req.model1_name = model1
        self.req.link1_name = link1
        self.req.model2_name = model2
        self.req.link2_name = link2
        # 2. SERVICE CALL:
        self.future = self.client.call_async(self.req)

# Function to move object from one position to another:
def move_object(start_position, goal_position, object, object_link):
    global RES

    # Extract coordinates from start_position and goal_position
    x1, y1, z1 = start_position
    x2, y2, z2 = goal_position

    # Move the robot above the box
    R_base_to_EE = Rotation.from_euler('ZYX', [90, 0, 90], degrees=True) # Desired orientation of the end effector
    R_base_to_tool = R_base_to_EE * R_tool_to_EE.inv()  # Compute orientation of tool so that the end effector has the desired orientation
    yaw, pitch, roll = R_base_to_tool.as_euler('ZYX', degrees=True) # Convert orientation to Euler angles
    XYZWclient.send_goal(x1 - HAND_TO_EE, y1, z1 + EE_TO_BOX, yaw, pitch, roll, 1.0) # Move the robot to the desired position and orientation
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(XYZWclient)
                if (RES != "null"):
                    break
    RES = "null"
    
    # Move the robot down to the box
    Lclient.send_goal(0.0, 0.0, -EE_TO_BOX, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(Lclient)
                if (RES != "null"):
                    break
    print(RES)
    RES = "null"
    # Attach the box to the robot's gripper
    attacher_control.call(robot_model, ee_link, object, object_link)
    # Wait for the box to be attached
    rclpy.spin_once(attacher_control)
    # Move the robot up
    Lclient.send_goal(0.0, 0.0, EE_TO_BOX, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(Lclient)
                if (RES != "null"):
                    break
    RES = "null"
    # Move the robot to the new position
    XYZclient.send_goal(x2 - HAND_TO_EE, y2, z2 + EE_TO_BOX + 0.1, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(XYZclient)
                if (RES != "null"):
                    break
    RES = "null"
    # Move the robot down
    Lclient.send_goal(0.0, 0.0, -EE_TO_BOX, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(Lclient)
                if (RES != "null"):
                    break
    RES = "null"
    # Detach the box from the robot's gripper
    detacher_control.call(robot_model, ee_link, object, object_link)
    # Wait for the box to be detached
    rclpy.spin_once(detacher_control)
    # Move the robot up
    Lclient.send_goal(0.0, 0.0, EE_TO_BOX, 1.0)
    # Wait for the robot to move
    while rclpy.ok():
                rclpy.spin_once(Lclient)
                if (RES != "null"):
                    break
    RES = "null"



# Main function:
def main(args=None):
    global RES, spawn_object, XYZclient, Lclient, XYZWclient, attacher_control, detacher_control

    rclpy.init(args=args)

    spawn_object = SpawnObject()
    XYZclient = MoveXYZclient()
    Lclient = MoveLclient()
    XYZWclient = MoveXYZWclient()
    attacher_control = AttacherClient()
    detacher_control = DetacherClient()

    # Position of the box
    box_position = [0.6, -0.3, 0.75]


    # Spawn the box
    spawn_object.spawn_object('ros2_grasping', 'red_box.urdf', 'red_box', box_position[0], box_position[1], box_position[2])
    # Wait for the object to spawn
    rclpy.spin_once(spawn_object)

    # Goal position
    goal_position = [0.6, 0.3, 0.75]
    
    # Move the object from one position to another
    move_object(box_position, goal_position, 'red_box', 'red_box')

    
    rclpy.shutdown()
