import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from scipy.spatial.transform import Rotation
import numpy

from ros2_data.action import MoveXYZ, MoveL, MoveXYZW, MoveG

from linkattacher_msgs.srv import AttachLink, DetachLink

from pick_place_interface.action import PickPlace

# Global variables:
RES = "null"
HAND_TO_EE = 0.103
EE_TO_BOX = 0.15
R_tool_to_EE = Rotation.from_euler('XYZ', [90, 45, 90], degrees=True)
robot_model = "panda"
ee_link = "end_effector_frame"
initial_position = numpy.array([0.3, 0.0, 1.0])

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

class PickPlaceActionServer(Node):
    
        def __init__(self):
            super().__init__('pick_place_action_server')
            self._action_server = ActionServer(
                self,
                PickPlace,
                'pick_place',
                self.execute_callback
            )

            self.XYZclient = MoveXYZclient()
            self.Lclient = MoveLclient()
            self.XYZWclient = MoveXYZWclient()
            self.attacher_control = AttacherClient()
            self.detacher_control = DetacherClient()
            self.Gclient = MoveGclient()
    
        def execute_callback(self, goal_handle):
            global RES

            feedback = PickPlace.Feedback()

            # Extract the goal parameters
            goal = goal_handle.request
            object_name = goal.object_name
            start_position = numpy.array([goal.start_position_x, goal.start_position_y, goal.start_position_z])
            end_position = numpy.array([goal.end_position_x, goal.end_position_y, goal.end_position_z])

            # Open the gripper
            self.Gclient.send_goal(0.04)
            # Update feedback
            feedback.stage = "Opening the gripper"
            goal_handle.publish_feedback(feedback)
            # Wait for the gripper to open
            while rclpy.ok():
                        rclpy.spin_once(self.Gclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Move the robot to the start position above the object
            R_base_to_EE = Rotation.from_euler('ZYX', [0, 90, 0], degrees=True) # Desired orientation of the end effector
            R_base_to_tool = R_base_to_EE * R_tool_to_EE.inv()  # Compute orientation of tool so that the end effector has the desired orientation
            yaw, pitch, roll = R_base_to_tool.as_euler('ZYX', degrees=True) # Convert orientation to Euler angles
            self.XYZWclient.send_goal(start_position[0], start_position[1], start_position[2] + EE_TO_BOX + HAND_TO_EE, yaw, pitch, roll, 1.0) # Move the robot to the desired position and orientation
            # Update feedback 
            feedback.stage = "Moving robot above the object"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.XYZWclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Move the robot down to the object
            self.Lclient.send_goal(0.0, 0.0, -EE_TO_BOX, 1.0)
            # Update feedback
            feedback.stage = "Moving robot down to the object"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.Lclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Attach the box to the robot's gripper
            self.attacher_control.call(robot_model, ee_link, object_name, object_name)
            # Update feedback
            feedback.stage = "Attaching the object to the robot"
            goal_handle.publish_feedback(feedback)
            # Wait for the box to be attached
            rclpy.spin_once(self.attacher_control)

            # Move the robot up
            self.Lclient.send_goal(0.0, 0.0, EE_TO_BOX, 1.0)
            # Update feedback
            feedback.stage = "Moving robot up"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.Lclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Move the robot to the new position
            self.XYZclient.send_goal(end_position[0], end_position[1], end_position[2] + EE_TO_BOX + HAND_TO_EE, 1.0)
            # Update feedback
            feedback.stage = "Moving robot to the new position"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.XYZclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Move the robot down
            self.Lclient.send_goal(0.0, 0.0, -EE_TO_BOX, 1.0)
            # Update feedback
            feedback.stage = "Moving robot down"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.Lclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Detach the box from the robot's gripper
            self.detacher_control.call(robot_model, ee_link, object_name, object_name)
            # Update feedback
            feedback.stage = "Detaching the object from the robot"
            goal_handle.publish_feedback(feedback)
            # Wait for the box to be detached
            rclpy.spin_once(self.detacher_control)

            # Move the robot up
            self.Lclient.send_goal(0.0, 0.0, EE_TO_BOX, 1.0)
            # Update feedback
            feedback.stage = "Moving robot up"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.Lclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Move the robot to the initial position
            self.XYZclient.send_goal(initial_position[0] - HAND_TO_EE, initial_position[1], initial_position[2], 1.0)
            # Update feedback
            feedback.stage = "Moving robot to the initial position"
            goal_handle.publish_feedback(feedback)
            # Wait for the robot to move
            while rclpy.ok():
                        rclpy.spin_once(self.XYZclient)
                        if (RES != "null"):
                            break
            RES = "null"

            # Finish the action
            goal_handle.succeed()
            result = PickPlace.Result()
            result.success = True
            return result
        
def main(args=None):
    rclpy.init(args=args)
    pick_place_action_server = PickPlaceActionServer()
    rclpy.spin(pick_place_action_server)
    pick_place_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()