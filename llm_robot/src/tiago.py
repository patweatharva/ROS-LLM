#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String, Int64
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from llm_interfaces.srv import ChatGPTs, ChatGPTsResponse
from move_base_msgs.msg import MoveBaseActionResult
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, GoalStatus
import json
import threading

ARUCO_IDS = {582: ["apple"], 582: ["banana"], 582: ["book"]}


# Global Initialization
from llm_config.user_config import UserConfig

def get_status_string(status_code):
    return GoalStatus.to_string(status_code)

config = UserConfig()

class TiagoRobot:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        self.go_to_point = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.go_to_point_result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.goal_status_callback)
        self.client = SimpleActionClient('/play_motion', PlayMotionAction)
        self.feedback_pub = rospy.Publisher("llm_feedback", String, queue_size=10)
        self.llm_feedback_to_user_publisher = rospy.Publisher("/llm_feedback_to_user", String, queue_size=1)
        rospy.loginfo("Waiting for Action Server...")
        self.client.wait_for_server()

        self.marker_set = rospy.Publisher("/set_marker", Int64, queue_size=10)
        self.pick_gui_service = rospy.ServiceProxy("/pick_gui", Empty)
        self.place_gui_service = rospy.ServiceProxy("/place_gui", Empty)
        self.pick_place_feedback = rospy.Subscriber("/pick_place_status", String, self.pick_place_status_callback)

        self.goal_status = GoalStatus.SUCCEEDED

        self.instructions = []
        self.lock = threading.Lock()
        self.current_function = None
        self.function_call_server = rospy.Service("/ChatGPT_function_call_service", ChatGPTs, self.function_call_callback)
        self.rate = rospy.Timer(rospy.Duration(1), self.sequence_looping)
        self.last_pick_place_status = None
    
    def pick_place_status_callback(self, msg):
        
        if msg.data == "IDLE" and (self.last_pick_place_status == "PICKING" or self.last_pick_place_status == "PLACING"):
            
            if self.last_pick_place_status == "PICKING":
                feedback = "I succeeded in picking the object."
            else:
                feedback = "I have placed the object."

            self.publish_string(feedback, self.llm_feedback_to_user_publisher)
            self.goal_status = GoalStatus.SUCCEEDED
        # HOW TO KNOW WHEN IT FAILED???

        self.last_pick_place_status = msg.data


    def publish_status(self, function_name, status):
        feedback = json.dumps({
            "function": function_name,
            "status": status
        })
        self.feedback_pub.publish(feedback)

    def function_call_callback(self, req):
        print(f"Received request: {req.request_text}")
        try:
            req_data = json.loads(req.request_text)
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse JSON: {e}")
            response = ChatGPTsResponse()
            response.response_text = f"Invalid JSON format: {e}"
            return response

        with self.lock:
            self.instructions = self.sort_instructions(req_data)

        response = ChatGPTsResponse()
        response.response_text = "Functions have been received"
        return response
    
    def sequence_looping(self, event):
        if hasattr(self, "goal_status") and self.current_function is not None:
            if self.goal_status == GoalStatus.ABORTED:
                feedback = self.current_function + " execution failed"
                rospy.loginfo(feedback)
                self.publish_status(self.current_function, "failed")
                # self.publish_string(feedback, self.llm_feedback_to_user_publisher)
                self.current_function = None
                return
            elif self.goal_status == GoalStatus.SUCCEEDED:
                feedback = self.current_function + " executed successfully"
                rospy.loginfo(feedback)
                self.publish_status(self.current_function, "succeeded")
                # self.publish_string(feedback, self.llm_feedback_to_user_publisher)
                self.current_function = None
            else:
                rospy.loginfo("Waiting for current function to complete...")
                self.publish_status(self.current_function, "ongoing")
                return
        
        with self.lock:
            if not hasattr(self, "instructions") or not self.instructions:
                rospy.loginfo("No instructions to process.")
                return


            # Get the next instruction
            next_instruction = self.instructions.pop(0)

        function_name = next_instruction.get("type")
        self.current_function = function_name
        args = next_instruction
        args.pop("type", None)
        args.pop("sequence_index", None)

        if hasattr(self, function_name):
            func_obj = getattr(self, function_name)

            try:
                self.publish_status(function_name, "in_progress")
                rospy.loginfo(f"Executing {function_name} with args: {args}")
                func_obj(**args)
                rospy.loginfo(f"{function_name} execution started.")
                self.publish_status(function_name, "succeeded")
            except TypeError as e:
                rospy.logerr(f"Error calling {function_name}: {e}")
                self.publish_status(function_name, "failed")
            except Exception as e:
                rospy.logerr(f"Unexpected error in {function_name}: {e}")
                self.publish_status(function_name, "failed")
        else:
            rospy.logerr(f"Function {function_name} not found.")
            self.publish_status(function_name, "failed")

    # Sort function
    def sort_instructions(self, data):
        # Mapping of keys to instruction types

        # Flatten and transform data with type labels
        instructions = []
        for key, items in data.items():
            for item in items:
                item["type"] = key  # Add type field
                instructions.append(item)

        # Sort the list by sequence_index
        instructions.sort(key=lambda i: i["sequence_index"])

        return instructions


    def goal_status_callback(self, msg):
        self.goal_status = msg.status.status

        if self.goal_status == GoalStatus.ABORTED:
            feedback = "I failed to move to the specified location."
            self.publish_string(feedback, self.llm_feedback_to_user_publisher)
        elif self.goal_status == GoalStatus.SUCCEEDED:
            feedback = "I have arrived at the specified location."
            self.publish_string(feedback, self.llm_feedback_to_user_publisher)


    def publish_cmd_vel(self, **kwargs):
        """
        Publishes cmd_vel message to control the movement of Tiago
        """
        linear_x = kwargs.get("linear_x", 0.0)
        linear_y = kwargs.get("linear_y", 0.0)
        linear_z = kwargs.get("linear_z", 0.0)
        angular_x = kwargs.get("angular_x", 0.0)
        angular_y = kwargs.get("angular_y", 0.0)
        angular_z = kwargs.get("angular_z", 0.0)

        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = float(linear_y)
        twist_msg.linear.z = float(linear_z)
        twist_msg.angular.x = float(angular_x)
        twist_msg.angular.y = float(angular_y)
        twist_msg.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(twist_msg)
        self.goal_status = GoalStatus.SUCCEEDED
        rospy.loginfo(f"Publishing cmd_vel message successfully: {twist_msg}")
        return twist_msg
    
    def publish_goal_pose(self, **kwargs):
        """
        Publishes goal_pose message
        """

        x_value = kwargs.get("x", 0.2)
        y_value = kwargs.get("y", 0.2)
        z_value = kwargs.get("z", 0.2)
        feedback = kwargs.get("feedback_to_user", "")

        roll_value = kwargs.get("roll", 0.2)
        pitch_value = kwargs.get("pitch", 0.2)
        yaw_value = kwargs.get("yaw", 0.2)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = x_value
        pose.pose.position.y = y_value
        pose.pose.position.z = z_value

        quaternion = tf.transformations.quaternion_from_euler(roll_value, pitch_value, yaw_value)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.go_to_point.publish(pose)
        self.publish_string(str(feedback), self.llm_feedback_to_user_publisher)
        self.goal_status = GoalStatus.ACTIVE
        rospy.loginfo(f"Publishing goal pose: {pose}")
        return pose
    
    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        rospy.loginfo(f"Topic: {publisher_to_use.name} Message published: {msg.data}")
    
    def provide_answer(self, **kwargs):
        """
        Provides answers and responses to the user\'s queries.
        """
        pass


    def execute_motion(self, **kwargs):
        """
        Plays predefined motions
        """

        goal = PlayMotionGoal()
        goal.motion_name = kwargs.get("motion_name", "home")
        goal.skip_planning = False
        goal.priority = 0  # Optional
        feedback = kwargs.get("feedback_to_user", "")

        rospy.loginfo("Sending goal with motion: " + goal.motion_name)
        self.client.send_goal(goal)

        rospy.loginfo("Waiting for result...")
        action_ok = self.client.wait_for_result(rospy.Duration(30.0))

        state = self.client.get_state()

        if action_ok:
            self.goal_status =  GoalStatus.SUCCEEDED
            rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
        else:
            self.goal_status =  GoalStatus.ABORTED
            rospy.logwarn("Action failed with state: " + str(get_status_string(state)))

        goal = PlayMotionGoal()
        goal.motion_name = "home"
        goal.skip_planning = False
        goal.priority = 0  # Optional

        rospy.loginfo("Sending goal with motion: " + goal.motion_name)
        self.publish_string(str(feedback), self.llm_feedback_to_user_publisher)
        self.client.send_goal(goal)

        rospy.loginfo("Waiting for result...")
        action_ok = self.client.wait_for_result(rospy.Duration(30.0))
    

    def pick_object(self, **kwargs):
        object_name = kwargs.get("object_name", 0.0)
        object_id = None
        feedback = kwargs.get("feedback_to_user", "")

        for id, num in ARUCO_IDS.items():
            if object_name in num:
                object_id = id

        if object_id is not None:
            rospy.loginfo(f"Aruco ID is {object_id}")
            x = Int64()
            x.data = object_id
            self.marker_set.publish(x)
        
        rospy.loginfo(f"Picking up {object_name}")
        self.publish_string(str(feedback), self.llm_feedback_to_user_publisher)
        self.pick_gui_service.call()
        self.goal_status = GoalStatus.ACTIVE


    def place_object(self, **kwargs):
        object_name = kwargs.get("object_name", 0.0)
        object_id = None
        feedback = kwargs.get("feedback_to_user", "")

        for id, num in ARUCO_IDS.items():
            if object_name in num:
                object_id = id

        if object_id is not None:
            x = Int64()
            x.data = object_id
            self.marker_set.publish(x)
        
        self.publish_string(str(feedback), self.llm_feedback_to_user_publisher)
        self.place_gui_service.call()
        self.goal_status = GoalStatus.ACTIVE
    


if __name__ == "__main__":
    try:
        rospy.init_node('tiago_robot_llm_node', anonymous=True)   
        node = TiagoRobot()
        
        # Run forever
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
