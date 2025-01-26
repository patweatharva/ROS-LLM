#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from llm_interfaces.srv import ChatGPTs, ChatGPTsResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, GoalStatus
import json

# Global Initialization
from llm_config.user_config import UserConfig

def get_status_string(status_code):
    return GoalStatus.to_string(status_code)

config = UserConfig()

class TiagoRobot:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        self.go_to_point = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.client = SimpleActionClient('/play_motion', PlayMotionAction)
        # rospy.loginfo("Waiting for Action Server...")
        # self.client.wait_for_server()

        # Server for model function call
        self.function_call_server = rospy.Service("/ChatGPT_function_call_service", ChatGPTs, self.function_call_callback)


    def function_call_callback(self, req):
        print(f"Received request: {req.request_text}")

        # Parse the request text safely
        try:
            req_data = json.loads(req.request_text)
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse JSON: {e}")
            response = ChatGPTsResponse()
            response.response_text = f"Invalid JSON format: {e}"
            return response

        # Process each function call dynamically
        response_texts = []
        for function_name, args_list in req_data.items():
            try:
                # Check if the function exists in the class
                if hasattr(self, function_name):
                    func_obj = getattr(self, function_name)

                    for args in args_list:
                        if isinstance(args, dict):
                            # Call the function with unpacked arguments
                            result = func_obj(**args)
                            response_texts.append(f"{function_name} executed: {result}")
                        else:
                            rospy.logerr(f"Invalid arguments for {function_name}: {args}")
                            response_texts.append(f"Invalid arguments for {function_name}")
                else:
                    rospy.logerr(f"Function {function_name} not found.")
                    response_texts.append(f"Function {function_name} not found.")

            except Exception as error:
                rospy.logerr(f"Error calling {function_name}: {error}")
                response_texts.append(f"Error calling {function_name}: {error}")

        response = ChatGPTsResponse()
        response.response_text = " | ".join(response_texts)
        return response


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
        rospy.loginfo(f"Publishing cmd_vel message successfully: {twist_msg}")
        return twist_msg
    
    def publish_goal_pose(self, **kwargs):
        """
        Publishes goal_pose message
        """

        x_value = kwargs.get("x", 0.2)
        y_value = kwargs.get("y", 0.2)
        z_value = kwargs.get("z", 0.2)

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
        rospy.loginfo(f"Publishing goal pose: {pose}")
        return pose
    
    def provide_answer(self, **kwargs):
        """
        Provides answers and responses to the user\'s queries.
        """
        pass


    def motion(self, **kwargs):
        """
        Plays predefined motions
        """

        goal = PlayMotionGoal()
        goal.motion_name = kwargs.get("name", "home")
        goal.skip_planning = False
        goal.priority = 0  # Optional

        rospy.loginfo("Sending goal with motion: " + goal.motion_name)
        self.client.send_goal(goal)

        rospy.loginfo("Waiting for result...")
        action_ok = self.client.wait_for_result(rospy.Duration(30.0))

        state = self.client.get_state()

        if action_ok:
            rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
        else:
            rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
    


if __name__ == "__main__":
    try:
        rospy.init_node('tiago_robot_llm_node', anonymous=True)   
        node = TiagoRobot()
        
        # Run forever
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
