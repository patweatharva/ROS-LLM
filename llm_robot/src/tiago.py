#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Twist
from llm_interfaces.srv import ChatGPT

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()

class TiagoRobot:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
        self.go_to_point = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    def plan(self, **kwargs):
        pass

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
        self.get_logger().info(f"Publishing cmd_vel message successfully: {twist_msg}")
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


if __name__ == "__main__":
    try:
        rospy.init_node('tiago_robot_llm_node', anonymous=True)   
        node = TiagoRobot()
        
        # Run forever
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
