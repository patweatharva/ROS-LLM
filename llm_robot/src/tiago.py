#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from llm_interfaces.srv import ChatGPT

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()

class TiagoRobot:
    def __init__(self):
        pass

    def plan(self, **kwargs):
        pass


if __name__ == "__main__":
    try:
        rospy.init_node('tiago_robot_llm_node', anonymous=True)   
        node = TiagoRobot()
        
        # Run forever
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
