#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class LangGraphNode:
    def __init__(self):
        # Initialize node
        rospy.init_node("lang_graph_node", anonymous=True)

        # Initialize publishers
        self.state_publisher = rospy.Publisher(
            "/llm_state", String, queue_size=10
        )

        # Initialize subscribers
        rospy.Subscriber(
            "/llm_state", String, self.state_callback
        )

    def state_callback(self, msg):
        rospy.loginfo("Received state: %s", msg.data)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        rospy.loginfo(
            f"Topic: {publisher_to_use.resolved_name}\nMessage published: {msg.data}"
        )

def main():
    try:
        lang_graph_node = LangGraphNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down lang_graph_node")

if __name__ == "__main__":
    main()
