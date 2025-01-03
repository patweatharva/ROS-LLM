#!/usr/bin/env python

import datetime
import json
import requests
import time

# AWS ASR related
import boto3
import os

# Audio recording related
import sounddevice as sd
from scipy.io.wavfile import write

# ROS related
import rospy
from std_msgs.msg import String

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()

class AudioOutput:
    def __init__(self):
        # Initialize node
        rospy.init_node("audio_output", anonymous=True)

        # Initialization publisher
        self.initialization_publisher = rospy.Publisher(
            "/llm_initialization_state", String, queue_size=10
        )

        # LLM state publisher
        self.llm_state_publisher = rospy.Publisher(
            "/llm_state", String, queue_size=10
        )

        # Feedback for user listener
        rospy.Subscriber(
            "/llm_feedback_to_user", String, self.feedback_for_user_callback
        )

        # AWS parameters
        self.aws_access_key_id = config.aws_access_key_id
        self.aws_secret_access_key = config.aws_secret_access_key
        self.aws_region_name = config.aws_region_name
        self.aws_session = boto3.Session(
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.aws_region_name,
        )

        # Initialization ready
        self.publish_string("output ready", self.initialization_publisher)

    def feedback_for_user_callback(self, msg):
        rospy.loginfo("Received text: '%s'" % msg.data)

        # Call AWS Polly service to synthesize speech
        polly_client = self.aws_session.client("polly")
        rospy.loginfo("Polly client successfully initialized.")
        response = polly_client.synthesize_speech(
            Text=msg.data, OutputFormat="mp3", VoiceId=config.aws_voice_id
        )

        # Save the audio output to a file
        output_file_path = "/tmp/speech_output.mp3"
        with open(output_file_path, "wb") as file:
            file.write(response["AudioStream"].read())

        # Play the audio output
        os.system("mpv" + " " + output_file_path)
        rospy.loginfo("Finished Polly playing.")
        self.publish_string("feedback finished", self.llm_state_publisher)
        self.publish_string("listening", self.llm_state_publisher)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        rospy.loginfo(
            f"Topic: {publisher_to_use.resolved_name}\nMessage published: {msg.data}"
        )


def main():

    try:
        rospy.init_node('audio_output', anonymous=True)
        audio_output = AudioOutput()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down audio_output node.")


if __name__ == "__main__":
    main()
