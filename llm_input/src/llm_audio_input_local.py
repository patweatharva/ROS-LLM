#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Open Whisper related
import whisper

# Audio recording related
import sounddevice as sd
from scipy.io.wavfile import write

# ROS related
import rospy
from std_msgs.msg import String

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()

class AudioInput:
    def __init__(self):
        # Initialize node
        rospy.init_node("llm_audio_input", anonymous=True)

        # tmp audio file
        self.tmp_audio_file = "/tmp/user_audio_input.flac"

        # Initialization publisher
        self.initialization_publisher = rospy.Publisher(
            "/llm_initialization_state", String, queue_size=10
        )

        # LLM state publisher
        self.llm_state_publisher = rospy.Publisher(
            "/llm_state", String, queue_size=10
        )

        # LLM state listener
        rospy.Subscriber(
            "/llm_state", String, self.state_listener_callback
        )

        self.audio_to_text_publisher = rospy.Publisher(
            "/llm_input_audio_to_text", String, queue_size=10
        )
        # Initialization ready
        self.publish_string("llm_audio_input", self.initialization_publisher)

    def state_listener_callback(self, msg):
        if msg.data == "listening":
            rospy.loginfo(f"STATE: {msg.data}")
            self.action_function_listening()

    def action_function_listening(self):
        # Recording settings
        duration = config.duration  # Audio recording duration, in seconds
        sample_rate = config.sample_rate  # Sample rate
        volume_gain_multiplier = config.volume_gain_multiplier  # Volume gain multiplier

        # Step 1: Record audio
        rospy.loginfo("Start local recording...")
        audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1
        )
        sd.wait()  # Wait until recording is finished

        # Step 2: Increase the volume by a multiplier
        audio_data *= volume_gain_multiplier

        # Step 3: Save audio to file
        write(self.tmp_audio_file, sample_rate, audio_data)
        rospy.loginfo("Stop local recording!")

        # action_function_input_processing
        self.publish_string("input_processing", self.llm_state_publisher)

        # Step 4: Process audio with OpenAI Whisper
        whisper_model = whisper.load_model(config.whisper_model_size)

        # Step 6: Wait until the conversion is complete
        rospy.loginfo("Local Converting...")

        # Step 7: Get the transcribed text
        whisper_result = whisper_model.transcribe(self.tmp_audio_file, language=config.whisper_language)

        transcript_text = whisper_result["text"]
        rospy.loginfo("Audio to text conversion complete!")

        # Step 8: Publish the transcribed text to ROS
        if transcript_text == "":  # Empty input
            rospy.loginfo("Empty input!")
            self.publish_string("listening", self.llm_state_publisher)
        else:
            self.publish_string(transcript_text, self.audio_to_text_publisher)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        rospy.loginfo(
            f"Topic: {publisher_to_use.resolved_name}\nMessage published: {msg.data}"
        )


def main():

    try:
        rospy.init_node('audio_input', anonymous=True)
        audio_input = AudioInput()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down llm_audio_input node.")


if __name__ == "__main__":
    main()
