#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Open Whisper related
from datetime import datetime
import whisper
import rospkg
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

class AudioInput:
    def __init__(self):
        # Initialize node
        rospy.init_node("llm_audio_input", anonymous=True)

        # tmp audio file
        self.audio_file = "/tmp/user_audio_input.flac"
        
        # Get package path and create directories for model and audio files
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path("llm_input")
        
        # Create model and audio directories
        self.model_dir = os.path.join(self.package_path, "models")
        self.audio_dir = os.path.join(self.package_path, "audio")
        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir, exist_ok=True)
        if not os.path.exists(self.audio_dir):
            os.makedirs(self.audio_dir, exist_ok=True)
            
        self.duration = config.duration  # Audio recording duration, in seconds
        self.sample_rate = config.sample_rate  # Sample rate
        self.volume_gain_multiplier = config.volume_gain_multiplier  # Volume gain multiplier
        self.model_device = config.model_device
        self.whisper_model = whisper.load_model(config.whisper_model_size, 
                                                device=self.model_device, 
                                                in_memory=True, 
                                                download_root=self.model_dir)
        
            
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
        # Get current time
        current_time = datetime.now().strftime("%Y%m%d%H%M%S")
        # Create audio file name (using WAV instead of FLAC/MP3)
        audio_file_name = f"user_audio_input_{current_time}.wav"
        self.audio_file = os.path.join(self.audio_dir, audio_file_name)
        
        # Step 1: Record audio
        rospy.loginfo("Start local recording...")
        audio_data = sd.rec(
            int(self.duration * self.sample_rate), samplerate=self.sample_rate, channels=1
        )
        sd.wait()  # Wait until recording is finished

        # Step 2: Increase the volume by a multiplier
        audio_data *= self.volume_gain_multiplier

        # Step 3: Save audio to WAV file only
        write(self.audio_file, self.sample_rate, audio_data)
        rospy.loginfo("Stop local recording!")
        
        # action_function_input_processing
        self.publish_string("input_processing", self.llm_state_publisher)
        
        # Step 6: Wait until the conversion is complete
        rospy.loginfo("Local Converting...")
        self.publish_string("done listening", self.llm_state_publisher)

        # Step 7: Get the transcribed text
        whisper_result = self.whisper_model.transcribe(
            self.audio_file,
            language=config.whisper_language,
            temperature=config.whisper_temperature,
            initial_prompt=config.initial_prompt,
            condition_on_previous_text=True
        )
        transcript_text = whisper_result["text"]
        rospy.loginfo("Audio to text conversion complete!")

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
        audio_input = AudioInput()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down llm_audio_input node.")


if __name__ == "__main__":
    main()
