#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import requests
from pydub import AudioSegment
from pydub.playback import play
import os
import rospkg
import json
import threading


# Get the package path dynamically using rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('llm_output')

# Path to the file that stores the Eleven Labs API key
API_KEY_FILE = os.path.join(package_path, 'config', '.elevenlabs_api_key')


# Replace with your preferred Eleven Labs voice ID
VOICE_ID = "iP95p4xoKVk53GoZ742B"  # Change to the desired voice ID


def read_api_key(file_path):
    """Read the Eleven Labs API key from a file."""
    try:
        with open(file_path, "r") as file:
            return file.read().strip()
    except FileNotFoundError:
        rospy.logerr(f"API key file not found at {file_path}")
        return None


class ElevenLabsTTSNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('audio_output', anonymous=True)

        # Read the API key from the external file
        self.api_key = read_api_key(API_KEY_FILE)

        if not self.api_key:
            rospy.logerr("Failed to read the Eleven Labs API key. Exiting...")
            rospy.signal_shutdown("No API key provided.")
            return
        
        # Initialization publisher
        self.initialization_publisher = rospy.Publisher(
            "/llm_initialization_state", String, queue_size=10
        )

        # LLM state publisher
        self.llm_state_publisher = rospy.Publisher(
            "/llm_state", String, queue_size=10
        )

        self.lock = threading.Lock()

        # Subscribe to the text input topic
        rospy.Subscriber('/llm_feedback_to_user', String, self.text_callback, queue_size=10)
        # Initialization ready
        self.publish_string("output ready", self.initialization_publisher)
    
    def extract_response(self, json_string):
        """
        Parses a JSON string to extract and concatenate the question and answer from specific function calls.
        """

        try:
            data = json.loads(json_string)
            keys_to_check = ["provide_answer_to_user", "ask_clarifying_question"]

            for key in keys_to_check:
                if key in data and data[key]:
                    response = data[key][0]
                    question = response.get("question", "") if key == "ask_clarifying_question" else ""
                    answer = response.get("answer", "") if key == "provide_answer_to_user" else ""
                    
                    result = f"{question} {answer}".strip()
                    return result if result else ""

            return ""
        except json.JSONDecodeError:
            return ""
    
    def is_json(self, message):
        try:
            json.loads(message)
            return True  # It's a valid JSON
        except ValueError:
            return False  # Not a JSON
        

    def text_callback(self, msg):
        with self.lock:

            text = msg.data
            if self.is_json(text):
                text = self.extract_response(str(text))
            rospy.loginfo(f"Received text: {text}")

            # Call Eleven Labs API to generate speech
            if text:
                audio_data = self.generate_speech(text)

                if audio_data:
                    self.play_audio(audio_data)
                else:
                    rospy.logerr("Failed to generate speech audio.")

    def generate_speech(self, text):
        """Send text to Eleven Labs API and receive audio data."""
        try:
            url = f"https://api.elevenlabs.io/v1/text-to-speech/{VOICE_ID}"
            headers = {
                "Content-Type": "application/json",
                "xi-api-key": self.api_key
            }
            payload = {
                "text": text,
                "voice_settings": {
                    "stability": 0.7,
                    "similarity_boost": 0.8
                }
            }
            response = requests.post(url, json=payload, headers=headers)

            if response.status_code == 200:
                return response.content
            else:
                rospy.logerr(f"Eleven Labs API Error: {response.status_code} - {response.text}")
                return None

        except Exception as e:
            rospy.logerr(f"Error calling Eleven Labs API: {e}")
            return None

    def play_audio(self, audio_data):
        """Play received audio data."""
        audio_file_path = "/tmp/eleven_labs_tts.mp3"
        with open(audio_file_path, "wb") as audio_file:
            audio_file.write(audio_data)

        # Convert and play the audio using pydub
        audio = AudioSegment.from_file(audio_file_path, format="mp3")
        play(audio)

        rospy.loginfo("Finished playing.")
        self.publish_string("feedback finished", self.llm_state_publisher)
        # self.publish_string("listening", self.llm_state_publisher)


    def run(self):
        rospy.spin()
    
    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        rospy.loginfo(
            f"Topic: {publisher_to_use.resolved_name}\nMessage published: {msg.data}"
        )


if __name__ == '__main__':
    try:
        tts_node = ElevenLabsTTSNode()
        tts_node.run()
    except rospy.ROSInterruptException:
        pass
