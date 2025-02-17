#!/usr/bin/env python

# Other libraries
import datetime
import json
import requests
import time

# AWS ASR related
import boto3

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

        # AWS service initialization
        self.aws_audio_file = "/tmp/user_audio_input.flac"
        self.aws_access_key_id = config.aws_access_key_id
        self.aws_secret_access_key = config.aws_secret_access_key
        self.aws_region_name = config.aws_region_name
        self.aws_session = boto3.Session(
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.aws_region_name,
        )

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
        # AWS S3 settings
        bucket_name = config.bucket_name
        audio_file_key = "gpt_audio.flac"  # Name of the audio file in S3
        transcribe_job_name = (
            f'my-transcribe-job-{datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}'
        )
        # Name the conversion task based on time to ensure uniqueness
        # Path of the audio file in S3
        transcribe_job_uri = f"s3://{bucket_name}/{audio_file_key}"

        # Step 1: Record audio
        rospy.loginfo("Start recording...")
        audio_data = sd.rec(
            int(duration * sample_rate), samplerate=sample_rate, channels=1
        )
        sd.wait()  # Wait until recording is finished

        # Step 2: Increase the volume by a multiplier
        audio_data *= volume_gain_multiplier

        # Step 3: Save audio to file
        write(self.aws_audio_file, sample_rate, audio_data)
        rospy.loginfo("Stop recording!")

        # action_function_input_processing
        self.publish_string("input_processing", self.llm_state_publisher)
        # Step 4: Upload audio to AWS S3
        s3 = self.aws_session.client("s3")
        rospy.loginfo("Uploading audio to AWS S3...")
        with open(self.aws_audio_file, "rb") as f:
            s3.upload_fileobj(Fileobj=f, Bucket=bucket_name, Key=audio_file_key)
        rospy.loginfo("Upload complete!")

        # Step 5: Convert audio to text
        transcribe = self.aws_session.client("transcribe")
        rospy.loginfo("Converting audio to text...")
        transcribe.start_transcription_job(
            TranscriptionJobName=transcribe_job_name,
            LanguageCode=config.aws_transcription_language,
            Media={"MediaFileUri": transcribe_job_uri},
        )

        # Step 6: Wait until the conversion is complete
        while True:
            status = transcribe.get_transcription_job(
                TranscriptionJobName=transcribe_job_name
            )
            if status["TranscriptionJob"]["TranscriptionJobStatus"] in [
                "COMPLETED",
                "FAILED",
            ]:
                break

            rospy.loginfo("Converting...")
            time.sleep(0.5)

        # Step 7: Get the transcribed text
        if status["TranscriptionJob"]["TranscriptionJobStatus"] == "COMPLETED":
            transcript_file_url = status["TranscriptionJob"]["Transcript"][
                "TranscriptFileUri"
            ]
            response = requests.get(transcript_file_url)
            transcript_data = json.loads(response.text)
            transcript_text = transcript_data["results"]["transcripts"][0]["transcript"]
            rospy.loginfo("Audio to text conversion complete!")
            # Step 8: Publish the transcribed text to ROS
            if transcript_text == "":  # Empty input
                rospy.loginfo("Empty input!")
                self.publish_string("listening", self.llm_state_publisher)
            else:
                self.publish_string(transcript_text, self.audio_to_text_publisher)
            # Step 9: Delete the temporary audio file from AWS S3
            s3.delete_object(Bucket=bucket_name, Key=audio_file_key)

        else:
            rospy.logerr(
                f"Failed to transcribe audio: {status['TranscriptionJob']['FailureReason']}"
            )

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
