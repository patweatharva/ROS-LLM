#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from llm_interfaces.srv import ChatGPTs
import json
import os
import time
import rospkg
import ollama

class LLMNode:
    def __init__(self):
        rospy.init_node('llm_node', anonymous=True)
        
        # Initialize Ollama client
        self.model = "tiago:latest"  # You can change this to any model available in Ollama
        self.ollama_client = ollama.Client(host="http://localhost:11434")
        self.rospack = rospkg.RosPack()
        
        # Publishers
        self.initialization_publisher = rospy.Publisher(
            "/llm_initialization_state", String, queue_size=1
        )
        self.llm_state_publisher = rospy.Publisher("/llm_state", String, queue_size=1)
        self.llm_response_type_publisher = rospy.Publisher(
            "/llm_response_type", String, queue_size=1
        )
        self.llm_feedback_publisher = rospy.Publisher(
            "/llm_feedback_to_user", String, queue_size=1
        )
        
        # Subscribers
        rospy.Subscriber("/llm_state", String, self.state_listener_callback)
        rospy.Subscriber("/llm_input_audio_to_text", String, self.llm_callback)
        
        # Service client for function calls
        self.function_call_client = rospy.ServiceProxy(
            '/ChatGPT_function_call_service', ChatGPTs
        )
        
        # Chat history initialization
        self.start_timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.chat_history_path = self.rospack.get_path('llm_model')
        self.chat_history_file = os.path.join(
            self.chat_history_path, f"llm_history/chat_history_{self.start_timestamp}.json"
        )
        self.chat_history = [{"role": "system", "content": "You are a helpful assistant robot which has the ability to move around the house and help the user with their needs. If the user asks for something, you will try to help them with it. If the user asks for something that is not in your capabilities, you will tell them that you are not able to help them with that."}]
        self.chat_history_max_length = 4000
        
        # Create llm_history directory if it doesn't exist
        os.makedirs(os.path.dirname(self.chat_history_file), exist_ok=True)
        self.write_chat_history_to_json()
        
        # Function name for tracking function calls
        self.function_name = "null"
        
        # Publish initialization state
        self.publish_string("llm_model_processing", self.initialization_publisher)
        rospy.loginfo("LLM Node initialized with Ollama model: " + self.model)

    def state_listener_callback(self, msg):
        rospy.logdebug(f"Model node get current State: {msg.data}")

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        rospy.loginfo(f"Topic: {publisher_to_use.name} Message published: {msg.data}")

    def add_message_to_history(self, role, content="null", function_call=None, name=None):
        message_element_object = {
            "role": role,
            "content": content,
        }
        if name is not None:
            message_element_object["name"] = name
        if function_call is not None:
            message_element_object["function_call"] = function_call
            
        self.chat_history.append(message_element_object)
        rospy.loginfo(f"Chat history updated with {message_element_object}")
        
        if len(self.chat_history) > self.chat_history_max_length:
            rospy.loginfo(
                f"Chat history is too long, popping the oldest message: {self.chat_history[0]}"
            )
            self.chat_history.pop(0)
        
        return self.chat_history

    def generate_llm_response(self, messages_input):
        rospy.loginfo(f"Sending messages to Ollama: {messages_input}")
        try:
            response = self.ollama_client.chat(
                model=self.model,
                messages=messages_input
            )
            return response
        except Exception as e:
            rospy.logerr(f"Error calling Ollama API: {e}")
            return None
    def get_response_information(self, llm_response):
        if llm_response is None:
            return None, "Sorry, I encountered an error while processing your request.", None, 0
            
        message = llm_response['message']
        content = message.get('content')
        
        # Check for tool calls in the response
        tool_calls = message.get('tool_calls', [])
        
        if tool_calls:
            # Convert tool call to function call format for compatibility
            function_call = {
                'name': tool_calls[0]['function']['name'],
                'arguments': tool_calls[0]['function']['arguments']
            }
            function_flag = 1
            content = None
        else:
            function_call = None
            function_flag = 0
            
        rospy.loginfo(f"Get message from Ollama: {message}, type: {type(message)}")
        rospy.loginfo(f"Get content from Ollama: {content}, type: {type(content)}")
        if function_call:
            rospy.loginfo(f"Get function call from Ollama: {function_call}")
        
        return message, content, function_call, function_flag

    def write_chat_history_to_json(self):
        try:
            json_data = json.dumps(self.chat_history)
            with open(self.chat_history_file, "w", encoding="utf-8") as file:
                file.write(json_data)
            rospy.loginfo("Chat history has been written to JSON")
            return True
        except IOError as error:
            rospy.logerr(f"Error writing chat history to JSON: {error}")
            return False

    def llm_callback(self, msg):
        rospy.loginfo("STATE: model_processing")
        rospy.loginfo(f"Input message received: {msg.data}")
        
        # Add user message to chat history
        user_prompt = msg.data
        self.add_message_to_history("user", user_prompt)
        
        # Generate chat completion
        llm_response = self.generate_llm_response(self.chat_history)
        
        # Get response information
        message, text, function_call, function_flag = self.get_response_information(llm_response)
        
        # Append response to chat history
        self.add_message_to_history(
            role="assistant", content=text, function_call=function_call
        )
        
        # Write chat history to JSON
        self.write_chat_history_to_json()
        
        # Log output_processing
        rospy.loginfo("STATE: output_processing")
        
        # Return text response
        llm_response_type = "feedback_for_user"
        rospy.loginfo("STATE: feedback_for_user")
        self.publish_string(llm_response_type, self.llm_response_type_publisher)
        self.publish_string(text, self.llm_feedback_publisher)

def main():
    try:
        llm_node = LLMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
