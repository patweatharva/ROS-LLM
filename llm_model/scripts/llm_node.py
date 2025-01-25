#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from llm_interfaces.srv import ChatGPTs
import json
import os
import time
import ollama
from openai import OpenAI
from model_context import ModelContext
from dotenv import load_dotenv

load_dotenv()

class LLMNode:
    def __init__(self):
        rospy.init_node('llm_node', anonymous=True)
        
        self.use_deepseek = True
        
        if self.use_deepseek:
            self.model = "deepseek-chat"
            self.llm_client = OpenAI(api_key=os.getenv("DEEPSEEK_API_KEY"),base_url=os.getenv("DEEPSEEK_BASE_URL"))
        else:
            self.model = "tiago:latest"  # You can change this to any model available in Ollama
            self.llm_client = ollama.Client(host="http://localhost:11434")
        
        self.model_context = ModelContext()
        self.system_prompt = self.model_context.system_prompt
        self.tools = self.model_context.tools
        
        # Publishers
        self.initialization_publisher = rospy.Publisher("/llm_initialization_state", String, queue_size=1)
        self.llm_state_publisher = rospy.Publisher("/llm_state", String, queue_size=1)
        self.llm_response_type_publisher = rospy.Publisher("/llm_response_type", String, queue_size=1)
        self.llm_feedback_to_user_publisher = rospy.Publisher("/llm_feedback_to_user", String, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/llm_state", String, self.state_listener_callback)
        rospy.Subscriber("/llm_input_audio_to_text", String, self.llm_callback)
        
        # Service client for function calls
        self.function_call_client = rospy.ServiceProxy('/ChatGPT_function_call_service', ChatGPTs)
        
        # Chat history initialization
        self.start_timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.chat_history_path = "src/llm_model/src/llm_model/llm_history"
        if not os.path.exists(self.chat_history_path):
            os.makedirs(self.chat_history_path)
        
        self.chat_history_file = os.path.join(self.chat_history_path, f"chat_history_{self.start_timestamp}.json")
        self.chat_history = [{"role": "system", "content": self.system_prompt}]
        self.chat_history_max_length = 1000
        
        # Create llm_history directory if it doesn't exist
        self.write_chat_history_to_json()
        
        # Function name for tracking function calls
        self.tool_calls = {}
        
        # Publish initialization state
        self.publish_string("llm_model_processing", self.initialization_publisher)
        if self.use_deepseek:
            rospy.loginfo("LLM Node initialized with Deepseek model: " + self.model)
        else:
            rospy.loginfo("LLM Node initialized with Ollama model: " + self.model)

    def state_listener_callback(self, msg):
        rospy.logdebug(f"Model node get current State: {msg.data}")

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        rospy.loginfo(f"Topic: {publisher_to_use.name} Message published: {msg.data}")

    def add_message_to_history(self, role, content="null", function_call=None):
        message_element_object = {
            "role": role,
            "content": content,
        }
        if function_call is not None:
            message_element_object["function_call"] = function_call
            
        self.chat_history.append(message_element_object)
        rospy.loginfo(f"Chat history updated with {message_element_object}")
        
        if len(self.chat_history) > self.chat_history_max_length:
            rospy.loginfo(
                f"Chat history is too long, popping the oldest message: {self.chat_history[0]}"
            )
            self.chat_history.pop(1)
        
        return self.chat_history

    def generate_llm_response(self, messages_input):
        if self.use_deepseek:
            rospy.loginfo(f"Sending messages to Deepseek: {messages_input}")
            try:
                response = self.llm_client.chat.completions.create(
                    model=self.model,
                    messages=messages_input,
                    max_tokens=4096,
                    temperature=0.2,
                    top_p=0.5,
                    tools = self.tools,
                    tool_choice="required",
                    stream=False,
                    parallel_tool_calls=True,
                    reasoning_effort="high"
                )
                return response
            except Exception as e:
                rospy.logerr(f"Error calling Deepseek API: {e}")
                return None
        else:
            rospy.loginfo(f"Sending messages to Ollama: {messages_input}")
            try:
                response = self.llm_client.chat(
                    model=self.model,
                    messages=messages_input,
                    max_tokens=4096,
                    temperature=0.5,
                    top_p=0.95,
                    frequency_penalty=0,
                    presence_penalty=0,
                )
                return response
            except Exception as e:
                rospy.logerr(f"Error calling Ollama API: {e}")
                return None
        
    def get_response_information(self, llm_response):
        if ((llm_response is not None) and (len(llm_response.choices[0].message.tool_calls) > 0)):
            for tool_call in llm_response.choices[0].message.tool_calls:
                if tool_call.function.name not in self.tool_calls:
                    self.tool_calls[tool_call.function.name] = []
                self.tool_calls[tool_call.function.name].append(json.loads(tool_call.function.arguments))
            
            tool_calls_data = [{
                "id": tool_call.id,
                "type": tool_call.type,
                "function": {
                    "name": tool_call.function.name,
                    "arguments": tool_call.function.arguments
                }
            } for tool_call in llm_response.choices[0].message.tool_calls]
            self.add_message_to_history("assistant", "", function_call=tool_calls_data)
            tool_calls_text = ", ".join([f"{name} ({len(calls)} times)" for name, calls in self.tool_calls.items()])
            text = f"<|BOC|>Please confirm that you have called following tools: {tool_calls_text}.<|EOC|>"
            return text
        else:
            text = "<|BOE|>Sorry, Your request could not be processed. I encountered an error while processing your request. Please try again.<|EOE|>"
            self.add_message_to_history("assistant", text)
            return text

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
        text = self.get_response_information(llm_response)
        
        # Write chat history to JSON
        self.write_chat_history_to_json()
        
        # Log output_processing
        rospy.loginfo("STATE: output_processing")
        
        self.generate_and_publish_response_to_user(text)
        
        self.generate_and_publish_tool_calls()
        
        self.tool_calls = {}

    def generate_and_publish_response_to_user(self, text):
        response_data = {"text": text}
        if "ask_clarifying_question" in self.tool_calls:
            response_data["ask_clarifying_question"] = self.tool_calls["ask_clarifying_question"]
        if "provide_answer_to_user" in self.tool_calls:
            response_data["provide_answer_to_user"] = self.tool_calls["provide_answer_to_user"]
        message = json.dumps(response_data)
        print("--------------------------------")
        print(message)
        print("--------------------------------")
        self.publish_string(message, self.llm_feedback_to_user_publisher)

    def generate_and_publish_tool_calls(self):
        tool_calls_data = {
            key: value for key, value in self.tool_calls.items() 
            if key not in ["ask_clarifying_question", "provide_answer_to_user"]
        }
        message = json.dumps(tool_calls_data)
        print("--------------------------------")
        print(message)
        print("--------------------------------")
        # Create service request
        request = ChatGPTs()
        request.request_text = message
        
        try:
            # Call the service
            response = self.function_call_client(request)
            rospy.loginfo(f"Service response: {response.response_text}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
        

def main():
    try:
        llm_node = LLMNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
