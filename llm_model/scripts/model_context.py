import json
import rospkg
import os
import sys

package_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'llm_config/src')
sys.path.append(package_dir)

from llm_config.robot_behavior import RobotBehavior


class ModelContext:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        llm_config_path = self.rospack.get_path('llm_config')
        self.environment_data = self.load_environment_data(os.path.join(llm_config_path, "src/llm_config/environment.json"))
        self.system_prompt_path = os.path.join(llm_config_path, "src/llm_config/system_prompt.txt")
        self.system_prompt = self.create_system_prompt()
        self.tools = self.create_tools()
        
    def create_system_prompt(self):
        system_prompt = self.generate_system_prompt(self.system_prompt_path)
        return system_prompt
    
    def load_environment_data(self, json_file_path):
        with open(json_file_path, 'r') as file:
            environment_data = json.load(file)
        return environment_data

    def generate_system_prompt(self, system_prompt_path):
        prompt = open(system_prompt_path, 'r').read()
        prompt += "\n\nRoom Information:\n"
        for room, details in self.environment_data["rooms"].items():
            coordinates = details["coordinates"]
            yaw = details["yaw"]
            items = ", ".join(details["items"])
            prompt += f"- {room.capitalize()} at coordinates {coordinates}, yaw: {yaw}, containing items: {items}.\n"

        prompt += "\nAvailable motions include:\n"
        motions_list = self.environment_data["motions"]["list"]
        prompt += ", ".join(motions_list) + ".\n"
    
        return prompt
    
    def create_tools(self):
        tools = RobotBehavior().robot_functions_list
        return tools
    
    
if __name__ == "__main__":
    model_context = ModelContext()
    print(model_context.system_prompt)
    print(model_context.tools)