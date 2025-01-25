import ollama
import robot_behavior
import json

robot_functions_list = robot_behavior.robot_functions_list

# Load environment data from a JSON file
def load_environment_data(json_file_path):
    with open(json_file_path, 'r') as file:
        environment_data = json.load(file)
    return environment_data

# Generate system prompt including rooms, items, and available motions
def generate_system_prompt(environment_data):
    prompt = "You are a home assistant robot. The environment consists of the following rooms:\n\n"
    for room, details in environment_data["rooms"].items():
        coordinates = details["coordinates"]
        items = ", ".join(details["items"])
        prompt += f"- {room.capitalize()} at coordinates {coordinates}, containing items: {items}.\n"
    
    prompt += "\nAvailable motions include:\n"
    motions_list = environment_data["motions"]["list"]
    prompt += ", ".join(motions_list) + ".\n"

    prompt += (
        "When the user asks you to retrieve an item, identify its location and provide navigation instructions. "
        "You can also perform the following motions when requested.\n"
        "Example commands: 'Go to the kitchen', 'Get the book from the living room', 'Execute the waving motion'."
    )
    return prompt

# Load data and create prompt
json_file_path = "environment.json"  # Replace with your actual file path
environment_data = load_environment_data(json_file_path)
system_prompt = generate_system_prompt(environment_data)

messages = [
    {"role": "system", "content": system_prompt},
    {"role": "user", "content": "what is the capital of germany? Also, what is the capital of india?"}
]

ollama_client = ollama.Client(host="http://localhost:11434")

response = ollama_client.chat(model='tiago:latest', messages=messages, tools=robot_functions_list)

# Print tool call responses
print(response['message']['tool_calls'])
