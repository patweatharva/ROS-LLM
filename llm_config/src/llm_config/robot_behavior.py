#!/usr/bin/env python3
# -*- coding: utf-8 -*-

robot_functions_list = [
    {
        "type": "function",
        "function": {
            "name": "publish_cmd_vel",
            "description": "Publishes a cmd_vel (command velocity) message to control the movement of the robot, including linear and angular velocities. The linear velocity is in meters per second, the angular velocity is in radians per second. Only when the user asks the robot to move, the robot should use this function to move.",
            "parameters": {
                "type": "object",
                "properties": {
                    'sequence_index': {
                        'type': 'number',
                        'description': 'The index of the sequence of the tool call. This is used to keep track of the order of the tool calls. The first tool call should have a sequence index of 0, the second tool call should have a sequence index of 1, and so on. This will be used to determine the execution order of the tool calls based on the user\'s request.'
                    },
                    "linear_x": {
                        "type": "number",
                        "description": "Linear velocity along the x-axis.",
                    },
                    "linear_y": {
                        "type": "number",
                        "description": "Linear velocity along the y-axis.",
                    },
                    "linear_z": {
                        "type": "number",
                        "description": "Linear velocity along the z-axis.",
                    },
                    "angular_x": {
                        "type": "number",
                        "description": "Angular velocity around the x-axis.",
                    },
                    "angular_y": {
                        "type": "number",
                        "description": "Angular velocity around the y-axis.",
                    },
                    "angular_z": {
                        "type": "number",
                        "description": "Angular velocity around the z-axis.",
                    },
                },
                "required": [
                    "sequence_index",
                    "linear_x",
                    "linear_y",
                    "linear_z",
                    "angular_x",
                    "angular_y",
                    "angular_z",
                ],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "publish_goal_pose",
            "description": "Publishes a target pose to move the robot to a specific position and orientation in 3D space. The target pose is in meters and radians. Only when the user asks the robot to move to a specific location, the robot should use this function to move.",
            "parameters": {
                "type": "object",
                "properties": {
                    'sequence_index': {
                        'type': 'number',
                        'description': 'The index of the sequence of the tool call. This is used to keep track of the order of the tool calls. The first tool call should have a sequence index of 0, the second tool call should have a sequence index of 1, and so on. This will be used to determine the execution order of the tool calls based on the user\'s request.'
                    },
                    "x": {
                        "type": "number",
                        "description": "The x-coordinate of the target pose.",
                    },
                    "y": {
                        "type": "number",
                        "description": "The y-coordinate of the target pose.",
                    },
                    "z": {
                        "type": "number",
                        "description": "The z-coordinate of the target pose.",
                    },
                    "roll": {
                        "type": "number",
                        "description": "The roll angle (rotation around x-axis) in radians.",
                    },
                    "pitch": {
                        "type": "number",
                        "description": "The pitch angle (rotation around y-axis) in radians.",
                    },
                    "yaw": {
                        "type": "number",
                        "description": "The yaw angle (rotation around z-axis) in radians.",
                    },
                },
                "required": ["sequence_index", "x", "y", "z", "roll", "pitch", "yaw"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "provide_answer_to_user",
            "description": "Delivers an answer to question asked by the user, the answer should be in natural language and will be used to answer the user's question in speech output by calling the text to speech function.",
            "parameters": {
                "type": "object",
                "properties": {
                    "sequence_index": {
                        'type': 'number',
                        'description': 'The index of the sequence of the tool call. This is used to keep track of the order of the tool calls. The first tool call should have a sequence index of 0, the second tool call should have a sequence index of 1, and so on. This will be used to determine the execution order of the tool calls based on the user\'s request.'
                    },
                    "question":{
                        "type": "string",
                        "description": "The question asked by the user. This is used to determine the context of the answer."
                    },
                    "answer": {
                        "type": "string",
                        "description": "An answer to be delivered by the robot.",
                    }
                },
                "required": ["sequence_index", "question", "answer"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "ask_clarifying_question",
            "description": "Asks a clarifying question to the user to get more information about their request",
            "parameters": {
                "type": "object",
                "properties": {
                    "sequence_index": {
                        'type': 'number',
                        'description': 'The index of the sequence of the tool call. This is used to keep track of the order of the tool calls. The first tool call should have a sequence index of 0, the second tool call should have a sequence index of 1, and so on. This will be used to determine the execution order of the tool calls based on the user\'s request.'
                    },
                    "question": {
                        "type": "string",
                        "description": "A question to ask the user to get more information about their request",
                    }
                },
                "required": ["sequence_index", "question"],
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'execute_motion',
            'description': 'Executes a predefined motion sequence for the robot.',
            'parameters': {
                'type': 'object',
                'properties': {
                    "sequence_index": {
                        'type': 'number',
                        'description': 'The index of the sequence of the tool call. This is used to keep track of the order of the tool calls. The first tool call should have a sequence index of 0, the second tool call should have a sequence index of 1, and so on. This will be used to determine the execution order of the tool calls based on the user\'s request.'
                    },
                    'motion_name': {
                        'type': 'string',
                        'description': 'The name of the predefined motion to execute.'
                    }
                },
                'required': ["sequence_index", 'motion_name']
            }
        }
    },
]


class RobotBehavior:
    """
    This class contains the behavior of the robot.
    It is used in llm_config/user_config.py to customize the behavior of the robot.
    """

    def __init__(self):
        self.robot_functions_list = robot_functions_list


if __name__ == "__main__":
    pass
