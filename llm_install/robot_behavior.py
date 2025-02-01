#!/usr/bin/env python3
# -*- coding: utf-8 -*-

robot_functions_list = [
    {
        'type': 'function',
        'function': {
            'name': 'publish_cmd_vel',
            'description': 'Publishes a cmd_vel message to control the movement of the robot, including linear and angular velocities.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'linear_x': {
                        'type': 'number',
                        'description': 'Linear velocity along the x-axis.'
                    },
                    'linear_y': {
                        'type': 'number',
                        'description': 'Linear velocity along the y-axis.'
                    },
                    'linear_z': {
                        'type': 'number',
                        'description': 'Linear velocity along the z-axis.'
                    },
                    'angular_x': {
                        'type': 'number',
                        'description': 'Angular velocity around the x-axis.'
                    },
                    'angular_y': {
                        'type': 'number',
                        'description': 'Angular velocity around the y-axis.'
                    },
                    'angular_z': {
                        'type': 'number',
                        'description': 'Angular velocity around the z-axis.'
                    }
                },
                'required': ['linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z']
            }
        }
    },
    {
        'type': 'function',
        'function': {
            'name': 'publish_goal_pose',
            'description': 'Publishes a target pose to move the robot to a specific position and orientation in 3D space.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'x': {
                        'type': 'number',
                        'description': 'The x-coordinate of the target pose.'
                    },
                    'y': {
                        'type': 'number',
                        'description': 'The y-coordinate of the target pose.'
                    },
                    'z': {
                        'type': 'number',
                        'description': 'The z-coordinate of the target pose.'
                    },
                    'roll': {
                        'type': 'number',
                        'description': 'The roll angle (rotation around x-axis) in radians.'
                    },
                    'pitch': {
                        'type': 'number',
                        'description': 'The pitch angle (rotation around y-axis) in radians.'
                    },
                    'yaw': {
                        'type': 'number',
                        'description': 'The yaw angle (rotation around z-axis) in radians.'
                    }
                },
                'required': ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            }
        }
    },
    {
        'type': 'function',
        'function': {
            'name': 'provide_answer',
            'description': 'Delivers an answer to question',
            'parameters': {
                'type': 'object',
                'properties': {
                    'answer': {
                        'type': 'string',
                        'description': 'An answer to be delivered by the robot.'
                    }
                },
                'required': ['answer']
            }
        }
    }
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