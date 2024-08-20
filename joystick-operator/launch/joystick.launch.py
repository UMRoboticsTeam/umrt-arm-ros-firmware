from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('joystick_operator'),'launch','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    joystick_operator_node = Node(
            package='joystick_operator', 
            executable='joystick_operator',
            name = 'joystick_operator_node',
            parameters=[joy_params]
            )

    return LaunchDescription([
        joy_node,
        joystick_operator_node
    ])

