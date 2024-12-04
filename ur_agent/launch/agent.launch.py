#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    cartesian_motion_control_node = Node(
        package='ur_agent',  
        executable='cartesian_motion_server.py', 
        name='cartesian_motion_server',
        output='screen',
        respawn=False,  
    )
    ur_agent_node = Node(
        package='ur_agent', 
        executable='ur_agent.py',
        name='ur_agent_node',
        output='screen',
    )

    # Build the launch description
    return LaunchDescription([
        cartesian_motion_control_node,
       # ur_agent_node,
    ])

