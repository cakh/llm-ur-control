#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rclpy
import rclpy.publisher
from langchain.agents import tool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import List
from sensor_msgs.msg import JointState
import threading
import time 
from custom_msgs.srv import MoveToPose, CustomSwitchController
from geometry_msgs.msg import Pose, PoseStamped

_shared_node = None
joint_traj_publisher = None
current_joint_states = None
cartesian_motion_client = None
joint_states_received = False 
cartesian_motion_client = None
current_ef_pose = None
controller_switcher_client = None


def initialize_node():
    global _shared_node, joint_traj_publisher, cartesian_motion_client,current_ef_pose, controller_switcher_client
    if _shared_node is None:
        print("Initializing ROS node and starting spin thread...")
        rclpy.init()
        _shared_node = rclpy.create_node('ur_agent_node')
        joint_traj_publisher = _shared_node.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        _shared_node.create_subscription(JointState, '/joint_states', joint_state_callback, 10)
        cartesian_motion_client = _shared_node.create_client(MoveToPose, "move_to_pose")
        controller_switcher_client = _shared_node.create_client(CustomSwitchController, "controller_switcher")
        _shared_node.create_subscription(PoseStamped, '/cartesian_motion_controller/current_pose', pose_callback, 10)

        def spin_node():
            try:
                rclpy.spin(_shared_node)
            except Exception as e:
                print(f"Error in spin thread: {e}")
                rclpy.shutdown()
        # Start spinning the node in a background thread
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        print("Spin thread started.")


def joint_state_callback(msg):
    global current_joint_states,joint_states_received
    current_joint_states = [msg.position[-1]] + list(msg.position[:-1])
    joint_states_received = True

def pose_callback(msg):
    global current_ef_pose
    current_ef_pose = msg
    
@tool
def publish_joint_positions(joint_positions: List[float], duration_sec: int = 5) -> str:
    """
        Publishes a `JointTrajectory` message to command the UR5e robot to move its joints to specified positions.
        Crictical: activate scaled_joint_trajectory_controller before running.
        
        :param joint_positions: List of up to six joint positions in radians.
        :param duration_sec: Motion duration in seconds (default: 5).
        :return: Success message or error details.

    """
    global _shared_node, joint_traj_publisher, current_joint_states
    initialize_node()

    print("Waiting for joint states to be available...")

    try:
        
        print("scaled_joint_trajectory_controller is active.")

        # Timeout for waiting for joint states
        timeout = 10  # seconds
        start_time = time.time()

        while current_joint_states is None:
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return "Error: Timed out waiting for joint states to become available."
            
        print("Current joint states are initialized. Testing publish_joint_positions...")
        # Validate input length
        if len(joint_positions) > 6:
            return "Error: Too many joint positions provided. Expected at most 6."

        # Handle uninitialized current_joint_states
        if current_joint_states is None:
            return "Error: Current joint states are not initialized yet. Wait for /joint_states messages."

        # Retrieve current joint states for unspecified joints
        full_joint_positions = current_joint_states[:]
        for i, pos in enumerate(joint_positions):
            full_joint_positions[i] = pos  # Update specified positions

        # Prepare the message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = _shared_node.get_clock().now().to_msg()
        trajectory_msg.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = full_joint_positions
        point.time_from_start.sec = duration_sec
        trajectory_msg.points.append(point)

        # Publish the trajectory message
        joint_traj_publisher.publish(trajectory_msg)

        result = f"Published joint trajectory: {joint_positions} over {duration_sec} seconds."
        # Confirm execution
        return(result)

    except Exception as e:
        error_msg = f"Failed to publish joint trajectory: {e}"
        return error_msg

@tool
def retrieve_joint_states() -> str:
    """
        Retrieves the current joint states of the UR5e robot.

        :return: Joint states as a formatted string or an error message.
    """
    global _shared_node, current_joint_states, joint_states_received
    initialize_node()
    print("Waiting for joint states to be available...")
    try:
        timeout = 10  # seconds
        start_time = time.time()

        # Wait for the current_joint_states to be initialized
        while not joint_states_received:
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return "Error: Timed out waiting for joint states to become available."
       
        # Spin to ensure the latest joint state message is processed
        for _ in range(3):  # Spin multiple times to avoid timing issues
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
        
        # Format the joint states as a string for return
        joint_states_str = ", ".join([f"{state:.4f}" for state in current_joint_states])
        result = f"Current joint states: [{joint_states_str}]"
        print(result)
        return result

    except Exception as e:
        error_msg = f"Error retrieving joint states: {e}"
        print(error_msg)
        return error_msg

@tool
def activate_controller_request(controller_name: str) -> str:
    """
        Activates the specified controller.

        :param controller_name: The name of the controller to activate ("cartesian_motion_controller" or "scaled_joint_trajectory_controller")
        :return: Service response or error message.
    """
    global controller_switcher_client
    initialize_node()
    valid_controllers = [
        "cartesian_motion_controller",
        "scaled_joint_trajectory_controller",
    ]
    try:
        if controller_name not in valid_controllers:
            return f"Error: Controller {controller_name} is not valid. Valid controllers are: {valid_controllers}"

        if not controller_switcher_client.wait_for_service(timeout_sec=5.0):
            return "Error: Service switch_controller not available."

        request = CustomSwitchController.Request()
        request.controller_name = controller_name
        future = controller_switcher_client.call_async(request)

        # Wait for the response with a timeout
        timeout = 10  # seconds
        start_time = time.time()

        while not future.done():
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return f"Error: Timed out waiting for the ControllerSwitcher service response for {controller_name}."

        response = future.result()
        if response.success:
            return f"Controller {controller_name} activated successfully. Message: {response.message}"
        else:
            return f"Error: {response.message}"
    
    except Exception as e:
        error_msg = f"Error switching controller: {e}"
        print(error_msg)
        return error_msg
    

@tool
def cartesian_motion_request(x: float, y: float, z: float) -> str:
    """
        Moves the robot TCP to the mentioned cartesian position,
        Crictical: activate cartesian_motion_controller before running.

        :param x: Target x-coordinate (meters).
        :param y: Target y-coordinate (meters).
        :param z: Target z-coordinate (meters).
        :return: Service response or error message.
    """
    global cartesian_motion_client
    initialize_node()
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0

    try:
        print("cartesian_motion_controller is active.")

        if not cartesian_motion_client.wait_for_service(timeout_sec=5.0):
            return "Error: Service move_to_pose not available."
        request = MoveToPose.Request()
        request.target_pose = pose
        future = cartesian_motion_client.call_async(request)

        # Wait for the response with a timeout
        timeout = 10  # seconds
        start_time = time.time()

        while not future.done():
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return "Error: Timed out waiting for the MoveToPose service response."

        response = future.result()
        return f"Response: {response.message}"
    
    except Exception as e:
        error_msg = f"Error moving the robot: {e}"
        print(error_msg)
        return error_msg
    
@tool
def get_current_pose() -> str:
    """
        Retrieves the current end effector pose of the robot in Cartesian coordinates.
        Crictical: activate cartesian_motion_controller before running.

        :return: Current pose as a formatted string or an error message.
    """
    global _shared_node, current_ef_pose
    initialize_node()
    print("Waiting for current pose to be available...")
    try:
        print("cartesian_motion_controller is active.")

        start_time = time.time()
        timeout = 10  # seconds

        while current_ef_pose is None:
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                print("Error: Timed out waiting for the current pose to become available.")
                return "Error: Timed out waiting for the current pose to become available."
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
        # Format the current pose as a string for return
        pose_str = (
            f"Position: ({current_ef_pose.pose.position.x:.4f}, "
            f"{current_ef_pose.pose.position.y:.4f}, "
            f"{current_ef_pose.pose.position.z:.4f}), "
            f"Orientation: ({current_ef_pose.pose.orientation.x:.4f}, "
            f"{current_ef_pose.pose.orientation.y:.4f}, "
            f"{current_ef_pose.pose.orientation.z:.4f}, "
            f"{current_ef_pose.pose.orientation.w:.4f})"
        )
        print(f"Current pose: {pose_str}")
        return f"Current pose: {pose_str}"

    except Exception as e:
        error_msg = f"Error retrieving joint states: {e}"
        print(error_msg)
        return error_msg
