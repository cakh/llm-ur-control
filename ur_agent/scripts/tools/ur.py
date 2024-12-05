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
from typing import List, Dict, Tuple
from sensor_msgs.msg import JointState
import threading
from controller_manager_msgs.srv import ListControllers
import time 
from controller_manager_msgs.srv import SwitchController
from custom_msgs.srv import MoveToPose
from geometry_msgs.msg import Pose, PoseStamped

_shared_node = None
joint_traj_publisher = None
current_joint_states = None
controller_status_client = None
switch_service_client = None
cartesian_motion_client = None
joint_states_received = False 
cartesian_motion_client = None
current_ef_pose = None


def initialize_node():
    global _shared_node, joint_traj_publisher, controller_status_client, switch_service_client, cartesian_motion_client,current_ef_pose
    if _shared_node is None:
        print("Initializing ROS node and starting spin thread...")
        rclpy.init()
        _shared_node = rclpy.create_node('ur_agent_node')
        joint_traj_publisher = _shared_node.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        _shared_node.create_subscription(JointState, '/joint_states', joint_state_callback, 10)
        controller_status_client = _shared_node.create_client(ListControllers, '/controller_manager/list_controllers')
        switch_service_client = _shared_node.create_client(SwitchController, '/controller_manager/switch_controller')
        cartesian_motion_client = _shared_node.create_client(MoveToPose, "move_to_pose")
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
    current_joint_states = list(msg.position)
    joint_states_received = True

def pose_callback(msg):
    global current_ef_pose
    current_ef_pose = msg

@tool
def publish_joint_positions(joint_positions: List[float], duration_sec: int = 5) -> str:
    """
        Publishes a `JointTrajectory` message to command the UR5e robot to move its joints to specified positions.
        Crictical: Check if `scaled_joint_trajectory_controller` is active. If not active, activate it before running.
        :param joint_positions: List of up to six joint positions in radians.
        :param duration_sec: Motion duration in seconds (default: 5).
        :return: Success message or error details.

    """
    global _shared_node, joint_traj_publisher, current_joint_states
    initialize_node()

    print("Waiting for joint states to be available...")

    try:

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
def list_controllers() -> str:
    """
    Retrieves the list of controllers and their statuses (active/inactive) from the robot.

    :return: A formatted string listing each controller's name and its status or an error message in case of failure.
    """
    global _shared_node, controller_status_client
    initialize_node()
    print("Fetching the list of controllers and their statuses...")

    try:
    # Ensure the service client is available
        if not controller_status_client.wait_for_service(timeout_sec=5.0):
            return "Error: Service /controller_manager/list_controllers not available."
        service_type = ListControllers        
        req = service_type.Request()
        future = controller_status_client.call_async(req)

        # Wait for the future to complete with a timeout
        start_time = time.time()
        timeout = 10  # seconds
        # Wait for the response
        while not future.done():
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return "Error: Timed out waiting for the list of controllers."

        response = future.result()
        # Format the controllers and their statuses
        controllers_info = []
        for controller in response.controller:
            status = "active" if controller.state == "active" else "inactive"
            controllers_info.append(f"{controller.name}: {status}")
        result = "\n".join(controllers_info)
        print(f"Controllers and statuses:\n{result}")
        return f"Controllers and their statuses:\n{result}"


    except Exception as e:
        error_msg = f"Error fetching controllers and their statuses: {e}"
        print(error_msg)
        return error_msg
    
@tool
def switch_controllers(enable_controller: int, disable_controller: int) -> str:
    """
    Switches robot controllers by enabling one and disabling another in a single operation.

    Controllers are identified by numbers:
    - 1: scaled_joint_trajectory_controller
    - 2: cartesian_motion_controller

    :param enable_controller: Number of the controller to enable.
    :param disable_controller: Number of the controller to disable.
    :return: Status message indicating success or failure."""
    global _shared_node
    initialize_node()
    # Map controller numbers to their names
    controller_map = {
        1: "scaled_joint_trajectory_controller",
        2: "cartesian_motion_controller"
    }

    if enable_controller not in controller_map or disable_controller not in controller_map:
        return ("Error: Invalid controller number(s). "
                "Available options: 1 (scaled_joint_trajectory_controller), 2 (cartesian_motion_controller).")
    # Get controller names
    start_controller = controller_map[enable_controller]
    stop_controller = controller_map[disable_controller]

    print(f"Enabling controller: {start_controller}, Disabling controller: {stop_controller}...")

    try:
        # Ensure the service client for switch_controllers is available
        if not switch_service_client.wait_for_service(timeout_sec=5.0):
            return "Error: Service /controller_manager/switch_controller not available."

        # Create and send the request
        req = SwitchController.Request()
        req.start_controllers = [start_controller]
        req.stop_controllers = [stop_controller]
        req.strictness = SwitchController.Request.STRICT

        future = switch_service_client.call_async(req)
        start_time = time.time()
        timeout = 10  # seconds
        while not future.done():
            rclpy.spin_once(_shared_node, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return "Error: Timed out waiting for controller switch response."
        
        response = future.result()
        # Check the response success
        if response.ok:
            return f"Successfully switched controllers: Enabled {start_controller}, Disabled {stop_controller}."
        else:
            return "Error: Failed to switch controllers. Service returned failure."

    except Exception as e:
        error_msg = f"Error switching controllers: {e}"
        print(error_msg)
        return error_msg
    
@tool
def cartesian_motion_request(x: float, y: float, z: float) -> str:
    """
        Moves the robot TCP to the mentioned cartesian position,
        If `cartesian_motion_controller` is not active, activate it before running.

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
        Retrieves the current end effector pose of the robot in Cartesian coordinates (`x`, `y`, `z`) and quaternion orientation (`qx`, `qy`, `qz`, `qw`).
        If `cartesian_motion_controller` is not active, activate it before running.

        :return: Current pose as a formatted string or an error message.
    """
    global _shared_node, current_ef_pose
    initialize_node()
    print("Waiting for current pose to be available...")
    try:
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
