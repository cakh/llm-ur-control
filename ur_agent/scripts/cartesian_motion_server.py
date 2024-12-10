#!/usr/bin/env python3
# Copyright (c) 2024. Chair of Handling and Assembly Technology, TU Chemnitz. All rights reserved.
# This software is licensed under the MIT License. Permission is hereby granted, 
# free of charge, to any person obtaining a copy of this software and associated 
# documentation files (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, merge, publish, 
# distribute, sublicense, and/or sell copies of the Software, and to permit persons 
# to whom the Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.srv import MoveToPose
import time
from controller_manager_msgs.srv import ListControllers, LoadController, ConfigureController

class CartesianMotionController(Node):
    def __init__(self):
        super().__init__("cartesian_motion_controller")

        # Ensure the controller is loaded and configured
        if not self.ensure_controller_loaded_and_configured():
            self.get_logger().error("Failed to prepare the Cartesian Motion Controller. Exiting.")
            rclpy.shutdown()
            return
        
        # Subscriber to get the current pose of the robot
        self.current_pose_subscription = self.create_subscription(
            PoseStamped,
            "/cartesian_motion_controller/current_pose",
            self.pose_callback,
            10,
        )
        # Publisher to send the target pose to the robot
        self.target_pose_publisher = self.create_publisher(
            PoseStamped, "/cartesian_motion_controller/target_frame", 10
        )
        # Service to handle pose requests
        self.move_to_pose_service = self.create_service(
            MoveToPose, "move_to_pose", self.handle_move_to_pose
        )

        # Variables to store robot state
        self.current_pose = None
        self.position_tolerance = 0.01  # meters
        self.orientation_tolerance = 0.01  # quaternion tolerance
        self.timeout = 10.0  # seconds

        self.get_logger().info("Cartesian Motion Controller Node Initialized")

    def pose_callback(self, msg):
        """Callback to update the current pose of the robot."""
        self.current_pose = msg

    def handle_move_to_pose(self, request, response):
        """Service handler to move the robot to the requested pose."""
        # Extract target pose from the service request
        target_pose = request.target_pose
        self.get_logger().info(f"Received target pose: {target_pose}")

        # Publish the target pose
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = "base_link"
        target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        target_pose_stamped.pose = target_pose

        self.target_pose_publisher.publish(target_pose_stamped)
        self.get_logger().info("Target pose published.")

        # Wait for the robot to reach the target pose
        if self.wait_until_target_reached(target_pose_stamped):
            response.success = True
            response.message = "Target pose reached successfully."
        else:
            response.success = False
            response.message = "Failed to reach target pose within timeout."

        return response

    def wait_until_target_reached(self, target_pose):
        """Wait until the robot reaches the target pose."""
        start_time = self.get_clock().now().nanoseconds / 1e9

        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < self.timeout:
            if self.current_pose is None:
                self.get_logger().info("Waiting for current pose...")
                time.sleep(0.1)
                continue

            # Check position and orientation differences
            position_error = (
                abs(self.current_pose.pose.position.x - target_pose.pose.position.x),
                abs(self.current_pose.pose.position.y - target_pose.pose.position.y),
                abs(self.current_pose.pose.position.z - target_pose.pose.position.z),
            )
            orientation_error = (
                abs(self.current_pose.pose.orientation.x - target_pose.pose.orientation.x),
                abs(self.current_pose.pose.orientation.y - target_pose.pose.orientation.y),
                abs(self.current_pose.pose.orientation.z - target_pose.pose.orientation.z),
                abs(self.current_pose.pose.orientation.w - target_pose.pose.orientation.w),
            )

            # Check if within tolerances
            if all(diff <= self.position_tolerance for diff in position_error) and \
               all(diff <= self.orientation_tolerance for diff in orientation_error):
                self.get_logger().info("Target pose reached!")
                return True

            self.get_logger().info("Moving towards target pose...")
            time.sleep(0.1)  # Small delay to avoid busy looping

        self.get_logger().warn("Timeout reached before target pose. check if cartesian_motion_controller is active")
        return False
    
    def ensure_controller_loaded_and_configured(self):
        """Ensure the controller is loaded, configured, and ready."""
        if not self.is_controller_loaded():
            self.get_logger().info("Controller is not loaded. Attempting to load it...")
            if not self.load_controller():
                self.get_logger().error("Failed to load the controller.")
                return False

        if not self.is_controller_configured():
            self.get_logger().info("Controller is not configured. Attempting to configure it...")
            if not self.configure_controller():
                self.get_logger().error("Failed to configure the controller.")
                return False

        self.get_logger().info("Controller is loaded and configured.")
        return True

    def is_controller_loaded(self):
        """Check if the controller is loaded."""
        controller_state = self.get_controller_state()
        return controller_state != "not loaded"

    def is_controller_configured(self):
        """Check if the controller is configured."""
        controller_state = self.get_controller_state()
        return controller_state in ["inactive", "active"]
    
    def get_controller_state(self):
        """Get the current state of the controller."""
        list_controllers_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        if not list_controllers_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Controller Manager service is not available.")
            return "not loaded"

        request = ListControllers.Request()
        future = list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error("Failed to call ListControllers service.")
            return "not loaded"

        response = future.result()
        for controller in response.controller:
            if controller.name == "cartesian_motion_controller":
                return controller.state

        return "not loaded"


    def load_controller(self):
        """Load the controller."""
        load_controller_client = self.create_client(LoadController, "/controller_manager/load_controller")
        if not load_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Load Controller service is not available.")
            return False

        request = LoadController.Request()
        request.name = "cartesian_motion_controller"
        future = load_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error("Failed to call LoadController service.")
            return False

        response = future.result()
        if not response.ok:
            self.get_logger().error(f"Failed to load controller: {response}")
        return response.ok


    def configure_controller(self):
        """Configure the controller."""
        configure_controller_client = self.create_client(ConfigureController, "/controller_manager/configure_controller")
        if not configure_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Configure Controller service is not available.")
            return False

        request = ConfigureController.Request()
        request.name = "cartesian_motion_controller"
        future = configure_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error("Failed to call ConfigureController service.")
            return False

        response = future.result()
        return response.ok

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CartesianMotionController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
