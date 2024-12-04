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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.srv import MoveToPose
import time

class CartesianMotionController(Node):
    def __init__(self):
        super().__init__("cartesian_motion_controller")

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


def main(args=None):
    rclpy.init(args=args)
    node = CartesianMotionController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
