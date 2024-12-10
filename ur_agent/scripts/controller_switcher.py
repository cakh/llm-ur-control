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
from controller_manager_msgs.srv import SwitchController
from custom_msgs.srv import CustomSwitchController

class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')

        # Define conflicting controllers
        self.conflicting_controllers = [
            "cartesian_motion_controller",
            "scaled_joint_trajectory_controller",
        ]

        # Create a service to switch controllers
        self.controller_switcher_service = self.create_service(
            CustomSwitchController, "controller_switcher", self.handle_controller_switcher
        )
        self.get_logger().info("Controller Switcher Node Initialized")
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        # Instance variable to store the result
        self.switch_result = None
        while not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')
    
    def handle_controller_switcher(self, request, response):
        """Handle the service request to switch controllers."""
        target_controller = request.controller_name  # The name of the controller to activate
        self.get_logger().info(f"Request to activate controller: {target_controller}")

        if target_controller not in self.conflicting_controllers:
            response.success = False
            response.message = f"Controller {target_controller} is not in the predefined list of controllers."
            self.get_logger().error(response.message)
            return response
        request = SwitchController.Request()
        request.start_controllers.append(target_controller)
        for controller in self.conflicting_controllers:
            if controller != target_controller:
                request.stop_controllers.append(controller)
        request.strictness = 2  # STRICT

        # Call the service asynchronously
        future = self.switch_controller_client.call_async(request)
        # Wait for the future to complete
        # Assign a callback to handle the result of the future
        future.add_done_callback(lambda f: self.future_callback(f, response))
        response.message = ("Forwarded switch task successful")
        response.success = True
        return response

    def future_callback(self, future, response):
        """Callback to handle the result of the future."""
        try:
            result = future.result()
            # Store the result in an instance variable
            self.switch_result = result.ok
            if(result.ok):
                self.get_logger().info(f"Controller switch successful: {result}")
            else:
                self.get_logger().info(f"Controller switch failed: {result}")
        except Exception as e:
            self.switch_result = False
            self.get_logger().error(f"Controller switch failed: {e}")

def main(args=None):
    rclpy.init(args=args)

    # Create a client node
    controller_switcher = ControllerSwitcher()
    rclpy.spin(controller_switcher)
    #controller_switcher.send_request()
    #controller_switcher.handle_response()

    # Shutdown the client
    controller_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
    def send_request(self):
        # Create a request message
        request = SwitchController.Request()
        request.start_controllers = ['scaled_joint_trajectory_controller']
        request.stop_controllers = ['cartesian_motion_controller']
        request.strictness = 2  # STRICT

        # Send the request scaled_joint_trajectory_controller
        self.future = self.switch_controller_client.call_async(request)

    def handle_response(self):
        # Wait for the response
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info(f'Response: {self.future.result()}')
        else:
            self.get_logger().error(f'Service call failed: {self.future.exception()}')

"""