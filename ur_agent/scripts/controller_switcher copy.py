from controller_manager_msgs.srv import ListControllers, LoadController, ConfigureController
from custom_msgs.srv import CustomSwitchController
import rclpy
from rclpy.node import Node


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__("controller_switcher")

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

    def handle_controller_switcher(self, request, response):
        """Handle the service request to switch controllers."""
        target_controller = request.controller_name  # The name of the controller to activate
        self.get_logger().info(f"Request to activate controller: {target_controller}")

        # Validate target controller
        if target_controller not in self.conflicting_controllers:
            response.success = False
            response.message = f"Controller {target_controller} is not in the predefined list of controllers."
            self.get_logger().error(response.message)
            return response

        # Ensure the target controller is loaded and activated
        if not self.prepare_controller(target_controller):
            response.success = False
            response.message = f"Failed to prepare controller: {target_controller}"
            self.get_logger().error(response.message)
            return response

        # Deactivate conflicting controllers
        for controller in self.conflicting_controllers:
            if controller != target_controller and self.is_controller_active(controller):
                self.get_logger().info(f"Deactivating conflicting controller: {controller}")
                if not self.controller_switcher(controller, deactivate=True):
                    response.success = False
                    response.message = f"Failed to deactivate conflicting controller: {controller}"
                    self.get_logger().error(response.message)
                    return response

        # Activate the target controller
        self.get_logger().info(f"Activating controller: {target_controller}")
        if self.controller_switcher(target_controller, activate=True):
            response.success = True
            response.message = f"Controller {target_controller} activated successfully."
        else:
            response.success = False
            response.message = f"Failed to activate controller: {target_controller}"
            self.get_logger().error(response.message)

        return response

    def prepare_controller(self, controller_name):
        """Ensure the controller is loaded, configured, and ready to be activated."""
        if not self.is_controller_loaded(controller_name):
            self.get_logger().info(f"Loading controller: {controller_name}")
            if not self.load_controller(controller_name):
                self.get_logger().error(f"Failed to load controller: {controller_name}")
                return False

        if not self.is_controller_configured(controller_name):
            self.get_logger().info(f"Configuring controller: {controller_name}")
            if not self.configure_controller(controller_name):
                self.get_logger().error(f"Failed to configure controller: {controller_name}")
                return False

        return True

    def is_controller_loaded(self, controller_name):
        """Check if the controller is loaded."""
        return self.get_controller_state(controller_name) != "not loaded"

    def is_controller_configured(self, controller_name):
        """Check if the controller is configured."""
        return self.get_controller_state(controller_name) in ["inactive", "active"]

    def is_controller_active(self, controller_name):
        """Check if the controller is active."""
        return self.get_controller_state(controller_name) == "active"

    def get_controller_state(self, controller_name):
        """Get the current state of a controller."""
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
            if controller.name == controller_name:
                return controller.state

        return "not loaded"

    def load_controller(self, controller_name):
        """Load the specified controller."""
        load_controller_client = self.create_client(LoadController, "/controller_manager/load_controller")
        if not load_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Load Controller service is not available.")
            return False

        request = LoadController.Request()
        request.name = controller_name
        future = load_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error("Failed to call LoadController service.")
            return False

        response = future.result()
        return response.ok

    def configure_controller(self, controller_name):
        """Configure the specified controller."""
        configure_controller_client = self.create_client(ConfigureController, "/controller_manager/configure_controller")
        if not configure_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Configure Controller service is not available.")
            return False

        request = ConfigureController.Request()
        request.name = controller_name
        future = configure_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error("Failed to call ConfigureController service.")
            return False

        response = future.result()
        return response.ok

    def controller_switcher(self, controller_name, activate=False, deactivate=False):
        """Switch the state of the specified controller."""
        controller_switcher_client = self.create_client(CustomSwitchController, "/controller_manager/controller_switcher")
        if not controller_switcher_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Switch Controller service is not available.")
            return False

        request = CustomSwitchController.Request()
        if activate:
            request.activate_controllers.append(controller_name)
        if deactivate:
            request.deactivate_controllers.append(controller_name)

        request.strictness = CustomSwitchController.Request.BEST_EFFORT

        future = controller_switcher_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self.get_logger().error("Failed to call _SwitchController service.")
            return False

        response = future.result()
        return response.ok


def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()