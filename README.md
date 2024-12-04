# llm-ur-control

A ROS2 package integrating large language models (LLMs) to control Universal Robots, leveraging the ROSA framework for natural langauge based robotic manipulation. This integration bridges the gap between robotics and intuitive, user-friendly control interfaces.

## Features

- **LLM Integration of Universal Robots:** Seamlessly interact with and control the Universal Robot (UR) using natural language commands through large language models (LLMs) such as ChatGPT.
- **Read Current Joint States:** Query the robot's state with intuitive commands like, _"What are the current joint angles in degrees?"_ to retrieve the joint positions. The agent listens to the /joint_states topic to fetch real-time joint positions and processes the data for user-friendly responses.
- **Read Current TCP Pose:** Query the robot's Tool Center Point (TCP) Pose with commands like, _"What is the current TCP Position"_ to retrieve the current end effector pose. The agent reads the position from the custom service (cartesian_motion_server).
- **Joint Motion Commands:**Issue joint motion commands with natural language, such as, _"Move the robot to joint positions -90, -90, -90, 90, 45, 180,"_ to initiate joint movements. The agent leverages the _/scaled_joint_trajectory_controller_ to move the robot joints.
- **Cartesian Motion Commands:** Control the robot's TCP directly in Cartesian space using commands like, _"Move the end effector 30cm in the z direction"_ to move the TCP upward. The agent communicates with the custom service _(cartesian_motion_server)_ which uses _/cartesian_motion_controller_ to change the TCP Pose.
- **Controller Management:** You can switch the controller that is controlling the robot between _cartesian_motion_controller_ and scaled_joint_trajectory_controller.

## Dependencies

Before using this package, ensure the following packages are installed and configured:

1. **[UR Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)**  
   Establishes a connection between ROS2 and Universal Robots.

2. **[ROSA Package](https://github.com/nasa-jpl/rosa)**  
   Enables integration of large language models with ROS2.

3. **[Cartesian Controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)**  
   Provides Cartesian control for use in ROS2.

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/cakh/llm-ur-control.git
   cd llm-ur-control
   ```

2. Install dependencies:
   Ensure the UR robot driver, ROSA package, and Cartesian controller are installed and sourced.

3. Build the package:
   ```bash
   colcon build
   source install/setup.bash
   ```

## Usage

### Tools and Capabilities

This package provides several tools for controlling the robot and monitoring its state:

1. **Publish Joint Positions**
   Publishes a `JointTrajectory` message to command the robot's joints.
   ```python
   publish_joint_positions(joint_positions: List[float], duration_sec: int = 5)
   ```
