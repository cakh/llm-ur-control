# llm-ur-control

A ROS2 package integrating large language models (LLMs) to control Universal Robots, leveraging the ROSA framework for natural langauge based robotic manipulation. This integration bridges the gap between robotics and intuitive, user-friendly control interfaces.
## llm-ur-control Demo
[Watch the Demo Video](videos/uragent_demo.mp4)
## Features

- **LLM Integration of Universal Robots:** Seamlessly interact with and control the Universal Robot (UR) using natural language commands through large language models (LLMs) such as ChatGPT.
- **Read Current Joint States:** Query the robot's state with intuitive commands like, _"What are the current joint angles in degrees?"_.
- **Joint Motion Commands:** Issue joint motion commands with natural language, such as, _"Move the robot to joint positions -90, -90, -90, 90, 45, 180,"_. 
- **Cartesian Motion Commands:** Control the robot's TCP directly in Cartesian space using commands like, _"Move the end effector 30cm in the z direction"_. 
- **More features in planning** If you want any features, open up an issue.
- For more features and capabilites refer to the [Tools and Capabilities](#tools-and-capabilities) section.

## ⚠️ Warning: Test Phase

This project is currently in the **test phase** and may cause the robot to move unexpectedly or randomly. The project has been tested with ROS2 humble and UR5e robot.

### Recommendations:
- **Use a simulated robot** for testing to avoid potential damage to physical hardware.
- Exercise extreme caution when using a real robot.

### Disclaimer:
The author is **not responsible** for any issues, damages, or malfunctions that occur as a result of using this package. Use it at your own risk.

## Dependencies

Before using this package, ensure the following packages are installed and configured:

1. **[UR Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)**  
   Establishes a connection between ROS2 and Universal Robots. Source (!) install this package and ensure that you are able to command the robot, for instance using MoveIt!

   **Tip** If the robot does not move, check if the _scaled_joint_trajectory_controller_ is active.

3. **[ROSA Package](https://github.com/nasa-jpl/rosa)**  
   Enables integration of large language models with ROS2. It is not required to install the package from source, instead you can install the ROSA using
   ```bash
   pip3 install jpl-rosa
   ```
To ensure proper functionality, test the package using the Docker container provided in its repository. This step is crucial for verifying that all necessary configurations for integrating the LLM with ROS (such as adding the API key) have been completed.

3. **[Cartesian Controller](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)**  
   Provides Cartesian control for use in ROS2. The package has to be installed in the same workspace as the llm-ur-control. llm-ur-control utilizes the _cartesian_motion_controller_ from this package to enable TCP motion in cartesian space.

   **Tip** After cloning the repo, you can safely delete the folders _cartesian_controller_simulation_ and _cartesian_controller_tests_ for colcon build to complete successfully.

## Installation

1. Install [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). The package has been tested on ROS2 humble. Support for any other distribution cannot be guaranteed.

3. Make sure that all the dependencies are properly installed and working.

1. Clone the repository:

   ```bash
   cd ros2_ws/src
   git clone https://github.com/cakh/llm-ur-control.git
   cd ..
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Update UR Controller:
   llm-ur-control utilizes the cartesian_motion_controller which is not used by the ur_robot_driver as default controller. Hence the file _ur_robot_driver/config/ur_controllers.yaml_ has to be modified with this controller. For this purpose, add the following lines to the above mentioned file in the _ur_robot_driver_ package in your workspace:
    ```yaml
       cartesian_motion_controller:
         type: cartesian_motion_controller/CartesianMotionController
   ```
   
   ```yaml
   cartesian_motion_controller:
     ros__parameters:
       # See the cartesian_compliance_controller
       end_effector_link: "$(var tf_prefix)tool0"
       robot_base_link: "$(var tf_prefix)base_link"
       joints:
         - $(var tf_prefix)shoulder_pan_joint
         - $(var tf_prefix)shoulder_lift_joint
         - $(var tf_prefix)elbow_joint
         - $(var tf_prefix)wrist_1_joint
         - $(var tf_prefix)wrist_2_joint
         - $(var tf_prefix)wrist_3_joint
   
       # See the cartesian_compliance_controller
       command_interfaces:
         - position
           #- velocity
   
       solver:
         error_scale: 1.0
         iterations: 10
         publish_state_feedback: True
   
       pd_gains:
         trans_x: { p: 1.0 }
         trans_y: { p: 1.0 }
         trans_z: { p: 1.0 }
         rot_x: { p: 0.5 }
         rot_y: { p: 0.5 }
         rot_z: { p: 0.5 }
   ```
An example of the modified _ur_controllers.yaml_ can be found in this repo under _ur_agent/config_

4. Update the launch file of _ur_moveit_config_ under the _ur_robot_driver_ repo to spawn necessary controllers. Edit the _ur_moveit_config/launch/ur_moveit.launch.py_ to add the following code above the line 274 (nodes_to_start = ...):
   ```py
   cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="cartesian_motion_controller_spawner",
        output="screen",
        arguments=["cartesian_motion_controller"],
    )
    scaled_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="scaled_joint_trajectory_controller",
        output="screen",
        arguments=["scaled_joint_trajectory_controller"],
    )
   ```
   and modify the _nodes_to_start_ variable to include both these controllers:
      ```py
    nodes_to_start = [move_group_node, rviz_node, servo_node, cartesian_motion_controller_spawner, scaled_trajectory_controller_spawner]

   ```
      An example of the modified _ur_moveit_config/launch/ur_moveit.launch.py_ can be found in this repo under _src/ur_agent/config_

3. Build the package:
   ```bash
   cd ros2_ws/
   colcon build
   source install/setup.bash
   ```

4. If you are using the OpenAI API, you need to add your API key to your environment variables. 
    ```bash
      gedit ~/.bashrc
      ```
   Add the following line at the end of the file:
    ```plaintext
      export OPENAI_API_KEY=<your_openai_api_key>
      ```
   Save and close the file, then apply the changes:
    ```bash
      source ~/.bashrc
   ```

## Usage

1. **Establish a connection between the simulated (or real) robot and ROS2** using the _[UR Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)_. Ensure that you can control the robot using MoveIt!.

   - **To start the simulated UR5e robot**, run:
     ```bash
     ros2 run ur_client_library start_ursim.sh ur_type:=ur5e
     ```

   - **To start the driver**, in a new terminal, run:
     ```bash
     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false
     ```
     Make sure you see the following:
     ```bash
     [UR_Client_Library:]: Robot connected to reverse interface. Ready to receive control commands.
     ```

   - **To start MoveIt!**, in a new terminal, run:
     ```bash
     ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
     ```

2. **Verify Robot Control**:
   - Open RViz and ensure that you can move the robot using the interactive marker.
   - Use _Plan & Execute_ in Rviz to execute robot motion.
   - If this step does not work, check whether the _scaled_joint_trajectory_controller_ is active.
   - Without the robot (simulated or real) being successfully controlled in this step, the agent will not be able to execute any motion commands.
     
3. **Start the services for ur_agent**
   - Launch the services for the ur_agent by running:
   ```bash
     ros2 launch ur_agent agent.launch.py
     ```
   To know more about the services, check the [How the Agent Controls the Robot](#how-the-agent-controls-the-robot) section.

4. **Start the ur_agent**
   ```bash
     ros2 run ur_agent ur_agent.py 
     ```
You should see the following on your terminal:

5. **Control the robot**
   You should now be able to control the robot using natural language. Try out the following examples:
   
   ```plaintext
   what are the current joint positions?
   ```
   ```plaintext
   can you convert these values to degrees?
   ```
   ```plaintext
   What is the current tcp position?
   ```
   ```plaintext
   move the tcp 25cm in the y direction
   ```

### How the Agent Controls the Robot

- The **Agent** primarily interacts with the robot using custom **ROS2 services**.
- Each _custom_ service is implemented as an executable located in the `scripts` folder.
- A launch file is provided to start all the required _custom_ services. You can start the services using the following command:
  ```bash
  ros2 launch ur_agent 
  ```

### Tools and Capabilities

This package provides several tools for controlling the robot and monitoring its state. Apart from the custom tools described below, ROSA can utilize a wide range of ROS2 topics and services to perform its actions autonomously. In this context, a tool is a functional module integrated into ROSA that provides a specific capability, such as retrieving data, commanding motions, or managing controllers.
To see the full list of available tools, simply type:
```plaintext
list available tools
```
in the command prompt.
Below is an overview of one of the key tools available:

#### Publish Joint Positions
The `publish_joint_positions` tool enables you to command the UR5e robot to move its six joints to specified positions over a defined duration.

**How it Works**
- The tool first checks if the scaled_joint_trajectory_controller is active.
- It initializes the current joint states and updates only the specified joints, keeping the rest unchanged.
- The tool constructs a trajectory message, including the target positions and motion duration, and publishes it to the appropriate topic.

**Example**:
To move the robot's joints to specific positions over 5 seconds, you might issue:
```plaintext
Move the robot's joints to positions [-90, -90, -120, 0, -90, 180] in 5 seconds.
```

#### Retrieve Joint States
The `retrieve_joint_states` tool allows you to query the current positions of the robot's joints.

**How it Works**
- It waits for the robot's joint states to be published on the /joint_states topic.
- Once the joint states are received, it formats the data into a human-readable string.
- If joint states are unavailable within a timeout period, the tool returns an error message.
  
**Example**:
```plaintext
What are the current joint states in degrees
```

#### List Controllers
The `list_controllers` tool allows you to check the currently loaded controllers on the robot and their statuses.

**How it Works**
- The tool sends a service request to /controller_manager/list_controllers
- It parses the response to extract the controller names and their statuses.
- The output is formatted into a human-readable list, making it easy for the user to identify active and inactive controllers.
  
**Example**:
```plaintext
Give me a list of all controllers
```


#### Switch Controllers
The `switch_controllers` tool allows you to dynamically switch between controllers on the robot, enabling one controller while disabling another in a single operation.

**Key Features**
- Supports switching between:
   - _scaled_joint_trajectory_controller_ (ID: 1)
   - _cartesian_motion_controller_ (ID: 2)
- Ensures the correct controller is active for specific robot operations, such as joint motion or Cartesian motion.
  
**How it Works**
- The tool maps controller IDs to their respective names (_scaled_joint_trajectory_controller_ and _cartesian_motion_controller_).
- It constructs a service request with the desired controllers to enable and disable.
- The tool waits for the service to respond and checks the success status of the operation.
  
**Example**:
```plaintext
Can you switch to scaled joint trajectory controller?
```

#### Cartesian Motion Request
The `Cartesian Motion Request` tool enables precise Cartesian control of the robot’s Tool Center Point (TCP) by specifying its desired position in Cartesian coordinates.

**How it Works**
- The tool validates the input values to ensure they are provided as floats.
- It initializes a Pose message with the desired position and a default orientation.
- The tool communicates with the _move_to_pose_ (*custom*) service, sending the request and waiting for a synchronous response.
  
**Example**:
```plaintext
can you move the tcp to 0.2,0.5,0.7
```
#### Get Current Pose
The `Get Current Pose` tool allows you to retrieve the current position and orientation of the robot’s Tool Center Point (TCP).

**How it Works**
- The tool subscribes to the /cartesian_motion_controller/current_pose topic to fetch the pose.
- Waits for the topic to update with the current TCP pose.
- Formats the position and orientation into a human-readable string.
  
**Example**:
```plaintext
What is the current position of the TCP
```
## Bugs
- The commands may not work consistently, as the same request to the Agent can sometimes succeed and other times fail. This behavior appears to stem from the inherent limitations of large language models (LLMs), which can introduce variability in interpreting and processing commands.
- If there is an error finding the pose of the tcp or moving the tcp, ask the agent to switch from _scaled_joint_trajectory_controller_ to _cartesian_motion_controller_. The agent is supposed to do this on its own as mentioned in the _prompts.py_ but misses this step sometimes.
