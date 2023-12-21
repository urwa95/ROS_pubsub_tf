# ROS 2 Transform Communication Package

## Description
This ROS 2 package comprises two nodes, `TransformPublisher` and `TransformSubscriber`, designed to demonstrate the handling and communication of transformation data. The `TransformPublisher` node periodically publishes transformation data, including translation and rotation, simulating a scenario like a camera's movement in relation to a drone. The `TransformSubscriber` node subscribes to this transformation data and logs the received information, displaying the frame IDs, translation, and rotation. This package exemplifies the publisher-subscriber model in ROS 2 using the `geometry_msgs/msg/TransformStamped` message type.

## File Structure

your_package/
|-- src/
| |-- transform_publisher.cpp
| |-- transform_subscriber.cpp
|-- CMakeLists.txt
|-- package.xml

markdown


- **src/**: Contains the source files.
  - **transform_publisher.cpp**: Implements the transform publisher node.
  - **transform_subscriber.cpp**: Implements the transform subscriber node.
- **CMakeLists.txt**: CMake file for configuring the build of the ROS 2 package.
- **package.xml**: Provides metadata and dependencies of the package.

## Build Instructions
1. **Place the Package**: Copy your package directory into the `src` folder of your ROS 2 workspace.

2. **Compile the Package**:
   - Navigate to the root of your ROS 2 workspace.
   - Execute `colcon build --packages-select your_package_name` to build your package. Replace `your_package_name` with the actual name of your package.
   - Source the setup script after building: `source install/setup.bash`.

3. **Run the Nodes**:
   - Start the publisher node with `ros2 run your_package_name transform_publisher`.
   - In a separate terminal (after sourcing the setup script), start the subscriber node with `ros2 run your_package_name transform_subscriber`.

4. **Verify Operation**: Check the output in the subscriber node's terminal to confirm it is correctly receiving and logging the transform data published by the publisher node.

This package is a practical example for understanding ROS 2's approach to handling and communicating transformation data, suitable for applications in robotics and simulations.
