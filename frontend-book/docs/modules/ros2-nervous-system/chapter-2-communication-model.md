# Chapter 2: ROS 2 Communication Model

## Understanding Nodes, Topics, Services, and Actions

In the previous chapter, we introduced ROS 2 as the nervous system of a robot. Now, let's dive deeper into the specific communication patterns that make this nervous system function. Understanding these patterns is crucial for developing AI systems that can effectively interact with humanoid robots.

### Nodes: The Computational Units

Nodes are the fundamental building blocks of any ROS 2 system. Think of them as specialized cells or regions in a biological nervous system, each with a specific function. A node is essentially a process that performs computation in your robot system. Each node typically handles a specific task or set of related tasks.

In a humanoid robot system, you might have nodes for:
- Sensor processing (camera, LIDAR, IMU)
- Motion planning
- Control systems
- Perception and recognition
- Behavior management
- Communication with external systems

Nodes are language-agnostic, meaning they can be written in different programming languages (C++, Python, etc.) and still communicate with each other seamlessly. This flexibility is particularly valuable in AI applications, where you might want to leverage Python's rich ecosystem for machine learning while using C++ for performance-critical control tasks.

### Topics: The Information Pathways

Topics are named buses over which nodes exchange messages in a publish/subscribe pattern. They're analogous to neural pathways in a biological nervous system, carrying continuous streams of information between different parts of the robot.

In a publish/subscribe model:
- Publishers send data to a topic without knowing who will receive it
- Subscribers receive data from a topic without knowing who sent it
- Multiple publishers can publish to the same topic
- Multiple subscribers can subscribe to the same topic

This decoupling is powerful for humanoid robots because it allows for flexible system architectures. For example, a camera sensor node can publish image data to a "camera/image_raw" topic, and multiple AI perception nodes can subscribe to this topic simultaneously – one for object detection, another for facial recognition, and another for navigation planning.

Topics are ideal for continuous data streams like sensor readings, robot state information, or control commands. The data flows continuously from publishers to subscribers, making it perfect for real-time applications.

### Services: The Request-Response Mechanisms

Services provide synchronous request/response communication between nodes. Unlike topics, which provide continuous data streams, services are used for specific interactions where a node sends a request and waits for a response.

The service model is like asking a specific question and waiting for an answer. For example, an AI planning node might request a path planning service to calculate a route from the current location to a goal location. The service receives the request, processes it, and returns the result.

Services are appropriate for tasks that:
- Have a clear start and end
- Require specific input parameters
- Return a specific result
- Don't need to happen continuously

In humanoid robots, services might be used for tasks like:
- Requesting a specific movement
- Querying robot status
- Loading a new map
- Saving a configuration

### Actions: The Goal-Oriented Behaviors

Actions are designed for long-running tasks that provide feedback during execution and can be canceled. They're particularly important for humanoid robots, which often need to perform complex behaviors that take time to complete.

An action involves three components:
- **Goal**: The desired outcome
- **Feedback**: Updates on progress during execution
- **Result**: The final outcome when the task completes

Actions are perfect for complex robot behaviors like:
- Navigation to a distant location
- Manipulation tasks that involve multiple steps
- Humanoid walking patterns
- Complex interaction sequences

The ability to cancel actions is crucial for humanoid robots that need to respond to changing environments or emergencies.

## Data Flow Between Sensors, AI, and Actuators

Understanding how data flows between sensors, AI modules, and actuators is essential for developing effective AI systems for humanoid robots. This flow represents the complete loop from perception to action, with AI processing in the middle.

### Sensor Data Flow

Sensors generate continuous streams of data that flow through the ROS 2 system:

1. **Raw Sensor Data**: Sensors like cameras, LIDAR, and IMUs generate raw data streams
2. **Preprocessing**: Specialized nodes preprocess sensor data (e.g., image calibration, point cloud filtering)
3. **Fusion**: Multiple sensor streams might be combined to create a more complete picture of the environment
4. **AI Perception**: AI modules consume processed sensor data to understand the environment

This flow typically uses topics for continuous data streams, with each sensor publishing to its own topic and multiple AI modules potentially subscribing to the same sensor data.

### AI Processing Flow

AI modules receive sensor data, process it, and generate commands:

1. **Perception**: AI algorithms analyze sensor data to identify objects, understand scenes, or detect events
2. **Planning**: Based on perception and goals, AI algorithms plan appropriate actions
3. **Control**: High-level commands are converted into low-level control signals
4. **Output**: Commands are sent to actuators through various communication patterns

### Actuator Command Flow

Commands flow from AI modules to actuators:

1. **High-level Commands**: AI modules generate high-level commands (e.g., "move to location," "grasp object")
2. **Service Requests**: Some commands might use services for immediate responses
3. **Action Goals**: Complex behaviors use actions with feedback and cancellation capabilities
4. **Low-level Control**: Control nodes translate high-level commands into specific actuator commands

### Complete Example: Visual Servoing

Let's consider a complete example: a humanoid robot using visual servoing to reach for an object.

1. **Sensors**: Camera nodes publish images to `/camera/image_raw` topic
2. **AI Perception**: Object detection node subscribes to the image topic, detects the target object, and publishes its position to `/object_position` topic
3. **AI Planning**: A planning node subscribes to object position, calculates required arm movement, and sends an action goal to `/arm_controller` action server
4. **Actuator Control**: The arm controller executes the movement, providing feedback on progress
5. **Monitoring**: Other nodes monitor the action and can cancel if obstacles appear

This example demonstrates how different communication patterns work together to achieve complex behaviors in humanoid robots.

In the next chapter, we'll explore how these communication patterns connect to the physical robot through rclpy and URDF, completing the bridge between AI software and physical hardware.