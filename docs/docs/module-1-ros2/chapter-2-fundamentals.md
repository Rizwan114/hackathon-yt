---
sidebar_position: 2
title: "Chapter 2: ROS 2 Fundamentals - Nodes, Topics, and Services"
---

# Chapter 2: ROS 2 Fundamentals - Nodes, Topics, and Services

## ROS 2 Node Lifecycle and Conventions

A ROS 2 node is a process that performs computation and communicates with other nodes in the ROS graph. Understanding the node lifecycle is fundamental to creating robust robotic applications.

### Node Creation and Initialization

When a node is created, it follows a specific lifecycle:

1. **Initialization**: The node is created and registers with the ROS master
2. **Configuration**: Publishers, subscribers, services, and parameters are set up
3. **Activation**: The node begins processing callbacks and communicating
4. **Execution**: The node runs its main loop or waits for callbacks
5. **Shutdown**: The node cleanly disconnects and releases resources

### Node Conventions

ROS 2 follows specific naming and structure conventions:

- Node names should be descriptive and unique within the system
- Node names should use lowercase with underscores separating words
- Each node should have a single, well-defined responsibility
- Nodes should handle their own errors gracefully
- Nodes should use ROS parameters for configuration

## Topics and QoS Models

Topics enable asynchronous communication through a publish/subscribe pattern. Understanding Quality of Service (QoS) settings is crucial for reliable communication.

### Publish/Subscribe Pattern

The publish/subscribe pattern allows nodes to communicate without direct connections:

- **Publishers** send messages to topics without knowing who will receive them
- **Subscribers** receive messages from topics without knowing who sent them
- **ROS Middleware** handles message routing between publishers and subscribers

### Quality of Service (QoS) Profiles

QoS settings determine how messages are delivered:

- **Reliability**: Whether messages must be delivered (RELIABLE) or can be dropped (BEST_EFFORT)
- **Durability**: Whether late-joining subscribers get old messages (TRANSIENT_LOCAL) or not (VOLATILE)
- **History**: How many messages to keep (KEEP_ALL or KEEP_LAST N)
- **Deadline**: Maximum time between consecutive messages
- **Lifespan**: Maximum age of messages before they're dropped

For humanoid robots, choosing the right QoS profile is critical for safety and performance.

## Services and Parameter Servers

Services provide synchronous request/response communication for operations that require confirmation or return data.

### Service Architecture

- **Service Server**: Waits for requests and sends responses
- **Service Client**: Sends requests and waits for responses
- **Service Interface**: Defines the request and response message types

Services are appropriate for operations that:
- Need to return data to the caller
- Should complete before the caller continues
- Are infrequent or low-bandwidth

### Parameter Servers

Parameter servers provide a centralized way to configure nodes:
- Parameters can be set at launch time or changed during runtime
- Nodes can declare parameters with default values and constraints
- Parameters support various types: integers, floats, strings, booleans, and lists

## Quality of Service (QoS) Models

Quality of Service (QoS) settings determine how messages are delivered between nodes. Understanding QoS is crucial for creating robust robotic systems, especially for humanoid robots where timing and reliability are critical.

### QoS Profiles

ROS 2 provides several predefined QoS profiles for common use cases:

- **Default**: Standard settings for general communication
- **Sensor Data**: Optimized for high-frequency sensor data (BEST_EFFORT, VOLATILE)
- **Services**: Optimized for service calls (RELIABLE, VOLATILE)
- **Parameters**: Optimized for parameter updates (RELIABLE, TRANSIENT_LOCAL)

### Key QoS Settings

#### Reliability
- **RELIABLE**: Every message is guaranteed to be delivered (may block publishers)
- **BEST_EFFORT**: Messages may be dropped if network conditions are poor (no blocking)

For humanoid robots:
- Use RELIABLE for critical control commands
- Use BEST_EFFORT for high-frequency sensor data where some loss is acceptable

#### Durability
- **TRANSIENT_LOCAL**: Late-joining subscribers receive the last message published
- **VOLATILE**: Late-joining subscribers only receive future messages

For humanoid robots:
- Use TRANSIENT_LOCAL for configuration parameters or state updates
- Use VOLATILE for streaming sensor data

#### History
- **KEEP_ALL**: Store all messages (limited by memory)
- **KEEP_LAST**: Store only the most recent N messages

### Example: Setting QoS in Code

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile for critical control commands
control_qos = QoSProfile(
    depth=10,  # queue size
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Create a publisher with the custom QoS
self.control_publisher = self.create_publisher(
    ControlMessage,
    'robot_control',
    qos_profile=control_qos
)

# Create a QoS profile for sensor data
sensor_qos = QoSProfile(
    depth=5,  # smaller queue for high-frequency data
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# Create a subscription with the sensor QoS
self.sensor_subscription = self.create_subscription(
    SensorMessage,
    'sensor_data',
    self.sensor_callback,
    qos_profile=sensor_qos
)
```

## Hands-on: Building Minimal Publisher/Subscriber Nodes in rclpy

Now let's implement basic publisher and subscriber nodes using rclpy, the Python client library for ROS 2.

### Publisher Node Implementation

The publisher node creates a publisher for a specific topic and sends messages at regular intervals:

```python
#!/usr/bin/env python3

"""
Simple ROS 2 publisher node example
This node publishes messages to a topic at regular intervals
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    Simple publisher node that sends messages to a topic
    """

    def __init__(self):
        """
        Initialize the publisher node
        """
        super().__init__('simple_publisher')

        # Create a publisher for the 'chatter' topic with String messages
        # The queue size is set to 10
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of the number of messages published
        self.i = 0

        self.get_logger().info('Simple publisher node initialized')

    def timer_callback(self):
        """
        Callback method that publishes a message to the topic
        """
        msg = String()
        msg.data = f'Hello ROS 2 World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the publisher node
    simple_publisher = SimplePublisher()

    try:
        # Spin the node so the callback function is called
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Node Implementation

The subscriber node creates a subscription to a topic and processes incoming messages:

```python
#!/usr/bin/env python3

"""
Simple ROS 2 subscriber node example
This node subscribes to messages from a topic and logs them
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    Simple subscriber node that receives messages from a topic
    """

    def __init__(self):
        """
        Initialize the subscriber node
        """
        super().__init__('simple_subscriber')

        # Create a subscription to the 'chatter' topic with String messages
        # The callback method message_callback will be called when a message is received
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.message_callback,
            10)  # queue size of 10

        # Don't forget to declare that subscription is not used to avoid a warning
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Simple subscriber node initialized')

    def message_callback(self, msg):
        """
        Callback method that is called when a message is received
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the subscriber node
    simple_subscriber = SimpleSubscriber()

    try:
        # Spin the node so the callback function is called
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

To run these nodes, first source your ROS 2 installation, then run each in separate terminals:

```bash
# Terminal 1
python3 simple_publisher.py

# Terminal 2
python3 simple_subscriber.py
```

## Debugging Using ROS 2 Tools

ROS 2 provides several tools for debugging communication issues:

### Topic Tools

- `ros2 topic list`: Show all active topics
- `ros2 topic info <topic_name>`: Show information about a specific topic
- `ros2 topic echo <topic_name>`: Print messages published to a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish messages to a topic

Example usage:
```bash
# List all topics
ros2 topic list

# Echo messages from the 'chatter' topic
ros2 topic echo /chatter std_msgs/msg/String

# Publish a message to the 'chatter' topic
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from command line'"
```

### Service Tools

- `ros2 service list`: Show all active services
- `ros2 service info <service_name>`: Show information about a specific service
- `ros2 service call <service_name> <srv_type> <args>`: Call a service

Example usage:
```bash
# List all services
ros2 service list

# Call the 'add_two_ints' service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### Node Tools

- `ros2 node list`: Show all active nodes
- `ros2 node info <node_name>`: Show information about a specific node
- `ros2 run <package_name> <executable>`: Run a node from a package

### Graph Visualization

- `rqt_graph`: Visualize the ROS graph showing nodes and topics
  ```bash
  rqt_graph
  ```

### System Health Check

- `ros2 doctor`: Check the health of your ROS installation
  ```bash
  ros2 doctor
  ```

### Parameter Tools

- `ros2 param list`: List parameters for a node
- `ros2 param get <node_name> <param_name>`: Get a parameter value
- `ros2 param set <node_name> <param_name> <value>`: Set a parameter value

These tools are essential for understanding and troubleshooting robotic systems. Use them to:
- Verify that nodes are communicating properly
- Check message rates and content
- Identify network issues
- Validate system configuration
- Monitor system performance