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