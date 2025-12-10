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