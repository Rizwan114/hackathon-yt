#!/usr/bin/env python3

"""
Simple ROS 2 service server example
This node provides a service that adds two integers
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleService(Node):
    """
    Simple service server that adds two integers
    """

    def __init__(self):
        """
        Initialize the service server node
        """
        super().__init__('simple_service_server')

        # Create a service that takes AddTwoInts requests and calls add_two_ints_callback
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        self.get_logger().info('Simple service server initialized')

    def add_two_ints_callback(self, request, response):
        """
        Callback method that processes the service request
        """
        # Perform the addition
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')

        # Return the response
        return response


def main(args=None):
    """
    Main function
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the service server node
    simple_service = SimpleService()

    try:
        # Spin the node so the callback function is called
        rclpy.spin(simple_service)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        simple_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()