#!/usr/bin/env python3

"""
Simple ROS 2 service client example
This node calls the add_two_ints service to add two integers
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceClient(Node):
    """
    Simple service client that calls the add_two_ints service
    """

    def __init__(self):
        """
        Initialize the service client node
        """
        super().__init__('simple_service_client')

        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Simple service client initialized')

    def send_request(self, a, b):
        """
        Send a request to the service
        """
        # Create the request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(request)
        self.get_logger().info(f'Sent request: {a} + {b}')


def main(args=None):
    """
    Main function
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the service client node
    simple_service_client = SimpleServiceClient()

    # Check if command line arguments are provided
    if len(sys.argv) != 3:
        print('Usage: simple_service_client.py <int1> <int2>')
        sys.exit(1)

    # Parse the command line arguments
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    # Send the request
    simple_service_client.send_request(a, b)

    try:
        # Spin until the future is complete
        while rclpy.ok():
            rclpy.spin_once(simple_service_client)
            if simple_service_client.future.done():
                try:
                    response = simple_service_client.future.result()
                    simple_service_client.get_logger().info(
                        f'Result of {a} + {b} = {response.sum}')
                except Exception as e:
                    simple_service_client.get_logger().error(
                        f'Service call failed: {e}')
                break
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        simple_service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()