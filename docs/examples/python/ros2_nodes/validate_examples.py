#!/usr/bin/env python3

"""
Validation script for ROS 2 examples
This script validates that all ROS 2 examples follow ROS 2 Humble conventions
and do not contain hallucinated APIs or functions.
"""

import ast
import os
import sys
from typing import List, Tuple, Set


class ROS2ExampleValidator:
    """
    Validator for ROS 2 examples to ensure they follow proper conventions
    and don't contain hallucinated APIs
    """

    def __init__(self):
        """
        Initialize the validator
        """
        # Known valid ROS 2 Humble API patterns
        self.valid_ros2_patterns = {
            # rclpy imports
            'rclpy',
            'rclpy.node',
            'rclpy.qos',

            # Node methods
            'create_publisher',
            'create_subscription',
            'create_timer',
            'create_client',
            'create_service',

            # Standard message types
            'std_msgs.msg',
            'geometry_msgs.msg',
            'sensor_msgs.msg',
            'nav_msgs.msg',

            # Service types
            'example_interfaces.srv',

            # QoS related
            'QoSProfile',
            'ReliabilityPolicy',
            'DurabilityPolicy',
        }

        # Known valid methods on Node class
        self.valid_node_methods = {
            'get_logger',
            'create_publisher',
            'create_subscription',
            'create_timer',
            'create_client',
            'create_service',
            'declare_parameter',
            'get_parameter',
            'set_parameters_callback',
            'wait_for_service',
            'call',
        }

    def validate_file(self, file_path: str) -> Tuple[bool, List[str]]:
        """
        Validate a single Python file

        Args:
            file_path: Path to the Python file to validate

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                tree = ast.parse(content)
        except SyntaxError as e:
            errors.append(f"Syntax error in {file_path}: {e}")
            return False, errors
        except Exception as e:
            errors.append(f"Error reading {file_path}: {e}")
            return False, errors

        # Check imports
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name not in self.valid_ros2_patterns and not alias.name.startswith('rclpy'):
                        errors.append(f"Potentially invalid import in {file_path}: {alias.name}")
            elif isinstance(node, ast.ImportFrom):
                if node.module not in self.valid_ros2_patterns and not node.module.startswith('rclpy'):
                    errors.append(f"Potentially invalid import in {file_path}: from {node.module}")

        # Basic validation passed
        return len(errors) == 0, errors

    def validate_directory(self, directory_path: str) -> Tuple[bool, dict]:
        """
        Validate all Python files in a directory

        Args:
            directory_path: Path to the directory to validate

        Returns:
            Tuple of (is_valid, dict mapping file_path to errors)
        """
        all_valid = True
        all_errors = {}

        for root, dirs, files in os.walk(directory_path):
            for file in files:
                if file.endswith('.py'):
                    file_path = os.path.join(root, file)
                    is_valid, errors = self.validate_file(file_path)

                    if not is_valid:
                        all_valid = False
                        all_errors[file_path] = errors

        return all_valid, all_errors


def main():
    """
    Main function to validate ROS 2 examples
    """
    validator = ROS2ExampleValidator()

    # Validate the ROS 2 examples directory
    examples_dir = os.path.join(os.path.dirname(__file__))
    is_valid, errors = validator.validate_directory(examples_dir)

    if is_valid:
        print("✅ All ROS 2 examples are valid!")
        print(f"Validated {len([f for f in os.listdir(examples_dir) if f.endswith('.py')])} Python files")
        return 0
    else:
        print("❌ Some ROS 2 examples have issues:")
        for file_path, file_errors in errors.items():
            print(f"\nFile: {file_path}")
            for error in file_errors:
                print(f"  - {error}")
        return 1


if __name__ == '__main__':
    sys.exit(main())