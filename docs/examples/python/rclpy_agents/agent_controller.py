#!/usr/bin/env python3
"""
Python Agent Controller for ROS 2 Integration
This example demonstrates how to create a Python-based AI agent that can send commands
to ROS 2 controllers using rclpy, with comprehensive safety mechanisms for humanoid robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Duration
import time
import threading
import math
from typing import Dict, List, Optional


class SafeController(Node):
    """
    A safe controller with comprehensive safety mechanisms including rate limiting,
    timeouts, joint limits, velocity limits, and emergency stop capabilities.
    """
    def __init__(self):
        super().__init__('safe_agent_controller')

        # Rate limiting: minimum time between commands (in seconds)
        self.min_command_interval = 0.01  # 10ms minimum between commands
        self.last_command_time = 0

        # Timeout configuration
        self.default_timeout = 5.0  # seconds

        # Publishers for different command types
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/agent_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 1)

        # Subscriber for sensor feedback
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # IMU subscriber for balance feedback
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )

        # Store current joint states and IMU data
        self.current_joint_states = JointState()
        self.current_imu_data = None

        # Safety parameters
        self.max_velocity = 1.0  # rad/s
        self.max_effort = 100.0  # N*m
        self.max_position = 3.14  # rad (pi), reasonable limit for most joints
        self.balance_threshold = 0.5  # rad, maximum acceptable tilt
        self.safety_enabled = True
        self.emergency_stop_triggered = False

        # Joint limits (example for a humanoid - would be loaded from URDF in practice)
        self.joint_limits = {
            'hip_joint': (-1.57, 1.57),      # -90 to 90 degrees
            'knee_joint': (0, 2.35),         # 0 to 135 degrees (flex only)
            'ankle_joint': (-0.52, 0.52),    # -30 to 30 degrees
            'shoulder_joint': (-2.09, 2.09), # -120 to 120 degrees
            'elbow_joint': (-2.35, 0),       # -135 to 0 degrees (flex only)
        }

        self.get_logger().info('Safe Agent Controller initialized with comprehensive safety features')

    def joint_state_callback(self, msg: JointState):
        """Callback to update current joint states"""
        self.current_joint_states = msg

    def imu_callback(self, msg: Imu):
        """Callback to update IMU data for balance monitoring"""
        self.current_imu_data = msg

    def send_command_if_safe(self, command: JointState) -> bool:
        """
        Send a command if all safety checks pass and rate limit allows
        """
        if self.emergency_stop_triggered:
            self.get_logger().warn('Command blocked: Emergency stop active')
            return False

        current_time = time.time()

        # Rate limiting check
        if current_time - self.last_command_time < self.min_command_interval:
            self.get_logger().warn('Command throttled due to rate limiting')
            return False

        # Safety validation
        if self.safety_enabled and not self.validate_command(command):
            self.get_logger().error('Command failed safety validation')
            return False

        # Balance check if IMU data is available
        if self.current_imu_data and not self.is_balanced():
            self.get_logger().error('Command blocked: Robot is not balanced')
            return False

        # Publish the command
        self.joint_pub.publish(command)
        self.last_command_time = current_time

        # Publish status update
        status_msg = String()
        status_msg.data = f'Command sent: {len(command.position)} joints updated'
        self.status_pub.publish(status_msg)

        return True

    def validate_command(self, command: JointState) -> bool:
        """
        Validate command for comprehensive safety constraints
        """
        if not self.safety_enabled:
            return True

        # Check joint position limits
        for i, joint_name in enumerate(command.name):
            if i < len(command.position):
                pos = command.position[i]

                # Check general position limits
                if abs(pos) > self.max_position:
                    self.get_logger().error(f'Joint {joint_name} position {pos} exceeds max {self.max_position}')
                    return False

                # Check specific joint limits if defined
                if joint_name in self.joint_limits:
                    min_limit, max_limit = self.joint_limits[joint_name]
                    if pos < min_limit or pos > max_limit:
                        self.get_logger().error(f'Joint {joint_name} position {pos} outside limits [{min_limit}, {max_limit}]')
                        return False

        # Check velocity limits
        if command.velocity:
            for vel in command.velocity:
                if abs(vel) > self.max_velocity:
                    self.get_logger().error(f'Velocity {vel} exceeds max {self.max_velocity}')
                    return False

        # Check effort limits
        if command.effort:
            for eff in command.effort:
                if abs(eff) > self.max_effort:
                    self.get_logger().error(f'Effort {eff} exceeds max {self.max_effort}')
                    return False

        return True

    def is_balanced(self) -> bool:
        """
        Check if the robot is within acceptable balance thresholds using IMU data
        """
        if not self.current_imu_data:
            return True  # If no IMU data, assume balanced

        # Extract roll and pitch from quaternion (simplified)
        # In practice, you'd use proper quaternion to euler conversion
        roll = math.asin(2.0 * (self.current_imu_data.orientation.w * self.current_imu_data.orientation.x +
                                self.current_imu_data.orientation.y * self.current_imu_data.orientation.z))
        pitch = math.atan2(2.0 * (self.current_imu_data.orientation.w * self.current_imu_data.orientation.y -
                                  self.current_imu_data.orientation.z * self.current_imu_data.orientation.x),
                          1.0 - 2.0 * (self.current_imu_data.orientation.x * self.current_imu_data.orientation.x +
                                       self.current_imu_data.orientation.y * self.current_imu_data.orientation.y))

        # Check if roll or pitch exceed balance threshold
        return abs(roll) <= self.balance_threshold and abs(pitch) <= self.balance_threshold

    def execute_with_timeout(self, func, timeout_duration: float = None):
        """
        Execute a function with a timeout
        """
        if timeout_duration is None:
            timeout_duration = self.default_timeout

        result = [None]
        exception = [None]
        completed = [False]

        def target():
            try:
                result[0] = func()
                completed[0] = True
            except Exception as e:
                exception[0] = e
                completed[0] = True

        thread = threading.Thread(target=target)
        thread.daemon = True
        thread.start()
        thread.join(timeout_duration)

        if not completed[0]:
            self.get_logger().error(f'Function did not complete within {timeout_duration}s')
            raise TimeoutError(f'Function did not complete within {timeout_duration}s')

        if exception[0]:
            raise exception[0]

        return result[0]

    def send_joint_trajectory(self, joint_names: List[str], target_positions: List[float],
                            duration: float = 2.0) -> bool:
        """
        Send a joint trajectory command with comprehensive safety checks
        """
        # Create joint state message
        joint_cmd = JointState()
        joint_cmd.name = joint_names
        joint_cmd.position = target_positions
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Calculate velocity based on duration and current position to ensure smooth motion
        if self.current_joint_states.position and len(self.current_joint_states.name) > 0:
            velocities = []
            for joint_name, target_pos in zip(joint_names, target_positions):
                # Find current position for this joint
                current_pos = 0.0
                for i, curr_name in enumerate(self.current_joint_states.name):
                    if curr_name == joint_name and i < len(self.current_joint_states.position):
                        current_pos = self.current_joint_states.position[i]
                        break

                # Calculate required velocity
                pos_diff = abs(target_pos - current_pos)
                required_vel = pos_diff / duration
                clamped_vel = min(required_vel, self.max_velocity)
                velocities.append(clamped_vel)

            joint_cmd.velocity = velocities
        else:
            # If no current state, set default velocities
            joint_cmd.velocity = [self.max_velocity * 0.5] * len(target_positions)  # Use half max velocity as default

        # Send command with safety checks
        return self.send_command_if_safe(joint_cmd)

    def trigger_emergency_stop(self):
        """
        Trigger emergency stop to halt all robot motion
        """
        self.emergency_stop_triggered = True
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)
        self.get_logger().warn('EMERGENCY STOP TRIGGERED - All robot motion halted')

    def clear_emergency_stop(self):
        """
        Clear emergency stop condition
        """
        self.emergency_stop_triggered = False
        stop_msg = Bool()
        stop_msg.data = False
        self.emergency_stop_pub.publish(stop_msg)
        self.get_logger().info('Emergency stop cleared - Robot motion enabled')


def main():
    """Main function to demonstrate the agent controller with safety features"""
    rclpy.init()

    controller = SafeController()

    # Example: Send a simple joint command
    joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']
    target_positions = [0.1, 0.2, 0.05]  # radians

    # Send command in a safe manner
    try:
        success = controller.send_joint_trajectory(joint_names, target_positions, duration=1.0)
        if success:
            controller.get_logger().info('Joint trajectory command sent successfully')
        else:
            controller.get_logger().error('Failed to send joint trajectory command')
    except Exception as e:
        controller.get_logger().error(f'Error sending command: {e}')

    # Example: Test balance monitoring
    if controller.current_imu_data:
        if controller.is_balanced():
            controller.get_logger().info('Robot is currently balanced')
        else:
            controller.get_logger().warn('Robot is not balanced - movement restricted')
            controller.trigger_emergency_stop()

    # Spin for a few seconds to allow command to be processed
    timer = controller.create_timer(0.1, lambda: None)  # Keep node alive

    # Run for 5 seconds
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 5.0:
        rclpy.spin_once(controller, timeout_sec=0.1)

    # Shutdown
    controller.destroy_timer(timer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()