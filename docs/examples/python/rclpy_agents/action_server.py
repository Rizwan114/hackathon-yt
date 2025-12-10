#!/usr/bin/env python3
"""
Action Server for Humanoid Robot Tasks
This example demonstrates how to create an action server for humanoid robot tasks
like walking, grasping, and balancing using ROS 2 actions with rclpy.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState, Imu
import time
import math
from typing import List, Tuple


class HumanoidActionServer(Node):
    """
    Action server for humanoid robot tasks including walking, grasping, and balancing
    with comprehensive safety mechanisms
    """
    def __init__(self):
        super().__init__('humanoid_action_server')

        # Publishers for commanding the robot
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.status_pub = self.create_publisher(JointState, '/action_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 1)

        # Subscribers for sensor feedback
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )

        # Store current robot state
        self.current_joint_positions = []
        self.current_imu_data = None
        self.is_executing = False
        self.emergency_stop_triggered = False

        # Safety parameters
        self.max_velocity = 1.0  # rad/s
        self.max_effort = 100.0  # N*m
        self.max_position = 3.14  # rad (pi)
        self.balance_threshold = 0.5  # rad, maximum acceptable tilt

        # Initialize action servers for different humanoid tasks
        self.walk_action_server = ActionServer(
            self,
            # Using a generic action type - in practice, you'd define specific action files
            # For now, we'll simulate with a custom approach
            self.execute_walk_callback,
            'walk_to_pose',
            cancel_callback=self.cancel_callback
        )

        self.grasp_action_server = ActionServer(
            self,
            self.execute_grasp_callback,
            'grasp_object',
            cancel_callback=self.cancel_callback
        )

        self.balance_action_server = ActionServer(
            self,
            self.execute_balance_callback,
            'maintain_balance',
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Humanoid Action Server initialized with safety mechanisms')

    def joint_state_callback(self, msg: JointState):
        """Callback to update current joint states"""
        self.current_joint_positions = list(msg.position)

    def imu_callback(self, msg: Imu):
        """Callback to update IMU data for balance control"""
        self.current_imu_data = msg

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info('Received cancel request')
        self.is_executing = False
        return CancelResponse.ACCEPT

    def validate_joint_command(self, joint_positions: List[float]) -> bool:
        """
        Validate joint positions for safety constraints
        """
        if self.emergency_stop_triggered:
            self.get_logger().warn('Joint command blocked: Emergency stop active')
            return False

        # Check position limits
        for pos in joint_positions:
            if abs(pos) > self.max_position:
                self.get_logger().error(f'Joint position {pos} exceeds max {self.max_position}')
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
        roll = self.current_imu_data.orientation.x  # Simplified - proper conversion needed
        pitch = self.current_imu_data.orientation.y  # Simplified - proper conversion needed

        # Check if roll or pitch exceed balance threshold
        return abs(roll) <= self.balance_threshold and abs(pitch) <= self.balance_threshold

    def execute_walk_callback(self, goal_handle):
        """Execute walking action with safety checks"""
        self.get_logger().info('Executing walk to pose action')
        self.is_executing = True

        # In a real implementation, you'd extract goal parameters from the goal message
        # For this example, we'll simulate walking to a target pose
        target_x = 1.0  # meters
        target_y = 0.0  # meters
        target_theta = 0.0  # radians

        # Create feedback and result messages
        # Using simple types for simulation - in practice would use action-specific types
        feedback_msg = JointState()
        result_msg = JointState()

        # Simulate walking with feedback and safety checks
        for step in range(100):  # Simulate 100 steps
            if not self.is_executing or goal_handle.is_cancel_requested:
                result_msg.name = ["walk_result"]
                result_msg.position = [0.0]  # Failed
                goal_handle.canceled()
                return result_msg

            # Check balance before proceeding
            if not self.is_balanced():
                self.get_logger().error('Walking stopped: Robot is not balanced')
                result_msg.name = ["walk_result"]
                result_msg.position = [0.0]  # Failed due to balance
                goal_handle.abort()
                return result_msg

            # Calculate intermediate pose (simplified walking simulation)
            alpha = step / 99.0
            current_x = alpha * target_x
            current_y = alpha * target_y
            current_theta = alpha * target_theta

            # Generate joint commands for walking (simplified)
            # In reality, this would involve complex inverse kinematics and gait planning
            joint_positions = self.generate_walking_trajectory(
                current_x, current_y, current_theta, alpha
            )

            # Validate joint positions before publishing
            if not self.validate_joint_command(joint_positions):
                self.get_logger().error('Walking stopped: Joint command validation failed')
                result_msg.name = ["walk_result"]
                result_msg.position = [0.0]  # Failed due to safety
                goal_handle.abort()
                return result_msg

            # Publish joint commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = joint_positions
            self.joint_pub.publish(cmd_msg)

            # Update feedback
            feedback_msg.name = [f"step_{step}"]
            feedback_msg.position = [alpha * 100.0]  # Progress percentage
            feedback_msg.header.stamp = self.get_clock().now().to_msg()

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to control execution speed
            time.sleep(0.1)

        # Complete successfully
        result_msg.name = ["walk_result"]
        result_msg.position = [1.0]  # Success
        goal_handle.succeed()
        return result_msg

    def generate_walking_trajectory(self, x: float, y: float, theta: float, progress: float) -> List[float]:
        """
        Generate a simplified walking trajectory for humanoid robot
        """
        # This is a highly simplified walking gait - in practice, this would be much more complex
        # involving inverse kinematics, balance control, and footstep planning

        # Base joint positions (standing pose)
        base_positions = [0.0] * 28  # Assuming 28 DOF humanoid

        # Add simple walking motion based on progress
        # This is just a simulation - real walking would require complex control
        walk_motion = math.sin(progress * 2 * math.pi) * 0.1  # Small oscillation

        # Modify some joints to simulate walking
        # Hip joints
        base_positions[0] += walk_motion * 0.1  # Left hip
        base_positions[1] -= walk_motion * 0.1  # Right hip
        # Knee joints
        base_positions[2] += abs(walk_motion) * 0.05  # Left knee
        base_positions[3] -= abs(walk_motion) * 0.05  # Right knee

        # Apply safety limits to ensure positions are within bounds
        for i in range(len(base_positions)):
            base_positions[i] = max(-self.max_position, min(base_positions[i], self.max_position))

        return base_positions

    def execute_grasp_callback(self, goal_handle):
        """Execute grasping action with safety checks"""
        self.get_logger().info('Executing grasp object action')
        self.is_executing = True

        feedback_msg = JointState()
        result_msg = JointState()

        # Check initial balance before starting grasp
        if not self.is_balanced():
            self.get_logger().error('Grasping stopped: Robot not balanced at start')
            result_msg.name = ["grasp_result"]
            result_msg.position = [0.0]  # Failed
            goal_handle.abort()
            return result_msg

        # Simulate grasping motion
        for step in range(50):  # Simulate 50 steps for grasping
            if not self.is_executing or goal_handle.is_cancel_requested:
                result_msg.name = ["grasp_result"]
                result_msg.position = [0.0]  # Failed
                goal_handle.canceled()
                return result_msg

            # Check balance during grasp
            if not self.is_balanced():
                self.get_logger().error('Grasping stopped: Robot lost balance')
                result_msg.name = ["grasp_result"]
                result_msg.position = [0.0]  # Failed due to balance
                goal_handle.abort()
                return result_msg

            # Calculate grasping motion
            alpha = step / 49.0
            joint_positions = self.generate_grasping_trajectory(alpha)

            # Validate joint positions before publishing
            if not self.validate_joint_command(joint_positions):
                self.get_logger().error('Grasping stopped: Joint command validation failed')
                result_msg.name = ["grasp_result"]
                result_msg.position = [0.0]  # Failed due to safety
                goal_handle.abort()
                return result_msg

            # Publish joint commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = joint_positions
            self.joint_pub.publish(cmd_msg)

            # Update feedback
            feedback_msg.name = [f"grasp_step_{step}"]
            feedback_msg.position = [alpha * 100.0]  # Progress percentage
            feedback_msg.header.stamp = self.get_clock().now().to_msg()

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to control execution speed
            time.sleep(0.1)

        # Complete successfully
        result_msg.name = ["grasp_result"]
        result_msg.position = [1.0]  # Success
        goal_handle.succeed()
        return result_msg

    def generate_grasping_trajectory(self, progress: float) -> List[float]:
        """
        Generate a simplified grasping trajectory for humanoid robot
        """
        # Base joint positions (standing pose)
        base_positions = [0.0] * 28

        # Add grasping motion based on progress
        grasp_motion = progress * 0.5  # Maximum 0.5 rad movement

        # Modify arm joints to simulate reaching and grasping
        # Shoulder joints
        base_positions[4] += grasp_motion * 0.5  # Shoulder pitch
        base_positions[5] += grasp_motion * 0.3  # Shoulder yaw
        # Elbow joint
        base_positions[6] += grasp_motion * 0.7  # Elbow
        # Wrist joints
        base_positions[7] += grasp_motion * 0.2  # Wrist
        base_positions[8] += grasp_motion * 0.1  # Wrist

        # Apply safety limits to ensure positions are within bounds
        for i in range(len(base_positions)):
            base_positions[i] = max(-self.max_position, min(base_positions[i], self.max_position))

        return base_positions

    def execute_balance_callback(self, goal_handle):
        """Execute balancing action with safety checks"""
        self.get_logger().info('Executing maintain balance action')
        self.is_executing = True

        feedback_msg = JointState()
        result_msg = JointState()

        # Balance for a set period (or until cancelled)
        balance_duration = 100  # 10 seconds at 10Hz

        for step in range(balance_duration):
            if not self.is_executing or goal_handle.is_cancel_requested:
                result_msg.name = ["balance_result"]
                result_msg.position = [0.0]  # Failed or cancelled
                goal_handle.canceled()
                return result_msg

            # Get current IMU data for balance control
            if self.current_imu_data:
                # Simple balance control based on IMU data
                roll = self.current_imu_data.orientation.x
                pitch = self.current_imu_data.orientation.y

                # Generate balance correction
                joint_positions = self.generate_balance_correction(roll, pitch)

                # Validate joint positions before publishing
                if not self.validate_joint_command(joint_positions):
                    self.get_logger().error('Balancing stopped: Joint command validation failed')
                    result_msg.name = ["balance_result"]
                    result_msg.position = [0.0]  # Failed due to safety
                    goal_handle.abort()
                    return result_msg

                # Publish joint commands for balance
                cmd_msg = Float64MultiArray()
                cmd_msg.data = joint_positions
                self.joint_pub.publish(cmd_msg)
            else:
                # If no IMU data, send a safe standing position
                safe_positions = [0.0] * 28
                cmd_msg = Float64MultiArray()
                cmd_msg.data = safe_positions
                self.joint_pub.publish(cmd_msg)

            # Update feedback
            feedback_msg.name = [f"balance_step_{step}"]
            feedback_msg.position = [step * 0.1]  # Time in seconds
            feedback_msg.header.stamp = self.get_clock().now().to_msg()

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to control execution speed (10Hz)
            time.sleep(0.1)

        # Complete successfully
        result_msg.name = ["balance_result"]
        result_msg.position = [1.0]  # Success
        goal_handle.succeed()
        return result_msg

    def generate_balance_correction(self, roll: float, pitch: float) -> List[float]:
        """
        Generate balance correction based on IMU data
        """
        # Base joint positions (standing pose)
        base_positions = [0.0] * 28

        # Simple proportional control for balance
        # Adjust hip and ankle joints based on roll/pitch errors
        hip_correction = roll * 0.5
        ankle_correction = -roll * 0.3

        # Apply corrections to balance joints
        base_positions[0] += hip_correction  # Left hip roll
        base_positions[1] -= hip_correction  # Right hip roll
        base_positions[2] += ankle_correction  # Left ankle
        base_positions[3] -= ankle_correction  # Right ankle

        # Add pitch corrections
        pitch_correction = pitch * 0.4
        base_positions[0] += pitch_correction  # Adjust forward/back balance
        base_positions[1] += pitch_correction

        # Apply safety limits to ensure positions are within bounds
        for i in range(len(base_positions)):
            base_positions[i] = max(-self.max_position, min(base_positions[i], self.max_position))

        return base_positions


def main():
    """Main function to run the action server"""
    rclpy.init()

    server = HumanoidActionServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Action server shutting down')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()