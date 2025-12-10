#!/usr/bin/env python3
"""
Action Client for Humanoid Robot Tasks
This example demonstrates how to create an action client that can send goals
to the humanoid action server and handle feedback and results with safety checks.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState, Imu
import time
from typing import List


class HumanoidActionClient(Node):
    """
    Action client for humanoid robot tasks with safety monitoring
    """
    def __init__(self):
        super().__init__('humanoid_action_client')

        # Create action clients for different humanoid tasks
        # Using placeholder action types - in practice, you'd define specific action files
        self.walk_client = ActionClient(self, None, 'walk_to_pose')  # Placeholder
        self.grasp_client = ActionClient(self, None, 'grasp_object')  # Placeholder
        self.balance_client = ActionClient(self, None, 'maintain_balance')  # Placeholder

        # Publisher for monitoring
        self.monitor_pub = self.create_publisher(JointState, '/action_client_monitor', 10)

        # Subscriber for safety monitoring
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Safety monitoring
        self.current_imu_data = None
        self.current_joint_states = None
        self.safety_thresholds = {
            'max_tilt': 0.5,  # radians
            'max_joint_velocity': 1.0,  # rad/s
        }

        self.get_logger().info('Humanoid Action Client initialized with safety monitoring')

    def imu_callback(self, msg: Imu):
        """Callback to update IMU data for safety monitoring"""
        self.current_imu_data = msg

    def joint_state_callback(self, msg: JointState):
        """Callback to update joint states for safety monitoring"""
        self.current_joint_states = msg

    def is_safe_to_proceed(self) -> bool:
        """Check if it's safe to proceed with actions based on sensor data"""
        if self.current_imu_data:
            # Check if robot is tilted beyond safe threshold
            roll = self.current_imu_data.orientation.x  # Simplified
            pitch = self.current_imu_data.orientation.y  # Simplified

            if abs(roll) > self.safety_thresholds['max_tilt'] or \
               abs(pitch) > self.safety_thresholds['max_tilt']:
                self.get_logger().warn('Robot tilt exceeds safety threshold')
                return False

        return True

    def send_walk_goal(self, target_x: float = 1.0, target_y: float = 0.0, target_theta: float = 0.0):
        """Send a walk goal to the action server with safety checks"""
        self.get_logger().info(f'Sending walk goal: x={target_x}, y={target_y}, theta={target_theta}')

        # Safety check before sending goal
        if not self.is_safe_to_proceed():
            self.get_logger().error('Walk goal rejected: Safety conditions not met')
            return

        # In a real implementation, you'd create a proper goal message
        # For this example, we'll simulate the process
        goal_msg = None  # Placeholder - would be a proper action goal message

        # Wait for the action server
        if not self.walk_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Walk action server not available')
            return

        # Send the goal with feedback callback
        send_goal_future = self.walk_client.send_goal_async(
            goal_msg,
            feedback_callback=self.walk_feedback_callback
        )

        # Add done callback for goal response
        send_goal_future.add_done_callback(self.walk_goal_response_callback)

    def walk_goal_response_callback(self, future):
        """Handle response when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Walk goal rejected')
            return

        self.get_logger().info('Walk goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.walk_result_callback)

    def walk_feedback_callback(self, feedback_msg):
        """Handle feedback during walk execution"""
        self.get_logger().info(f'Walk progress: {feedback_msg.feedback.progress_percentage:.1f}%')

        # Additional safety check during execution based on feedback
        if hasattr(feedback_msg, 'current_pose'):
            # Could implement additional checks based on current pose
            pass

    def walk_result_callback(self, future):
        """Handle the final result of the walk action"""
        result = future.result().result
        self.get_logger().info(f'Walk result: {result.success}')

    def send_grasp_goal(self, object_name: str = "red_cube", approach_pose: List[float] = None):
        """Send a grasp goal to the action server with safety checks"""
        if approach_pose is None:
            approach_pose = [0.5, 0.0, 0.3, 0.0, 0.0, 0.0]  # Default approach pose [x, y, z, roll, pitch, yaw]

        self.get_logger().info(f'Sending grasp goal for {object_name}')

        # Safety check before sending goal
        if not self.is_safe_to_proceed():
            self.get_logger().error('Grasp goal rejected: Safety conditions not met')
            return

        goal_msg = None  # Placeholder - would be a proper action goal message

        # Wait for the action server
        if not self.grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Grasp action server not available')
            return

        # Send the goal with feedback callback
        send_goal_future = self.grasp_client.send_goal_async(
            goal_msg,
            feedback_callback=self.grasp_feedback_callback
        )

        send_goal_future.add_done_callback(self.grasp_goal_response_callback)

    def grasp_goal_response_callback(self, future):
        """Handle response when grasp goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Grasp goal rejected')
            return

        self.get_logger().info('Grasp goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.grasp_result_callback)

    def grasp_feedback_callback(self, feedback_msg):
        """Handle feedback during grasp execution"""
        self.get_logger().info(f'Grasp progress: {feedback_msg.feedback.progress_percentage:.1f}%')

    def grasp_result_callback(self, future):
        """Handle the final result of the grasp action"""
        result = future.result().result
        self.get_logger().info(f'Grasp result: {result.success}')

    def send_balance_goal(self, duration: float = 10.0):
        """Send a balance goal to the action server with safety checks"""
        self.get_logger().info(f'Sending balance goal for {duration} seconds')

        # Safety check before sending goal
        if not self.is_safe_to_proceed():
            self.get_logger().error('Balance goal rejected: Safety conditions not met')
            return

        goal_msg = None  # Placeholder - would be a proper action goal message

        # Wait for the action server
        if not self.balance_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Balance action server not available')
            return

        # Send the goal with feedback callback
        send_goal_future = self.balance_client.send_goal_async(
            goal_msg,
            feedback_callback=self.balance_feedback_callback
        )

        send_goal_future.add_done_callback(self.balance_goal_response_callback)

    def balance_goal_response_callback(self, future):
        """Handle response when balance goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Balance goal rejected')
            return

        self.get_logger().info('Balance goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.balance_result_callback)

    def balance_feedback_callback(self, feedback_msg):
        """Handle feedback during balance execution"""
        self.get_logger().info(f'Balance time: {feedback_msg.feedback.elapsed_time:.1f}s')

        # Monitor balance during execution
        if hasattr(feedback_msg, 'roll_error') and hasattr(feedback_msg, 'pitch_error'):
            if abs(feedback_msg.roll_error) > self.safety_thresholds['max_tilt'] or \
               abs(feedback_msg.pitch_error) > self.safety_thresholds['max_tilt']:
                self.get_logger().warn('Balance error exceeds safety threshold during execution')

    def balance_result_callback(self, future):
        """Handle the final result of the balance action"""
        result = future.result().result
        self.get_logger().info(f'Balance result: {result.success}')

    def send_cancel_request(self, goal_handle):
        """Send a cancel request for a running goal"""
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Handle response to cancel request"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


def main():
    """Main function to demonstrate the action client with safety features"""
    rclpy.init()

    client = HumanoidActionClient()

    # Example: Send a walk goal
    client.send_walk_goal(target_x=2.0, target_y=1.0, target_theta=0.5)

    # Example: Send a grasp goal
    client.send_grasp_goal(object_name="blue_box", approach_pose=[0.6, 0.1, 0.35, 0.0, 0.0, 0.0])

    # Example: Send a balance goal
    client.send_balance_goal(duration=5.0)

    # Keep the client alive to receive feedback and results
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Action client shutting down')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()