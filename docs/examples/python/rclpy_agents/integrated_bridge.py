#!/usr/bin/env python3
"""
Python Agent → ROS Bridge Example
This example demonstrates how to create an integrated bridge between Python-based
AI agents and the ROS 2 control pipeline, including perception, planning, and actuation.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
import numpy as np
import math
import json
import time
from typing import Dict, List, Optional, Tuple
import threading
import queue


class PerceptionBridge(Node):
    """
    Bridge component that processes sensor data for AI agents
    """
    def __init__(self):
        super().__init__('perception_bridge')

        # QoS profile for sensor data
        qos_profile = QoSProfile(depth=10)

        # Sensor subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu_data', self.imu_callback, qos_profile
        )
        self.perception_sub = self.create_subscription(
            String, '/processed_perception', self.perception_callback, qos_profile
        )

        # Publisher for AI-ready data
        self.ai_data_pub = self.create_publisher(String, '/ai_input_data', qos_profile)

        # Internal state
        self.current_joint_state = None
        self.current_imu_data = None
        self.current_perception_data = None
        self.last_ai_update = 0

        # Data queue for AI agent
        self.data_queue = queue.Queue()

        # Timer for processing and publishing data
        self.processing_timer = self.create_timer(0.1, self.process_and_publish_data)

        self.get_logger().info('Perception Bridge initialized')

    def joint_state_callback(self, msg: JointState):
        """Handle joint state updates"""
        self.current_joint_state = msg

    def imu_callback(self, msg: Imu):
        """Handle IMU updates"""
        self.current_imu_data = msg

    def perception_callback(self, msg: String):
        """Handle processed perception updates"""
        try:
            self.current_perception_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse perception data')

    def process_and_publish_data(self):
        """Process sensor data and make it available for AI agents"""
        if not self.current_joint_state or not self.current_imu_data:
            return

        # Create AI-ready data structure
        ai_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'robot_state': self.extract_robot_state(),
            'balance_info': self.calculate_balance_info(),
            'joint_info': self.extract_joint_info(),
        }

        # Add perception data if available
        if self.current_perception_data:
            ai_data['environment'] = self.current_perception_data.get('environment_state', {})

        # Publish to AI agent
        data_msg = String()
        data_msg.data = json.dumps(ai_data)
        self.ai_data_pub.publish(data_msg)

        # Add to queue for direct access by AI agent
        try:
            self.data_queue.put_nowait(ai_data)
        except queue.Full:
            # Remove oldest item if queue is full
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(ai_data)
            except queue.Empty:
                pass

    def extract_robot_state(self) -> Dict:
        """Extract robot state from joint data"""
        if not self.current_joint_state:
            return {}

        positions = list(self.current_joint_state.position)
        velocities = list(self.current_joint_state.velocity) if self.current_joint_state.velocity else [0.0] * len(positions)
        efforts = list(self.current_joint_state.effort) if self.current_joint_state.effort else [0.0] * len(positions)

        return {
            'joint_count': len(positions),
            'avg_position': sum(positions) / len(positions) if positions else 0.0,
            'max_position': max(positions) if positions else 0.0,
            'min_position': min(positions) if positions else 0.0,
            'avg_velocity': sum(velocities) / len(velocities) if velocities else 0.0,
            'avg_effort': sum(efforts) / len(efforts) if efforts else 0.0,
        }

    def calculate_balance_info(self) -> Dict:
        """Calculate balance information from IMU data"""
        if not self.current_imu_data:
            return {'balanced': False, 'roll': 0.0, 'pitch': 0.0}

        # Extract roll and pitch from quaternion
        orientation = self.current_imu_data.orientation
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Check if robot is balanced
        balance_threshold = 0.5  # radians
        is_balanced = abs(roll) <= balance_threshold and abs(pitch) <= balance_threshold

        return {
            'roll': roll,
            'pitch': pitch,
            'balanced': is_balanced,
            'tilt_angle': math.sqrt(roll**2 + pitch**2)
        }

    def extract_joint_info(self) -> Dict:
        """Extract detailed joint information"""
        if not self.current_joint_state:
            return {}

        joint_info = {}
        for i, name in enumerate(self.current_joint_state.name):
            pos = self.current_joint_state.position[i] if i < len(self.current_joint_state.position) else 0.0
            vel = self.current_joint_state.velocity[i] if i < len(self.current_joint_state.velocity) else 0.0
            eff = self.current_joint_state.effort[i] if i < len(self.current_joint_state.effort) else 0.0

            joint_info[name] = {
                'position': pos,
                'velocity': vel,
                'effort': eff
            }

        return joint_info


class PlanningBridge(Node):
    """
    Bridge component that translates AI decisions to ROS trajectories
    """
    def __init__(self):
        super().__init__('planning_bridge')

        # Subscribers
        self.ai_command_sub = self.create_subscription(
            String, '/ai_commands', self.ai_command_callback, 10
        )
        self.perception_bridge_sub = self.create_subscription(
            String, '/ai_input_data', self.perception_data_callback, 10
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(Float64MultiArray, '/planned_trajectory', 10)
        self.goal_pub = self.create_publisher(Pose, '/target_pose', 10)

        # Internal state
        self.current_ai_command = None
        self.current_perception = None
        self.joint_names = []  # Will be populated from perception data

        self.get_logger().info('Planning Bridge initialized')

    def perception_data_callback(self, msg: String):
        """Handle perception data from bridge"""
        try:
            data = json.loads(msg.data)
            self.current_perception = data

            # Update joint names if available
            if 'joint_info' in data:
                self.joint_names = list(data['joint_info'].keys())
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse perception data')

    def ai_command_callback(self, msg: String):
        """Handle AI commands"""
        try:
            command = json.loads(msg.data)
            self.current_ai_command = command
            self.process_ai_command(command)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse AI command')

    def process_ai_command(self, command: Dict):
        """Process AI command and generate appropriate trajectory"""
        command_type = command.get('type', 'unknown')

        if command_type == 'move_to_pose':
            self.handle_move_to_pose_command(command)
        elif command_type == 'balance':
            self.handle_balance_command(command)
        elif command_type == 'reach':
            self.handle_reach_command(command)
        elif command_type == 'walk':
            self.handle_walk_command(command)
        else:
            self.get_logger().warn(f'Unknown command type: {command_type}')

    def handle_move_to_pose_command(self, command: Dict):
        """Handle move to pose command"""
        target_pose = command.get('target_pose', {})
        if not target_pose:
            self.get_logger().warn('No target pose provided')
            return

        # Generate trajectory to reach target pose
        trajectory = self.generate_pose_trajectory(target_pose)
        if trajectory:
            self.publish_trajectory(trajectory)

    def handle_balance_command(self, command: Dict):
        """Handle balance command"""
        # Generate trajectory to maintain balance
        trajectory = self.generate_balance_trajectory()
        if trajectory:
            self.publish_trajectory(trajectory)

    def handle_reach_command(self, command: Dict):
        """Handle reach command"""
        target_position = command.get('target_position', {})
        if not target_position:
            self.get_logger().warn('No target position provided for reach command')
            return

        # Generate trajectory for reaching
        trajectory = self.generate_reach_trajectory(target_position)
        if trajectory:
            self.publish_trajectory(trajectory)

    def handle_walk_command(self, command: Dict):
        """Handle walk command"""
        target_location = command.get('target_location', {})
        if not target_location:
            self.get_logger().warn('No target location provided for walk command')
            return

        # Publish goal for walking
        goal_pose = Pose()
        goal_pose.position.x = target_location.get('x', 0.0)
        goal_pose.position.y = target_location.get('y', 0.0)
        goal_pose.position.z = target_location.get('z', 0.0)
        goal_pose.orientation.w = 1.0  # No rotation

        self.goal_pub.publish(goal_pose)

    def generate_pose_trajectory(self, target_pose: Dict) -> Optional[List[float]]:
        """Generate trajectory to reach a specific pose"""
        if not self.current_perception:
            return None

        # This is a simplified trajectory generation
        # In practice, this would involve inverse kinematics and path planning
        current_state = self.current_perception.get('robot_state', {})
        balance_info = self.current_perception.get('balance_info', {})

        # Create base trajectory (28 DOF humanoid)
        trajectory = [0.0] * 28

        # Apply balance corrections if needed
        if not balance_info.get('balanced', True):
            # Adjust for balance first
            roll_correction = -balance_info.get('roll', 0.0) * 0.5
            pitch_correction = -balance_info.get('pitch', 0.0) * 0.3

            # Apply to hip and ankle joints (simplified)
            trajectory[0] += roll_correction  # Left hip roll
            trajectory[1] -= roll_correction  # Right hip roll
            trajectory[2] += pitch_correction  # Left ankle pitch
            trajectory[3] -= pitch_correction  # Right ankle pitch

        # Apply target pose adjustments (simplified)
        # This would be more sophisticated with proper inverse kinematics
        pose_weight = 0.1
        for i in range(min(6, len(trajectory))):  # Adjust first 6 joints
            trajectory[i] += pose_weight

        return trajectory

    def generate_balance_trajectory(self) -> List[float]:
        """Generate trajectory to maintain balance"""
        if not self.current_perception:
            return [0.0] * 28

        balance_info = self.current_perception.get('balance_info', {})
        trajectory = [0.0] * 28

        # Apply balance corrections
        roll_correction = -balance_info.get('roll', 0.0) * 0.5
        pitch_correction = -balance_info.get('pitch', 0.0) * 0.3

        # Adjust joints for balance
        trajectory[0] += roll_correction  # Left hip roll
        trajectory[1] -= roll_correction  # Right hip roll
        trajectory[2] += pitch_correction  # Left ankle pitch
        trajectory[3] -= pitch_correction  # Right ankle pitch

        return trajectory

    def generate_reach_trajectory(self, target_position: Dict) -> Optional[List[float]]:
        """Generate trajectory for reaching a target position"""
        if not self.current_perception:
            return None

        # Simplified reaching trajectory generation
        trajectory = [0.0] * 28

        # Apply reaching motion to arm joints (simplified)
        # In practice, this would use inverse kinematics
        reach_weight = 0.3
        # Left arm joints (indices 4-10)
        for i in range(4, 11):
            if i < len(trajectory):
                trajectory[i] += reach_weight

        return trajectory

    def publish_trajectory(self, trajectory: List[float]):
        """Publish generated trajectory"""
        trajectory_msg = Float64MultiArray()
        trajectory_msg.data = trajectory
        self.trajectory_pub.publish(trajectory_msg)


class ActionBridge(Node):
    """
    Bridge component that handles ROS actions for complex humanoid tasks
    """
    def __init__(self):
        super().__init__('action_bridge')

        # Action clients for complex tasks
        self.walk_action_client = ActionClient(self, None, 'walk_to_pose')  # Placeholder
        self.grasp_action_client = ActionClient(self, None, 'grasp_object')  # Placeholder
        self.balance_action_client = ActionClient(self, None, 'maintain_balance')  # Placeholder

        # Subscribers for AI action commands
        self.ai_action_sub = self.create_subscription(
            String, '/ai_action_commands', self.ai_action_callback, 10
        )

        self.get_logger().info('Action Bridge initialized')

    def ai_action_callback(self, msg: String):
        """Handle AI action commands"""
        try:
            command = json.loads(msg.data)
            action_type = command.get('action_type', 'unknown')

            if action_type == 'walk':
                self.send_walk_action(command)
            elif action_type == 'grasp':
                self.send_grasp_action(command)
            elif action_type == 'balance':
                self.send_balance_action(command)
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse AI action command')

    def send_walk_action(self, command: Dict):
        """Send walk action"""
        # In practice, this would create a proper action goal message
        self.get_logger().info(f'Sending walk action to: {command.get("target", "unknown")}')

    def send_grasp_action(self, command: Dict):
        """Send grasp action"""
        # In practice, this would create a proper action goal message
        self.get_logger().info(f'Sending grasp action for: {command.get("object", "unknown")}')

    def send_balance_action(self, command: Dict):
        """Send balance action"""
        # In practice, this would create a proper action goal message
        self.get_logger().info('Sending balance action')


class AIAgentBridge(Node):
    """
    Main bridge node that integrates all components for AI agent interaction
    """
    def __init__(self):
        super().__init__('ai_agent_bridge')

        # Create bridge components
        self.perception_bridge = PerceptionBridge()
        self.planning_bridge = PlanningBridge()
        self.action_bridge = ActionBridge()

        # Publishers for AI commands
        self.ai_command_pub = self.create_publisher(String, '/ai_commands', 10)
        self.ai_action_command_pub = self.create_publisher(String, '/ai_action_commands', 10)

        # Subscribers for system status
        self.system_status_sub = self.create_subscription(
            String, '/system_status', self.system_status_callback, 10
        )

        # Internal state
        self.system_status = {}
        self.ai_commands_queue = queue.Queue()

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.5, self.ai_decision_cycle)

        self.get_logger().info('AI Agent Bridge initialized and ready')

    def system_status_callback(self, msg: String):
        """Handle system status updates"""
        try:
            self.system_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse system status')

    def ai_decision_cycle(self):
        """Main AI decision cycle"""
        # Get latest perception data
        perception_data = self.get_latest_perception_data()

        if perception_data:
            # Make AI decisions based on perception
            ai_decision = self.make_ai_decision(perception_data)
            if ai_decision:
                self.send_ai_command(ai_decision)

    def get_latest_perception_data(self) -> Optional[Dict]:
        """Get the latest perception data from the queue"""
        try:
            # Get the most recent data, discarding older ones
            latest_data = None
            while not self.perception_bridge.data_queue.empty():
                latest_data = self.perception_bridge.data_queue.get_nowait()
            return latest_data
        except queue.Empty:
            return None

    def make_ai_decision(self, perception_data: Dict) -> Optional[Dict]:
        """Make AI decision based on perception data"""
        # This is a simplified decision-making process
        # In practice, this would involve more sophisticated AI algorithms

        balance_info = perception_data.get('balance_info', {})
        robot_state = perception_data.get('robot_state', {})

        # Check if robot needs to balance
        if not balance_info.get('balanced', True):
            return {
                'type': 'balance',
                'priority': 'high',
                'reason': 'Robot is unbalanced'
            }

        # Check if robot should move based on some criteria
        # For example, if tilt angle is too high
        tilt_angle = balance_info.get('tilt_angle', 0.0)
        if tilt_angle > 0.3:  # If tilt is significant
            return {
                'type': 'balance',
                'priority': 'high',
                'reason': f'Significant tilt detected: {tilt_angle:.2f} rad'
            }

        # Example: Move to a new position periodically
        current_time = time.time()
        if current_time % 10 < 1:  # Every 10 seconds
            return {
                'type': 'move_to_pose',
                'target_pose': {
                    'x': 1.0,
                    'y': 0.0,
                    'z': 0.0
                },
                'priority': 'medium',
                'reason': 'Periodic movement command'
            }

        return None  # No action needed

    def send_ai_command(self, command: Dict):
        """Send AI command through appropriate channel"""
        command['timestamp'] = self.get_clock().now().nanoseconds

        command_msg = String()
        command_msg.data = json.dumps(command)

        # Route command to appropriate bridge
        if command.get('type') in ['move_to_pose', 'balance', 'reach', 'walk']:
            self.planning_bridge.current_ai_command = command
            self.planning_bridge.process_ai_command(command)
            self.ai_command_pub.publish(command_msg)
        elif command.get('type') in ['walk_action', 'grasp_action', 'balance_action']:
            self.action_bridge.ai_action_callback(command_msg)
            self.ai_action_command_pub.publish(command_msg)

    def send_direct_command(self, command_type: str, **kwargs):
        """Send a command directly from external code"""
        command = {
            'type': command_type,
            'timestamp': self.get_clock().now().nanoseconds,
            **kwargs
        }

        self.send_ai_command(command)


def main(args=None):
    """Main function to run the AI Agent Bridge"""
    rclpy.init(args=args)

    # Create the AI Agent Bridge
    ai_bridge = AIAgentBridge()

    try:
        # Run with MultiThreadedExecutor to handle multiple bridge components
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(ai_bridge)
        executor.add_node(ai_bridge.perception_bridge)
        executor.add_node(ai_bridge.planning_bridge)
        executor.add_node(ai_bridge.action_bridge)

        executor.spin()
    except KeyboardInterrupt:
        ai_bridge.get_logger().info('Shutting down AI Agent Bridge')
    finally:
        # Cleanup
        ai_bridge.destroy_node()
        ai_bridge.perception_bridge.destroy_node()
        ai_bridge.planning_bridge.destroy_node()
        ai_bridge.action_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()