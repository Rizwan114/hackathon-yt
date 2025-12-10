#!/usr/bin/env python3
"""
Multi-Node ROS 2 System Example: Complete Humanoid Control Pipeline
This example demonstrates a complete perception → planning → actuation pipeline
with multiple interconnected ROS 2 nodes for humanoid robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, Imu, Image, LaserScan
from geometry_msgs.msg import Pose, Twist
from builtin_interfaces.msg import Duration
import numpy as np
import math
import json
from typing import List, Dict, Optional


class PerceptionNode(Node):
    """
    Perception Node: Processes sensor data and publishes processed information
    """
    def __init__(self):
        super().__init__('perception_node')

        # QoS profile for sensor data
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        # Sensor subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu_data', self.imu_callback, qos_profile
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, qos_profile
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile
        )

        # Publisher for processed perception data
        self.perception_pub = self.create_publisher(
            String, '/processed_perception', qos_profile
        )

        # Store current sensor states
        self.current_joint_state = None
        self.current_imu_data = None
        self.current_camera_data = None
        self.current_lidar_data = None

        # Timer for perception processing (10Hz)
        self.perception_timer = self.create_timer(0.1, self.process_perception)

        # Statistics
        self.perception_count = 0

        self.get_logger().info('Perception Node initialized')

    def joint_callback(self, msg: JointState):
        """Handle joint state updates"""
        self.current_joint_state = msg

    def imu_callback(self, msg: Imu):
        """Handle IMU data updates"""
        self.current_imu_data = msg

    def camera_callback(self, msg: Image):
        """Handle camera image updates"""
        self.current_camera_data = msg

    def lidar_callback(self, msg: LaserScan):
        """Handle LiDAR scan updates"""
        self.current_lidar_data = msg

    def process_perception(self):
        """Process all sensor data and publish processed information"""
        if not self.current_joint_state or not self.current_imu_data:
            return

        # Extract robot state from joint and IMU data
        robot_state = self.extract_robot_state()

        # Extract environment state from sensors
        environment_state = self.extract_environment_state()

        # Create perception message
        perception_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'robot_state': robot_state,
            'environment_state': environment_state,
            'perception_count': self.perception_count
        }

        perception_msg = String()
        perception_msg.data = json.dumps(perception_data)
        self.perception_pub.publish(perception_msg)

        self.perception_count += 1

        self.get_logger().debug(f'Published perception data: {robot_state["balance_status"]}')

    def extract_robot_state(self) -> Dict:
        """Extract meaningful robot state from sensor data"""
        if not self.current_joint_state or not self.current_imu_data:
            return {'error': 'No sensor data'}

        # Calculate balance from IMU
        roll, pitch = self.quaternion_to_euler(
            self.current_imu_data.orientation
        )

        # Calculate joint statistics
        joint_positions = list(self.current_joint_state.position)
        avg_position = sum(joint_positions) / len(joint_positions) if joint_positions else 0.0
        max_position = max(joint_positions) if joint_positions else 0.0
        min_position = min(joint_positions) if joint_positions else 0.0

        # Determine balance status
        balance_threshold = 0.5  # radians
        is_balanced = abs(roll) <= balance_threshold and abs(pitch) <= balance_threshold

        return {
            'joint_count': len(self.current_joint_state.position),
            'avg_joint_position': avg_position,
            'max_joint_position': max_position,
            'min_joint_position': min_position,
            'roll': roll,
            'pitch': pitch,
            'balance_status': 'balanced' if is_balanced else 'unbalanced',
            'is_balanced': is_balanced
        }

    def extract_environment_state(self) -> Dict:
        """Extract environment information from sensors"""
        env_state = {}

        # Process LiDAR data for obstacles
        if self.current_lidar_data:
            valid_ranges = [r for r in self.current_lidar_data.ranges if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                env_state['nearest_obstacle'] = min(valid_ranges)
                env_state['obstacle_count'] = len(valid_ranges)
            else:
                env_state['nearest_obstacle'] = float('inf')
                env_state['obstacle_count'] = 0

        # Process camera data (simplified - in practice would do object detection)
        if self.current_camera_data:
            env_state['camera_available'] = True
            env_state['image_width'] = self.current_camera_data.width
            env_state['image_height'] = self.current_camera_data.height

        return env_state

    def quaternion_to_euler(self, quaternion) -> tuple:
        """Convert quaternion to Euler angles (roll, pitch) - simplified for z-axis up"""
        # Simplified conversion - in practice use proper quaternion math
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        return roll, pitch


class PlanningNode(Node):
    """
    Planning Node: Processes perception data and generates movement plans
    """
    def __init__(self):
        super().__init__('planning_node')

        # Subscribers
        self.perception_sub = self.create_subscription(
            String, '/processed_perception', self.perception_callback, 10
        )
        self.goal_sub = self.create_subscription(
            Pose, '/target_pose', self.goal_callback, 10
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(
            Float64MultiArray, '/planned_trajectory', 10
        )
        self.plan_status_pub = self.create_publisher(
            String, '/plan_status', 10
        )

        # Internal state
        self.current_perception = None
        self.current_goal = None
        self.planning_active = True
        self.plan_count = 0

        # Timer for planning (5Hz - slower than perception)
        self.planning_timer = self.create_timer(0.2, self.plan_trajectory)

        self.get_logger().info('Planning Node initialized')

    def perception_callback(self, msg: String):
        """Handle perception data updates"""
        try:
            self.current_perception = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse perception data')

    def goal_callback(self, msg: Pose):
        """Handle goal pose updates"""
        self.current_goal = msg
        self.get_logger().info(f'Received new goal: ({msg.position.x}, {msg.position.y})')

    def plan_trajectory(self):
        """Plan trajectory based on current perception and goal"""
        if not self.planning_active or not self.current_perception:
            return

        # Check if robot is balanced before planning movement
        robot_state = self.current_perception.get('robot_state', {})
        is_balanced = robot_state.get('is_balanced', True)

        if not is_balanced:
            self.get_logger().warn('Robot is unbalanced - pausing movement planning')
            # Still publish a "stop" trajectory to maintain current position
            self.publish_stop_trajectory()
            return

        # Generate trajectory based on goal and current state
        if self.current_goal:
            planned_trajectory = self.generate_trajectory_to_goal()
        else:
            # Default: maintain current position
            planned_trajectory = self.generate_maintenance_trajectory()

        if planned_trajectory:
            trajectory_msg = Float64MultiArray()
            trajectory_msg.data = planned_trajectory
            self.trajectory_pub.publish(trajectory_msg)

            # Publish plan status
            status_msg = String()
            status_msg.data = json.dumps({
                'plan_id': self.plan_count,
                'timestamp': self.get_clock().now().nanoseconds,
                'trajectory_length': len(planned_trajectory),
                'goal_reached': self.check_goal_reached(planned_trajectory)
            })
            self.plan_status_pub.publish(status_msg)

            self.plan_count += 1

    def generate_trajectory_to_goal(self) -> Optional[List[float]]:
        """Generate trajectory to reach the current goal"""
        if not self.current_perception or not self.current_goal:
            return None

        # Extract current robot state
        robot_state = self.current_perception['robot_state']

        # This is a simplified trajectory generation
        # In practice, this would involve inverse kinematics, path planning, etc.

        # For demonstration, create a simple adjustment based on balance
        base_positions = [0.0] * 28  # Assuming 28 DOF humanoid

        # Apply balance corrections if needed
        roll = robot_state.get('roll', 0.0)
        pitch = robot_state.get('pitch', 0.0)

        # Simple balance adjustment
        balance_correction = 0.1
        base_positions[0] += -roll * balance_correction  # Left hip roll
        base_positions[1] += roll * balance_correction   # Right hip roll
        base_positions[2] += -pitch * balance_correction  # Left ankle pitch
        base_positions[3] += pitch * balance_correction   # Right ankle pitch

        # Apply small goal-directed adjustment (simplified)
        goal_weight = 0.05
        base_positions[4] += goal_weight  # Example: slight shoulder adjustment

        return base_positions

    def generate_maintenance_trajectory(self) -> List[float]:
        """Generate trajectory to maintain current position"""
        # Return neutral joint positions
        return [0.0] * 28  # 28 DOF humanoid in neutral position

    def check_goal_reached(self, trajectory: List[float]) -> bool:
        """Check if the goal has been reached"""
        # Simplified goal checking
        # In practice, this would compare actual robot pose to goal pose
        return False  # Placeholder

    def publish_stop_trajectory(self):
        """Publish trajectory to stop and maintain position"""
        stop_trajectory = [0.0] * 28  # Neutral position for all joints
        trajectory_msg = Float64MultiArray()
        trajectory_msg.data = stop_trajectory
        self.trajectory_pub.publish(trajectory_msg)


class URDFIntegrationNode(Node):
    """
    URDF Integration Node: Loads and manages URDF model information
    """
    def __init__(self):
        super().__init__('urdf_integration_node')

        # Publishers for URDF-related information
        self.joint_limits_pub = self.create_publisher(String, '/joint_limits', 10)

        # Load URDF model
        self.robot_model = self.load_urdf_model()

        # Extract joint information from URDF
        self.joint_names = []
        self.joint_limits = {}
        self.link_names = []

        if self.robot_model:
            self.extract_urdf_info()
            self.publish_urdf_info()
        else:
            # Use default humanoid configuration if URDF not available
            self.setup_default_humanoid()

        self.get_logger().info(f'URDF Integration Node loaded model with {len(self.joint_names)} joints')

    def load_urdf_model(self):
        """Load URDF model from file"""
        try:
            # Try to import urdf_parser_py to load the URDF
            import os
            from urdf_parser_py.urdf import URDF

            # Look for the humanoid model URDF file
            urdf_paths = [
                '/path/to/humanoid_model.urdf',  # Default path
                os.path.expanduser('~/robot_models/humanoid_model.urdf'),
                os.path.join(os.path.dirname(__file__), '../../../urdf/humanoid_model.urdf'),
                os.path.join(os.path.dirname(__file__), '../../urdf/humanoid_model.urdf'),
                os.path.join(os.path.dirname(__file__), '../urdf/humanoid_model.urdf'),
                os.path.join(os.path.dirname(__file__), 'humanoid_model.urdf')
            ]

            for urdf_path in urdf_paths:
                if os.path.exists(urdf_path):
                    try:
                        robot = URDF.from_xml_file(urdf_path)
                        self.get_logger().info(f'Loaded URDF from: {urdf_path}')
                        return robot
                    except Exception as e:
                        self.get_logger().warn(f'Failed to load URDF from {urdf_path}: {e}')
                        continue

            self.get_logger().warn('URDF file not found, using default model')
            return None
        except ImportError:
            self.get_logger().warn('urdf_parser_py not available, using default model')
            return None
        except Exception as e:
            self.get_logger().error(f'Error loading URDF model: {e}')
            return None

    def extract_urdf_info(self):
        """Extract joint and link information from URDF model"""
        if not self.robot_model:
            return

        # Extract joint names and limits
        for joint in self.robot_model.joints:
            if joint.type != 'fixed':  # Skip fixed joints
                self.joint_names.append(joint.name)
                if joint.limit:
                    self.joint_limits[joint.name] = {
                        'lower': joint.limit.lower if joint.limit.lower is not None else -math.pi,
                        'upper': joint.limit.upper if joint.limit.upper is not None else math.pi,
                        'effort': joint.limit.effort if joint.limit.effort is not None else 100.0,
                        'velocity': joint.limit.velocity if joint.limit.velocity is not None else 2.0
                    }
                else:
                    # Default limits if not specified in URDF
                    self.joint_limits[joint.name] = {
                        'lower': -math.pi,
                        'upper': math.pi,
                        'effort': 100.0,
                        'velocity': 2.0
                    }

        # Extract link names
        for link in self.robot_model.links:
            self.link_names.append(link.name)

    def setup_default_humanoid(self):
        """Setup default humanoid configuration if URDF not available"""
        # Define a default 28 DOF humanoid joint structure
        self.joint_names = [
            # Torso
            'torso_joint',
            # Head
            'head_pan_joint', 'head_tilt_joint',
            # Left Arm
            'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
            'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint',
            # Right Arm
            'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
            'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint',
            # Left Leg
            'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint',
            'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint',
            # Right Leg
            'r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint',
            'r_knee_pitch_joint', 'r_ankle_pitch_joint', 'r_ankle_roll_joint'
        ]

        # Set default limits for each joint
        for joint_name in self.joint_names:
            if 'hip' in joint_name or 'knee' in joint_name or 'ankle' in joint_name:
                # Leg joints
                self.joint_limits[joint_name] = {
                    'lower': -2.0 if 'roll' not in joint_name else -0.5,
                    'upper': 1.0 if 'pitch' in joint_name or 'roll' in joint_name else 2.0,
                    'effort': 200.0,
                    'velocity': 3.0
                }
            elif 'shoulder' in joint_name or 'elbow' in joint_name or 'wrist' in joint_name:
                # Arm joints
                self.joint_limits[joint_name] = {
                    'lower': -2.5,
                    'upper': 2.5,
                    'effort': 100.0,
                    'velocity': 2.0
                }
            elif 'head' in joint_name:
                # Head joints
                self.joint_limits[joint_name] = {
                    'lower': -1.0,
                    'upper': 1.0,
                    'effort': 10.0,
                    'velocity': 1.0
                }
            else:
                # Default limits
                self.joint_limits[joint_name] = {
                    'lower': -math.pi,
                    'upper': math.pi,
                    'effort': 100.0,
                    'velocity': 2.0
                }

    def publish_urdf_info(self):
        """Publish URDF information for other nodes to use"""
        urdf_info = {
            'joint_names': self.joint_names,
            'joint_limits': self.joint_limits,
            'link_names': self.link_names,
            'total_joints': len(self.joint_names)
        }

        info_msg = String()
        info_msg.data = json.dumps(urdf_info)
        self.joint_limits_pub.publish(info_msg)

    def validate_joint_positions(self, joint_positions: List[float], joint_names: List[str] = None) -> List[float]:
        """Validate joint positions against URDF limits"""
        if not joint_positions:
            return []

        validated_positions = []

        for i, pos in enumerate(joint_positions):
            if i < len(self.joint_names):
                joint_name = self.joint_names[i]
                if joint_name in self.joint_limits:
                    limits = self.joint_limits[joint_name]
                    # Apply position limits
                    limited_pos = max(limits['lower'], min(pos, limits['upper']))
                    validated_positions.append(limited_pos)
                else:
                    # If no limits defined, apply conservative limits
                    limited_pos = max(-math.pi, min(pos, math.pi))
                    validated_positions.append(limited_pos)
            else:
                # If more positions than joints, apply conservative limits
                limited_pos = max(-math.pi, min(pos, math.pi))
                validated_positions.append(limited_pos)

        return validated_positions


class ActuationNode(Node):
    """
    Actuation Node: Executes planned trajectories with safety validation using URDF limits
    """
    def __init__(self):
        super().__init__('actuation_node')

        # Subscribers
        self.trajectory_sub = self.create_subscription(
            Float64MultiArray, '/planned_trajectory', self.trajectory_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.urdf_info_sub = self.create_subscription(
            String, '/joint_limits', self.urdf_info_callback, 10
        )

        # Publishers
        self.joint_command_pub = self.create_publisher(
            JointState, '/joint_commands', 10
        )
        self.actuation_status_pub = self.create_publisher(
            String, '/actuation_status', 10
        )

        # Internal state
        self.current_trajectory = None
        self.current_joint_state = None
        self.joint_names_from_urdf = []
        self.joint_limits_from_urdf = {}
        self.actuation_active = True
        self.command_count = 0

        # Safety parameters (will be overridden by URDF if available)
        self.max_joint_velocity = 2.0         # rad/s
        self.max_joint_effort = 100.0         # N*m
        self.control_frequency = 100          # Hz

        # Timer for actuation (100Hz - fast for control)
        self.actuation_timer = self.create_timer(
            1.0/self.control_frequency, self.execute_commands
        )

        self.get_logger().info('Actuation Node initialized with URDF-based safety validation')

    def urdf_info_callback(self, msg: String):
        """Handle URDF information updates"""
        try:
            urdf_info = json.loads(msg.data)
            self.joint_names_from_urdf = urdf_info.get('joint_names', [])
            self.joint_limits_from_urdf = urdf_info.get('joint_limits', {})

            # Update safety parameters based on URDF
            if self.joint_limits_from_urdf:
                # Use the most restrictive velocity limit from URDF
                vel_limits = [limits.get('velocity', 2.0) for limits in self.joint_limits_from_urdf.values()]
                if vel_limits:
                    self.max_joint_velocity = min(vel_limits) if vel_limits else 2.0

        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse URDF info')

    def trajectory_callback(self, msg: Float64MultiArray):
        """Handle planned trajectory updates"""
        self.current_trajectory = list(msg.data)

    def joint_state_callback(self, msg: JointState):
        """Handle current joint state updates"""
        self.current_joint_state = msg

    def execute_commands(self):
        """Execute joint commands with URDF-based safety validation"""
        if not self.actuation_active or not self.current_trajectory:
            return

        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Use joint names from URDF if available, otherwise generate generic names
        if self.joint_names_from_urdf:
            # Use URDF joint names but limit to the number of trajectory values
            joint_cmd.name = self.joint_names_from_urdf[:len(self.current_trajectory)]
        else:
            # Generate generic joint names
            joint_cmd.name = [f'joint_{i}' for i in range(len(self.current_trajectory))]

        # Apply URDF-based safety validation to trajectory
        validated_positions = self.validate_trajectory_with_urdf_limits(self.current_trajectory)
        joint_cmd.position = validated_positions

        # Calculate velocities for smooth motion based on URDF limits
        if self.current_joint_state and len(self.current_joint_state.position) >= len(validated_positions):
            velocities = []
            dt = 1.0 / self.control_frequency

            for i, (curr_pos, target_pos) in enumerate(zip(
                self.current_joint_state.position[:len(validated_positions)],
                validated_positions
            )):
                vel = (target_pos - curr_pos) / dt

                # Apply velocity limits based on URDF if available
                if i < len(joint_cmd.name):
                    joint_name = joint_cmd.name[i]
                    if joint_name in self.joint_limits_from_urdf:
                        max_vel = self.joint_limits_from_urdf[joint_name].get('velocity', self.max_joint_velocity)
                        vel = max(-max_vel, min(vel, max_vel))
                    else:
                        vel = max(-self.max_joint_velocity, min(vel, self.max_joint_velocity))

                velocities.append(vel)

            joint_cmd.velocity = velocities

        # Publish the command
        self.joint_command_pub.publish(joint_cmd)

        # Publish actuation status
        status_msg = String()
        status_msg.data = json.dumps({
            'command_id': self.command_count,
            'timestamp': self.get_clock().now().nanoseconds,
            'joints_controlled': len(validated_positions),
            'safety_validation_passed': True,
            'urdf_based_limits': bool(self.joint_limits_from_urdf)
        })
        self.actuation_status_pub.publish(status_msg)

        self.command_count += 1

    def validate_trajectory_with_urdf_limits(self, trajectory: List[float]) -> List[float]:
        """Validate trajectory using URDF joint limits"""
        if not trajectory:
            return []

        validated_trajectory = []
        for i, pos in enumerate(trajectory):
            if i < len(self.joint_names_from_urdf):
                joint_name = self.joint_names_from_urdf[i]
                if joint_name in self.joint_limits_from_urdf:
                    limits = self.joint_limits_from_urdf[joint_name]
                    # Apply position limits from URDF
                    limited_pos = max(limits['lower'], min(pos, limits['upper']))
                    validated_trajectory.append(limited_pos)
                else:
                    # If no limits in URDF for this joint, apply conservative limits
                    limited_pos = max(-math.pi, min(pos, math.pi))
                    validated_trajectory.append(limited_pos)
            else:
                # If more positions than URDF joints, apply conservative limits
                limited_pos = max(-math.pi, min(pos, math.pi))
                validated_trajectory.append(limited_pos)

        return validated_trajectory


class ControlPipelineManager(Node):
    """
    Manager Node: Coordinates the entire control pipeline
    """
    def __init__(self):
        super().__init__('control_pipeline_manager')

        # Publishers for pipeline control
        self.goal_pub = self.create_publisher(Pose, '/target_pose', 10)
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)

        # Subscribers for monitoring
        self.perception_sub = self.create_subscription(
            String, '/processed_perception', self.perception_monitor, 10
        )
        self.plan_status_sub = self.create_subscription(
            String, '/plan_status', self.plan_status_monitor, 10
        )
        self.actuation_status_sub = self.create_subscription(
            String, '/actuation_status', self.actuation_status_monitor, 10
        )

        # Internal state
        self.pipeline_active = True
        self.monitoring_active = True
        self.system_status = {
            'perception_active': True,
            'planning_active': True,
            'actuation_active': True,
            'last_update': 0
        }

        # Timer for system management (1Hz)
        self.management_timer = self.create_timer(1.0, self.manage_pipeline)

        # Timer for status publishing (2Hz)
        self.status_timer = self.create_timer(0.5, self.publish_system_status)

        # Demo goal counter
        self.goal_counter = 0

        self.get_logger().info('Control Pipeline Manager initialized')

    def perception_monitor(self, msg: String):
        """Monitor perception node status"""
        try:
            data = json.loads(msg.data)
            self.system_status['last_perception_time'] = data.get('timestamp', 0)
            self.system_status['perception_active'] = True
        except:
            self.system_status['perception_active'] = False

    def plan_status_monitor(self, msg: String):
        """Monitor planning node status"""
        try:
            data = json.loads(msg.data)
            self.system_status['last_plan_time'] = data.get('timestamp', 0)
            self.system_status['planning_active'] = True
        except:
            self.system_status['planning_active'] = False

    def actuation_status_monitor(self, msg: String):
        """Monitor actuation node status"""
        try:
            data = json.loads(msg.data)
            self.system_status['last_actuation_time'] = data.get('timestamp', 0)
            self.system_status['actuation_active'] = True
        except:
            self.system_status['actuation_active'] = False

    def manage_pipeline(self):
        """Manage the overall pipeline operation"""
        if not self.pipeline_active:
            return

        # Check system health
        self.check_system_health()

        # Send demo goals periodically
        if self.goal_counter % 10 == 0:  # Every 10 seconds
            self.send_demo_goal()

        self.goal_counter += 1

    def check_system_status(self):
        """Check if all pipeline components are active"""
        current_time = self.get_clock().now().nanoseconds

        # Check if nodes are publishing data (within last 5 seconds)
        perception_ok = (current_time - self.system_status.get('last_perception_time', 0)) < 5e9
        planning_ok = (current_time - self.system_status.get('last_plan_time', 0)) < 5e9
        actuation_ok = (current_time - self.system_status.get('last_actuation_time', 0)) < 5e9

        return perception_ok and planning_ok and actuation_ok

    def check_system_health(self):
        """Check overall system health"""
        health_ok = self.check_system_status()

        if not health_ok:
            self.get_logger().warn('System health check failed - some nodes may be unresponsive')

    def send_demo_goal(self):
        """Send a demonstration goal to the system"""
        goal_pose = Pose()
        goal_pose.position.x = 1.0 * (self.goal_counter // 10 % 2)  # Alternate between 0 and 1
        goal_pose.position.y = 0.5
        goal_pose.position.z = 0.0
        goal_pose.orientation.w = 1.0  # No rotation

        self.goal_pub.publish(goal_pose)
        self.get_logger().info(f'Sent demo goal {self.goal_counter//10}: ({goal_pose.position.x}, {goal_pose.position.y})')

    def publish_system_status(self):
        """Publish overall system status"""
        self.system_status['timestamp'] = self.get_clock().now().nanoseconds
        self.system_status['pipeline_active'] = self.pipeline_active
        self.system_status['health_ok'] = self.check_system_status()

        status_msg = String()
        status_msg.data = json.dumps(self.system_status)
        self.system_status_pub.publish(status_msg)


def main(args=None):
    """Main function to run the complete control pipeline with URDF integration"""
    rclpy.init(args=args)

    # Create all pipeline nodes
    urdf_node = URDFIntegrationNode()
    perception_node = PerceptionNode()
    planning_node = PlanningNode()
    actuation_node = ActuationNode()
    manager_node = ControlPipelineManager()

    # Use MultiThreadedExecutor to run all nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(urdf_node)
    executor.add_node(perception_node)
    executor.add_node(planning_node)
    executor.add_node(actuation_node)
    executor.add_node(manager_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        urdf_node.get_logger().info('Shutting down URDF integration node')
        perception_node.get_logger().info('Shutting down perception node')
        planning_node.get_logger().info('Shutting down planning node')
        actuation_node.get_logger().info('Shutting down actuation node')
        manager_node.get_logger().info('Shutting down manager node')
    finally:
        # Shutdown all nodes
        urdf_node.destroy_node()
        perception_node.destroy_node()
        planning_node.destroy_node()
        actuation_node.destroy_node()
        manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()