---
sidebar_position: 5
title: "Chapter 5: Mini-Project — Building the Humanoid Control Backbone"
---

# Chapter 5: Mini-Project — Building the Humanoid Control Backbone

## Introduction to the End-to-End Control Pipeline

In this chapter, you'll implement a complete humanoid control pipeline that integrates all the concepts learned in previous chapters. This mini-project will demonstrate how to connect perception → planning → actuation in a real ROS 2 system using your humanoid URDF model and Python-based AI agents.

The goal is to create a functional system where:
- Perception: Sensors provide environmental and robot state data
- Planning: AI agents process sensor data and generate movement plans
- Actuation: Commands are sent to robot joints to execute movements

## Perception → Planning → Actuation Architecture

The control pipeline follows a modular architecture where each component can be developed and tested independently. This three-layer architecture is fundamental to humanoid robotics:

### Perception Layer
The perception layer processes sensor data from multiple sources to create a comprehensive understanding of both the robot's state and its environment:

- **Joint state sensors**: Provide current joint positions, velocities, and efforts
- **IMU (Inertial Measurement Unit)**: Provides orientation, angular velocity, and linear acceleration
- **Camera**: Visual input for object detection, recognition, and scene understanding
- **LiDAR**: Environment mapping, obstacle detection, and distance measurement

The perception node aggregates and processes this data to extract meaningful information about the robot's current state and its surroundings. Here's how the perception node works:

```python
class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Sensor subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publisher for processed sensor data
        self.perception_pub = self.create_publisher(String, '/processed_perception', 10)

    def process_perception(self):
        """Process all sensor data and publish processed information"""
        if not all([self.current_joint_state, self.current_imu_data]):
            return

        # Process sensor data to extract relevant information
        robot_state = self.extract_robot_state()
        environment_state = self.extract_environment_state()

        # Combine into perception message
        perception_msg = String()
        perception_msg.data = f"Robot: {robot_state}, Environment: {environment_state}"
        self.perception_pub.publish(perception_msg)
```

### Planning Layer
The planning layer takes the processed perception data and generates appropriate movement plans:

- **State estimation**: Determines the current robot pose and joint configuration
- **Path planning**: Generates trajectories to reach target poses while avoiding obstacles
- **Motion planning**: Creates detailed joint trajectories for complex movements
- **Behavior selection**: Chooses appropriate actions based on goals and current state

The planning node operates at a moderate frequency (typically 2-5Hz) to allow sufficient time for complex planning algorithms while maintaining responsiveness:

```python
class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Subscribers
        self.perception_sub = self.create_subscription(String, '/processed_perception', self.perception_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/target_pose', self.goal_callback, 10)

        # Publisher for planned trajectories
        self.trajectory_pub = self.create_publisher(Float64MultiArray, '/planned_trajectory', 10)

    def plan_trajectory(self):
        """Plan a trajectory based on current perception and goal"""
        if not self.current_goal or not self.current_perception:
            return

        # Generate trajectory based on planning algorithm
        planned_trajectory = self.generate_trajectory(
            self.current_robot_state,
            self.current_goal
        )

        if planned_trajectory:
            trajectory_msg = Float64MultiArray()
            trajectory_msg.data = planned_trajectory
            self.trajectory_pub.publish(trajectory_msg)
```

### Actuation Layer
The actuation layer executes the planned movements with strict safety validation:

- **Joint command generation**: Converts planned trajectories to specific joint positions
- **Safety validation**: Ensures all commands are within safe operational limits
- **Command execution**: Publishes commands to robot joint controllers at high frequency
- **Real-time control**: Maintains precise control with minimal latency

The actuation node runs at a high frequency (typically 100Hz) to ensure smooth and responsive control:

```python
class ActuationNode(Node):
    def __init__(self):
        super().__init__('actuation_node')

        # Subscribers
        self.trajectory_sub = self.create_subscription(Float64MultiArray, '/planned_trajectory', self.trajectory_callback, 10)

        # Publishers
        self.joint_command_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Safety validators
        self.max_velocity = 1.0  # rad/s
        self.max_effort = 100.0  # N*m

        # Timer for actuation (100Hz for real-time control)
        self.timer = self.create_timer(0.01, self.execute_commands)

    def execute_commands(self):
        """Execute joint commands with safety validation"""
        if not self.current_trajectory:
            return

        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.name = [f'joint_{i}' for i in range(len(self.current_trajectory))]
        joint_cmd.position = self.current_trajectory
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Validate command for safety
        if self.validate_command(joint_cmd):
            self.joint_command_pub.publish(joint_cmd)
        else:
            self.get_logger().warn('Command validation failed - not executing')
```

This architecture ensures that perception, planning, and actuation operate at appropriate frequencies while maintaining clear data flow and safety boundaries between each layer.

## Multi-Node ROS 2 System Design

Your complete control pipeline will consist of multiple interconnected ROS 2 nodes:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Perception    │    │     Planning    │    │    Actuation    │
│     Node        │───▶│      Node       │───▶│      Node       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       │                       │
        ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Sensor Drivers │    │  AI Agent Core  │    │  Joint Command  │
│                 │    │                 │    │   Publisher     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Perception Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image, LaserScan
from std_msgs.msg import String
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Sensor subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publisher for processed sensor data
        self.perception_pub = self.create_publisher(String, '/processed_perception', 10)

        # Store current sensor states
        self.current_joint_state = None
        self.current_imu_data = None
        self.current_camera_data = None
        self.current_lidar_data = None

        # Timer for perception processing
        self.timer = self.create_timer(0.1, self.process_perception)  # 10Hz

    def joint_callback(self, msg):
        self.current_joint_state = msg

    def imu_callback(self, msg):
        self.current_imu_data = msg

    def camera_callback(self, msg):
        self.current_camera_data = msg

    def lidar_callback(self, msg):
        self.current_lidar_data = msg

    def process_perception(self):
        """Process all sensor data and publish processed information"""
        if not all([self.current_joint_state, self.current_imu_data]):
            return

        # Process sensor data to extract relevant information
        robot_state = self.extract_robot_state()
        environment_state = self.extract_environment_state()

        # Combine into perception message
        perception_msg = String()
        perception_msg.data = f"Robot: {robot_state}, Environment: {environment_state}"
        self.perception_pub.publish(perception_msg)

    def extract_robot_state(self):
        """Extract meaningful robot state from sensor data"""
        if self.current_joint_state:
            # Example: Calculate center of mass, joint angles, velocities
            avg_joint_pos = sum(self.current_joint_state.position) / len(self.current_joint_state.position)
            return f"pos={avg_joint_pos:.2f}, joints={len(self.current_joint_state.position)}"
        return "unknown"

    def extract_environment_state(self):
        """Extract environment information from sensors"""
        if self.current_lidar_data:
            # Example: Find nearest obstacle
            min_distance = min(self.current_lidar_data.ranges)
            return f"nearest_obstacle={min_distance:.2f}m"
        return "unknown"
```

### Planning Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Subscribers
        self.perception_sub = self.create_subscription(String, '/processed_perception', self.perception_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/target_pose', self.goal_callback, 10)

        # Publisher for planned trajectories
        self.trajectory_pub = self.create_publisher(Float64MultiArray, '/planned_trajectory', 10)

        # Internal state
        self.current_perception = None
        self.current_goal = None
        self.current_robot_state = None

        # Timer for planning
        self.timer = self.create_timer(0.5, self.plan_trajectory)  # 2Hz (slower than perception)

    def perception_callback(self, msg):
        self.current_perception = msg.data

    def goal_callback(self, msg):
        self.current_goal = msg

    def plan_trajectory(self):
        """Plan a trajectory based on current perception and goal"""
        if not self.current_goal or not self.current_perception:
            return

        # Simple planning algorithm (in practice, this would be more sophisticated)
        planned_trajectory = self.generate_trajectory(
            self.current_robot_state,
            self.current_goal
        )

        if planned_trajectory:
            trajectory_msg = Float64MultiArray()
            trajectory_msg.data = planned_trajectory
            self.trajectory_pub.publish(trajectory_msg)

    def generate_trajectory(self, start_state, goal_pose):
        """Generate a joint trajectory from start to goal"""
        # This is a simplified example - real trajectory planning would be complex
        # involving inverse kinematics, collision checking, and smooth interpolation
        if start_state and goal_pose:
            # Example: Simple linear interpolation in joint space
            # In practice, use proper inverse kinematics and trajectory optimization
            return [0.1, 0.2, 0.05, -0.1, -0.2, 0.05]  # Example joint positions
        return None
```

### Actuation Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class ActuationNode(Node):
    def __init__(self):
        super().__init__('actuation_node')

        # Subscribers
        self.trajectory_sub = self.create_subscription(Float64MultiArray, '/planned_trajectory', self.trajectory_callback, 10)

        # Publishers
        self.joint_command_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Safety validators
        self.max_velocity = 1.0  # rad/s
        self.max_effort = 100.0  # N*m

        # Timer for actuation
        self.timer = self.create_timer(0.01, self.execute_commands)  # 100Hz (fast for control)

        # Internal state
        self.current_trajectory = None
        self.last_command_time = self.get_clock().now()

    def trajectory_callback(self, msg):
        """Receive planned trajectory and prepare for execution"""
        self.current_trajectory = msg.data

    def execute_commands(self):
        """Execute joint commands with safety validation"""
        if not self.current_trajectory:
            return

        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.name = [f'joint_{i}' for i in range(len(self.current_trajectory))]
        joint_cmd.position = self.current_trajectory
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Validate command for safety
        if self.validate_command(joint_cmd):
            # Publish the command
            self.joint_command_pub.publish(joint_cmd)

            # Update timing
            self.last_command_time = self.get_clock().now()
        else:
            self.get_logger().warn('Command validation failed - not executing')

    def validate_command(self, command):
        """Validate joint command for safety"""
        # Check position limits (would come from URDF)
        for pos in command.position:
            if abs(pos) > 3.14:  # Reasonable joint limit
                return False

        # Check velocity limits (if provided)
        if command.velocity:
            for vel in command.velocity:
                if abs(vel) > self.max_velocity:
                    return False

        # Check effort limits (if provided)
        if command.effort:
            for eff in command.effort:
                if abs(eff) > self.max_effort:
                    return False

        return True
```

## Integration with URDF Model

Your control pipeline seamlessly integrates with the humanoid URDF model you created in Chapter 4. This integration is crucial for ensuring that all control commands respect the physical constraints and kinematic structure defined in your robot model. The system should:

1. **Load the URDF model** for accurate kinematic and dynamic calculations
2. **Use joint names from the URDF** for precise command mapping
3. **Validate all commands against URDF limits** to ensure safe operation
4. **Enable visualization** of the robot state in RViz2 with accurate kinematics

### URDF Integration Node

The URDF Integration Node is responsible for loading the robot model and sharing its information with other nodes in the pipeline:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import os

class URDFIntegrationNode(Node):
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
            from urdf_parser_py.urdf import URDF

            # Look for the humanoid model URDF file in common locations
            urdf_paths = [
                os.path.join(os.path.dirname(__file__), '../../../urdf/humanoid_model.urdf'),
                os.path.join(os.path.dirname(__file__), '../../urdf/humanoid_model.urdf'),
                os.path.join(os.path.dirname(__file__), '../urdf/humanoid_model.urdf'),
                os.path.expanduser('~/robot_models/humanoid_model.urdf'),
                '/path/to/humanoid_model.urdf'
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

        # Set default limits for each joint based on typical humanoid capabilities
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
```

### Actuation Node with URDF-Based Safety

The Actuation Node uses URDF information to enforce physical constraints:

```python
class ActuationNode(Node):
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

        # Internal state
        self.current_trajectory = None
        self.current_joint_state = None
        self.joint_names_from_urdf = []
        self.joint_limits_from_urdf = {}
        self.control_frequency = 100  # Hz

    def urdf_info_callback(self, msg: String):
        """Handle URDF information updates"""
        try:
            urdf_info = json.loads(msg.data)
            self.joint_names_from_urdf = urdf_info.get('joint_names', [])
            self.joint_limits_from_urdf = urdf_info.get('joint_limits', {})
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse URDF info')

    def execute_commands(self):
        """Execute joint commands with URDF-based safety validation"""
        if not self.current_trajectory:
            return

        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Use joint names from URDF if available
        if self.joint_names_from_urdf:
            joint_cmd.name = self.joint_names_from_urdf[:len(self.current_trajectory)]
        else:
            joint_cmd.name = [f'joint_{i}' for i in range(len(self.current_trajectory))]

        # Apply URDF-based safety validation to trajectory
        validated_positions = self.validate_trajectory_with_urdf_limits(self.current_trajectory)
        joint_cmd.position = validated_positions

        # Calculate velocities respecting URDF limits
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
                        max_vel = self.joint_limits_from_urdf[joint_name].get('velocity', 2.0)
                        vel = max(-max_vel, min(vel, max_vel))

                velocities.append(vel)

            joint_cmd.velocity = velocities

        # Publish the command
        self.joint_command_pub.publish(joint_cmd)

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
                    # Apply conservative limits if not in URDF
                    limited_pos = max(-math.pi, min(pos, math.pi))
                    validated_trajectory.append(limited_pos)
            else:
                # Apply conservative limits for extra positions
                limited_pos = max(-math.pi, min(pos, math.pi))
                validated_trajectory.append(limited_pos)

        return validated_trajectory
```

This URDF integration ensures that all control commands respect the physical constraints of your specific humanoid robot model, preventing damage and ensuring safe operation within the designed parameters.

## Python Agent → ROS Bridge Implementation

The AI agent interfaces with your ROS 2 system through a sophisticated bridge that handles perception, planning, and action execution. This bridge translates high-level AI decisions into low-level ROS 2 commands while ensuring safety and proper integration with your control pipeline.

### Bridge Architecture

The bridge consists of three main components:

1. **Perception Bridge**: Processes sensor data into AI-friendly formats
2. **Planning Bridge**: Translates AI decisions into ROS trajectories and goals
3. **Action Bridge**: Handles complex ROS actions for humanoid tasks

### Perception Bridge

The Perception Bridge processes raw sensor data and makes it available in a format suitable for AI agents:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Imu
import json
import math
import queue

class PerceptionBridge(Node):
    def __init__(self):
        super().__init__('perception_bridge')

        # Sensor subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu_data', self.imu_callback, 10
        )

        # Publisher for AI-ready data
        self.ai_data_pub = self.create_publisher(String, '/ai_input_data', 10)

        # Internal state
        self.current_joint_state = None
        self.current_imu_data = None
        self.data_queue = queue.Queue()

        # Timer for processing and publishing data
        self.processing_timer = self.create_timer(0.1, self.process_and_publish_data)

    def joint_state_callback(self, msg: JointState):
        """Handle joint state updates"""
        self.current_joint_state = msg

    def imu_callback(self, msg: Imu):
        """Handle IMU updates"""
        self.current_imu_data = msg

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

        # Publish to AI agent
        data_msg = String()
        data_msg.data = json.dumps(ai_data)
        self.ai_data_pub.publish(data_msg)

        # Add to queue for direct access
        try:
            self.data_queue.put_nowait(ai_data)
        except queue.Full:
            pass  # Skip if queue is full

    def extract_robot_state(self):
        """Extract robot state from joint data"""
        if not self.current_joint_state:
            return {}

        positions = list(self.current_joint_state.position)
        return {
            'joint_count': len(positions),
            'avg_position': sum(positions) / len(positions) if positions else 0.0,
            'max_position': max(positions) if positions else 0.0,
            'min_position': min(positions) if positions else 0.0,
        }

    def calculate_balance_info(self):
        """Calculate balance information from IMU data"""
        if not self.current_imu_data:
            return {'balanced': False, 'roll': 0.0, 'pitch': 0.0}

        # Extract roll and pitch from quaternion
        orientation = self.current_imu_data.orientation
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        # Convert quaternion to Euler angles
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
```

### Planning Bridge

The Planning Bridge translates high-level AI commands into specific ROS trajectories:

```python
class PlanningBridge(Node):
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

    def handle_balance_command(self, command: Dict):
        """Handle balance command"""
        # Generate trajectory to maintain balance
        trajectory = self.generate_balance_trajectory()
        if trajectory:
            trajectory_msg = Float64MultiArray()
            trajectory_msg.data = trajectory
            self.trajectory_pub.publish(trajectory_msg)

    def generate_balance_trajectory(self):
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
```

### Action Bridge

The Action Bridge handles complex humanoid tasks using ROS actions:

```python
from rclpy.action import ActionClient

class ActionBridge(Node):
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
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse AI action command')
```

### Main AI Agent Bridge

The main bridge node integrates all components:

```python
class AIAgentBridge(Node):
    def __init__(self):
        super().__init__('ai_agent_bridge')

        # Create bridge components
        self.perception_bridge = PerceptionBridge()
        self.planning_bridge = PlanningBridge()
        self.action_bridge = ActionBridge()

        # Publishers for AI commands
        self.ai_command_pub = self.create_publisher(String, '/ai_commands', 10)

        # Timer for AI decision making
        self.ai_timer = self.create_timer(0.5, self.ai_decision_cycle)

    def ai_decision_cycle(self):
        """Main AI decision cycle"""
        # Get latest perception data
        perception_data = self.get_latest_perception_data()

        if perception_data:
            # Make AI decisions based on perception
            ai_decision = self.make_ai_decision(perception_data)
            if ai_decision:
                self.send_ai_command(ai_decision)

    def make_ai_decision(self, perception_data: Dict):
        """Make AI decision based on perception data"""
        balance_info = perception_data.get('balance_info', {})

        # Check if robot needs to balance
        if not balance_info.get('balanced', True):
            return {
                'type': 'balance',
                'priority': 'high',
                'reason': 'Robot is unbalanced'
            }

        # Example: Move to a new position periodically
        tilt_angle = balance_info.get('tilt_angle', 0.0)
        if tilt_angle > 0.3:  # If tilt is significant
            return {
                'type': 'balance',
                'priority': 'high',
                'reason': f'Significant tilt detected: {tilt_angle:.2f} rad'
            }

        return None  # No action needed

    def send_ai_command(self, command: Dict):
        """Send AI command through appropriate channel"""
        command['timestamp'] = self.get_clock().now().nanoseconds

        command_msg = String()
        command_msg.data = json.dumps(command)

        self.ai_command_pub.publish(command_msg)
```

This bridge architecture provides a clean separation between AI decision-making and ROS 2 control, allowing sophisticated AI agents to interact with your humanoid robot while maintaining safety and proper integration with the control pipeline.

## Complete Control Pipeline Example

Here's how all components work together in a complete system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose
import numpy as np
import math
import json

class CompleteControlPipeline(Node):
    """
    Complete implementation of perception → planning → actuation pipeline
    """
    def __init__(self):
        super().__init__('complete_control_pipeline')

        # All subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.ai_command_sub = self.create_subscription(String, '/ai_commands', self.ai_command_callback, 10)

        # All publishers
        self.command_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.status_pub = self.create_publisher(String, '/system_status', 10)
        self.trajectory_pub = self.create_publisher(Float64MultiArray, '/planned_trajectory', 10)

        # Internal state
        self.current_joint_state = None
        self.current_imu_data = None
        self.current_ai_command = None
        self.target_pose = None
        self.is_balanced = True

        # Control parameters
        self.control_frequency = 100  # Hz
        self.balance_threshold = 0.3  # rad
        self.max_joint_velocity = 1.0  # rad/s

        # Create control timer
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)

        self.get_logger().info('Complete control pipeline initialized')

    def joint_callback(self, msg):
        """Update joint state"""
        self.current_joint_state = msg

    def imu_callback(self, msg):
        """Update IMU data and check balance"""
        self.current_imu_data = msg
        self.is_balanced = self.check_balance(msg)

    def ai_command_callback(self, msg):
        """Handle AI commands"""
        try:
            self.current_ai_command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse AI command')

    def check_balance(self, imu_msg):
        """Check if robot is within balance thresholds"""
        # Extract roll and pitch from quaternion
        orientation = imu_msg.orientation
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        # Convert to Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        return abs(roll) <= self.balance_threshold and abs(pitch) <= self.balance_threshold

    def control_loop(self):
        """Main control loop: perception → planning → actuation"""
        if not all([self.current_joint_state, self.current_imu_data]):
            return

        # PERCEPTION: Process sensor data
        perception_result = self.process_perception()

        # PLANNING: Generate control commands based on perception and AI commands
        control_commands = self.plan_control(perception_result)

        # ACTUATION: Execute commands with safety checks
        self.execute_control(control_commands)

        # Publish system status
        status_msg = String()
        status_msg.data = f"balanced: {self.is_balanced}, joints: {len(self.current_joint_state.position) if self.current_joint_state else 0}"
        self.status_pub.publish(status_msg)

    def process_perception(self):
        """Process sensor data to extract relevant information"""
        if not self.current_joint_state or not self.current_imu_data:
            return None

        # Extract meaningful information from sensors
        joint_info = {
            'positions': self.current_joint_state.position,
            'velocities': self.current_joint_state.velocity,
            'efforts': self.current_joint_state.effort
        }

        imu_info = {
            'orientation': self.current_imu_data.orientation,
            'angular_velocity': self.current_imu_data.angular_velocity,
            'linear_acceleration': self.current_imu_data.linear_acceleration
        }

        return {
            'joint_info': joint_info,
            'imu_info': imu_info,
            'is_balanced': self.is_balanced
        }

    def plan_control(self, perception_result):
        """Plan control commands based on perception and AI commands"""
        if not perception_result:
            return None

        # Prioritize balance over other commands
        if not perception_result['is_balanced']:
            # Generate balance correction commands
            return self.generate_balance_correction(perception_result)
        elif self.current_ai_command:
            # Process AI command
            return self.process_ai_command_for_planning(self.current_ai_command, perception_result)
        else:
            # Generate default behavior
            return self.generate_default_behavior(perception_result)

    def generate_balance_correction(self, perception_result):
        """Generate commands to restore balance"""
        if not self.current_joint_state:
            return None

        # Calculate balance correction based on IMU data
        imu_orientation = perception_result['imu_info']['orientation']

        # Simple proportional control for balance
        w, x, y, z = imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Create command with balance corrections
        current_positions = list(self.current_joint_state.position)

        # Apply corrections to balance joints (assuming specific joint indices)
        if len(current_positions) > 3:
            roll_correction = -roll * 0.5  # Adjust hip based on roll
            pitch_correction = -pitch * 0.3  # Adjust ankle based on pitch

            current_positions[0] += roll_correction  # Left hip
            current_positions[1] -= roll_correction  # Right hip
            current_positions[2] += pitch_correction  # Left ankle
            current_positions[3] -= pitch_correction  # Right ankle

        return current_positions

    def process_ai_command_for_planning(self, ai_command, perception_result):
        """Process AI command and generate appropriate trajectory"""
        cmd_type = ai_command.get('type', 'default')

        if cmd_type == 'balance':
            return self.generate_balance_correction(perception_result)
        elif cmd_type == 'move_to_pose':
            # This would involve more sophisticated trajectory planning
            return self.generate_movement_commands(perception_result)
        else:
            return self.generate_default_behavior(perception_result)

    def generate_movement_commands(self, perception_result):
        """Generate normal movement commands"""
        if self.current_joint_state:
            # Return current positions as default (no movement)
            return list(self.current_joint_state.position)
        return None

    def generate_default_behavior(self, perception_result):
        """Generate default behavior when no specific commands"""
        if self.current_joint_state:
            # Return current positions to maintain pose
            return list(self.current_joint_state.position)
        return [0.0] * 28  # Default neutral position

    def execute_control(self, control_commands):
        """Execute control commands with safety validation"""
        if not control_commands or not self.current_joint_state:
            return

        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()

        # Use joint names from current state
        if self.current_joint_state.name:
            joint_cmd.name = self.current_joint_state.name
        else:
            joint_cmd.name = [f'joint_{i}' for i in range(len(control_commands))]

        # Apply safety limits to commands
        limited_commands = self.apply_safety_limits(control_commands)
        joint_cmd.position = limited_commands

        # Calculate velocities for smooth motion
        if self.current_joint_state.position:
            velocities = []
            dt = 1.0 / self.control_frequency
            for curr_pos, target_pos in zip(self.current_joint_state.position[:len(limited_commands)], limited_commands):
                vel = (target_pos - curr_pos) / dt
                # Limit velocity
                vel = max(-self.max_joint_velocity, min(vel, self.max_joint_velocity))
                velocities.append(vel)
            joint_cmd.velocity = velocities

        # Publish the command
        self.command_pub.publish(joint_cmd)

    def apply_safety_limits(self, commands):
        """Apply safety limits to joint commands"""
        # Apply reasonable position limits (±π)
        limited_commands = []
        for cmd in commands:
            limited_cmd = max(-math.pi, min(cmd, math.pi))
            limited_commands.append(limited_cmd)
        return limited_commands

def main(args=None):
    rclpy.init(args=args)

    pipeline = CompleteControlPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Control pipeline shutting down')
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Mini-Project Requirements and Evaluation

The complete mini-project requires you to implement a functional humanoid robot control system with the following requirements:

### Core Requirements
1. **Perception System**: Process sensor data from joint states, IMU, and other sensors
2. **Planning System**: Generate appropriate trajectories based on goals and current state
3. **Actuation System**: Execute commands with safety validation
4. **URDF Integration**: Use your humanoid URDF model for kinematic constraints
5. **AI Agent Bridge**: Allow high-level commands from AI agents
6. **Safety Mechanisms**: Implement rate limiting, velocity limits, and emergency stops

### Evaluation Criteria
- **Functionality**: All components work together in a coordinated manner
- **Safety**: Proper validation and limits prevent damage to the robot
- **Modularity**: Components are well-separated and can be tested independently
- **URDF Compliance**: All commands respect the physical constraints of your URDF model
- **Performance**: Control loop runs at appropriate frequency (100Hz for actuation)
- **Extensibility**: Architecture allows for additional sensors and capabilities

### Testing the Complete System
To test your complete control pipeline:

1. **Start the ROS 2 system**:
   ```bash
   ros2 launch your_robot_control control_pipeline.launch.py
   ```

2. **Monitor the system**:
   ```bash
   # Check topics
   ros2 topic list

   # Monitor joint states
   ros2 topic echo /joint_states

   # Monitor commands
   ros2 topic echo /joint_commands
   ```

3. **Visualize in RViz2**:
   ```bash
   rviz2
   # Add RobotModel display and set Robot Description to your URDF parameter
   ```

4. **Send test commands**:
   ```bash
   # Send an AI command
   ros2 topic pub /ai_commands std_msgs/String "data: '{\"type\": \"balance\", \"priority\": \"high\"}'"
   ```

This complete implementation demonstrates the integration of all concepts learned in Module 1, creating a functional humanoid robot control system that can perceive its environment, process AI commands, plan movements, and execute them safely.