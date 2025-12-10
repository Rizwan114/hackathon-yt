---
sidebar_position: 3
title: "Chapter 3: Controlling Humanoids with rclpy"
---

# Chapter 3: Controlling Humanoids with rclpy

## Creating Python-based Robot Controllers

Python is a powerful language for creating robot controllers due to its simplicity, extensive libraries, and strong support in the ROS 2 ecosystem. The rclpy library provides Python bindings for ROS 2, enabling you to create nodes, publish/subscribe to topics, and use services and actions.

### Why Python for Robotics?

Python offers several advantages for robot control:
- **Simplicity**: Easy to learn and use, especially for AI and machine learning integration
- **Rich Ecosystem**: Extensive libraries for AI, computer vision, and scientific computing
- **Rapid Prototyping**: Quick development and testing of robot behaviors
- **Integration**: Seamless integration with ROS 2 through rclpy

### Basic Node Structure

Every rclpy node follows a similar structure:
1. Initialize the ROS 2 client library
2. Create a node class inheriting from `rclpy.node.Node`
3. Set up publishers, subscribers, services, or actions
4. Spin the node to process callbacks
5. Clean up resources on shutdown

## Integrating LLM/AI agents with ROS 2

One of the key advantages of the ROS 2 architecture is its ability to integrate high-level AI agents with low-level robot control systems. This integration enables sophisticated behaviors where AI agents can:
- Send commands to robot controllers
- Receive sensor feedback
- Plan complex sequences of actions
- Adapt to changing environments

### Communication Patterns

AI agents typically interact with ROS 2 using these patterns:
- **Topics**: For continuous sensor data streams and actuator commands
- **Services**: For goal-oriented tasks that require a response
- **Actions**: For long-running tasks with feedback and cancellation

## Action Servers for Humanoid Tasks

Actions in ROS 2 are designed for long-running tasks that require:
- **Goal requests**: The desired outcome
- **Feedback**: Continuous updates on progress
- **Result**: The final outcome when the task completes
- **Cancellation**: Ability to stop a running task

### Common Humanoid Actions
- **Walking**: Move to a specific location with balance control
- **Grasping**: Pick up an object with vision-based control
- **Balancing**: Maintain stability when disturbed

### Action Server Implementation

An action server for a humanoid task follows this pattern:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_action_interfaces.action import YourActionType

class HumanoidActionServer(Node):
    def __init__(self):
        super().__init__('humanoid_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            YourActionType,
            'action_name',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        # Process the goal
        feedback_msg = YourActionType.Feedback()
        result_msg = YourActionType.Result()

        # Perform the action with feedback
        for i in range(0, 100):
            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result_msg

            # Update feedback
            feedback_msg.progress = i
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for some time (or perform actual work)
            time.sleep(0.1)

        # Complete the goal
        goal_handle.succeed()
        result_msg.success = True
        return result_msg
```

## Example: A Python Agent Commanding Joint Targets over a ROS Action

Here's a complete example of a Python agent commanding joint targets for a humanoid robot:

### Action Definition

First, define the action in an action file (e.g., `JointTrajectory.action`):
```
# Goal: List of joint names and target positions
string[] joint_names
float64[] target_positions
duration execution_time

---
# Result: Success status
bool success
string message

---
# Feedback: Current progress
float64[] current_positions
float64[] error_positions
float64 progress_percentage
```

### Action Server (Robot Side)

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from your_interfaces.action import JointTrajectory

class JointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            JointTrajectory,
            'joint_trajectory',
            self.execute_callback,
            cancel_callback=self.cancel_callback)

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # Robot-specific parameters
        self.current_positions = [0.0] * 28  # For a 28-dof humanoid
        self.is_executing = False

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info('Received cancel request')
        self.is_executing = False
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the joint trajectory goal"""
        self.get_logger().info('Executing joint trajectory goal')
        self.is_executing = True

        feedback_msg = JointTrajectory.Feedback()
        result_msg = JointTrajectory.Result()

        # Command the joints to move to target positions
        for step in range(100):  # Simulate trajectory execution
            if not self.is_executing or goal_handle.is_cancel_requested:
                result_msg.success = False
                result_msg.message = "Trajectory canceled"
                goal_handle.canceled()
                return result_msg

            # Calculate intermediate positions (simplified interpolation)
            alpha = step / 99.0
            command_positions = []
            for i, (current, target) in enumerate(zip(self.current_positions, goal.target_positions)):
                pos = current + alpha * (target - current)
                command_positions.append(pos)

            # Publish the command
            cmd_msg = Float64MultiArray()
            cmd_msg.data = command_positions
            self.joint_pub.publish(cmd_msg)

            # Update current positions
            self.current_positions = command_positions

            # Provide feedback
            feedback_msg.current_positions = command_positions
            feedback_msg.error_positions = [0.0] * len(command_positions)  # Simplified
            feedback_msg.progress_percentage = alpha * 100.0

            goal_handle.publish_feedback(feedback_msg)

            # Sleep to control execution speed
            time.sleep(0.05)

        # Complete successfully
        result_msg.success = True
        result_msg.message = "Trajectory completed successfully"
        goal_handle.succeed()
        return result_msg
```

### Action Client (AI Agent Side)

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_interfaces.action import JointTrajectory

class JointTrajectoryClient(Node):
    def __init__(self):
        super().__init__('joint_trajectory_client')
        self._action_client = ActionClient(
            self,
            JointTrajectory,
            'joint_trajectory')

    def send_goal(self, joint_names, target_positions):
        """Send a joint trajectory goal"""
        goal_msg = JointTrajectory.Goal()
        goal_msg.joint_names = joint_names
        goal_msg.target_positions = target_positions
        goal_msg.execution_time.sec = 5  # Allow up to 5 seconds

        # Wait for the action server
        self._action_client.wait_for_server()

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback during execution"""
        self.get_logger().info(f'Progress: {feedback_msg.feedback.progress_percentage:.1f}%')

    def get_result_callback(self, future):
        """Handle the final result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')
```

## Advanced Humanoid Action Servers

For complex humanoid tasks like walking, grasping, and balancing, you'll need specialized action servers that handle the unique challenges of each task.

### Walking Action Server

Walking requires careful coordination of multiple joints and balance control. Here's an example implementation:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose
import math

class WalkingActionServer(Node):
    def __init__(self):
        super().__init__('walking_action_server')

        # Create action server for walking
        self._action_server = ActionServer(
            self,
            YourWalkingActionType,  # Replace with actual action type
            'walk_to_pose',
            self.execute_walk_callback)

        # Publishers and subscribers for walking control
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)

        # Walking parameters
        self.current_pose = Pose()
        self.balance_threshold = 0.1  # Maximum acceptable tilt

    def execute_walk_callback(self, goal_handle):
        """Execute walking action with balance feedback"""
        self.get_logger().info('Executing walking action')

        target_pose = goal_handle.target_pose
        feedback_msg = YourWalkingActionType.Feedback()
        result_msg = YourWalkingActionType.Result()

        # Generate walking trajectory
        steps = self.generate_walking_trajectory(self.current_pose, target_pose)

        for i, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                return result_msg

            # Check balance using IMU data
            if not self.is_balanced():
                goal_handle.abort()
                result_msg.success = False
                result_msg.message = "Robot lost balance during walking"
                return result_msg

            # Execute step
            self.execute_step(step)

            # Update feedback
            feedback_msg.progress = (i + 1) / len(steps) * 100.0
            feedback_msg.current_pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)

            # Small delay between steps
            time.sleep(0.1)

        result_msg.success = True
        result_msg.message = "Walking completed successfully"
        goal_handle.succeed()
        return result_msg

    def generate_walking_trajectory(self, start_pose, target_pose):
        """Generate a trajectory of steps from start to target"""
        # Simplified walking trajectory generation
        steps = []
        # Calculate intermediate poses
        for i in range(10):  # 10 steps example
            t = i / 9.0  # Normalize to 0-1
            step_pose = Pose()
            step_pose.position.x = start_pose.position.x + t * (target_pose.position.x - start_pose.position.x)
            step_pose.position.y = start_pose.position.y + t * (target_pose.position.y - start_pose.position.y)
            steps.append(step_pose)
        return steps

    def is_balanced(self):
        """Check if the robot is within balance thresholds"""
        # Check IMU data for balance
        # Return True if balanced, False otherwise
        return True  # Simplified for example

    def execute_step(self, step_pose):
        """Execute a single walking step"""
        # Calculate joint angles for this step
        # Publish joint commands
        pass
```

### Grasping Action Server

Grasping requires precise control and sensor feedback:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Point
import cv2

class GraspingActionServer(Node):
    def __init__(self):
        super().__init__('grasping_action_server')

        # Create action server for grasping
        self._action_server = ActionServer(
            self,
            YourGraspingActionType,  # Replace with actual action type
            'grasp_object',
            self.execute_grasp_callback)

        # Publishers and subscribers for grasping
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Grasping parameters
        self.object_detected = False
        self.grasp_success = False

    def execute_grasp_callback(self, goal_handle):
        """Execute grasping action with vision feedback"""
        self.get_logger().info('Executing grasping action')

        target_object = goal_handle.target_object
        feedback_msg = YourGraspingActionType.Feedback()
        result_msg = YourGraspingActionType.Result()

        # Approach phase
        self.get_logger().info('Approaching target object')
        approach_success = self.approach_object(target_object)
        if not approach_success:
            goal_handle.abort()
            result_msg.success = False
            result_msg.message = "Failed to approach object"
            return result_msg

        feedback_msg.phase = "approach"
        goal_handle.publish_feedback(feedback_msg)

        # Grasp phase
        self.get_logger().info('Executing grasp')
        grasp_success = self.execute_grasp(target_object)

        if grasp_success:
            result_msg.success = True
            result_msg.message = "Object grasped successfully"
            goal_handle.succeed()
        else:
            result_msg.success = False
            result_msg.message = "Grasp failed"
            goal_handle.abort()

        return result_msg

    def approach_object(self, target_object):
        """Move the arm to approach the target object"""
        # Calculate approach trajectory
        # Move arm to pre-grasp position
        return True  # Simplified for example

    def execute_grasp(self, target_object):
        """Execute the grasp motion"""
        # Close gripper
        # Apply appropriate force
        # Verify grasp success
        return True  # Simplified for example
```

### Balancing Action Server

Balance control is critical for humanoid robots:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class BalancingActionServer(Node):
    def __init__(self):
        super().__init__('balancing_action_server')

        # Create action server for balancing
        self._action_server = ActionServer(
            self,
            YourBalancingActionType,  # Replace with actual action type
            'maintain_balance',
            self.execute_balance_callback)

        # Publishers and subscribers for balance control
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)

        # PID controller parameters for balance
        self.roll_pid = {'kp': 1.0, 'ki': 0.1, 'kd': 0.05}
        self.pitch_pid = {'kp': 1.0, 'ki': 0.1, 'kd': 0.05}

    def execute_balance_callback(self, goal_handle):
        """Execute balancing action"""
        self.get_logger().info('Executing balancing action')

        feedback_msg = YourBalancingActionType.Feedback()
        result_msg = YourBalancingActionType.Result()

        balance_duration = goal_handle.duration  # How long to maintain balance

        for i in range(int(balance_duration * 100)):  # 100Hz control loop
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                return result_msg

            # Get current IMU data
            current_roll, current_pitch = self.get_imu_orientation()

            # Calculate balance corrections using PID
            roll_correction = self.calculate_pid_correction(
                0.0, current_roll, self.roll_pid  # Target roll is 0 (upright)
            )
            pitch_correction = self.calculate_pid_correction(
                0.0, current_pitch, self.pitch_pid  # Target pitch is 0 (upright)
            )

            # Apply corrections to joint positions
            balance_joints = self.calculate_balance_joints(roll_correction, pitch_correction)

            # Publish balance commands
            cmd_msg = JointState()
            cmd_msg.position = balance_joints
            self.joint_pub.publish(cmd_msg)

            # Update feedback
            feedback_msg.roll_error = current_roll
            feedback_msg.pitch_error = current_pitch
            feedback_msg.elapsed_time = i / 100.0
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for 10ms (100Hz control loop)
            time.sleep(0.01)

        result_msg.success = True
        result_msg.message = "Balance maintained successfully"
        goal_handle.succeed()
        return result_msg

    def get_imu_orientation(self):
        """Get current roll and pitch from IMU"""
        # Extract roll and pitch from IMU quaternion
        # Return as tuple (roll, pitch)
        return 0.0, 0.0  # Simplified for example

    def calculate_pid_correction(self, target, current, pid_params):
        """Calculate PID correction value"""
        error = target - current
        # Simplified PID calculation
        correction = pid_params['kp'] * error
        return correction

    def calculate_balance_joints(self, roll_correction, pitch_correction):
        """Calculate joint positions for balance correction"""
        # Calculate appropriate joint angles based on balance corrections
        # This would involve inverse kinematics for the humanoid
        base_joints = [0.0] * 28  # 28 DOF humanoid

        # Apply corrections to balance joints (hips, ankles, etc.)
        base_joints[0] += roll_correction * 0.5  # Left hip roll
        base_joints[1] -= roll_correction * 0.5  # Right hip roll
        base_joints[2] += pitch_correction * 0.3  # Left ankle
        base_joints[3] -= pitch_correction * 0.3  # Right ankle

        return base_joints
```

## Safety: Rate Limiting, Timeouts, and Control-loop Constraints

Safety is paramount in humanoid robotics. Python agents controlling humanoid robots must implement several safety measures:

### Rate Limiting

Prevent overwhelming the robot with commands:
```python
import time

class SafeController(Node):
    def __init__(self):
        super().__init__('safe_controller')
        self.last_command_time = 0
        self.min_command_interval = 0.01  # 10ms minimum between commands

    def send_command_if_safe(self, command):
        current_time = time.time()
        if current_time - self.last_command_time >= self.min_command_interval:
            self.publisher.publish(command)
            self.last_command_time = current_time
            return True
        return False  # Command throttled
```

### Timeouts

Ensure operations don't hang indefinitely:
```python
import threading
import time

def execute_with_timeout(func, timeout_duration):
    """Execute a function with a timeout"""
    result = [None]
    exception = [None]

    def target():
        try:
            result[0] = func()
        except Exception as e:
            exception[0] = e

    thread = threading.Thread(target=target)
    thread.daemon = True
    thread.start()
    thread.join(timeout_duration)

    if thread.is_alive():
        # Handle timeout
        raise TimeoutError(f"Function did not complete within {timeout_duration}s")

    if exception[0]:
        raise exception[0]

    return result[0]
```

### Control Loop Constraints

Implement proper control loop timing:
```python
import time

class ControlLoop:
    def __init__(self, frequency=100):  # 100 Hz
        self.period = 1.0 / frequency
        self.last_execution_time = time.time()

    def wait_for_next_cycle(self):
        """Wait until the next control cycle"""
        expected_next = self.last_execution_time + self.period
        current_time = time.time()

        if current_time < expected_next:
            sleep_time = expected_next - current_time
            time.sleep(sleep_time)

        self.last_execution_time = time.time()

    def execute(self, control_function):
        """Execute the control function with proper timing"""
        control_function()
        self.wait_for_next_cycle()
```

### Additional Safety Considerations

Beyond rate limiting and timeouts, humanoid robot controllers should implement:

**Joint Limit Enforcement**: Ensure all commanded joint positions remain within safe physical limits:
```python
def enforce_joint_limits(self, joint_positions, min_limits, max_limits):
    """Enforce joint position limits"""
    constrained_positions = []
    for pos, min_val, max_val in zip(joint_positions, min_limits, max_limits):
        constrained_positions.append(max(min_val, min(pos, max_val)))
    return constrained_positions
```

**Velocity and Acceleration Limits**: Control the rate of change to prevent jerky movements:
```python
def limit_velocity(self, current_positions, target_positions, max_velocity, dt):
    """Limit joint velocity"""
    max_change = max_velocity * dt
    limited_positions = []

    for curr, target in zip(current_positions, target_positions):
        change = target - curr
        limited_change = max(-max_change, min(change, max_change))
        limited_positions.append(curr + limited_change)

    return limited_positions
```

**Emergency Stop**: Implement an emergency stop mechanism that can halt all robot motion:
```python
class SafeController(Node):
    def __init__(self):
        super().__init__('safe_controller')
        self.emergency_stop = False
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 1)

    def emergency_stop_callback(self):
        """Trigger emergency stop"""
        self.emergency_stop = True
        # Publish emergency stop command
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Stop all joint commands
        # Implementation depends on your robot's emergency stop interface
```

These safety mechanisms are essential for preventing damage to the robot and ensuring safe interaction with the environment and humans.