---
sidebar_position: 4
title: "Chapter 4: URDF for Humanoid Robots"
---

# Chapter 4: URDF for Humanoid Robots

## URDF Structure: Links, Joints, Transmissions, Inertia

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF defines the physical structure including links, joints, and other components.

### Links

Links represent rigid bodies in the robot. Each link has:
- A unique name
- Visual properties (for display)
- Collision properties (for physics simulation)
- Inertial properties (for dynamics)

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

### Joints

Joints connect links and define their relative motion. Common joint types:
- **fixed**: No motion between links
- **revolute**: Rotational motion around an axis
- **continuous**: Unlimited rotational motion
- **prismatic**: Linear motion along an axis
- **floating**: 6-DOF motion (for base of floating robots)

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Transmissions

Transmissions define how actuators connect to joints, mapping from control commands to physical actuation:

```xml
<transmission name="tran1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint_name">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Inertial Properties

Inertial properties define how a link responds to forces:
- **Mass**: Resistance to linear acceleration
- **Inertia tensor**: Resistance to rotational acceleration
- Proper inertial properties are crucial for accurate simulation and control

## Links and Joints in Humanoid Robots

Humanoid robots have a complex kinematic structure that mimics the human body. The arrangement of links and joints determines the robot's range of motion and capabilities.

### Torso Structure
The torso typically consists of a base link (pelvis) with links for the spine and head. Joints allow for upper body movement while maintaining structural integrity.

### Limb Structure
Each limb (arm or leg) follows a chain-like structure:
- Proximal link (e.g., shoulder/hip) connects to the torso
- Intermediate links (e.g., upper arm/thigh, forearm/shin) provide reach
- Distal link (e.g., hand/foot) interacts with the environment

### Joint Limitations
Each joint has physical constraints that must be represented in URDF:
- Range of motion limits (lower/upper values)
- Maximum effort (torque) the joint can exert
- Maximum velocity for the joint motion

### Example: Humanoid Arm Kinematics
A typical humanoid arm has 7 degrees of freedom (DOF):
- 3 DOF in the shoulder (yaw, pitch, roll)
- 1 DOF in the elbow (pitch)
- 3 DOF in the wrist (yaw, pitch, roll)

This allows for positioning the hand in any orientation within the arm's reach.

## Modeling a Simple Humanoid Torso, Arms, and Legs

A basic humanoid model typically includes:

### Torso
- Base link (usually pelvis)
- Spine/upper body links
- Possible head link

### Arms
- Shoulder links (often with multiple segments)
- Upper arm
- Forearm
- Hand/wrist

### Legs
- Hip links
- Thigh
- Shin
- Foot

The kinematic structure follows a tree-like hierarchy, with the pelvis as the root for most configurations.

## Adding Sensors: Camera, IMU, LiDAR

Sensors are typically added as additional links or attached to existing links using fixed joints:

### Camera
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.05"/>
    </geometry>
  </visual>
  <sensor name="camera" type="camera">
    <camera name="my_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
    </camera>
  </sensor>
</link>
```

### IMU
```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
      </angular_velocity>
    </imu>
  </sensor>
</gazebo>
```

## Verification with RViz2 and check_urdf

### Using check_urdf
The `check_urdf` tool validates URDF syntax and structure:
```bash
check_urdf /path/to/robot.urdf
```

This command will:
- Validate XML syntax
- Check for proper link/joint connections
- Verify kinematic tree structure
- Report any errors or warnings

### Visualization in RViz2
To visualize your URDF in RViz2:
1. Launch RViz2
2. Add a RobotModel display
3. Set the Robot Description parameter to your URDF's parameter name
4. Ensure TF frames are being published if the robot has joints

## URDF Validation Techniques

Validating your URDF model is crucial to ensure it works correctly in simulation and real-world applications.

### Using check_urdf Tool

The `check_urdf` tool is the primary validation utility for URDF files:

```bash
# Install the tool if not already available
sudo apt-get install ros-humble-urdfdom-tools

# Validate your URDF file
check_urdf /path/to/your/robot.urdf
```

This tool will:
- Verify XML syntax
- Check for proper link/joint connections
- Validate kinematic tree structure
- Report any missing parent/child relationships
- Show warnings for potentially problematic configurations

### Common Validation Issues

- **Missing parent links**: Every joint must have a valid parent link
- **Disconnected links**: All links should be connected through joints
- **Inconsistent units**: Ensure consistent use of meters, kilograms, etc.
- **Invalid joint limits**: Check that joint limits are physically meaningful
- **Zero mass/inertia**: All links need positive mass and realistic inertia values

### Programmatic Validation

You can also validate URDF models programmatically:

```python
from urdf_parser_py.urdf import URDF
from urdf_validator import URDFValidator

# Load and validate URDF
try:
    robot = URDF.from_xml_file('/path/to/robot.urdf')
    print("URDF loaded successfully")

    # Validate structure
    validator = URDFValidator()
    is_valid, errors = validator.validate_urdf_model(robot.to_xml_string())

    if is_valid:
        print("URDF model is valid")
    else:
        print("URDF model has errors:")
        for error in errors:
            print(f"  - {error}")

except Exception as e:
    print(f"Error loading URDF: {e}")
```

## Exporting URDF for Later Use with Gazebo and Isaac (Module 2 & 3)

### Gazebo Integration
For Gazebo simulation, add Gazebo-specific tags to your URDF:
- Physics properties: `<gazebo><physics></physics></gazebo>`
- Material definitions: `<gazebo reference="link_name">...`
- Sensor configurations: `<sensor type="camera|imu|ray">`
- Plugin specifications: `<plugin filename="libgazebo_ros_*.so">`

Example Gazebo integration:
```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
</gazebo>

<gazebo>
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
  </plugin>
</gazebo>
```

### Isaac Integration
For NVIDIA Isaac compatibility:
- Follow Isaac URDF conventions: Use consistent naming schemes
- Use appropriate joint types and limits: Ensure joint limits are realistic
- Include necessary inertial properties: Isaac requires accurate mass and inertia
- Consider Isaac-specific extensions: Add Isaac-specific Gazebo plugins

Isaac-specific considerations:
- Use meters for all length units
- Ensure all joints have appropriate safety limits
- Include transmission elements for each joint
- Add proper collision geometry for physics simulation

### Export Best Practices
- Keep URDF files modular: Split complex robots into multiple files
- Use xacro for parameterization: Make URDF files configurable
- Validate before export: Always run `check_urdf` before using in simulation
- Document custom elements: Comment any custom Gazebo plugins or extensions

URDF serves as the foundational representation that connects perception, planning, and control across different simulation and real-world environments.