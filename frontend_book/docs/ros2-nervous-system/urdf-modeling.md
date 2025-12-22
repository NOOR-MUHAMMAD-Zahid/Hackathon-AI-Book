---
sidebar_position: 4
title: 'Modeling Humanoids with URDF'
---

# Modeling Humanoids with URDF

## Purpose and Structure of URDF Files

### What is URDF?
URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and other components.

### Basic URDF Structure
A URDF file typically contains:
- **Links**: Rigid bodies of the robot (e.g., base, arms, legs)
- **Joints**: Connections between links that allow relative motion
- **Visual**: How the robot appears in simulation and visualization
- **Collision**: How the robot interacts with the environment in physics simulation
- **Inertial**: Mass properties for physics simulation

### Example Basic URDF
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Defining Links, Joints, and Kinematic Chains

### Links
Links represent rigid bodies in the robot. Each link can have:
- Visual properties (shape, color, material)
- Collision properties (shape for physics simulation)
- Inertial properties (mass, center of mass, moments of inertia)

### Joint Types
- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limits
- **Fixed**: No movement between links
- **Floating**: 6-DOF movement (rarely used)
- **Planar**: Movement on a plane (rarely used)

### Example Link and Joint Definition
```xml
<link name="upper_arm">
  <visual>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<joint name="elbow_joint" type="revolute">
  <parent link="forearm"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Kinematic Chains
Kinematic chains are sequences of links connected by joints. In humanoid robots:
- **Arm chain**: Base → shoulder → elbow → wrist → hand
- **Leg chain**: Base → hip → knee → ankle → foot
- **Spine chain**: Base → spine → chest → neck → head

## Representing Humanoid Anatomy in URDF

### Humanoid Robot Structure
Humanoid robots typically have:
- **Trunk**: Torso/chest as the main body
- **Head**: With sensors (cameras, IMU)
- **Arms**: Shoulders, upper arms, forearms, hands
- **Legs**: Hips, thighs, shins, feet

### Example Humanoid URDF Snippet
```xml
<robot name="simple_humanoid">
  <!-- Trunk -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
  </joint>
</robot>
```

## Preparing Robot Descriptions for Simulation and Control

### Gazebo Simulation Considerations
When using URDF with Gazebo:
- Include `<gazebo>` tags for physics properties
- Define materials and colors
- Add transmission elements for actuator simulation

### Example with Gazebo Tags
```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
  </plugin>
</gazebo>
```

### Validation Guidelines
- Check that all links have proper mass and inertia values
- Verify that joint limits are appropriate for the robot's capabilities
- Ensure the kinematic chain forms a valid tree structure (no loops)
- Test the URDF file with `check_urdf` command

### Common URDF Validation Commands
```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Visualize URDF
urdf_to_graphiz /path/to/robot.urdf

# Parse and validate URDF
python -c "from urdf_parser_py.urdf import URDF; robot = URDF.from_xml_file('/path/to/robot.urdf')"
```

## Learning Objectives

After completing this chapter, you should be able to:

- Explain the purpose and structure of URDF files
- Define links, joints, and kinematic chains in URDF
- Represent humanoid anatomy using appropriate URDF elements
- Prepare robot descriptions for simulation and control

## Exercises

1. Create a simple URDF file for a 2-wheeled robot with a caster wheel.
2. Model a simple humanoid arm with shoulder, elbow, and wrist joints.
3. Validate your URDF file using ROS tools and visualize it in Rviz.