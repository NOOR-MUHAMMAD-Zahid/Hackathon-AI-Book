---
title: Physics Simulation with Gazebo
sidebar_position: 3
---

# Physics Simulation with Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Configure physics parameters** in Gazebo including gravity, friction, and collisions
2. **Create environment files** and world configurations for robotics simulation
3. **Run humanoid robots** in Gazebo and validate their behavior under physical constraints
4. **Implement validation techniques** to ensure realistic robot behavior in simulation

## Table of Contents
- [Simulating Physical Forces](#simulating-physical-forces)
- [Environment Creation and World Files](#environment-creation-and-world-files)
- [Running Humanoid Robots in Gazebo](#running-humanoid-robots-in-gazebo)
- [Validating Robot Behavior](#validating-robot-behavior)
- [Exercises](#exercises)

## Simulating Physical Forces

Gazebo provides a realistic physics simulation environment that models various physical forces and interactions. Understanding how to configure these parameters is crucial for creating accurate simulations.

### Gravity Configuration

Gravity is a fundamental force in Gazebo simulations. By default, Gazebo simulates Earth's gravity (9.8 m/s²), but this can be adjusted for different scenarios:

```xml
<!-- In a world file -->
<sdf version='1.6'>
  <world name='default'>
    <gravity>0 0 -9.8</gravity>
    <!-- Other world elements -->
  </world>
</sdf>
```

### Friction Modeling

Friction affects how objects interact with surfaces. Gazebo supports both static and dynamic friction parameters:

- **Static friction**: The force required to initiate motion
- **Dynamic friction**: The force that opposes motion once it has started

Friction parameters can be set in URDF files for individual links:

```xml
<link name="wheel_link">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>  <!-- Static friction coefficient -->
          <mu2>1.0</mu2>  <!-- Secondary friction coefficient -->
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

### Collision Detection

Gazebo uses collision detection algorithms to determine when objects interact. The accuracy of collision detection depends on:

- **Collision geometry**: The shape used for collision detection (box, sphere, cylinder, mesh)
- **Surface properties**: How objects respond to contact
- **Update rate**: How frequently collision detection is performed

*Figure 1: Gazebo physics simulation showing gravity, friction, and collision modeling. The diagram illustrates how physical forces are simulated in the Gazebo environment.*

> **Note**: Gazebo physics simulation diagram showing gravity, friction, and collision modeling. The diagram illustrates how physical forces are simulated in the Gazebo environment.

## Environment Creation and World Files

World files in Gazebo define the environment where robots operate. These files specify:

- Physical properties (gravity, magnetic field, etc.)
- Models and their initial positions
- Lighting conditions
- Plugins and sensors

### Basic World File Structure

```xml
<sdf version='1.6'>
  <world name='my_world'>
    <!-- Physics engine configuration -->
    <physics name='default_physics' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Models in the environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom models can be placed here -->
    <model name='my_robot'>
      <!-- Model definition -->
    </model>
  </world>
</sdf>
```

### Creating Custom Environments

Custom environments can be created to simulate specific scenarios:

1. **Indoor environments**: Rooms, corridors, furniture
2. **Outdoor environments**: Terrain, obstacles, weather conditions
3. **Specialized environments**: Factory floors, hospital rooms, home settings

## Running Humanoid Robots in Gazebo

Humanoid robots present unique challenges in simulation due to their complex kinematics and balance requirements.

### Model Preparation

Before running a humanoid robot in Gazebo, ensure:

1. **URDF is complete**: All links, joints, and transmissions are properly defined
2. **Inertial properties**: Mass and inertia values are realistic
3. **Joint limits**: Properly configured for safe operation
4. **Actuator models**: Appropriate for the intended control approach

### Control Strategies

Humanoid robots typically require sophisticated control strategies:

- **Balance control**: Maintaining center of mass over support polygon
- **Walking gaits**: Implementing stable locomotion patterns
- **Manipulation**: Coordinated arm and hand movements

### Example: Launching a Humanoid Robot

```xml
<!-- launch/humanoid_gazebo.launch.py -->
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'worlds',
                'humanoid_world.world'
            ])
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open('urdf/humanoid.urdf').read()
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0',
            '-y', '0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
```

## Validating Robot Behavior

Validation ensures that robot behavior in simulation matches expected real-world performance.

### Key Validation Metrics

1. **Kinematic accuracy**: Do joint movements match expected ranges?
2. **Dynamic behavior**: Does the robot respond appropriately to forces?
3. **Stability**: Can the robot maintain balance under various conditions?
4. **Sensor accuracy**: Do simulated sensors provide realistic data?

### Validation Techniques

- **Comparative analysis**: Compare simulation results with physical tests
- **Parameter sweeps**: Test behavior across different parameter values
- **Edge case testing**: Validate behavior under extreme conditions
- **Statistical validation**: Use multiple trials to ensure consistent behavior

*Figure 2: Gazebo validation process showing simulation results compared to expected behavior. The diagram illustrates how robot behavior is validated against physical constraints and expected outcomes.*

> **Note**: Gazebo validation process diagram showing simulation results compared to expected behavior. The diagram illustrates how robot behavior is validated against physical constraints and expected outcomes.

## Exercises

### Exercise 1: Gravity and Friction Configuration
1. Create a simple world file with custom gravity settings (e.g., moon gravity: 1.6 m/s²)
2. Implement a robot that behaves differently under various friction coefficients
3. Document how changing these parameters affects robot mobility

### Exercise 2: Environment Design
1. Design a custom world file for a humanoid robot to navigate
2. Include obstacles and terrain variations
3. Test the robot's ability to operate in this environment

### Exercise 3: Validation Challenge
1. Create a simple humanoid model with basic walking capability
2. Implement validation tests to ensure the robot maintains balance
3. Document any discrepancies between expected and actual behavior

## Summary

Physics simulation with Gazebo enables realistic testing of humanoid robots before physical deployment. By properly configuring gravity, friction, and collision parameters, developers can create accurate simulations that validate robot behavior under physical constraints. World files provide the flexibility to create custom environments for specific testing scenarios.

## Next Steps

In the next chapter, we'll explore high-fidelity visualization and sensor simulation using Unity, where you'll learn to create realistic perception and control pipelines.