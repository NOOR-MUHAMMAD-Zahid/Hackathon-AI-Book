---
title: Cognitive Planning with Vision-Language Models
sidebar_position: 3
---

# Cognitive Planning with Vision-Language Models

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain LLM-based task decomposition** for high-level goal planning in humanoid robotics and its role in cognitive planning systems
2. **Implement translation of natural language goals** into ROS 2 action sequences with appropriate grounding in perception data
3. **Integrate visual perception** into planning loops for grounded decision making that connects language to physical reality
4. **Design safety and constraint-aware planning** systems that ensure robot behaviors are safe and appropriate for the environment

## Table of Contents
- [Using LLMs for High-Level Task Decomposition](#using-llms-for-high-level-task-decomposition)
- [Translating Natural Language Goals into ROS 2 Action Sequences](#translating-natural-language-goals-into-ros-2-action-sequences)
- [Incorporating Visual Perception into Planning Loops](#incorporating-visual-perception-into-planning-loops)
- [Safety, Grounding, and Constraint-Aware Planning](#safety-grounding-and-constraint-aware-planning)
- [Exercises](#exercises)

## Using LLMs for High-Level Task Decomposition

Large Language Models (LLMs) excel at understanding and decomposing complex natural language instructions into simpler, executable steps. In robotics, this capability is particularly valuable for cognitive planning, where high-level goals need to be broken down into specific robot actions that can be executed by the robot's control system.

### The Role of LLMs in Cognitive Planning

LLMs serve as cognitive bridges between human intention and robot action by:

1. **Understanding Natural Language**: Interpreting complex, ambiguous, or underspecified human commands
2. **Task Decomposition**: Breaking complex goals into simpler, executable subtasks
3. **Knowledge Integration**: Applying world knowledge to fill in missing details or resolve ambiguities
4. **Contextual Reasoning**: Making decisions based on the current situation and environment

### LLM Integration Architecture for Robotics

```
Natural Language Goal
        ↓
LLM Task Decomposer
        ↓
Structured Action Sequence
        ↓
ROS 2 Action Planner
        ↓
Robot Execution
```

> **Figure 1**: LLM integration architecture for robotic task decomposition and planning. The diagram illustrates how natural language goals are processed by an LLM to generate structured action sequences that are then executed by the ROS 2 action system.

*Architecture: Natural Language Goal → LLM Task Decomposer → Structured Action Sequence → ROS 2 Action Planner → Robot Execution*

### Example: LLM-Based Task Decomposition

```python
# Example implementation of LLM-based task decomposition for robotics
import openai
import json
from typing import List, Dict, Any

class LLMTaskDecomposer:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.client = openai.OpenAI(api_key=api_key)
        self.model = model

        # System prompt to guide LLM behavior
        self.system_prompt = """
        You are an expert in robotics and task planning. Your role is to decompose high-level human commands into specific robot actions.

        Each action should be:
        - Specific and executable by a humanoid robot
        - Include necessary parameters (locations, objects, etc.)
        - Ordered logically for successful completion
        - Safe and appropriate for the environment

        Respond only in valid JSON format with the following structure:
        {
            "decomposed_tasks": [
                {
                    "task_id": "unique identifier",
                    "action_type": "navigation|manipulation|perception|information",
                    "action_name": "specific action name",
                    "parameters": {
                        "location": "specific location",
                        "object": "specific object",
                        "description": "any additional details"
                    },
                    "preconditions": ["what must be true before"],
                    "postconditions": ["what will be true after"]
                }
            ],
            "confidence": 0.0-1.0,
            "reasoning": "brief explanation of decomposition approach"
        }
        """

    def decompose_task(self, natural_language_goal: str, robot_capabilities: List[str], environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Decompose a natural language goal into structured robot tasks
        """
        user_prompt = f"""
        Human goal: "{natural_language_goal}"

        Robot capabilities: {robot_capabilities}

        Environment context: {json.dumps(environment_context, indent=2)}

        Please decompose this goal into specific robot actions.
        """

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.1,  # Low temperature for more consistent results
                response_format={"type": "json_object"}
            )

            # Parse the response
            result = json.loads(response.choices[0].message.content)

            # Validate the structure
            if "decomposed_tasks" not in result:
                raise ValueError("Invalid response format: missing 'decomposed_tasks'")

            return result

        except Exception as e:
            # Handle errors gracefully
            return {
                "decomposed_tasks": [],
                "confidence": 0.0,
                "reasoning": f"Error decomposing task: {str(e)}"
            }

# Example usage
decomposer = LLMTaskDecomposer(api_key="your-api-key")

# Example goal decomposition
goal = "Go to the kitchen, find a red apple, and bring it to the dining table"
capabilities = ["navigation", "object_recognition", "manipulation", "grasping"]
environment = {
    "locations": ["kitchen", "dining_table", "living_room"],
    "objects": ["apple", "banana", "orange", "cup", "plate"],
    "robot_location": "starting_position"
}

result = decomposer.decompose_task(goal, capabilities, environment)
print(json.dumps(result, indent=2))
```

### Task Decomposition Patterns

Different types of goals require different decomposition strategies:

1. **Navigation Goals**: "Go to the kitchen" → Path planning → Motion execution
2. **Manipulation Goals**: "Pick up the red apple" → Object recognition → Grasping planning → Execution
3. **Composite Goals**: "Go to kitchen and bring me an apple" → Navigation + Manipulation + Transport
4. **Information Goals**: "Tell me what's on the table" → Perception → Recognition → Reporting

## Translating Natural Language Goals into ROS 2 Action Sequences

The translation of natural language goals into ROS 2 action sequences requires careful mapping between high-level human concepts and low-level robot capabilities.

### Translation Architecture

```
Natural Language Goal
        ↓ (Intent Parser)
Structured Intent
        ↓ (Action Mapper)
ROS 2 Action Sequence
        ↓ (Action Executor)
Robot Behavior
```

### ROS 2 Action Sequence Structure

For humanoid robots, common action sequences include:

- **Navigation Actions**: `nav2_msgs/action/NavigateToPose`
- **Manipulation Actions**: `control_msgs/action/FollowJointTrajectory`
- **Perception Actions**: `vision_msgs/action/DetectObjects`
- **Transport Actions**: Sequence of navigation and manipulation actions

### Example: Natural Language to ROS 2 Action Mapping

```python
# Example implementation of natural language to ROS 2 action mapping
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

class NaturalLanguageActionMapper(Node):
    def __init__(self):
        super().__init__('natural_language_action_mapper')

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.perception_client = ActionClient(self, DetectObjects, 'detect_objects')

        # Publisher for commands
        self.command_pub = self.create_publisher(String, 'robot_commands', 10)

        # Subscription for natural language goals
        self.goal_sub = self.create_subscription(
            String,
            'natural_language_goals',
            self.goal_callback,
            10
        )

        # Initialize LLM task decomposer
        self.task_decomposer = LLMTaskDecomposer(api_key=self.get_parameter('openai_api_key').value)

    def goal_callback(self, msg):
        """Process incoming natural language goals"""
        try:
            # Decompose the natural language goal
            robot_capabilities = ["navigation", "object_recognition", "manipulation"]
            environment_context = self.get_environment_context()

            decomposition_result = self.task_decomposer.decompose_task(
                msg.data,
                robot_capabilities,
                environment_context
            )

            # Execute the decomposed tasks
            self.execute_task_sequence(decomposition_result['decomposed_tasks'])

        except Exception as e:
            self.get_logger().error(f"Error processing natural language goal: {e}")

    def get_environment_context(self) -> dict:
        """Get current environment context"""
        # In practice, this would query the robot's knowledge base
        # or perception systems for current state
        return {
            "robot_location": "starting_position",
            "known_locations": ["kitchen", "dining_room", "living_room"],
            "visible_objects": [],
            "robot_battery_level": 0.8
        }

    def execute_task_sequence(self, tasks: List[Dict[str, Any]]):
        """Execute a sequence of decomposed tasks"""
        for task in tasks:
            if task['action_type'] == 'navigation':
                self.execute_navigation_task(task)
            elif task['action_type'] == 'manipulation':
                self.execute_manipulation_task(task)
            elif task['action_type'] == 'perception':
                self.execute_perception_task(task)
            elif task['action_type'] == 'information':
                self.execute_information_task(task)

    def execute_navigation_task(self, task: Dict[str, Any]):
        """Execute a navigation task"""
        goal_msg = NavigateToPose.Goal()

        # Convert location name to pose
        pose = self.location_to_pose(task['parameters']['location'])
        goal_msg.pose = pose

        # Send goal to navigation system
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        # In a real implementation, you would handle the future response
        self.get_logger().info(f"Navigating to {task['parameters']['location']}")

    def location_to_pose(self, location_name: str) -> PoseStamped:
        """Convert a location name to a pose in the robot's coordinate system"""
        # In practice, this would look up pre-defined poses in a map
        # For this example, we'll use a simple mapping
        location_poses = {
            "kitchen": PoseStamped(pose=Pose(position=Point(x=1.0, y=2.0, z=0.0))),
            "dining_room": PoseStamped(pose=Pose(position=Point(x=3.0, y=1.0, z=0.0))),
            "living_room": PoseStamped(pose=Pose(position=Point(x=0.0, y=0.0, z=0.0)))
        }

        return location_poses.get(location_name, location_poses["living_room"])  # Default to living room

    def execute_manipulation_task(self, task: Dict[str, Any]):
        """Execute a manipulation task"""
        # Implementation would depend on the specific manipulator and gripper
        self.get_logger().info(f"Manipulation task: {task['action_name']} with {task['parameters']['object']}")

    def execute_perception_task(self, task: Dict[str, Any]):
        """Execute a perception task"""
        # Implementation would depend on the specific perception capabilities
        self.get_logger().info(f"Perception task: {task['action_name']} for {task['parameters']['object']}")

    def execute_information_task(self, task: Dict[str, Any]):
        """Execute an information task"""
        # Implementation would involve reporting information to the user
        self.get_logger().info(f"Information task: {task['action_name']} about {task['parameters']['subject']}")

def main(args=None):
    rclpy.init(args=args)
    mapper = NaturalLanguageActionMapper()

    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        mapper.get_logger().info("Shutting down natural language action mapper")
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Incorporating Visual Perception into Planning Loops

For effective cognitive planning, LLMs must be grounded in real-world perception data. This ensures that plans are based on actual environmental conditions rather than abstract knowledge alone.

### Perception-Grounded Planning Architecture

```
Natural Language Goal
        ↓
LLM Task Decomposer
        ↓
Partial Action Sequence
        ↓
Perception Request
        ↓
Environmental Data
        ↓
Plan Refinement
        ↓
Final Action Sequence
        ↓
Execution
```

> **Figure 2**: Perception-grounded planning showing how environmental perception data is integrated into cognitive planning loops. The diagram illustrates how LLM-generated plans are refined and validated using real-world perception data to ensure grounded and executable robot behaviors.

*Process: Natural Language Goal → LLM Task Decomposer → Partial Action Sequence → Perception Request → Environmental Data → Plan Refinement → Final Action Sequence → Execution*

### Example: Perception-Grounded Task Decomposition

```python
# Example implementation of perception-grounded task decomposition
class PerceptionGroundedDecomposer(LLMTaskDecomposer):
    def __init__(self, api_key: str, perception_client):
        super().__init__(api_key)
        self.perception_client = perception_client

    def decompose_task_with_perception(self, natural_language_goal: str, robot_capabilities: List[str]) -> Dict[str, Any]:
        """
        Decompose task with perception grounding
        """
        # First, get current environmental context
        environment_context = self.perception_client.get_current_environment()

        # Then decompose with environmental context
        return self.decompose_task(natural_language_goal, robot_capabilities, environment_context)

    def refine_plan_with_perception(self, initial_plan: Dict[str, Any], perception_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Refine an initial plan based on perception data
        """
        # Example refinement: update locations based on actual object positions
        refined_tasks = []

        for task in initial_plan['decomposed_tasks']:
            if task['action_type'] == 'manipulation':
                # Check if the object exists and update location if needed
                target_object = task['parameters'].get('object', '')
                if target_object in perception_data.get('detected_objects', []):
                    # Update the task with actual object location
                    actual_location = perception_data['detected_objects'][target_object]['location']
                    task['parameters']['location'] = actual_location

            refined_tasks.append(task)

        initial_plan['decomposed_tasks'] = refined_tasks
        return initial_plan

# Example usage
perception_client = PerceptionClient()  # Custom perception client
grounded_decomposer = PerceptionGroundedDecomposer(api_key="your-api-key", perception_client=perception_client)

goal = "Find the red apple and bring it to the table"
result = grounded_decomposer.decompose_task_with_perception(goal, ["navigation", "manipulation"])
refined_result = grounded_decomposer.refine_plan_with_perception(result, perception_client.get_latest_perception_data())
```

### Perception Integration Patterns

Several patterns emerge when integrating perception with LLM-based planning:

1. **Pre-Planning Perception**: Gather environmental context before decomposing the task
2. **In-Planning Perception**: Request specific perception data during task decomposition
3. **Post-Planning Perception**: Refine the plan based on detailed perception data
4. **Continuous Perception**: Update the plan dynamically as new perception data arrives

## Safety, Grounding, and Constraint-Aware Planning

When using LLMs for cognitive planning, it's crucial to ensure that generated plans are safe, grounded in reality, and respect environmental constraints.

### Safety Constraints

LLM-based plans must incorporate safety constraints such as:

- **Physical Safety**: Avoiding collisions, respecting joint limits
- **Operational Safety**: Following operational procedures, avoiding dangerous behaviors
- **Social Safety**: Respecting privacy, cultural norms, personal space

### Example: Safety-Constrained Task Decomposition

```python
# Example implementation of safety-constrained task decomposition
class SafetyConstrainedDecomposer(LLMTaskDecomposer):
    def __init__(self, api_key: str, safety_checker):
        super().__init__(api_key)
        self.safety_checker = safety_checker

    def decompose_task(self, natural_language_goal: str, robot_capabilities: List[str], environment_context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Decompose task with safety checking
        """
        # First, get the basic decomposition
        basic_result = super().decompose_task(natural_language_goal, robot_capabilities, environment_context)

        # Then check safety constraints
        safe_tasks = []
        for task in basic_result['decomposed_tasks']:
            if self.safety_checker.is_safe(task, environment_context):
                safe_tasks.append(task)
            else:
                # Replace unsafe task with safer alternative
                safe_alternative = self.safety_checker.get_safe_alternative(task, environment_context)
                if safe_alternative:
                    safe_tasks.append(safe_alternative)

        basic_result['decomposed_tasks'] = safe_tasks
        return basic_result

class SafetyChecker:
    def __init__(self):
        # Define safety rules
        self.safety_rules = {
            "avoid_collisions": True,
            "respect_personal_space": True,
            "follow_traffic_patterns": True,
            "avoid_dangerous_areas": True
        }

    def is_safe(self, task: Dict[str, Any], environment: Dict[str, Any]) -> bool:
        """Check if a task is safe to execute"""
        action_type = task.get('action_type', '')
        params = task.get('parameters', {})

        # Check for collision risk during navigation
        if action_type == 'navigation':
            target_location = params.get('location', '')
            if self.would_cause_collision(target_location, environment):
                return False

        # Check for personal space violation
        if action_type == 'navigation' and self.would_violate_personal_space(params, environment):
            return False

        # Additional safety checks...

        return True

    def get_safe_alternative(self, task: Dict[str, Any], environment: Dict[str, Any]) -> Dict[str, Any] or None:
        """Provide a safe alternative to an unsafe task"""
        # Implementation would provide safe alternatives
        # For example, if navigation to a location is unsafe due to obstacles,
        # return a task to navigate to a nearby safe location instead
        pass

    def would_cause_collision(self, location: str, environment: Dict[str, Any]) -> bool:
        """Check if navigation to location would cause collision"""
        # Implementation would check environment for obstacles
        return False

    def would_violate_personal_space(self, params: Dict[str, Any], environment: Dict[str, Any]) -> bool:
        """Check if action would violate personal space"""
        # Implementation would check for people in proximity
        return False
```

### Grounding Techniques

To ensure plans are grounded in reality:

1. **Reality Checking**: Verify that objects and locations exist before planning
2. **Constraint Integration**: Incorporate environmental constraints into the planning process
3. **Feedback Loops**: Continuously update plans based on perception data
4. **Uncertainty Handling**: Account for uncertainty in perception and planning

## Exercises

### Exercise 1: LLM Task Decomposition Implementation
1. Set up an OpenAI API client for task decomposition
2. Implement a basic LLM task decomposer that can handle simple navigation goals like "Go to the kitchen"
3. Test your implementation with various natural language goals and evaluate decomposition accuracy
4. Add safety checking to your task decomposer to ensure generated plans are safe

### Exercise 2: Natural Language to ROS 2 Action Mapping
1. Create a ROS 2 node that subscribes to natural language goals and publishes structured action sequences
2. Implement action mapping for navigation, manipulation, and perception tasks
3. Test the complete pipeline from natural language input to action execution
4. Validate that actions are properly sequenced and executed

### Exercise 3: Perception-Grounded Planning
1. Integrate perception data into your task decomposition process
2. Implement plan refinement based on actual environmental conditions
3. Test how your system handles cases where initial assumptions don't match reality
4. Evaluate the improvement in task success rate when using perception-grounded planning

## Summary

Cognitive planning with vision-language models enables humanoid robots to understand and execute complex natural language goals by leveraging LLMs for task decomposition and incorporating visual perception for grounded decision making. The key components include LLM-based task decomposition, natural language to ROS 2 action mapping, perception integration for grounded planning, and safety/constraint-aware planning. When properly implemented, these systems can translate high-level human intentions into executable robot behaviors that are both effective and safe.

## Next Steps

In the next chapter, we'll explore the complete autonomous humanoid system that integrates all the components we've covered - voice command reception, cognitive planning, and coordinated perception-planning-execution in simulation.