# Nav2 Stack Architecture Diagram

[Diagram showing the components of Nav2 navigation stack for humanoid robots]

## Description
This diagram illustrates the Navigation2 (Nav2) stack architecture specifically adapted for humanoid robots. It shows the core components including Navigation Server, Global Planner, Local Planner, Controller, Costmaps, and Recovery Behaviors, with emphasis on humanoid-specific adaptations for bipedal locomotion.

## Key Elements
- Navigation Server (central coordinator)
- Global Planner (long-term path planning)
- Local Planner (short-term trajectory planning)
- Controller (low-level motion control)
- Global Costmap (static + dynamic obstacles)
- Local Costmap (short-term obstacles)
- Recovery Behaviors (failure recovery actions)
- Lifecycle Manager (component state management)
- Humanoid-specific adaptations (step planning, balance considerations)