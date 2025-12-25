# ROS 2 Robotic Nervous System Course

This website hosts educational content for the ROS 2 Robotic Nervous System course, designed for AI and robotics students learning about ROS 2 architecture, communication patterns, and URDF modeling.

## Course Overview

This course covers:
- ROS 2 as the middleware nervous system of humanoid robots
- Communication primitives (nodes, topics, services) for robot control
- Bridging Python-based AI agents to physical robot controllers
- Structural modeling of humanoid robots using URDF

## Course Modules

- **ROS 2 Robotic Nervous System**: Introduction to ROS 2 architecture and its comparison to biological nervous systems
- **ROS 2 Communication**: Nodes, topics, and services for robot control
- **URDF Modeling**: Creating humanoid robot models using URDF files
- **Digital Twin Simulation**: Physics simulation and sensor modeling with Gazebo
- **AI-Robot Brain with NVIDIA Isaac**: Isaac Sim and Isaac ROS for perception and navigation
- **Vision-Language-Action Integration**: Voice interfaces and cognitive planning for humanoid robots

## Installation

```bash
npm i
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

The site is deployed at: https://ai-book-flame.vercel.app/

The site is configured for GitHub Pages deployment and will automatically deploy when changes are pushed to the main branch via the GitHub Actions workflow in `.github/workflows/deploy.yml`.