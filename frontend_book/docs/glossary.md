# Glossary of ROS 2 Concepts

## A

### Action
A communication pattern in ROS 2 that allows for long-running tasks with feedback, goals, and results. Actions are built on top of services and provide more sophisticated communication for tasks that may take a significant amount of time to complete.

## D

### DDS (Data Distribution Service)
The underlying communication middleware used by ROS 2. DDS provides a standardized interface for real-time, high-performance, scalable data distribution between publishers and subscribers.

## J

### Joint
In URDF, a connection between two links that allows relative motion. Joint types include revolute, continuous, prismatic, fixed, floating, and planar.

## L

### Link
In URDF, a rigid body that represents a part of the robot. Links have visual, collision, and inertial properties that define how they appear and behave in simulation.

## N

### Node
A process that performs computation in ROS 2. Nodes are the fundamental building blocks of ROS 2 applications and communicate with other nodes through topics, services, and actions.

## Q

### QoS (Quality of Service)
Settings that define how messages are delivered in ROS 2, including reliability, durability, deadline, lifespan, and liveliness policies.

## S

### Service
A synchronous request-response communication pattern in ROS 2 where a client sends a request to a server and waits for a response.

## T

### Topic
An asynchronous publish-subscribe communication pattern in ROS 2 where publishers send messages to topics and subscribers receive messages from topics.