---
sidebar_position: 3
title: 'ROS 2 Communication: Nodes, Topics, and Services'
---

# ROS 2 Communication: Nodes, Topics, and Services

## Creating and Running ROS 2 Nodes

### Node Structure
A ROS 2 node is a program that uses ROS 2 client libraries to communicate with other nodes. Each node should have a single responsibility and communicate with other nodes through ROS 2's communication primitives.

### Basic Node Example
Here's a simple ROS 2 node structure in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle
1. Initialize ROS 2 client library
2. Create node instance
3. Configure publishers, subscribers, services, etc.
4. Enter spin loop to process callbacks
5. Clean up resources when shutting down

## Asynchronous Communication with Topics

### Publishers and Subscribers
ROS 2 uses a publish-subscribe pattern for asynchronous communication. Publishers send messages to topics, and subscribers receive messages from topics.

### Publisher Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Quality of Service (QoS) Settings
QoS settings allow you to configure how messages are delivered:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep last N messages vs. keep all messages

## Request-Response Patterns Using Services

### Service Architecture
Services provide a request-response communication pattern where a client sends a request to a server and receives a response.

### Service Definition
Services are defined using .srv files that specify request and response message types:

```
# Request
string name
int32 age
---
# Response
bool success
string message
```

### Service Server Example
```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response
```

### Service Client Example
```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## Connecting Python AI Agents to ROS 2 Controllers Using rclpy

### AI Agent Integration
Python-based AI agents can be integrated into ROS 2 systems by creating nodes that interface between AI processing and robot control.

### Example: AI Decision Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def laser_callback(self, msg):
        # Process sensor data with AI algorithm
        command = self.ai_decision_algorithm(msg.ranges)

        # Publish command to robot
        self.publisher.publish(command)

    def ai_decision_algorithm(self, sensor_data):
        # AI logic here
        cmd = Twist()
        # ... decision making based on sensor_data
        return cmd
```

### Best Practices for AI Integration
1. Use appropriate QoS settings for sensor data based on latency and reliability requirements
2. Implement proper error handling for AI algorithm failures
3. Consider computational load when designing AI processing nodes
4. Use actions for long-running AI tasks that may take time to complete

## Learning Objectives

After completing this chapter, you should be able to:

- Create and run ROS 2 nodes with proper lifecycle management
- Implement publisher-subscriber communication patterns
- Configure Quality of Service settings appropriately
- Create and use service-based request-response patterns
- Connect Python AI agents to ROS 2 controllers using rclpy

## Exercises

1. Create a simple publisher node that publishes sensor data (e.g., temperature readings) and a subscriber node that logs this data.
2. Implement a service that performs a mathematical calculation and test it with a client node.
3. Design a simple AI agent that makes decisions based on sensor input and publishes commands to control a robot.