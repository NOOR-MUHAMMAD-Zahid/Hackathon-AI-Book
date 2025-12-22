---
title: Voice-to-Action Interfaces
sidebar_position: 2
---

# Voice-to-Action Interfaces

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain voice interaction** as a control modality for humanoid robots and its role in Physical AI systems
2. **Implement speech-to-text pipelines** using OpenAI Whisper for robot command processing
3. **Convert voice commands** into structured robot intents that can be processed by action systems
4. **Integrate voice inputs** into ROS 2 action flows for natural human-robot interaction

## Table of Contents
- [Voice Interaction as a Control Modality](#voice-interaction-as-a-control-modality)
- [Speech-to-Text Pipelines with OpenAI Whisper](#speech-to-text-pipelines-with-openai-whisper)
- [Converting Voice Commands to Structured Robot Intents](#converting-voice-commands-to-structured-robot-intents)
- [Integrating Voice Inputs into ROS 2 Action Flows](#integrating-voice-inputs-into-ros-2-action-flows)
- [Exercises](#exercises)

## Voice Interaction as a Control Modality

Voice interaction represents a natural and intuitive way for humans to communicate with humanoid robots. Unlike traditional interfaces that require physical interaction (buttons, touchscreens, remotes), voice commands allow humans to interact with robots using the same modality they use for human-to-human communication.

### Advantages of Voice Control for Humanoid Robots

1. **Natural Interaction**: Humans naturally use voice to communicate, making it intuitive for robot control
2. **Hands-Free Operation**: Allows users to control robots while performing other tasks
3. **Accessibility**: Enables interaction for users with motor impairments or when hands are occupied
4. **Social Integration**: Makes robots feel more like collaborative partners than tools

### Challenges in Voice Control for Robotics

1. **Ambient Noise**: Background sounds can interfere with speech recognition accuracy
2. **Distance Sensitivity**: Microphone sensitivity decreases with distance from speaker
3. **Homonym Resolution**: Same-sounding words with different meanings (e.g., "to", "too", "two")
4. **Ambiguous Commands**: Natural language often contains implicit assumptions or underspecification

### Voice Command Architecture for Humanoid Robots

```
Human Speaker
       ↓ (Voice Command)
Voice Capture (Microphones)
       ↓ (Audio Stream)
Speech-to-Text (Whisper/ASR)
       ↓ (Transcribed Text)
Intent Parser/NLU
       ↓ (Structured Intent)
ROS 2 Action Client
       ↓ (Action Request)
Robot Action Server
       ↓ (Execution Result)
Feedback System
```

> **Figure 1**: Voice command processing pipeline for humanoid robots showing the complete flow from audio capture to action execution. The diagram illustrates how voice input is captured by microphones, processed through Whisper for transcription, parsed for intent understanding, and converted to ROS 2 actions for robot execution.

*Pipeline: Human Speaker → Microphones → Speech-to-Text (Whisper) → Intent Parser → ROS 2 Action Client → Robot Action Server → Execution Result → Feedback System*

## Speech-to-Text Pipelines with OpenAI Whisper

OpenAI Whisper is a state-of-the-art speech recognition model that excels at transcribing audio in multiple languages and handling various accents and background noise conditions. Its multilingual capabilities make it particularly suitable for robotics applications where users may speak different languages.

### Whisper Model Characteristics

- **Multilingual Support**: Trained on 98 languages including low-resource languages
- **Robustness**: Performs well on various accents, background noise, and technical speech
- **Versatility**: Supports transcription, translation, language identification, and speech detection
- **Open Source**: Available under MIT license for commercial and research use

### Whisper Integration Architecture

```python
# Example Whisper integration for robotics
import whisper
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class VoiceCommandProcessor:
    def __init__(self):
        # Initialize Whisper model
        self.model = whisper.load_model("medium")

        # ROS 2 subscribers and publishers
        self.audio_sub = rospy.Subscriber("/audio_input", AudioData, self.audio_callback)
        self.command_pub = rospy.Publisher("/parsed_commands", String, queue_size=10)

        # Configuration parameters
        self.noise_threshold = 0.01
        self.confidence_threshold = 0.7

    def audio_callback(self, audio_msg):
        """Process incoming audio and convert to text"""
        # Convert audio data to appropriate format for Whisper
        audio_array = self.convert_audio_format(audio_msg.data)

        # Transcribe audio using Whisper
        result = self.model.transcribe(audio_array)

        # Extract text and confidence score
        transcribed_text = result['text']
        confidence = result.get('avg_logprob', 0)

        # Filter low-confidence transcriptions
        if confidence > self.confidence_threshold:
            self.process_command(transcribed_text)

    def process_command(self, text):
        """Parse and publish structured commands"""
        # Parse natural language command
        structured_intent = self.parse_intent(text)

        # Publish structured command to ROS 2 system
        command_msg = String(data=str(structured_intent))
        self.command_pub.publish(command_msg)
```

### Whisper Configuration for Robotics

```yaml
# Configuration for Whisper-based voice command processing
voice_processor:
  ros__parameters:
    # Whisper model settings
    model_size: "medium"  # Options: tiny, base, small, medium, large
    device: "cuda"        # Options: cpu, cuda (GPU acceleration)
    language: "english"   # Optional: specify language for better accuracy

    # Audio processing settings
    sample_rate: 16000    # Audio sample rate in Hz
    audio_buffer_size: 1024  # Size of audio chunks for processing
    silence_threshold: 0.01  # Threshold for voice activity detection

    # Confidence settings
    transcription_confidence_threshold: 0.7  # Minimum confidence for valid transcription
    noise_suppression: true  # Enable noise suppression preprocessing

    # Output settings
    publish_transcription: true  # Whether to publish raw transcriptions
    publish_confidence: true     # Whether to publish confidence scores
```

## Converting Voice Commands to Structured Robot Intents

The conversion of natural language voice commands to structured robot intents is a critical step in the voice-to-action pipeline. This process involves understanding the user's intent and mapping it to specific robot actions that can be executed by the robot's control system.

### Intent Classification Architecture

Intent classification for robotics typically involves multiple levels of understanding:

1. **Command Type Recognition**: Identify the general category of action (navigation, manipulation, information request)
2. **Entity Extraction**: Identify specific objects, locations, or parameters mentioned in the command
3. **Action Mapping**: Map the recognized intent to specific robot action services or topics
4. **Parameter Validation**: Verify that extracted parameters are valid and safe for robot execution

> **Figure 2**: Intent processing flow showing the conversion from voice command to structured robot intent. The diagram illustrates how natural language commands are processed through transcription, entity recognition, intent classification, and action mapping to create structured robot intents.

*Flow: Raw Voice Input → Audio Processing → Transcription → Natural Language Processing → Entity Recognition → Intent Classification → Action Mapping → Structured Robot Intent*

### Example Intent Classification Pipeline

```python
# Example intent parser for humanoid robot commands
import re
from typing import Dict, Any, Optional

class IntentParser:
    def __init__(self):
        # Define command patterns
        self.patterns = {
            'navigation': [
                r'go to (.+)',
                r'go to the (.+)',
                r'move to (.+)',
                r'walk to (.+)',
                r'navigate to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'grab (.+)',
                r'bring me (.+)',
                r'fetch (.+)'
            ],
            'information': [
                r'what is (.+)',
                r'where is (.+)',
                r'tell me about (.+)',
                r'explain (.+)'
            ]
        }

    def parse_intent(self, text: str) -> Dict[str, Any]:
        """Parse natural language command into structured intent"""
        text = text.lower().strip()

        # Normalize text (remove punctuation, normalize whitespace)
        normalized_text = re.sub(r'[^\w\s]', ' ', text)
        normalized_text = ' '.join(normalized_text.split())

        # Match against patterns
        for intent_type, patterns in self.patterns.items():
            for pattern in patterns:
                match = re.search(pattern, normalized_text)
                if match:
                    entities = match.groups()
                    return {
                        'intent_type': intent_type,
                        'primary_entity': entities[0].strip(),
                        'original_text': text,
                        'confidence': 0.9  # Simplified confidence calculation
                    }

        # Default to unknown intent if no pattern matches
        return {
            'intent_type': 'unknown',
            'primary_entity': '',
            'original_text': text,
            'confidence': 0.0
        }

# Example usage
parser = IntentParser()
result = parser.parse_intent("Go to the kitchen and bring me a cup")
print(result)
# Output: {'intent_type': 'navigation', 'primary_entity': 'the kitchen', 'original_text': 'go to the kitchen and bring me a cup', 'confidence': 0.9}
```

### ROS 2 Message Types for Voice Commands

For voice command integration with ROS 2, you'll typically define custom message types to carry structured command information:

```yaml
# voice_command.msg
string intent_type  # navigation, manipulation, information, etc.
string primary_entity  # object or location name
string original_text  # original transcribed text
float32 confidence  # transcription confidence score
float32[] parameters  # additional numeric parameters
string[] entities  # additional extracted entities
```

### Example Voice Command Processing Node

```python
# voice_command_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from your_msgs.msg import VoiceCommand  # Custom message type
import whisper
import re

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Initialize Whisper model
        self.model = whisper.load_model("medium")

        # Subscribers and publishers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/microphone/audio_raw',
            self.audio_callback,
            10
        )

        self.command_pub = self.create_publisher(
            VoiceCommand,
            '/voice_commands',
            10
        )

        self.text_pub = self.create_publisher(
            String,
            '/transcriptions',
            10
        )

        # Configuration
        self.confidence_threshold = 0.7
        self.intent_parser = IntentParser()

    def audio_callback(self, msg):
        """Process incoming audio data"""
        try:
            # Convert audio data to numpy array (implementation depends on format)
            audio_array = self.convert_audio_to_numpy(msg.data)

            # Transcribe using Whisper
            result = self.model.transcribe(audio_array)

            transcription = result['text']
            confidence = result.get('avg_logprob', 0)

            # Publish transcription for monitoring
            text_msg = String()
            text_msg.data = f"{transcription} (conf: {confidence:.2f})"
            self.text_pub.publish(text_msg)

            # Only process if confidence is above threshold
            if confidence > self.confidence_threshold:
                # Parse intent
                intent_data = self.intent_parser.parse_intent(transcription)

                # Create and publish structured command
                command_msg = VoiceCommand()
                command_msg.intent_type = intent_data['intent_type']
                command_msg.primary_entity = intent_data['primary_entity']
                command_msg.original_text = intent_data['original_text']
                command_msg.confidence = intent_data['confidence']

                self.command_pub.publish(command_msg)

                self.get_logger().info(f'Published command: {command_msg.intent_type} - {command_msg.primary_entity}')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def convert_audio_to_numpy(self, audio_data):
        """Convert audio data to numpy array for Whisper processing"""
        # Implementation depends on audio format
        # This is a simplified example
        import numpy as np
        # Assuming audio_data is raw bytes at 16kHz
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0  # Normalize to [-1, 1]
        return audio_array

def main(args=None):
    rclpy.init(args=args)
    processor = VoiceCommandProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down voice command processor')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating Voice Inputs into ROS 2 Action Flows

Once voice commands are converted to structured intents, they need to be integrated into ROS 2 action flows. Actions are the preferred method for long-running tasks that provide feedback during execution, making them ideal for robot navigation, manipulation, and other complex behaviors.

### ROS 2 Action Structure for Voice Commands

```
Voice Command Processor
         ↓ (VoiceCommand message)
Action Client (Voice Command Handler)
         ↓ (Action Goal)
Action Server (Robot Behavior)
         ↓ (Feedback during execution)
         ↑ (Result upon completion)
```

### Example Voice Command Action Server

```python
# voice_command_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_msgs.action import VoiceCommandAction  # Custom action type
from your_msgs.msg import VoiceCommand  # Custom message type
import threading
import time

class VoiceCommandActionServer(Node):
    def __init__(self):
        super().__init__('voice_command_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            VoiceCommandAction,
            'voice_command_action',
            self.execute_callback
        )

        # Publishers for robot control
        self.nav_publisher = self.create_publisher(String, '/navigation_goals', 10)
        self.manip_publisher = self.create_publisher(String, '/manipulation_goals', 10)

        self.get_logger().info('Voice command action server started')

    def execute_callback(self, goal_handle):
        """Execute the voice command action"""
        feedback_msg = VoiceCommandAction.Feedback()
        result_msg = VoiceCommandAction.Result()

        command = goal_handle.request.voice_command

        self.get_logger().info(f'Executing voice command: {command.intent_type} - {command.primary_entity}')

        # Provide initial feedback
        feedback_msg.status = f'Processing {command.intent_type} command'
        goal_handle.publish_feedback(feedback_msg)

        try:
            # Route command based on intent type
            if command.intent_type == 'navigation':
                result = self.execute_navigation_command(command)
            elif command.intent_type == 'manipulation':
                result = self.execute_manipulation_command(command)
            elif command.intent_type == 'information':
                result = self.execute_information_command(command)
            else:
                result = {'success': False, 'message': f'Unknown command type: {command.intent_type}'}

            # Set result
            result_msg.success = result['success']
            result_msg.message = result.get('message', '')

            if result['success']:
                goal_handle.succeed()
                self.get_logger().info(f'Successfully completed {command.intent_type} command')
            else:
                goal_handle.abort()
                self.get_logger().warn(f'Failed to complete {command.intent_type} command: {result.get("message")}')

        except Exception as e:
            self.get_logger().error(f'Error executing voice command: {e}')
            result_msg.success = False
            result_msg.message = f'Error: {str(e)}'
            goal_handle.abort()

        return result_msg

    def execute_navigation_command(self, command):
        """Execute navigation command"""
        # Publish navigation goal
        nav_msg = String()
        nav_msg.data = command.primary_entity
        self.nav_publisher.publish(nav_msg)

        # Simulate navigation execution (in real implementation, this would wait for actual navigation completion)
        time.sleep(2.0)  # Simulated navigation time

        return {'success': True, 'message': f'Navigated to {command.primary_entity}'}

    def execute_manipulation_command(self, command):
        """Execute manipulation command"""
        # Publish manipulation goal
        manip_msg = String()
        manip_msg.data = command.primary_entity
        self.manip_publisher.publish(manip_msg)

        # Simulate manipulation execution
        time.sleep(3.0)  # Simulated manipulation time

        return {'success': True, 'message': f'Manipulated {command.primary_entity}'}

    def execute_information_command(self, command):
        """Execute information command"""
        # In a real implementation, this might query a knowledge base or sensor data
        time.sleep(1.0)  # Simulated information retrieval time

        return {'success': True, 'message': f'Retrieved information about {command.primary_entity}'}

def main(args=None):
    rclpy.init(args=args)
    server = VoiceCommandActionServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down voice command action server')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File for Voice Command System

```xml
<!-- voice_command_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    model_arg = DeclareLaunchArgument(
        'whisper_model_size',
        default_value='medium',
        description='Size of Whisper model to use'
    )

    ld.add_action(model_arg)

    # Voice command processor node
    voice_processor = Node(
        package='your_voice_package',
        executable='voice_command_processor',
        name='voice_command_processor',
        parameters=[
            {'model_size': LaunchConfiguration('whisper_model_size')},
            {'confidence_threshold': 0.7}
        ],
        remappings=[
            ('/microphone/audio_raw', '/audio_input'),
            ('/voice_commands', '/parsed_voice_commands')
        ]
    )

    # Voice command action server
    action_server = Node(
        package='your_voice_package',
        executable='voice_command_action_server',
        name='voice_command_action_server'
    )

    # Navigation system (example)
    navigation_system = Node(
        package='nav2_bringup',
        executable='nav2',
        name='navigation_system'
    )

    ld.add_action(voice_processor)
    ld.add_action(action_server)
    ld.add_action(navigation_system)

    return ld
```

## Exercises

### Exercise 1: Voice Command Processing
1. Implement a simple Whisper-based voice command processor that can recognize basic navigation commands
2. Create a custom ROS 2 message type for voice commands with appropriate fields
3. Test the system with different voice commands and evaluate transcription accuracy

### Exercise 2: Intent Classification
1. Extend the intent parser to recognize more complex commands with multiple entities
2. Implement validation for extracted entities (e.g., check if location exists in robot's map)
3. Add confidence scoring to intent classification results

### Exercise 3: Action Integration
1. Create a ROS 2 action server that responds to voice commands
2. Implement feedback during action execution
3. Test the complete voice-to-action pipeline with simulated robot behaviors

## Summary

Voice-to-action interfaces provide a natural and intuitive way for humans to interact with humanoid robots. By leveraging OpenAI Whisper for robust speech-to-text conversion and properly structuring voice commands as ROS 2 actions, we can create systems that understand and execute natural language commands reliably. The key components include audio capture, speech recognition, intent parsing, and action execution, all integrated into the ROS 2 framework for consistency with other robot systems.

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain voice interaction** as a control modality for humanoid robots and its role in Physical AI systems
2. **Implement speech-to-text pipelines** using OpenAI Whisper for robot command processing with appropriate confidence scoring
3. **Convert voice commands** into structured robot intents that can be processed by ROS 2 action systems
4. **Integrate voice inputs** into ROS 2 action flows for natural human-robot interaction with proper error handling and feedback mechanisms

## Exercises

### Exercise 1: Voice Command Processing Implementation
1. Set up a basic Whisper model for speech-to-text processing
2. Create a simple intent parser that can recognize basic navigation commands like "Go to the kitchen" or "Move to the table"
3. Implement confidence threshold filtering to avoid acting on low-confidence transcriptions
4. Test your implementation with various voice commands and evaluate accuracy

### Exercise 2: ROS 2 Integration
1. Create a custom ROS 2 message type for voice commands with fields for intent type, entities, and confidence
2. Implement a ROS 2 node that subscribes to audio input and publishes parsed voice commands
3. Create a simple action server that responds to voice commands with appropriate feedback
4. Test the complete voice-to-action pipeline with simulated audio input

### Exercise 3: Error Handling and Validation
1. Implement validation for extracted entities (e.g., check if a requested location exists in the robot's map)
2. Add error handling for cases where Whisper fails to transcribe audio accurately
3. Create a feedback mechanism that informs the user when commands are misunderstood
4. Test edge cases such as ambiguous commands, unrecognized vocabulary, or noisy environments

## Example Code: Whisper Integration

```python
# Example implementation of Whisper integration for robotics
import whisper
import rclpy
import numpy as np
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from your_msgs.msg import VoiceCommand

class WhisperVoiceProcessor:
    def __init__(self):
        # Load Whisper model (adjust size based on computational resources)
        self.model = whisper.load_model("medium")

        # ROS 2 initialization
        rclpy.init()
        self.node = rclpy.create_node('whisper_voice_processor')

        # Publishers and subscribers
        self.audio_sub = self.node.create_subscription(AudioData, "/audio_input", self.audio_callback, 10)
        self.command_pub = self.node.create_publisher(VoiceCommand, "/voice_commands", 10)
        self.feedback_pub = self.node.create_publisher(String, "/voice_feedback", 10)

        # Configuration parameters
        self.confidence_threshold = 0.7
        self.language = "english"

        self.node.get_logger().info("Whisper voice processor initialized")

    def audio_callback(self, audio_msg):
        """Process incoming audio data and convert to structured commands"""
        try:
            # Convert audio data to appropriate format
            audio_array = self.convert_audio_format(audio_msg.data)

            # Transcribe audio using Whisper
            result = self.model.transcribe(
                audio_array,
                language=self.language,
                temperature=0.0  # More deterministic results
            )

            transcription = result['text'].strip()
            confidence = result.get('avg_logprob', -float('inf'))

            # Log transcription for debugging
            self.node.get_logger().info(f"Transcription: '{transcription}' (confidence: {confidence:.2f})")

            # Only process if confidence is above threshold
            if confidence > self.confidence_threshold:
                # Parse intent from transcription
                intent_data = self.parse_intent(transcription)

                # Create and publish structured command
                command_msg = VoiceCommand()
                command_msg.intent_type = intent_data.get('intent_type', 'unknown')
                command_msg.primary_entity = intent_data.get('primary_entity', '')
                command_msg.original_text = transcription
                command_msg.confidence = confidence

                self.command_pub.publish(command_msg)

                # Provide feedback
                feedback_msg = String()
                feedback_msg.data = f"Processed command: {transcription}"
                self.feedback_pub.publish(feedback_msg)
            else:
                # Low confidence - provide feedback
                feedback_msg = String()
                feedback_msg.data = f"Low confidence transcription ignored: {transcription} ({confidence:.2f})"
                self.feedback_pub.publish(feedback_msg)

        except Exception as e:
            self.node.get_logger().error(f"Error processing audio: {e}")
            # Provide error feedback
            feedback_msg = String()
            feedback_msg.data = f"Error processing voice command: {str(e)}"
            self.feedback_pub.publish(feedback_msg)

    def convert_audio_format(self, audio_bytes):
        """Convert audio bytes to format suitable for Whisper"""
        # Assuming audio is 16-bit PCM at 16kHz
        # Adjust based on your microphone configuration
        audio_array = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0  # Normalize to [-1, 1]
        return audio_array

    def parse_intent(self, text):
        """Simple intent parser - in practice, you might use more sophisticated NLP"""
        text_lower = text.lower()

        # Navigation intents
        if any(keyword in text_lower for keyword in ['go to', 'move to', 'navigate to', 'walk to']):
            # Extract destination
            for keyword in ['go to', 'move to', 'navigate to', 'walk to']:
                if keyword in text_lower:
                    destination = text_lower.split(keyword)[1].strip()
                    return {
                        'intent_type': 'navigation',
                        'primary_entity': destination
                    }

        # Manipulation intents
        elif any(keyword in text_lower for keyword in ['pick up', 'grasp', 'grab', 'bring me', 'fetch']):
            for keyword in ['pick up', 'grasp', 'grab', 'bring me', 'fetch']:
                if keyword in text_lower:
                    object_name = text_lower.split(keyword)[1].strip()
                    return {
                        'intent_type': 'manipulation',
                        'primary_entity': object_name
                    }

        # Information intents
        elif any(keyword in text_lower for keyword in ['what is', 'where is', 'tell me about', 'explain']):
            for keyword in ['what is', 'where is', 'tell me about', 'explain']:
                if keyword in text_lower:
                    subject = text_lower.split(keyword)[1].strip()
                    return {
                        'intent_type': 'information',
                        'primary_entity': subject
                    }

        # Unknown intent
        return {
            'intent_type': 'unknown',
            'primary_entity': text_lower
        }

def main():
    processor = WhisperVoiceProcessor()

    try:
        rclpy.spin(processor.node)
    except KeyboardInterrupt:
        processor.node.get_logger().info("Shutting down voice processor")
    finally:
        processor.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

In the next chapter, we'll explore cognitive planning with vision-language models, where you'll learn how to use LLMs for high-level task decomposition and translating natural language goals into ROS 2 action sequences.