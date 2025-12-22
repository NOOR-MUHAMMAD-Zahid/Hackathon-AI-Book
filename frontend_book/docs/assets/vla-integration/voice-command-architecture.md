# Voice Command Architecture Diagram

[Diagram showing the architecture of voice command processing for humanoid robots]

## Description
This diagram illustrates the complete voice command processing pipeline for humanoid robots, from human speech to robot action execution. The diagram shows how voice input is captured, processed through Whisper for transcription, parsed for intent, and converted to ROS 2 actions.

## Key Elements
- Human Speaker
- Voice Capture (Microphones)
- Speech-to-Text (Whisper)
- Intent Parser/NLU
- ROS 2 Action Client
- Robot Action Server
- Feedback System
- Confidence Scoring