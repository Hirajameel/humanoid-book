# Vision-Language-Action (VLA) Integration

This documentation covers the VLA system that integrates speech recognition, LLM-based cognitive planning, and ROS 2-based robot action execution.

## Overview

The VLA system processes natural language commands and converts them into sequences of actions that can be executed by humanoid robots in both simulation and real-world environments.

## Architecture

The system consists of three main components:
1. **Voice Input Pipeline**: Captures and processes voice commands using OpenAI Whisper
2. **Cognitive Planning Layer**: Interprets commands and generates action plans using GPT
3. **ROS 2 Action Execution**: Maps plans to ROS 2 actions and executes them on robots

## Getting Started

For setup and usage instructions, see the main project documentation.