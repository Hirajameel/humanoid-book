# Quickstart Guide: Vision-Language-Action (VLA) Integration

## Prerequisites

- Python 3.8 or higher
- ROS 2 (Humble Hawksbill or later)
- OpenAI API key
- Docker (for simulation environments)
- System dependencies for audio capture

## Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd humanoid-book
   ```

2. **Install Python dependencies:**
   ```bash
   pip install openai openai-whisper pyaudio rclpy
   ```

3. **Set up environment variables:**
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

4. **Install ROS 2 dependencies:**
   ```bash
   # Install ROS 2 Humble and required packages
   sudo apt update && sudo apt install ros-humble-desktop
   source /opt/ros/humble/setup.bash
   ```

## Running the VLA Pipeline

### 1. Start the VLA Service

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Run the VLA service
python -m vla_service.main
```

### 2. Test Voice Command Processing

```bash
# Submit an audio file for processing
curl -X POST http://localhost:8000/voice/commands \
  -H "Content-Type: multipart/form-data" \
  -F "audio_file=@command.wav"

# Or submit a text command directly
curl -X POST http://localhost:8000/action-plans \
  -H "Content-Type: application/json" \
  -d '{"text_command": "Go to the kitchen and bring me the red cup"}'
```

### 3. Run in Simulation

```bash
# Start Gazebo simulation
ros2 launch my_robot_gazebo robot_world.launch.py

# Execute action plan in simulation
curl -X POST http://localhost:8000/action-plans/{plan_id}/execute \
  -H "Content-Type: application/json" \
  -d '{"simulation": true, "environment": "gazebo"}'
```

## Key Components

### Voice Processing Node
- Captures audio input
- Converts speech to text using Whisper
- Validates and normalizes commands

### Cognitive Planning Module
- Interprets text commands using GPT
- Generates structured action plans
- Validates plans against robot capabilities

### ROS 2 Action Executor
- Translates action plans to ROS 2 actions
- Manages multi-step task execution
- Handles error recovery and fallbacks

## Configuration

### Environment Variables
- `OPENAI_API_KEY`: OpenAI API key for GPT and Whisper
- `ROS_DOMAIN_ID`: ROS 2 domain ID (default: 0)
- `AUDIO_DEVICE_INDEX`: Audio input device index (default: 0)

### Settings
- Adjust audio capture parameters in `config/audio.yaml`
- Configure LLM parameters in `config/llm.yaml`
- Set simulation options in `config/simulation.yaml`

## Troubleshooting

1. **Audio capture not working:**
   - Check microphone permissions
   - Verify audio device index with `python -m sounddevice.query_devices()`

2. **OpenAI API errors:**
   - Verify API key is set correctly
   - Check internet connectivity
   - Ensure account has sufficient credits

3. **ROS 2 communication issues:**
   - Verify ROS_DOMAIN_ID matches across nodes
   - Check that ROS 2 network is properly configured