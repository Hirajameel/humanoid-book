import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ..utils.base_node import BaseVLANode
from ..utils.config_manager import ConfigManager
from ..utils.error_handlers import handle_vla_error, VLAErrorCode
from ..models.voice_command import VoiceCommand
from .audio_capture import AudioCaptureService, AudioData
from .whisper_client import WhisperClient
from .audio_preprocessor import AudioPreprocessor
from .command_validator import CommandValidator
import uuid
from datetime import datetime


class VoiceInputNode(BaseVLANode):
    """
    ROS 2 node for capturing voice commands and converting them to text.
    """
    def __init__(self):
        # Load configuration
        self.config_manager = ConfigManager("config/vla_config.yaml")

        # Initialize base node
        super().__init__(
            node_name=self.config_manager.get('ros2.node_name', 'vla_voice_input'),
            config_path="config/vla_config.yaml"
        )

        # Initialize services
        self.audio_capture = AudioCaptureService(self.config_manager)
        self.whisper_client = WhisperClient(self.config_manager)
        self.preprocessor = AudioPreprocessor(self.config_manager)
        self.validator = CommandValidator()

        # Setup publisher for voice commands
        voice_topic = self.config_manager.get('ros2.voice_command_topic', '/vla/voice_command')
        self.voice_publisher = self.setup_publisher(String, voice_topic)

        # Setup audio callback
        self.audio_capture.set_audio_callback(self._handle_audio_data)

        # Store the last recognized command to avoid duplicates
        self.last_command_text = ""
        self.command_cooldown = 2.0  # seconds
        self.last_command_time = datetime.min

        self.logger.info("Voice Input Node initialized")

    def start_listening(self):
        """Start listening for voice commands."""
        self.audio_capture.start_recording()
        self.logger.info("Voice input node started listening")

    def stop_listening(self):
        """Stop listening for voice commands."""
        self.audio_capture.stop_recording()
        self.logger.info("Voice input node stopped listening")

    def _handle_audio_data(self, audio_data: AudioData):
        """
        Handle incoming audio data and process it into voice commands.
        """
        try:
            # Check if we're still in cooldown period
            time_since_last = (datetime.now() - self.last_command_time).total_seconds()
            if time_since_last < self.command_cooldown:
                return

            # Preprocess the audio
            processed_audio = self.preprocessor.preprocess_for_whisper(audio_data.audio_data)

            # Transcribe the audio
            transcribed_text = self.whisper_client.transcribe_audio(processed_audio)

            # Skip if transcription is empty or same as last command
            if not transcribed_text.strip() or transcribed_text == self.last_command_text:
                return

            # Create a VoiceCommand object
            voice_command = VoiceCommand(
                id=str(uuid.uuid4()),
                text=transcribed_text,
                timestamp=datetime.now(),
                confidence=0.9,  # Placeholder - Whisper doesn't provide confidence directly
                status="pending"
            )

            # Validate the command
            is_valid, error_msg = self.validator.validate_voice_command(voice_command)
            if not is_valid:
                self.logger.warning(f"Invalid command: {error_msg}")
                return

            # Update last command tracking
            self.last_command_text = transcribed_text
            self.last_command_time = datetime.now()

            # Publish the command
            self._publish_voice_command(voice_command)

        except Exception as e:
            self.logger.error(f"Error processing audio data: {e}")

    @handle_vla_error(VLAErrorCode.SPEECH_RECOGNITION_ERROR)
    def _publish_voice_command(self, voice_command: VoiceCommand):
        """
        Publish a voice command to the ROS 2 topic.
        """
        msg = String()
        msg.data = voice_command.text
        self.voice_publisher.publish(msg)
        self.logger.info(f"Published voice command: {voice_command.text}")

    def process_single_command(self, duration: float = 5.0) -> str:
        """
        Record and process a single voice command.

        Args:
            duration: Duration to record audio in seconds

        Returns:
            The transcribed command text
        """
        try:
            # Record audio for specified duration
            audio_data = self.audio_capture.record_single_clip(duration)

            # Preprocess the audio
            processed_audio = self.preprocessor.preprocess_for_whisper(audio_data.audio_data)

            # Transcribe the audio
            transcribed_text = self.whisper_client.transcribe_audio(processed_audio)

            # Validate the command
            if transcribed_text.strip():
                voice_command = VoiceCommand(
                    id=str(uuid.uuid4()),
                    text=transcribed_text,
                    timestamp=datetime.now(),
                    confidence=0.9,  # Placeholder
                    status="pending"
                )

                is_valid, error_msg = self.validator.validate_voice_command(voice_command)
                if is_valid:
                    self._publish_voice_command(voice_command)
                    return transcribed_text
                else:
                    self.logger.warning(f"Invalid command: {error_msg}")
                    return ""

            return transcribed_text

        except Exception as e:
            self.logger.error(f"Error processing single command: {e}")
            return ""

    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.audio_capture.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    voice_input_node = VoiceInputNode()
    voice_input_node.start_listening()

    try:
        rclpy.spin(voice_input_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_input_node.stop_listening()
        voice_input_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()