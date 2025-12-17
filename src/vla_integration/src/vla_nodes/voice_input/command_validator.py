from typing import Tuple, Optional
from ..models.voice_command import VoiceCommand
from ..utils.error_handlers import VLAError


class CommandValidator:
    """
    Validates voice commands to ensure they meet the system's requirements.
    """
    def __init__(self):
        # Define valid command patterns or keywords if needed
        self.valid_commands = set()  # Will be populated based on robot capabilities
        self.min_confidence = 0.5  # Minimum confidence for a command to be considered valid

    def validate_command_text(self, text: str) -> Tuple[bool, Optional[str]]:
        """
        Validate the text of a voice command.

        Args:
            text: The transcribed text

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not text or not text.strip():
            return False, "Command text cannot be empty"

        text = text.strip()

        # Check for minimum length
        if len(text) < 3:
            return False, "Command text is too short (minimum 3 characters)"

        # Check for maximum length
        if len(text) > 200:
            return False, "Command text is too long (maximum 200 characters)"

        # Basic validation - command should contain at least one word
        words = text.split()
        if len(words) == 0:
            return False, "Command text must contain at least one word"

        return True, None

    def validate_voice_command(self, command: VoiceCommand) -> Tuple[bool, Optional[str]]:
        """
        Validate a complete VoiceCommand object.

        Args:
            command: The VoiceCommand object to validate

        Returns:
            Tuple of (is_valid, error_message)
        """
        # Validate the text
        is_text_valid, text_error = self.validate_command_text(command.text)
        if not is_text_valid:
            return False, text_error

        # Validate confidence level
        if command.confidence < self.min_confidence:
            return False, f"Command confidence too low ({command.confidence}), minimum required: {self.min_confidence}"

        # Validate status
        if command.status.value not in ['pending', 'processing', 'completed', 'failed']:
            return False, f"Invalid command status: {command.status.value}"

        # Additional validations can be added here based on robot capabilities

        return True, None

    def validate_and_clean_command(self, command: VoiceCommand) -> VoiceCommand:
        """
        Validate and clean a voice command, raising an exception if invalid.

        Args:
            command: The VoiceCommand object to validate and clean

        Returns:
            The validated and cleaned VoiceCommand object

        Raises:
            VLAError: If the command is invalid
        """
        is_valid, error_msg = self.validate_voice_command(command)
        if not is_valid:
            raise VLAError(f"Invalid voice command: {error_msg}")

        # Clean the command text (remove extra whitespace, etc.)
        command.text = command.text.strip()

        return command

    def add_valid_command(self, command: str):
        """Add a valid command to the list of recognized commands."""
        self.valid_commands.add(command.lower())

    def add_valid_commands(self, commands: list):
        """Add multiple valid commands to the list of recognized commands."""
        for cmd in commands:
            self.add_valid_command(cmd)

    def is_command_recognized(self, command_text: str) -> bool:
        """
        Check if the command is in the list of recognized commands.
        This is optional validation that can be enabled based on requirements.

        Args:
            command_text: The command text to check

        Returns:
            True if the command is recognized, False otherwise
        """
        if not self.valid_commands:
            # If no valid commands are defined, accept all commands
            return True

        # Check if any of the words in the command match known commands
        words = command_text.lower().split()
        for word in words:
            if word in self.valid_commands:
                return True

        return False