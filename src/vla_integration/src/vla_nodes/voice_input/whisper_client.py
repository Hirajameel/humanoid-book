import openai
import io
import tempfile
import wave
from typing import Optional
from pathlib import Path
from .config_manager import ConfigManager
from ..utils.error_handlers import handle_vla_error, VLAErrorCode


class WhisperClient:
    """
    Client for OpenAI Whisper API for speech-to-text conversion.
    """
    def __init__(self, config_manager: ConfigManager):
        self.config = config_manager
        api_key = self.config.get('openai.api_key')

        if not api_key:
            raise ValueError("OpenAI API key is required for WhisperClient")

        openai.api_key = api_key
        self.whisper_model = self.config.get('openai.whisper_model', 'whisper-1')

    @handle_vla_error(VLAErrorCode.SPEECH_RECOGNITION_ERROR, default_return="")
    def transcribe_audio(self, audio_data: bytes, file_format: str = "wav") -> str:
        """
        Transcribe audio data to text using OpenAI Whisper API.

        Args:
            audio_data: Raw audio bytes
            file_format: Format of the audio file (e.g., wav, mp3)

        Returns:
            Transcribed text
        """
        # Create a temporary file to save the audio data
        with tempfile.NamedTemporaryFile(suffix=f".{file_format}", delete=False) as temp_file:
            temp_path = Path(temp_file.name)

            # Write audio data to temporary file
            temp_file.write(audio_data)
            temp_file.flush()

            try:
                # Open the temporary file and send to Whisper API
                with open(temp_path, "rb") as audio_file:
                    transcript = openai.audio.transcriptions.create(
                        model=self.whisper_model,
                        file=audio_file
                    )

                return transcript.text
            finally:
                # Clean up the temporary file
                temp_path.unlink()

    @handle_vla_error(VLAErrorCode.SPEECH_RECOGNITION_ERROR, default_return="")
    def transcribe_audio_file(self, file_path: str) -> str:
        """
        Transcribe an audio file to text using OpenAI Whisper API.

        Args:
            file_path: Path to the audio file

        Returns:
            Transcribed text
        """
        with open(file_path, "rb") as audio_file:
            transcript = openai.audio.transcriptions.create(
                model=self.whisper_model,
                file=audio_file
            )

        return transcript.text

    def transcribe_audio_with_context(self, audio_data: bytes, prompt: Optional[str] = None) -> str:
        """
        Transcribe audio with additional context prompt to improve accuracy.

        Args:
            audio_data: Raw audio bytes
            prompt: Optional context prompt to guide transcription

        Returns:
            Transcribed text
        """
        # Create a temporary file to save the audio data
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_file:
            temp_path = Path(temp_file.name)

            # Write audio data to temporary file
            temp_file.write(audio_data)
            temp_file.flush()

            try:
                # Open the temporary file and send to Whisper API with prompt
                with open(temp_path, "rb") as audio_file:
                    transcript = openai.audio.transcriptions.create(
                        model=self.whisper_model,
                        file=audio_file,
                        prompt=prompt
                    )

                return transcript.text
            finally:
                # Clean up the temporary file
                temp_path.unlink()