from typing import Optional, Callable, Any
import functools
import logging
from ..utils.error_handlers import VLAError, VLAErrorCode, handle_vla_error


def handle_audio_processing_error(default_return: Any = None):
    """
    Decorator to handle audio processing errors specifically.
    """
    return handle_vla_error(VLAErrorCode.AUDIO_CAPTURE_ERROR, default_return)


def handle_speech_recognition_error(default_return: Any = None):
    """
    Decorator to handle speech recognition errors specifically.
    """
    return handle_vla_error(VLAErrorCode.SPEECH_RECOGNITION_ERROR, default_return)


class AudioProcessingError(VLAError):
    """Exception raised for audio processing errors."""
    pass


class SpeechRecognitionError(VLAError):
    """Exception raised for speech recognition errors."""
    pass


class AudioErrorHandler:
    """
    Error handler specifically for audio processing components.
    """
    @staticmethod
    def handle_audio_capture_error(error: Exception) -> AudioProcessingError:
        """Handle audio capture related errors."""
        return AudioProcessingError(f"Audio capture error: {str(error)}")

    @staticmethod
    def handle_microphone_access_error(error: Exception) -> AudioProcessingError:
        """Handle microphone access errors."""
        return AudioProcessingError(f"Microphone access error: {str(error)}")

    @staticmethod
    def handle_audio_format_error(error: Exception) -> AudioProcessingError:
        """Handle audio format related errors."""
        return AudioProcessingError(f"Audio format error: {str(error)}")

    @staticmethod
    def handle_whisper_api_error(error: Exception) -> SpeechRecognitionError:
        """Handle Whisper API related errors."""
        return SpeechRecognitionError(f"Whisper API error: {str(error)}")

    @staticmethod
    def handle_transcription_error(error: Exception) -> SpeechRecognitionError:
        """Handle general transcription errors."""
        return SpeechRecognitionError(f"Transcription error: {str(error)}")