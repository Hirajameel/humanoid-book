from dataclasses import dataclass
from typing import Optional, Dict, Any
from datetime import datetime


@dataclass
class TextTranscript:
    """
    Represents the transcribed text from speech recognition.
    """
    id: str
    transcript: str
    original_audio_path: Optional[str] = None
    language: Optional[str] = "en"
    duration: Optional[float] = 0.0
    processing_metadata: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate the TextTranscript after initialization."""
        if not self.transcript.strip():
            raise ValueError("TextTranscript transcript must not be empty")

        if self.duration is not None and self.duration < 0:
            raise ValueError("TextTranscript duration must be non-negative")