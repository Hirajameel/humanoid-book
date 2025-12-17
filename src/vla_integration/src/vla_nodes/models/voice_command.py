from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any
from enum import Enum


class VoiceCommandStatus(Enum):
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class VoiceCommand:
    """
    Represents a voice command received from the user.
    """
    id: str
    text: str
    timestamp: datetime
    confidence: float
    user_id: Optional[str] = None
    status: VoiceCommandStatus = VoiceCommandStatus.PENDING

    def __post_init__(self):
        """Validate the VoiceCommand after initialization."""
        if not self.text.strip():
            raise ValueError("VoiceCommand text must not be empty")

        if not 0 <= self.confidence <= 1:
            raise ValueError("VoiceCommand confidence must be between 0 and 1")

        if isinstance(self.status, str):
            self.status = VoiceCommandStatus(self.status)