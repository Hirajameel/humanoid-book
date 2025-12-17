from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict, Any


@dataclass
class ExecutionContext:
    """
    Represents the execution context for an action plan.
    """
    id: str
    timestamp: datetime
    environment: str  # simulation or real environment
    obstacles: Optional[List[Dict[str, Any]]] = None
    constraints: Optional[Dict[str, Any]] = None
    feedback: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate the ExecutionContext after initialization."""
        if self.environment not in ["simulation", "real"]:
            raise ValueError("ExecutionContext environment must be 'simulation' or 'real'")

        if self.obstacles is None:
            self.obstacles = []

        if self.constraints is None:
            self.constraints = {}

        if self.feedback is None:
            self.feedback = {}