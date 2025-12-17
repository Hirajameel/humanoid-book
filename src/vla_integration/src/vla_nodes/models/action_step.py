from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from enum import Enum


class ActionType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    PERCEPTION = "perception"
    COMMUNICATION = "communication"


@dataclass
class ActionStep:
    """
    Represents a single step in an action plan.
    """
    id: str
    plan_id: str
    step_number: int
    action_type: ActionType
    parameters: Optional[Dict[str, Any]] = None
    dependencies: Optional[List[str]] = None
    timeout: int = 30  # seconds
    retry_count: int = 0

    def __post_init__(self):
        """Validate the ActionStep after initialization."""
        if self.step_number < 0:
            raise ValueError("ActionStep step_number must be non-negative")

        if self.timeout <= 0:
            raise ValueError("ActionStep timeout must be positive")

        if self.retry_count < 0:
            raise ValueError("ActionStep retry_count must be non-negative")

        if self.retry_count > 3:
            raise ValueError("ActionStep retry_count must not exceed maximum allowed retries (3)")

        if isinstance(self.action_type, str):
            self.action_type = ActionType(self.action_type)

        if self.parameters is None:
            self.parameters = {}

        if self.dependencies is None:
            self.dependencies = []