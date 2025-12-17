from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Dict, Any
from enum import Enum
from .action_step import ActionStep


class ActionPlanStatus(Enum):
    PLANNED = "planned"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class ActionPlan:
    """
    Represents a sequence of actions generated from a voice command.
    """
    id: str
    command_id: str
    steps: List[ActionStep]
    created_at: datetime
    status: ActionPlanStatus = ActionPlanStatus.PLANNED
    updated_at: Optional[datetime] = None
    execution_context: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate the ActionPlan after initialization."""
        if not self.steps:
            raise ValueError("ActionPlan steps array must not be empty")

        if isinstance(self.status, str):
            self.status = ActionPlanStatus(self.status)

        if self.updated_at and self.created_at > self.updated_at:
            raise ValueError("ActionPlan created_at must be before updated_at if both present")