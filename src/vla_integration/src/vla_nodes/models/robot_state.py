from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any, List
from .position import Position
from .orientation import Orientation


@dataclass
class RobotState:
    """
    Represents the current state of the robot.
    """
    id: str
    timestamp: datetime
    position: Optional[Position] = None
    orientation: Optional[Orientation] = None
    joint_states: Optional[Dict[str, Any]] = None
    battery_level: float = 100.0  # percentage
    active_sensors: Optional[Dict[str, bool]] = None
    current_task: Optional[str] = None

    def __post_init__(self):
        """Validate the RobotState after initialization."""
        if not 0 <= self.battery_level <= 100:
            raise ValueError("RobotState battery_level must be between 0 and 100")

        if self.position is None:
            self.position = Position(0.0, 0.0, 0.0)

        if self.orientation is None:
            self.orientation = Orientation(0.0, 0.0, 0.0, 1.0)  # Default quaternion (no rotation)

        if self.joint_states is None:
            self.joint_states = {}

        if self.active_sensors is None:
            self.active_sensors = {}