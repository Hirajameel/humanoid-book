from dataclasses import dataclass
from typing import Union


@dataclass
class Orientation:
    """
    Represents orientation as a quaternion (x, y, z, w).
    Alternatively, can represent Euler angles (roll, pitch, yaw).
    """
    x: float
    y: float
    z: float
    w: float = 0.0  # w component for quaternion; if 0, assumes Euler angles

    def __post_init__(self):
        """Validate the Orientation after initialization."""
        if not isinstance(self.x, (int, float)):
            raise ValueError("Orientation x must be a number")
        if not isinstance(self.y, (int, float)):
            raise ValueError("Orientation y must be a number")
        if not isinstance(self.z, (int, float)):
            raise ValueError("Orientation z must be a number")
        if not isinstance(self.w, (int, float)):
            raise ValueError("Orientation w must be a number")