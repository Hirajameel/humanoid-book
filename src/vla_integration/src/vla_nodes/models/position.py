from dataclasses import dataclass
from typing import Union


@dataclass
class Position:
    """
    Represents a 3D position in space.
    """
    x: float
    y: float
    z: float

    def __post_init__(self):
        """Validate the Position after initialization."""
        if not isinstance(self.x, (int, float)):
            raise ValueError("Position x must be a number")
        if not isinstance(self.y, (int, float)):
            raise ValueError("Position y must be a number")
        if not isinstance(self.z, (int, float)):
            raise ValueError("Position z must be a number")