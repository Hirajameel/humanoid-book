from typing import Dict, Any, Optional
from ..models.robot_state import RobotState
from ..models.exec_context import ExecutionContext
from datetime import datetime


class SensorIntegration:
    """
    Integration with robot sensors for perception and feedback during action execution.
    """
    def __init__(self):
        self.sensors = {}
        self.last_sensor_data = {}
        self.perception_callbacks = {}

    def register_sensor(self, sensor_name: str, sensor_type: str, callback=None):
        """
        Register a sensor with the perception system.

        Args:
            sensor_name: Name of the sensor
            sensor_type: Type of the sensor (camera, lidar, imu, etc.)
            callback: Optional callback function for sensor data
        """
        self.sensors[sensor_name] = {
            "type": sensor_type,
            "active": True,
            "last_update": None,
            "callback": callback
        }

    def get_sensor_data(self, sensor_name: str) -> Optional[Dict[str, Any]]:
        """
        Get the latest data from a specific sensor.

        Args:
            sensor_name: Name of the sensor to get data from

        Returns:
            Sensor data dictionary or None if sensor not available
        """
        if sensor_name not in self.last_sensor_data:
            return None
        return self.last_sensor_data[sensor_name]

    def get_all_sensor_data(self) -> Dict[str, Any]:
        """
        Get data from all registered sensors.

        Returns:
            Dictionary with all sensor data
        """
        return self.last_sensor_data.copy()

    def update_sensor_data(self, sensor_name: str, data: Dict[str, Any]):
        """
        Update the sensor data for a specific sensor.

        Args:
            sensor_name: Name of the sensor
            data: Sensor data to update
        """
        if sensor_name in self.sensors:
            self.last_sensor_data[sensor_name] = {
                "data": data,
                "timestamp": datetime.now()
            }

            # Call the sensor's callback if it exists
            callback = self.sensors[sensor_name].get("callback")
            if callback:
                callback(sensor_name, data)

    def get_robot_state_from_sensors(self) -> RobotState:
        """
        Generate a RobotState object from current sensor data.

        Returns:
            RobotState object with information from sensors
        """
        from uuid import uuid4
        from ..models.position import Position
        from ..models.orientation import Orientation

        # Extract position and orientation from sensor data
        position = self._extract_position_from_sensors()
        orientation = self._extract_orientation_from_sensors()

        # Extract joint states from sensor data
        joint_states = self._extract_joint_states_from_sensors()

        # Extract active sensors
        active_sensors = {name: sensor["active"] for name, sensor in self.sensors.items()}

        # Create RobotState object
        robot_state = RobotState(
            id=str(uuid4()),
            timestamp=datetime.now(),
            position=position,
            orientation=orientation,
            joint_states=joint_states,
            battery_level=self._get_battery_level_from_sensors(),
            active_sensors=active_sensors,
            current_task="idle"
        )

        return robot_state

    def _extract_position_from_sensors(self) -> Optional[Position]:
        """
        Extract position information from sensor data.

        Returns:
            Position object or None if not available
        """
        # In a real implementation, this would extract position from localization sensors
        # For now, return a default position
        return Position(0.0, 0.0, 0.0)

    def _extract_orientation_from_sensors(self) -> Optional[Orientation]:
        """
        Extract orientation information from sensor data.

        Returns:
            Orientation object or None if not available
        """
        # In a real implementation, this would extract orientation from IMU or other sensors
        # For now, return a default orientation
        return Orientation(0.0, 0.0, 0.0, 1.0)

    def _extract_joint_states_from_sensors(self) -> Dict[str, float]:
        """
        Extract joint states from sensor data.

        Returns:
            Dictionary of joint names to positions
        """
        # In a real implementation, this would extract joint positions from encoders
        # For now, return a default set of joint states
        return {
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0
        }

    def _get_battery_level_from_sensors(self) -> float:
        """
        Get battery level from sensor data.

        Returns:
            Battery level as a percentage
        """
        # In a real implementation, this would get battery level from power sensors
        # For now, return a simulated battery level
        return 95.0

    def detect_obstacles(self) -> Dict[str, Any]:
        """
        Detect obstacles using sensor data.

        Returns:
            Dictionary with obstacle information
        """
        obstacles = []

        # Process data from different sensors to detect obstacles
        for sensor_name, sensor_data in self.last_sensor_data.items():
            if self.sensors[sensor_name]["type"] == "lidar":
                # Process LIDAR data to detect obstacles
                lidar_data = sensor_data["data"]
                # In a real implementation, this would process the actual LIDAR data
                obstacles.extend(self._process_lidar_data_for_obstacles(lidar_data))

        return {
            "obstacles": obstacles,
            "timestamp": datetime.now()
        }

    def _process_lidar_data_for_obstacles(self, lidar_data: Dict[str, Any]) -> list:
        """
        Process LIDAR data to detect obstacles.

        Args:
            lidar_data: Raw LIDAR data

        Returns:
            List of detected obstacles
        """
        # In a real implementation, this would process the actual LIDAR point cloud
        # For now, return a simulated list of obstacles
        return [
            {"id": "obstacle_1", "distance": 1.5, "angle": 45.0, "size": "medium"},
            {"id": "obstacle_2", "distance": 2.3, "angle": -30.0, "size": "small"}
        ]

    def detect_objects(self, target_class: Optional[str] = None) -> Dict[str, Any]:
        """
        Detect objects using sensor data (typically camera data).

        Args:
            target_class: Optional specific object class to look for

        Returns:
            Dictionary with object detection results
        """
        objects = []

        # Process data from different sensors to detect objects
        for sensor_name, sensor_data in self.last_sensor_data.items():
            if self.sensors[sensor_name]["type"] == "camera":
                # Process camera data to detect objects
                camera_data = sensor_data["data"]
                # In a real implementation, this would process the actual camera image
                objects.extend(self._process_camera_data_for_objects(camera_data, target_class))

        return {
            "objects": objects,
            "timestamp": datetime.now()
        }

    def _process_camera_data_for_objects(self, camera_data: Dict[str, Any], target_class: Optional[str]) -> list:
        """
        Process camera data to detect objects.

        Args:
            camera_data: Raw camera data
            target_class: Optional specific object class to look for

        Returns:
            List of detected objects
        """
        # In a real implementation, this would run object detection on the actual image
        # For now, return a simulated list of objects
        return [
            {"id": "object_1", "class": "cup", "confidence": 0.9, "position": {"x": 1.0, "y": 0.5, "z": 0.8}},
            {"id": "object_2", "class": "box", "confidence": 0.85, "position": {"x": 2.0, "y": -0.2, "z": 0.6}}
        ]

    def update_execution_context(self, execution_context: ExecutionContext) -> ExecutionContext:
        """
        Update execution context with current sensor information.

        Args:
            execution_context: The execution context to update

        Returns:
            Updated execution context
        """
        # Get current sensor data
        sensor_data = self.get_all_sensor_data()

        # Detect obstacles and objects
        obstacles = self.detect_obstacles()
        objects = self.detect_objects()

        # Update execution context with perception data
        execution_context.obstacles = obstacles["obstacles"]
        execution_context.feedback = {
            "sensor_data": sensor_data,
            "obstacles": obstacles,
            "objects": objects,
            "timestamp": datetime.now()
        }

        return execution_context

    def is_sensor_active(self, sensor_name: str) -> bool:
        """
        Check if a sensor is currently active.

        Args:
            sensor_name: Name of the sensor to check

        Returns:
            True if sensor is active, False otherwise
        """
        if sensor_name not in self.sensors:
            return False
        return self.sensors[sensor_name]["active"]

    def activate_sensor(self, sensor_name: str) -> bool:
        """
        Activate a sensor.

        Args:
            sensor_name: Name of the sensor to activate

        Returns:
            True if activation was successful, False otherwise
        """
        if sensor_name not in self.sensors:
            return False
        self.sensors[sensor_name]["active"] = True
        return True

    def deactivate_sensor(self, sensor_name: str) -> bool:
        """
        Deactivate a sensor.

        Args:
            sensor_name: Name of the sensor to deactivate

        Returns:
            True if deactivation was successful, False otherwise
        """
        if sensor_name not in self.sensors:
            return False
        self.sensors[sensor_name]["active"] = False
        return True