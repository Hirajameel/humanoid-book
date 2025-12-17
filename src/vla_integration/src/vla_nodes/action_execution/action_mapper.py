from typing import Dict, Any, Optional
from ..models.action_step import ActionStep, ActionType
from ..models.robot_state import RobotState
from ..utils.error_handlers import VLAError


class ActionMappingError(VLAError):
    """Exception raised when action mapping fails."""
    pass


class ActionMapper:
    """
    Maps action steps to specific ROS 2 actions that can be executed by the robot.
    """
    def __init__(self):
        # Define mapping from action types to ROS 2 action names
        self.action_mapping = {
            ActionType.NAVIGATION: {
                "action_name": "NavigateToPose",
                "package": "nav2_msgs",
                "default_parameters": {
                    "yaw_goal_tolerance": 0.3,
                    "xy_goal_tolerance": 0.3,
                    "max_velocity": 0.5,
                    "min_velocity": 0.1
                }
            },
            ActionType.MANIPULATION: {
                "action_name": "MoveGroup",
                "package": "moveit_msgs",
                "default_parameters": {
                    "end_effector": "gripper",
                    "max_effort": 100.0,
                    "velocity_scaling": 0.5
                }
            },
            ActionType.PERCEPTION: {
                "action_name": "ObjectDetection",
                "package": "vision_msgs",
                "default_parameters": {
                    "detection_timeout": 10.0,
                    "confidence_threshold": 0.7
                }
            },
            ActionType.COMMUNICATION: {
                "action_name": "Speak",
                "package": "sound_play_msgs",
                "default_parameters": {
                    "voice": "default",
                    "volume": 1.0
                }
            }
        }

    def map_action_step(self, step: ActionStep, robot_state: Optional[RobotState] = None) -> Dict[str, Any]:
        """
        Map an ActionStep to specific ROS 2 action parameters.

        Args:
            step: The action step to map
            robot_state: Optional current state of the robot

        Returns:
            A dictionary with ROS 2 action parameters
        """
        if step.action_type not in self.action_mapping:
            raise ActionMappingError(f"Action type '{step.action_type}' is not supported")

        mapping_info = self.action_mapping[step.action_type]

        # Start with default parameters for this action type
        action_params = mapping_info["default_parameters"].copy()

        # Override with step-specific parameters
        action_params.update(step.parameters)

        # Add metadata about the action
        ros_action = {
            "action_name": mapping_info["action_name"],
            "package": mapping_info["package"],
            "parameters": action_params,
            "step_id": step.id,
            "timeout": step.timeout,
            "original_step": step
        }

        # Apply robot state-specific modifications if provided
        if robot_state:
            ros_action = self._apply_robot_state_modifications(ros_action, step, robot_state)

        return ros_action

    def _apply_robot_state_modifications(self, ros_action: Dict[str, Any], step: ActionStep, robot_state: RobotState) -> Dict[str, Any]:
        """
        Apply modifications to the ROS action based on the current robot state.

        Args:
            ros_action: The ROS action dictionary to modify
            step: The original action step
            robot_state: Current state of the robot

        Returns:
            Modified ROS action dictionary
        """
        # Adjust parameters based on battery level for energy-intensive actions
        if robot_state.battery_level < 20:
            if step.action_type in [ActionType.NAVIGATION, ActionType.MANIPULATION]:
                # Reduce speed for energy conservation
                if "max_velocity" in ros_action["parameters"]:
                    ros_action["parameters"]["max_velocity"] *= 0.7
                if "velocity_scaling" in ros_action["parameters"]:
                    ros_action["parameters"]["velocity_scaling"] *= 0.7

        # Additional state-based modifications can be added here

        return ros_action

    def get_available_actions(self) -> Dict[ActionType, str]:
        """
        Get a mapping of available action types to their ROS 2 action names.

        Returns:
            Dictionary mapping action types to action names
        """
        return {action_type: info["action_name"] for action_type, info in self.action_mapping.items()}

    def validate_action_compatibility(self, step: ActionStep, robot_capabilities: Dict[str, Any]) -> bool:
        """
        Validate if the action step is compatible with the robot's capabilities.

        Args:
            step: The action step to validate
            robot_capabilities: Dictionary of robot capabilities

        Returns:
            True if action is compatible, False otherwise
        """
        # Check if action type is supported
        if step.action_type not in self.action_mapping:
            return False

        # Check action-specific constraints
        if step.action_type == ActionType.NAVIGATION:
            # Check if target is within navigation range
            max_distance = robot_capabilities.get("max_navigation_distance", 100.0)
            target_distance = self._get_target_distance(step.parameters, robot_capabilities)
            if target_distance and target_distance > max_distance:
                return False

        elif step.action_type == ActionType.MANIPULATION:
            # Check if target object is within reach
            max_reach = robot_capabilities.get("max_manipulation_reach", 1.0)
            target_distance = self._get_target_distance(step.parameters, robot_capabilities)
            if target_distance and target_distance > max_reach:
                return False

            # Check payload constraints
            payload = step.parameters.get("payload", 0.0)
            max_payload = robot_capabilities.get("max_payload", 5.0)
            if payload > max_payload:
                return False

        return True

    def _get_target_distance(self, parameters: Dict[str, Any], robot_capabilities: Dict[str, Any]) -> Optional[float]:
        """
        Estimate the distance to the target based on parameters.

        Args:
            parameters: Action parameters
            robot_capabilities: Robot capabilities

        Returns:
            Estimated distance or None if not applicable
        """
        # This is a simplified implementation
        # In a real system, this would calculate actual distances
        # based on current position and target location
        return None

    def create_fallback_action(self, step: ActionStep) -> Optional[Dict[str, Any]]:
        """
        Create a fallback action for when the primary action fails.

        Args:
            step: The original action step

        Returns:
            Fallback action dictionary or None if no fallback is possible
        """
        # For now, return None - in a real system this would provide
        # alternative actions for each action type
        return None