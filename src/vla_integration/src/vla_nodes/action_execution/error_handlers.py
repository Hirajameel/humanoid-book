from typing import Optional, Callable, Any
import functools
import logging
from ..utils.error_handlers import VLAError, VLAErrorCode, handle_vla_error


def handle_action_execution_error(default_return: Any = None):
    """
    Decorator to handle action execution errors specifically.
    """
    return handle_vla_error(VLAErrorCode.ACTION_EXECUTION_ERROR, default_return)


def handle_action_mapping_error(default_return: Any = None):
    """
    Decorator to handle action mapping errors specifically.
    """
    return handle_vla_error(VLAErrorCode.ACTION_EXECUTION_ERROR, default_return)


class ActionExecutionError(VLAError):
    """Exception raised for action execution errors."""
    pass


class ActionMappingError(VLAError):
    """Exception raised for action mapping errors."""
    pass


class ExecutionMonitoringError(VLAError):
    """Exception raised for execution monitoring errors."""
    pass


class ActionExecutionErrorHandler:
    """
    Error handler specifically for action execution components.
    """
    @staticmethod
    def handle_action_execution_error(error: Exception) -> ActionExecutionError:
        """Handle action execution related errors."""
        return ActionExecutionError(f"Action execution error: {str(error)}")

    @staticmethod
    def handle_robot_connection_error(error: Exception) -> ActionExecutionError:
        """Handle robot connection errors."""
        return ActionExecutionError(f"Robot connection error: {str(error)}")

    @staticmethod
    def handle_action_timeout_error(error: Exception) -> ActionExecutionError:
        """Handle action timeout errors."""
        return ActionExecutionError(f"Action timeout error: {str(error)}")

    @staticmethod
    def handle_action_preemption_error(error: Exception) -> ActionExecutionError:
        """Handle action preemption errors."""
        return ActionExecutionError(f"Action preemption error: {str(error)}")

    @staticmethod
    def handle_monitoring_error(error: Exception) -> ExecutionMonitoringError:
        """Handle execution monitoring errors."""
        return ExecutionMonitoringError(f"Execution monitoring error: {str(error)}")

    @staticmethod
    def handle_recovery_error(error: Exception) -> ActionExecutionError:
        """Handle action recovery errors."""
        return ActionExecutionError(f"Action recovery error: {str(error)}")

    @staticmethod
    def handle_sequence_execution_error(error: Exception) -> ActionExecutionError:
        """Handle sequence execution errors."""
        return ActionExecutionError(f"Sequence execution error: {str(error)}")

    @staticmethod
    def handle_ros_action_error(error: Exception) -> ActionExecutionError:
        """Handle ROS action interface errors."""
        return ActionExecutionError(f"ROS action error: {str(error)}")