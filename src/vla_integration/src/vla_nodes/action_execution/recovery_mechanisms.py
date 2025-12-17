from typing import Dict, Any, Optional, List
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.action_step import ActionStep
from ..models.robot_state import RobotState
from ..utils.error_handlers import VLAError


class RecoveryError(VLAError):
    """Exception raised when recovery mechanisms fail."""
    pass


class RecoveryMechanisms:
    """
    Implements various recovery mechanisms for handling failures during action execution.
    """
    def __init__(self):
        self.recovery_strategies = {
            "retry": self._retry_strategy,
            "fallback": self._fallback_strategy,
            "skip": self._skip_strategy,
            "abort": self._abort_strategy,
            "replan": self._replan_strategy
        }

    def handle_step_failure(self, step: ActionStep, error: Exception, robot_state: RobotState,
                           available_recovery_strategies: List[str] = None) -> str:
        """
        Handle a failed action step by applying appropriate recovery strategy.

        Args:
            step: The action step that failed
            error: The error that occurred
            robot_state: Current state of the robot
            available_recovery_strategies: List of available recovery strategies

        Returns:
            The recovery action taken
        """
        if available_recovery_strategies is None:
            available_recovery_strategies = list(self.recovery_strategies.keys())

        # Determine the best recovery strategy based on error type and context
        strategy = self._select_recovery_strategy(step, error, robot_state, available_recovery_strategies)

        # Apply the recovery strategy
        return self.recovery_strategies[strategy](step, error, robot_state)

    def _select_recovery_strategy(self, step: ActionStep, error: Exception, robot_state: RobotState,
                                available_strategies: List[str]) -> str:
        """
        Select the most appropriate recovery strategy based on error type and context.

        Args:
            step: The failed action step
            error: The error that occurred
            robot_state: Current state of the robot
            available_strategies: List of available strategies

        Returns:
            The selected strategy name
        """
        error_str = str(error).lower()

        # If retry is available and this is a temporary error
        if "retry" in available_strategies:
            if any(temp_error in error_str for temp_error in ["timeout", "connection", "network", "temporary"]):
                return "retry"

        # If fallback is available and the action has alternatives
        if "fallback" in available_strategies and self._has_fallback(step):
            return "fallback"

        # If replan is available and the plan is still valid to modify
        if "replan" in available_strategies:
            if step.retry_count < 2:  # Only allow replanning for early failures
                return "replan"

        # If skip is available and the step is not critical
        if "skip" in available_strategies and not self._is_critical_step(step):
            return "skip"

        # Default to abort for critical failures
        return "abort"

    def _has_fallback(self, step: ActionStep) -> bool:
        """
        Check if the step has a fallback action available.

        Args:
            step: The action step to check

        Returns:
            True if fallback is available, False otherwise
        """
        # In a real implementation, this would check for defined fallbacks
        # For now, return True for manipulation and navigation steps
        return step.action_type in ["manipulation", "navigation", "perception"]

    def _is_critical_step(self, step: ActionStep) -> bool:
        """
        Check if the step is critical to the overall plan.

        Args:
            step: The action step to check

        Returns:
            True if step is critical, False otherwise
        """
        # In a real implementation, this would analyze the step's role in the plan
        # For now, return True for communication steps that might be essential
        return step.action_type == "communication" and "essential" in str(step.parameters)

    def _retry_strategy(self, step: ActionStep, error: Exception, robot_state: RobotState) -> str:
        """
        Retry the failed step if retry count is not exceeded.

        Args:
            step: The failed action step
            error: The error that occurred
            robot_state: Current state of the robot

        Returns:
            The recovery action taken
        """
        if step.retry_count < step.timeout:  # Use timeout field to store max retries
            step.retry_count += 1
            return f"retry_{step.retry_count}"
        else:
            raise RecoveryError(f"Retry limit exceeded for step {step.id}")

    def _fallback_strategy(self, step: ActionStep, error: Exception, robot_state: RobotState) -> str:
        """
        Execute a fallback action instead of the failed step.

        Args:
            step: The failed action step
            error: The error that occurred
            robot_state: Current state of the robot

        Returns:
            The recovery action taken
        """
        # In a real implementation, this would execute a predefined fallback
        # For now, return a placeholder
        fallback_action = self._get_fallback_action(step, robot_state)
        if fallback_action:
            return f"fallback_executed:{fallback_action}"
        else:
            raise RecoveryError(f"No fallback available for step {step.id}")

    def _get_fallback_action(self, step: ActionStep, robot_state: RobotState) -> Optional[str]:
        """
        Get an appropriate fallback action for the failed step.

        Args:
            step: The failed action step
            robot_state: Current state of the robot

        Returns:
            Fallback action name or None if not available
        """
        # Define fallbacks based on action type
        fallbacks = {
            "navigation": "move_to_safe_location",
            "manipulation": "release_and_retry",
            "perception": "use_alternative_sensor",
            "communication": "use_backup_communication"
        }

        return fallbacks.get(step.action_type.value if hasattr(step.action_type, 'value') else step.action_type)

    def _skip_strategy(self, step: ActionStep, error: Exception, robot_state: RobotState) -> str:
        """
        Skip the failed step and continue with the plan.

        Args:
            step: The failed action step
            error: The error that occurred
            robot_state: Current state of the robot

        Returns:
            The recovery action taken
        """
        # Mark the step as skipped
        step.status = "skipped"
        return "skipped"

    def _abort_strategy(self, step: ActionStep, error: Exception, robot_state: RobotState) -> str:
        """
        Abort the entire plan execution.

        Args:
            step: The failed action step
            error: The error that occurred
            robot_state: Current state of the robot

        Returns:
            The recovery action taken
        """
        raise RecoveryError(f"Aborting plan execution due to critical failure in step {step.id}: {error}")

    def _replan_strategy(self, step: ActionStep, error: Exception, robot_state: RobotState) -> str:
        """
        Request a new plan based on the current state and remaining objectives.

        Args:
            step: The failed action step
            error: The error that occurred
            robot_state: Current state of the robot

        Returns:
            The recovery action taken
        """
        # In a real implementation, this would trigger the planning system to generate a new plan
        # For now, return a placeholder
        return "replan_requested"

    def handle_execution_failure(self, plan: ActionPlan, error: Exception, robot_state: RobotState) -> ActionPlan:
        """
        Handle failure of the entire plan execution.

        Args:
            plan: The action plan that failed
            error: The error that occurred
            robot_state: Current state of the robot

        Returns:
            A modified plan or None if recovery is not possible
        """
        plan.status = ActionPlanStatus.FAILED

        # Analyze the failure to determine if partial recovery is possible
        completed_steps = [step for step in plan.steps if hasattr(step, 'status') and step.status == 'completed']
        remaining_steps = [step for step in plan.steps if step not in completed_steps]

        if not remaining_steps:
            # All steps completed but plan still failed - return original plan
            return plan

        # Try to create a recovery plan with remaining steps
        recovery_plan = self._create_recovery_plan(plan, remaining_steps, robot_state, error)
        return recovery_plan

    def _create_recovery_plan(self, original_plan: ActionPlan, remaining_steps: List[ActionStep],
                            robot_state: RobotState, error: Exception) -> Optional[ActionPlan]:
        """
        Create a recovery plan with remaining steps adjusted for the current state.

        Args:
            original_plan: The original plan that failed
            remaining_steps: Steps that were not completed
            robot_state: Current state of the robot
            error: The error that caused the failure

        Returns:
            A new ActionPlan for recovery or None if not possible
        """
        from uuid import uuid4
        from datetime import datetime

        if not remaining_steps:
            return None

        # Create a new plan with remaining steps
        recovery_plan = ActionPlan(
            id=str(uuid4()),
            command_id=original_plan.command_id,
            steps=remaining_steps,
            created_at=datetime.now(),
            status=ActionPlanStatus.PLANNED,
            execution_context={
                "original_plan_id": original_plan.id,
                "recovery_from_error": str(error),
                "recovery_type": "partial_continuation"
            }
        )

        # Adjust step dependencies for the new plan
        for i, step in enumerate(recovery_plan.steps):
            step.plan_id = recovery_plan.id
            step.step_number = i

        return recovery_plan

    def handle_robot_state_recovery(self, robot_state: RobotState, error: Exception) -> RobotState:
        """
        Handle recovery at the robot state level.

        Args:
            robot_state: Current robot state
            error: The error that occurred

        Returns:
            A safe robot state
        """
        # Reset robot to a safe state
        safe_state = RobotState(
            id=robot_state.id,
            timestamp=datetime.now(),
            position=robot_state.position,  # Keep position unless unsafe
            orientation=robot_state.orientation,  # Keep orientation
            joint_states=robot_state.joint_states if robot_state.joint_states else {},
            battery_level=robot_state.battery_level,
            active_sensors=robot_state.active_sensors if robot_state.active_sensors else {},
            current_task=None  # Clear current task
        )

        # If the error is related to navigation, move to a safe location
        error_str = str(error).lower()
        if "navigation" in error_str or "collision" in error_str:
            # In a real implementation, this would command the robot to move to a safe location
            pass

        return safe_state