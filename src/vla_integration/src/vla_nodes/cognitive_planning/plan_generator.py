from typing import Dict, Any, List, Optional
from datetime import datetime
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.action_step import ActionStep, ActionType
from ..models.robot_state import RobotState
from .llm_service import LLMService
from .plan_validator import PlanValidator
from .action_schema import ActionSchema
from .ambiguity_handler import AmbiguityHandler


class PlanGenerator:
    """
    Generates action plans from natural language commands using LLM and validation.
    """
    def __init__(self, llm_service: LLMService, validator: PlanValidator):
        self.llm_service = llm_service
        self.validator = validator
        self.ambiguity_handler = AmbiguityHandler(llm_service)
        self.action_schema = ActionSchema()

    def generate_plan(self, command_text: str, robot_state: Optional[RobotState] = None) -> Optional[ActionPlan]:
        """
        Generate an action plan from a natural language command.

        Args:
            command_text: The natural language command
            robot_state: Optional current state of the robot

        Returns:
            An ActionPlan object or None if generation fails
        """
        # Check for ambiguity in the command
        is_ambiguous, ambiguity_sources = self.ambiguity_handler.detect_ambiguity(command_text)

        if is_ambiguous:
            # Try to resolve ambiguity
            resolved_command, was_resolved = self.ambiguity_handler.resolve_ambiguity(
                command_text,
                context=self._extract_context_from_state(robot_state) if robot_state else None
            )

            if not was_resolved:
                # If we can't resolve the ambiguity, we might want to request clarification
                # For now, we'll proceed with the original command
                pass
            else:
                command_text = resolved_command

        # Generate the plan using LLM
        action_plan = self.llm_service.generate_action_plan(command_text,
                                                          self._extract_context_from_state(robot_state) if robot_state else None)

        if not action_plan:
            return None

        # Validate the generated plan
        is_valid, validation_errors = self.validator.validate_plan(action_plan, robot_state)

        if not is_valid:
            print(f"Generated plan failed validation: {validation_errors}")
            # Try to fix the plan or return None
            fixed_plan = self._attempt_plan_fix(action_plan, validation_errors)
            if fixed_plan:
                # Re-validate the fixed plan
                is_fixed_valid, fixed_errors = self.validator.validate_plan(fixed_plan, robot_state)
                if is_fixed_valid:
                    return fixed_plan
            return None

        return action_plan

    def _extract_context_from_state(self, robot_state: RobotState) -> Dict[str, Any]:
        """
        Extract relevant context information from the robot state.

        Args:
            robot_state: The current robot state

        Returns:
            A dictionary with context information
        """
        context = {
            "position": {
                "x": robot_state.position.x if robot_state.position else 0,
                "y": robot_state.position.y if robot_state.position else 0,
                "z": robot_state.position.z if robot_state.position else 0
            } if robot_state.position else {},
            "orientation": {
                "x": robot_state.orientation.x if robot_state.orientation else 0,
                "y": robot_state.orientation.y if robot_state.orientation else 0,
                "z": robot_state.orientation.z if robot_state.orientation else 0,
                "w": robot_state.orientation.w if robot_state.orientation else 1
            } if robot_state.orientation else {},
            "battery_level": robot_state.battery_level,
            "current_task": robot_state.current_task,
            "timestamp": robot_state.timestamp.isoformat() if hasattr(robot_state.timestamp, 'isoformat') else str(robot_state.timestamp)
        }

        if robot_state.joint_states:
            context["joint_states"] = robot_state.joint_states

        if robot_state.active_sensors:
            context["active_sensors"] = robot_state.active_sensors

        return context

    def _attempt_plan_fix(self, plan: ActionPlan, validation_errors: List[str]) -> Optional[ActionPlan]:
        """
        Attempt to fix common validation errors in the action plan.

        Args:
            plan: The action plan to fix
            validation_errors: List of validation errors

        Returns:
            A fixed ActionPlan or None if fixing is not possible
        """
        # For now, we'll return the original plan
        # In a more sophisticated implementation, we could try to fix common issues
        # like invalid dependencies, missing parameters, etc.
        return plan

    def generate_simple_navigation_plan(self, target_location: str) -> ActionPlan:
        """
        Generate a simple navigation plan to a target location.

        Args:
            target_location: The target location

        Returns:
            An ActionPlan for navigation
        """
        from uuid import uuid4

        navigation_step = ActionStep(
            id=str(uuid4()),
            plan_id=str(uuid4()),  # Will be updated after plan creation
            step_number=0,
            action_type=ActionType.NAVIGATION,
            parameters={"target": target_location},
            dependencies=[],
            timeout=60,  # 60 seconds for navigation
            retry_count=0
        )

        plan = ActionPlan(
            id=str(uuid4()),
            command_id=str(uuid4()),
            steps=[navigation_step],
            created_at=datetime.now(),
            status=ActionPlanStatus.PLANNED,
            execution_context={"original_command": f"Go to {target_location}"}
        )

        # Update the plan_id in the step
        plan.steps[0].plan_id = plan.id

        return plan

    def generate_simple_manipulation_plan(self, target_object: str) -> ActionPlan:
        """
        Generate a simple manipulation plan for a target object.

        Args:
            target_object: The target object to manipulate

        Returns:
            An ActionPlan for manipulation
        """
        from uuid import uuid4

        steps = [
            ActionStep(
                id=str(uuid4()),
                plan_id=str(uuid4()),  # Will be updated after plan creation
                step_number=0,
                action_type=ActionType.PERCEPTION,
                parameters={"target": target_object},
                dependencies=[],
                timeout=30,
                retry_count=0
            ),
            ActionStep(
                id=str(uuid4()),
                plan_id="",  # Will be updated after plan creation
                step_number=1,
                action_type=ActionType.MANIPULATION,
                parameters={"target": target_object},
                dependencies=[],
                timeout=30,
                retry_count=0
            )
        ]

        plan = ActionPlan(
            id=str(uuid4()),
            command_id=str(uuid4()),
            steps=steps,
            created_at=datetime.now(),
            status=ActionPlanStatus.PLANNED,
            execution_context={"original_command": f"Manipulate {target_object}"}
        )

        # Update the plan_id in the steps
        for step in plan.steps:
            step.plan_id = plan.id
            # Set dependencies properly (second step depends on first)
            if step.step_number == 1:
                step.dependencies = [plan.steps[0].id]

        return plan

    def validate_and_optimize_plan(self, plan: ActionPlan, robot_state: Optional[RobotState] = None) -> Optional[ActionPlan]:
        """
        Validate and potentially optimize an action plan.

        Args:
            plan: The action plan to validate and optimize
            robot_state: Optional current state of the robot

        Returns:
            An optimized ActionPlan or None if validation fails
        """
        # Validate the plan
        is_valid, validation_errors = self.validator.validate_plan(plan, robot_state)

        if not is_valid:
            print(f"Plan validation failed: {validation_errors}")
            return None

        # Optimization could include:
        # - Combining similar sequential steps
        # - Reordering steps for efficiency
        # - Adjusting timeouts based on context
        optimized_plan = self._optimize_plan_steps(plan)

        return optimized_plan

    def _optimize_plan_steps(self, plan: ActionPlan) -> ActionPlan:
        """
        Apply optimizations to the plan steps.

        Args:
            plan: The action plan to optimize

        Returns:
            An optimized ActionPlan
        """
        # For now, we'll return the plan as is
        # In a more sophisticated implementation, we could optimize step sequences
        return plan