from typing import Dict, Any, List, Optional
import json
from jsonschema import validate, ValidationError
from ..models.action_plan import ActionStep, ActionPlan, ActionPlanStatus
from ..models.action_step import ActionType


class ActionSchema:
    """
    Defines and validates the schema for action plans and steps.
    """
    # Schema for individual action steps
    ACTION_STEP_SCHEMA = {
        "type": "object",
        "properties": {
            "id": {"type": "string"},
            "plan_id": {"type": "string"},
            "step_number": {"type": "integer", "minimum": 0},
            "action_type": {
                "type": "string",
                "enum": ["navigation", "manipulation", "perception", "communication"]
            },
            "parameters": {
                "type": "object",
                "properties": {
                    "target": {"type": ["string", "null"]},
                    "value": {"type": ["string", "number", "null"]},
                    "duration": {"type": ["number", "null"], "minimum": 0},
                    "position": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"}
                        },
                        "required": ["x", "y", "z"]
                    },
                    "orientation": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"},
                            "w": {"type": "number"}
                        },
                        "required": ["x", "y", "z", "w"]
                    }
                }
            },
            "dependencies": {
                "type": "array",
                "items": {"type": "string"}
            },
            "timeout": {"type": "integer", "minimum": 1},
            "retry_count": {"type": "integer", "minimum": 0, "maximum": 10}
        },
        "required": ["id", "plan_id", "step_number", "action_type", "parameters", "timeout"]
    }

    # Schema for complete action plans
    ACTION_PLAN_SCHEMA = {
        "type": "object",
        "properties": {
            "id": {"type": "string"},
            "command_id": {"type": "string"},
            "description": {"type": "string"},
            "steps": {
                "type": "array",
                "items": ACTION_STEP_SCHEMA
            },
            "status": {
                "type": "string",
                "enum": ["planned", "executing", "completed", "failed", "cancelled"]
            },
            "created_at": {"type": "string", "format": "date-time"},
            "updated_at": {"type": ["string", "null"], "format": "date-time"},
            "execution_context": {"type": "object"}
        },
        "required": ["id", "command_id", "steps", "status", "created_at"]
    }

    @staticmethod
    def validate_action_step(step: ActionStep) -> bool:
        """
        Validate an ActionStep object against the schema.

        Args:
            step: The ActionStep to validate

        Returns:
            True if valid, False otherwise
        """
        try:
            step_dict = {
                "id": step.id,
                "plan_id": step.plan_id,
                "step_number": step.step_number,
                "action_type": step.action_type.value if hasattr(step.action_type, 'value') else step.action_type,
                "parameters": step.parameters,
                "dependencies": step.dependencies,
                "timeout": step.timeout,
                "retry_count": step.retry_count
            }
            validate(instance=step_dict, schema=ActionSchema.ACTION_STEP_SCHEMA)
            return True
        except ValidationError as e:
            print(f"ActionStep validation error: {e.message}")
            return False

    @staticmethod
    def validate_action_plan(plan: ActionPlan) -> bool:
        """
        Validate an ActionPlan object against the schema.

        Args:
            plan: The ActionPlan to validate

        Returns:
            True if valid, False otherwise
        """
        try:
            # Convert plan to dictionary
            steps_dict = []
            for step in plan.steps:
                steps_dict.append({
                    "id": step.id,
                    "plan_id": step.plan_id,
                    "step_number": step.step_number,
                    "action_type": step.action_type.value if hasattr(step.action_type, 'value') else step.action_type,
                    "parameters": step.parameters,
                    "dependencies": step.dependencies,
                    "timeout": step.timeout,
                    "retry_count": step.retry_count
                })

            plan_dict = {
                "id": plan.id,
                "command_id": plan.command_id,
                "steps": steps_dict,
                "status": plan.status.value if hasattr(plan.status, 'value') else plan.status,
                "created_at": plan.created_at.isoformat() if hasattr(plan.created_at, 'isoformat') else str(plan.created_at),
                "updated_at": plan.updated_at.isoformat() if plan.updated_at and hasattr(plan.updated_at, 'isoformat') else None,
                "execution_context": plan.execution_context
            }

            validate(instance=plan_dict, schema=ActionSchema.ACTION_PLAN_SCHEMA)
            return True
        except ValidationError as e:
            print(f"ActionPlan validation error: {e.message}")
            return False

    @staticmethod
    def get_default_action_step() -> Dict[str, Any]:
        """
        Get a default action step template.

        Returns:
            A dictionary with default action step values
        """
        return {
            "id": "",
            "plan_id": "",
            "step_number": 0,
            "action_type": "communication",
            "parameters": {},
            "dependencies": [],
            "timeout": 30,
            "retry_count": 0
        }

    @staticmethod
    def get_default_action_plan() -> Dict[str, Any]:
        """
        Get a default action plan template.

        Returns:
            A dictionary with default action plan values
        """
        return {
            "id": "",
            "command_id": "",
            "steps": [],
            "status": "planned",
            "created_at": "",
            "updated_at": None,
            "execution_context": {}
        }

    @staticmethod
    def validate_action_parameters(action_type: ActionType, parameters: Dict[str, Any]) -> List[str]:
        """
        Validate action-specific parameters based on action type.

        Args:
            action_type: The type of action
            parameters: The parameters to validate

        Returns:
            A list of validation error messages (empty if valid)
        """
        errors = []

        if action_type == ActionType.NAVIGATION:
            # Navigation actions typically need a target location
            if "target" not in parameters and "position" not in parameters:
                errors.append("Navigation action requires 'target' or 'position' parameter")

        elif action_type == ActionType.MANIPULATION:
            # Manipulation actions typically need a target object
            if "target" not in parameters:
                errors.append("Manipulation action requires 'target' parameter")

        elif action_type == ActionType.PERCEPTION:
            # Perception actions might need a target to look at
            if "target" not in parameters:
                errors.append("Perception action requires 'target' parameter")

        elif action_type == ActionType.COMMUNICATION:
            # Communication actions need content to speak
            if "value" not in parameters or not parameters["value"]:
                errors.append("Communication action requires 'value' parameter with content to speak")

        return errors