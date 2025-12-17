from typing import Dict, Any, List, Optional, Tuple
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.action_step import ActionStep, ActionType
from ..models.robot_state import RobotState
from .action_schema import ActionSchema
from ..utils.error_handlers import VLAError


class PlanValidationError(VLAError):
    """Exception raised when an action plan fails validation."""
    pass


class PlanValidator:
    """
    Validates action plans against robot capabilities and constraints.
    """
    def __init__(self):
        self.action_schema = ActionSchema()
        # Define robot capabilities and constraints
        self.capabilities = {
            "max_navigation_distance": 100.0,  # meters
            "max_manipulation_reach": 1.0,     # meters
            "max_payload": 5.0,                # kg
            "max_execution_time": 300,         # seconds
            "supported_action_types": [ActionType.NAVIGATION, ActionType.MANIPULATION,
                                     ActionType.PERCEPTION, ActionType.COMMUNICATION]
        }

    def validate_plan(self, plan: ActionPlan, robot_state: Optional[RobotState] = None) -> Tuple[bool, List[str]]:
        """
        Validate an action plan against robot capabilities and constraints.

        Args:
            plan: The action plan to validate
            robot_state: Optional current state of the robot

        Returns:
            Tuple of (is_valid, list_of_error_messages)
        """
        errors = []

        # Validate against schema
        if not self.action_schema.validate_action_plan(plan):
            errors.append("Action plan does not conform to schema")
            return False, errors

        # Check if plan has steps
        if not plan.steps:
            errors.append("Action plan must have at least one step")
            return False, errors

        # Validate each step
        for i, step in enumerate(plan.steps):
            step_errors = self._validate_action_step(step, robot_state)
            if step_errors:
                errors.extend([f"Step {i}: {error}" for error in step_errors])

        # Validate plan dependencies
        dependency_errors = self._validate_dependencies(plan)
        if dependency_errors:
            errors.extend(dependency_errors)

        # Validate plan sequence logic
        sequence_errors = self._validate_sequence_logic(plan, robot_state)
        if sequence_errors:
            errors.extend(sequence_errors)

        return len(errors) == 0, errors

    def _validate_action_step(self, step: ActionStep, robot_state: Optional[RobotState] = None) -> List[str]:
        """
        Validate a single action step.

        Args:
            step: The action step to validate
            robot_state: Optional current state of the robot

        Returns:
            List of validation error messages
        """
        errors = []

        # Check if action type is supported
        if step.action_type not in self.capabilities["supported_action_types"]:
            errors.append(f"Action type '{step.action_type}' is not supported")

        # Validate action-specific parameters
        param_errors = self.action_schema.validate_action_parameters(step.action_type, step.parameters)
        errors.extend(param_errors)

        # Validate timeout
        if step.timeout <= 0:
            errors.append("Step timeout must be positive")

        # Validate step number
        if step.step_number < 0:
            errors.append("Step number must be non-negative")

        # Validate dependencies exist
        for dep_id in step.dependencies:
            if not self._dependency_exists(dep_id, step.plan_id):
                errors.append(f"Dependency '{dep_id}' does not exist")

        # Additional validation based on robot state if provided
        if robot_state:
            state_errors = self._validate_against_robot_state(step, robot_state)
            errors.extend(state_errors)

        return errors

    def _validate_against_robot_state(self, step: ActionStep, robot_state: RobotState) -> List[str]:
        """
        Validate an action step against the current robot state.

        Args:
            step: The action step to validate
            robot_state: Current state of the robot

        Returns:
            List of validation error messages
        """
        errors = []

        # Check if robot has sufficient battery for the action
        if robot_state.battery_level < 10 and step.action_type in [ActionType.NAVIGATION, ActionType.MANIPULATION]:
            errors.append("Insufficient battery level for navigation/manipulation action")

        # Additional state-based validations can be added here

        return errors

    def _validate_dependencies(self, plan: ActionPlan) -> List[str]:
        """
        Validate that all dependencies in the plan refer to existing steps.

        Args:
            plan: The action plan to validate

        Returns:
            List of validation error messages
        """
        errors = []

        # Create a set of all step IDs in the plan
        step_ids = {step.id for step in plan.steps}

        # Check each step's dependencies
        for step in plan.steps:
            for dep_id in step.dependencies:
                if dep_id not in step_ids:
                    errors.append(f"Step '{step.id}' depends on non-existent step '{dep_id}'")

        return errors

    def _validate_sequence_logic(self, plan: ActionPlan, robot_state: Optional[RobotState] = None) -> List[str]:
        """
        Validate the logical sequence of actions in the plan.

        Args:
            plan: The action plan to validate
            robot_state: Optional current state of the robot

        Returns:
            List of validation error messages
        """
        errors = []

        # Check for circular dependencies
        if self._has_circular_dependencies(plan):
            errors.append("Plan contains circular dependencies")

        # Check if steps are in logical order based on dependencies
        sorted_steps = self._topological_sort(plan.steps)
        if sorted_steps is None:
            errors.append("Plan contains circular dependencies")

        # Additional sequence logic validation can be added here

        return errors

    def _has_circular_dependencies(self, plan: ActionPlan) -> bool:
        """
        Check if the plan has circular dependencies between steps.

        Args:
            plan: The action plan to check

        Returns:
            True if circular dependencies exist, False otherwise
        """
        # Build dependency graph
        graph = {}
        for step in plan.steps:
            graph[step.id] = step.dependencies[:]

        # Use DFS to detect cycles
        visiting = set()
        visited = set()

        def has_cycle(node):
            if node in visited:
                return False
            if node in visiting:
                return True

            visiting.add(node)
            for dep in graph.get(node, []):
                if has_cycle(dep):
                    return True
            visiting.remove(node)
            visited.add(node)
            return False

        for step in plan.steps:
            if has_cycle(step.id):
                return True

        return False

    def _topological_sort(self, steps: List[ActionStep]) -> Optional[List[ActionStep]]:
        """
        Perform topological sort on steps based on dependencies.

        Args:
            steps: List of action steps

        Returns:
            Sorted list of steps, or None if circular dependencies exist
        """
        # Build dependency graph
        graph = {}
        for step in steps:
            graph[step.id] = step.dependencies[:]

        # Build reverse graph (adjacency list)
        reverse_graph = {step.id: [] for step in steps}
        for step in steps:
            for dep in step.dependencies:
                if dep in reverse_graph:
                    reverse_graph[dep].append(step.id)

        # Kahn's algorithm for topological sorting
        in_degree = {step.id: 0 for step in steps}
        for step in steps:
            for dep in step.dependencies:
                if dep in in_degree:
                    in_degree[step.id] += 1

        # Find nodes with no incoming edges
        queue = []
        for step in steps:
            if in_degree[step.id] == 0:
                queue.append(step)

        result = []
        while queue:
            current = queue.pop(0)
            result.append(current)

            # Reduce in-degree for all neighbors
            for neighbor_id in reverse_graph[current.id]:
                in_degree[neighbor_id] -= 1
                if in_degree[neighbor_id] == 0:
                    # Find the step object for this ID
                    for step in steps:
                        if step.id == neighbor_id:
                            queue.append(step)
                            break

        # If we didn't visit all nodes, there's a cycle
        if len(result) != len(steps):
            return None

        return result

    def _dependency_exists(self, dep_id: str, plan_id: str) -> bool:
        """
        Check if a dependency exists. This is a simplified check.
        In a real implementation, this would check against a database of available steps.

        Args:
            dep_id: The dependency ID to check
            plan_id: The plan ID (for context)

        Returns:
            True if dependency exists, False otherwise
        """
        # In a real implementation, this would check against available steps
        # For now, we'll return True to avoid blocking development
        return True

    def validate_plan_feasibility(self, plan: ActionPlan, robot_state: Optional[RobotState] = None) -> Tuple[bool, str]:
        """
        Validate if the plan is feasible given the robot's current state and capabilities.

        Args:
            plan: The action plan to validate
            robot_state: Optional current state of the robot

        Returns:
            Tuple of (is_feasible, reason)
        """
        # This method can include more complex feasibility checks
        # For now, we'll just run the standard validation
        is_valid, errors = self.validate_plan(plan, robot_state)

        if is_valid:
            return True, "Plan is feasible"
        else:
            return False, "; ".join(errors)