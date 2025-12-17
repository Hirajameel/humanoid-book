from typing import List, Dict, Any, Optional, Callable
from datetime import datetime, timedelta
import asyncio
import threading
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.action_step import ActionStep
from ..models.robot_state import RobotState
from .action_mapper import ActionMapper
from ..utils.error_handlers import VLAError, retry_on_failure
from .error_handlers import ActionExecutionError


class SequenceExecutor:
    """
    Executes sequences of actions according to an action plan, handling dependencies and monitoring progress.
    """
    def __init__(self, action_mapper: ActionMapper):
        self.action_mapper = action_mapper
        self.active_executions = {}  # plan_id -> execution state
        self.execution_callbacks = {}  # plan_id -> callback functions
        self.lock = threading.Lock()

    def execute_plan(self, plan: ActionPlan, robot_state: Optional[RobotState] = None,
                    on_step_complete: Optional[Callable] = None,
                    on_plan_complete: Optional[Callable] = None,
                    on_error: Optional[Callable] = None) -> bool:
        """
        Execute an action plan asynchronously.

        Args:
            plan: The action plan to execute
            robot_state: Optional current state of the robot
            on_step_complete: Callback function called when each step completes
            on_plan_complete: Callback function called when the plan completes
            on_error: Callback function called when an error occurs

        Returns:
            True if execution started successfully, False otherwise
        """
        with self.lock:
            if plan.id in self.active_executions:
                raise ActionExecutionError(f"Plan {plan.id} is already being executed")

            # Set up callbacks
            self.execution_callbacks[plan.id] = {
                "on_step_complete": on_step_complete,
                "on_plan_complete": on_plan_complete,
                "on_error": on_error
            }

            # Initialize execution state
            execution_state = {
                "plan": plan,
                "robot_state": robot_state,
                "start_time": datetime.now(),
                "current_step_index": 0,
                "completed_steps": set(),
                "status": "running",
                "results": {}
            }

            self.active_executions[plan.id] = execution_state

        # Execute the plan in a separate thread
        execution_thread = threading.Thread(
            target=self._execute_plan_thread,
            args=(plan.id, execution_state)
        )
        execution_thread.start()

        return True

    def _execute_plan_thread(self, plan_id: str, execution_state: Dict[str, Any]):
        """
        Execute the plan in a separate thread.
        """
        try:
            result = self._execute_plan_steps(plan_id, execution_state)
            with self.lock:
                if plan_id in self.active_executions:
                    self.active_executions[plan_id]["status"] = "completed" if result else "failed"
        except Exception as e:
            with self.lock:
                if plan_id in self.active_executions:
                    self.active_executions[plan_id]["status"] = "error"
            self._call_error_callback(plan_id, e)

    def _execute_plan_steps(self, plan_id: str, execution_state: Dict[str, Any]) -> bool:
        """
        Execute the steps of the plan in order, respecting dependencies.

        Args:
            plan_id: The ID of the plan to execute
            execution_state: The current execution state

        Returns:
            True if all steps completed successfully, False otherwise
        """
        plan = execution_state["plan"]
        robot_state = execution_state["robot_state"]

        # Sort steps by dependencies to ensure proper execution order
        sorted_steps = self._sort_steps_by_dependencies(plan.steps)

        if sorted_steps is None:
            error = ActionExecutionError(f"Plan {plan_id} has circular dependencies")
            self._call_error_callback(plan_id, error)
            return False

        # Execute each step
        for step in sorted_steps:
            if not self._execute_single_step(plan_id, step, robot_state, execution_state):
                # Step failed, decide whether to continue based on plan configuration
                if hasattr(plan, 'fail_fast') and plan.fail_fast:
                    plan.status = ActionPlanStatus.FAILED
                    self._call_error_callback(plan_id, ActionExecutionError(f"Step {step.id} failed and fail_fast is enabled"))
                    return False
                else:
                    # Continue with the next steps
                    continue

            # Update completed steps
            execution_state["completed_steps"].add(step.id)

            # Call step completion callback
            self._call_step_callback(plan_id, step)

        # All steps completed successfully
        plan.status = ActionPlanStatus.COMPLETED
        self._call_plan_callback(plan_id, plan)
        return True

    def _sort_steps_by_dependencies(self, steps: List[ActionStep]) -> Optional[List[ActionStep]]:
        """
        Sort steps by dependencies using topological sort.

        Args:
            steps: List of action steps

        Returns:
            Sorted list of steps, or None if circular dependencies exist
        """
        # Build dependency graph
        graph = {}
        step_map = {step.id: step for step in steps}

        for step in steps:
            graph[step.id] = step.dependencies[:]

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
            for neighbor_id in graph[current.id]:
                if neighbor_id in in_degree:
                    in_degree[neighbor_id] -= 1
                    if in_degree[neighbor_id] == 0:
                        # Find the step object for this ID
                        if neighbor_id in step_map:
                            queue.append(step_map[neighbor_id])

        # If we didn't visit all nodes, there's a cycle
        if len(result) != len(steps):
            return None

        return result

    def _execute_single_step(self, plan_id: str, step: ActionStep, robot_state: Optional[RobotState],
                           execution_state: Dict[str, Any]) -> bool:
        """
        Execute a single action step.

        Args:
            plan_id: The ID of the plan containing the step
            step: The action step to execute
            robot_state: Current state of the robot
            execution_state: The current execution state

        Returns:
            True if step completed successfully, False otherwise
        """
        try:
            # Map the action step to ROS 2 action
            ros_action = self.action_mapper.map_action_step(step, robot_state)

            # Check if action is compatible with robot capabilities
            robot_capabilities = self._get_robot_capabilities(robot_state)
            if not self.action_mapper.validate_action_compatibility(step, robot_capabilities):
                raise ActionExecutionError(f"Action {step.id} is not compatible with robot capabilities")

            # Execute the action with retry logic
            success = self._execute_ros_action_with_retry(ros_action, step)

            # Update execution state with result
            execution_state["results"][step.id] = {
                "success": success,
                "timestamp": datetime.now(),
                "step": step
            }

            return success

        except Exception as e:
            self._call_error_callback(plan_id, e)
            return False

    @retry_on_failure(max_retries=3, delay=1.0)
    def _execute_ros_action_with_retry(self, ros_action: Dict[str, Any], step: ActionStep) -> bool:
        """
        Execute a ROS action with retry logic.

        Args:
            ros_action: The ROS action to execute
            step: The original action step

        Returns:
            True if action completed successfully, False otherwise
        """
        # In a real implementation, this would interface with ROS 2 to execute the action
        # For now, we'll simulate the execution
        import time
        print(f"Executing action: {ros_action['action_name']} with parameters: {ros_action['parameters']}")
        time.sleep(min(2, ros_action.get('timeout', 5)))  # Simulate action execution time
        return True  # Simulate success

    def _get_robot_capabilities(self, robot_state: Optional[RobotState]) -> Dict[str, Any]:
        """
        Get robot capabilities based on state or default values.

        Args:
            robot_state: Optional robot state

        Returns:
            Dictionary of robot capabilities
        """
        # Default capabilities
        capabilities = {
            "max_navigation_distance": 100.0,
            "max_manipulation_reach": 1.0,
            "max_payload": 5.0,
            "max_execution_time": 300
        }

        # Update with any state-specific information
        if robot_state and robot_state.battery_level:
            # Adjust capabilities based on battery level
            battery_factor = robot_state.battery_level / 100.0
            capabilities["max_navigation_distance"] *= battery_factor

        return capabilities

    def cancel_execution(self, plan_id: str) -> bool:
        """
        Cancel the execution of a plan.

        Args:
            plan_id: The ID of the plan to cancel

        Returns:
            True if cancellation was successful, False otherwise
        """
        with self.lock:
            if plan_id not in self.active_executions:
                return False

            execution_state = self.active_executions[plan_id]
            execution_state["status"] = "cancelled"

            # Update plan status
            execution_state["plan"].status = ActionPlanStatus.CANCELLED

            # Remove from active executions
            del self.active_executions[plan_id]

            # Remove callbacks
            if plan_id in self.execution_callbacks:
                del self.execution_callbacks[plan_id]

            return True

    def get_execution_status(self, plan_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the current status of a plan execution.

        Args:
            plan_id: The ID of the plan

        Returns:
            Execution status dictionary or None if plan is not being executed
        """
        with self.lock:
            if plan_id not in self.active_executions:
                return None

            execution_state = self.active_executions[plan_id]
            return {
                "plan_id": plan_id,
                "status": execution_state["status"],
                "completed_steps": len(execution_state["completed_steps"]),
                "total_steps": len(execution_state["plan"].steps),
                "start_time": execution_state["start_time"],
                "elapsed_time": datetime.now() - execution_state["start_time"],
                "current_step_index": execution_state["current_step_index"]
            }

    def _call_step_callback(self, plan_id: str, step: ActionStep):
        """
        Call the step completion callback if it exists.
        """
        with self.lock:
            if plan_id in self.execution_callbacks:
                callback = self.execution_callbacks[plan_id].get("on_step_complete")
                if callback:
                    try:
                        callback(plan_id, step)
                    except Exception as e:
                        print(f"Error in step callback: {e}")

    def _call_plan_callback(self, plan_id: str, plan: ActionPlan):
        """
        Call the plan completion callback if it exists.
        """
        with self.lock:
            if plan_id in self.execution_callbacks:
                callback = self.execution_callbacks[plan_id].get("on_plan_complete")
                if callback:
                    try:
                        callback(plan_id, plan)
                    except Exception as e:
                        print(f"Error in plan callback: {e}")

    def _call_error_callback(self, plan_id: str, error: Exception):
        """
        Call the error callback if it exists.
        """
        with self.lock:
            if plan_id in self.execution_callbacks:
                callback = self.execution_callbacks[plan_id].get("on_error")
                if callback:
                    try:
                        callback(plan_id, error)
                    except Exception as e:
                        print(f"Error in error callback: {e}")

    def cleanup_completed_executions(self):
        """
        Clean up completed or failed executions to free up resources.
        """
        with self.lock:
            completed_plans = []
            for plan_id, execution_state in self.active_executions.items():
                if execution_state["status"] in ["completed", "failed", "cancelled", "error"]:
                    completed_plans.append(plan_id)

            for plan_id in completed_plans:
                del self.active_executions[plan_id]
                if plan_id in self.execution_callbacks:
                    del self.execution_callbacks[plan_id]