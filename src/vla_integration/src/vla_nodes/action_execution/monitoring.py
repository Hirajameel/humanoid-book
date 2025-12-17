from typing import Dict, Any, Optional, Callable
from datetime import datetime
import threading
import time
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.action_step import ActionStep
from ..models.robot_state import RobotState
from ..utils.error_handlers import VLAError


class ExecutionMonitor:
    """
    Monitors action execution and provides real-time status updates.
    """
    def __init__(self):
        self.active_monitors = {}
        self.status_callbacks = {}
        self.lock = threading.Lock()

    def start_monitoring(self, plan: ActionPlan,
                        on_status_update: Optional[Callable] = None,
                        on_completion: Optional[Callable] = None,
                        on_error: Optional[Callable] = None) -> str:
        """
        Start monitoring an action plan execution.

        Args:
            plan: The action plan to monitor
            on_status_update: Callback for status updates
            on_completion: Callback for completion events
            on_error: Callback for error events

        Returns:
            Monitor ID for tracking
        """
        monitor_id = plan.id
        with self.lock:
            self.status_callbacks[monitor_id] = {
                "on_status_update": on_status_update,
                "on_completion": on_completion,
                "on_error": on_error
            }

        # Store initial monitoring data
        with self.lock:
            self.active_monitors[monitor_id] = {
                "plan": plan,
                "start_time": datetime.now(),
                "last_update": datetime.now(),
                "completed_steps": 0,
                "total_steps": len(plan.steps),
                "current_status": "running",
                "step_status": {step.id: "pending" for step in plan.steps},
                "metrics": {
                    "execution_time": 0,
                    "success_rate": 0.0,
                    "error_count": 0
                }
            }

        # Start monitoring thread
        monitor_thread = threading.Thread(
            target=self._monitor_execution,
            args=(monitor_id,)
        )
        monitor_thread.daemon = True
        monitor_thread.start()

        return monitor_id

    def _monitor_execution(self, monitor_id: str):
        """
        Monitor the execution in a separate thread.
        """
        while True:
            with self.lock:
                if monitor_id not in self.active_monitors:
                    break

                monitor_data = self.active_monitors[monitor_id]
                plan = monitor_data["plan"]

                # Check if plan execution is complete
                if plan.status in [ActionPlanStatus.COMPLETED, ActionPlanStatus.FAILED, ActionPlanStatus.CANCELLED]:
                    self._call_completion_callback(monitor_id, plan)
                    # Clean up after a delay
                    time.sleep(1)
                    with self.lock:
                        if monitor_id in self.active_monitors:
                            del self.active_monitors[monitor_id]
                        if monitor_id in self.status_callbacks:
                            del self.status_callbacks[monitor_id]
                    break

                # Update metrics
                self._update_metrics(monitor_id)

                # Send status update
                self._send_status_update(monitor_id)

            time.sleep(1)  # Update every second

    def _update_metrics(self, monitor_id: str):
        """
        Update execution metrics for the monitored plan.
        """
        with self.lock:
            if monitor_id not in self.active_monitors:
                return

            monitor_data = self.active_monitors[monitor_id]
            plan = monitor_data["plan"]

            # Update completed steps count
            completed = sum(1 for step_id, status in monitor_data["step_status"].items()
                          if status == "completed")
            monitor_data["completed_steps"] = completed

            # Update execution time
            elapsed = datetime.now() - monitor_data["start_time"]
            monitor_data["metrics"]["execution_time"] = elapsed.total_seconds()

            # Update success rate
            if monitor_data["total_steps"] > 0:
                monitor_data["metrics"]["success_rate"] = completed / monitor_data["total_steps"]

    def _send_status_update(self, monitor_id: str):
        """
        Send a status update to the registered callback.
        """
        with self.lock:
            if monitor_id not in self.active_monitors:
                return

            monitor_data = self.active_monitors[monitor_id]
            status_info = {
                "plan_id": monitor_id,
                "status": monitor_data["current_status"],
                "completed_steps": monitor_data["completed_steps"],
                "total_steps": monitor_data["total_steps"],
                "progress_percentage": (monitor_data["completed_steps"] / monitor_data["total_steps"]) * 100 if monitor_data["total_steps"] > 0 else 0,
                "execution_time": monitor_data["metrics"]["execution_time"],
                "success_rate": monitor_data["metrics"]["success_rate"],
                "step_status": monitor_data["step_status"].copy(),
                "timestamp": datetime.now()
            }

            # Update last update time
            monitor_data["last_update"] = datetime.now()

        # Call the status update callback outside the lock
        self._call_status_callback(monitor_id, status_info)

    def update_step_status(self, plan_id: str, step_id: str, status: str,
                          result: Optional[Dict[str, Any]] = None):
        """
        Update the status of a specific step in the monitored plan.

        Args:
            plan_id: The ID of the plan
            step_id: The ID of the step
            status: The new status ('pending', 'running', 'completed', 'failed', 'skipped')
            result: Optional result data from the step execution
        """
        with self.lock:
            if plan_id not in self.active_monitors:
                return

            monitor_data = self.active_monitors[plan_id]
            if step_id in monitor_data["step_status"]:
                old_status = monitor_data["step_status"][step_id]
                monitor_data["step_status"][step_id] = status

                # Update plan status if needed
                if status == "failed":
                    monitor_data["current_status"] = "error"
                    monitor_data["metrics"]["error_count"] += 1
                elif status == "completed":
                    # Check if all steps are completed
                    if all(s == "completed" for s in monitor_data["step_status"].values()):
                        monitor_data["current_status"] = "completed"
                        # Update the actual plan status
                        monitor_data["plan"].status = ActionPlanStatus.COMPLETED

    def get_execution_status(self, plan_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the current execution status of a plan.

        Args:
            plan_id: The ID of the plan to check

        Returns:
            Status information dictionary or None if not monitored
        """
        with self.lock:
            if plan_id not in self.active_monitors:
                return None

            monitor_data = self.active_monitors[plan_id]
            return {
                "plan_id": plan_id,
                "status": monitor_data["current_status"],
                "completed_steps": monitor_data["completed_steps"],
                "total_steps": monitor_data["total_steps"],
                "progress_percentage": (monitor_data["completed_steps"] / monitor_data["total_steps"]) * 100 if monitor_data["total_steps"] > 0 else 0,
                "execution_time": monitor_data["metrics"]["execution_time"],
                "success_rate": monitor_data["metrics"]["success_rate"],
                "error_count": monitor_data["metrics"]["error_count"],
                "step_status": monitor_data["step_status"].copy(),
                "start_time": monitor_data["start_time"],
                "last_update": monitor_data["last_update"]
            }

    def stop_monitoring(self, monitor_id: str) -> bool:
        """
        Stop monitoring a specific plan.

        Args:
            monitor_id: The ID of the monitor to stop

        Returns:
            True if monitoring was stopped, False if monitor didn't exist
        """
        with self.lock:
            if monitor_id in self.active_monitors:
                del self.active_monitors[monitor_id]
                if monitor_id in self.status_callbacks:
                    del self.status_callbacks[monitor_id]
                return True
            return False

    def _call_status_callback(self, monitor_id: str, status_info: Dict[str, Any]):
        """
        Call the status update callback if it exists.
        """
        with self.lock:
            if monitor_id in self.status_callbacks:
                callback = self.status_callbacks[monitor_id].get("on_status_update")
                if callback:
                    try:
                        callback(monitor_id, status_info)
                    except Exception as e:
                        print(f"Error in status callback: {e}")

    def _call_completion_callback(self, monitor_id: str, plan: ActionPlan):
        """
        Call the completion callback if it exists.
        """
        with self.lock:
            if monitor_id in self.status_callbacks:
                callback = self.status_callbacks[monitor_id].get("on_completion")
                if callback:
                    try:
                        callback(monitor_id, plan)
                    except Exception as e:
                        print(f"Error in completion callback: {e}")

    def _call_error_callback(self, monitor_id: str, error: Exception):
        """
        Call the error callback if it exists.
        """
        with self.lock:
            if monitor_id in self.status_callbacks:
                callback = self.status_callbacks[monitor_id].get("on_error")
                if callback:
                    try:
                        callback(monitor_id, error)
                    except Exception as e:
                        print(f"Error in error callback: {e}")

    def get_all_monitored_plans(self) -> Dict[str, Dict[str, Any]]:
        """
        Get status information for all currently monitored plans.

        Returns:
            Dictionary mapping plan IDs to their status information
        """
        all_status = {}
        with self.lock:
            for plan_id in self.active_monitors:
                status = self.get_execution_status(plan_id)
                if status:
                    all_status[plan_id] = status
        return all_status

    def cleanup_inactive_monitors(self):
        """
        Clean up monitors that have been inactive for a certain period.
        """
        with self.lock:
            inactive_monitors = []
            current_time = datetime.now()

            for monitor_id, monitor_data in self.active_monitors.items():
                time_since_update = current_time - monitor_data["last_update"]
                # If no update for more than 5 minutes, consider it inactive
                if time_since_update.total_seconds() > 300:
                    inactive_monitors.append(monitor_id)

            for monitor_id in inactive_monitors:
                del self.active_monitors[monitor_id]
                if monitor_id in self.status_callbacks:
                    del self.status_callbacks[monitor_id]