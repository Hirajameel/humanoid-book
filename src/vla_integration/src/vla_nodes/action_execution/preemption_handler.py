from typing import Dict, Any, Optional, Callable
from datetime import datetime
import threading
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.action_step import ActionStep
from ..utils.error_handlers import VLAError


class PreemptionError(VLAError):
    """Exception raised when preemption operations fail."""
    pass


class PreemptionHandler:
    """
    Handles preemption and cancellation of action executions.
    """
    def __init__(self):
        self.active_preemptions = {}
        self.preemption_callbacks = {}
        self.lock = threading.Lock()

    def request_preemption(self, plan_id: str, reason: str = "user_request",
                          on_preemption_complete: Optional[Callable] = None,
                          on_preemption_failed: Optional[Callable] = None) -> bool:
        """
        Request preemption of a currently executing plan.

        Args:
            plan_id: The ID of the plan to preempt
            reason: The reason for preemption
            on_preemption_complete: Callback when preemption is complete
            on_preemption_failed: Callback when preemption fails

        Returns:
            True if preemption request was accepted, False otherwise
        """
        with self.lock:
            if plan_id not in self.active_preemptions:
                self.active_preemptions[plan_id] = {
                    "request_time": datetime.now(),
                    "reason": reason,
                    "status": "requested",
                    "callbacks": {
                        "on_complete": on_preemption_complete,
                        "on_failed": on_preemption_failed
                    }
                }
                return True
            return False

    def execute_preemption(self, plan_id: str) -> bool:
        """
        Execute the preemption of a plan.

        Args:
            plan_id: The ID of the plan to preempt

        Returns:
            True if preemption was successful, False otherwise
        """
        with self.lock:
            if plan_id not in self.active_preemptions:
                # Plan might not be registered for preemption, but we'll try anyway
                self.active_preemptions[plan_id] = {
                    "request_time": datetime.now(),
                    "reason": "force_preemption",
                    "status": "executing",
                    "callbacks": {}
                }

            preemption_data = self.active_preemptions[plan_id]
            preemption_data["status"] = "executing"

        try:
            # In a real implementation, this would interface with ROS 2 to cancel active goals
            # For now, we'll simulate the preemption process
            self._simulate_preemption(plan_id)

            with self.lock:
                preemption_data["status"] = "completed"
                preemption_data["completion_time"] = datetime.now()

            # Call completion callback
            self._call_preemption_complete_callback(plan_id)
            return True

        except Exception as e:
            with self.lock:
                preemption_data["status"] = "failed"
                preemption_data["error"] = str(e)

            self._call_preemption_failed_callback(plan_id, e)
            return False

    def _simulate_preemption(self, plan_id: str):
        """
        Simulate the preemption process.
        In a real implementation, this would cancel active ROS 2 goals.
        """
        import time
        # Simulate time needed to cancel active actions
        time.sleep(0.5)

    def can_preempt(self, plan_id: str, plan: ActionPlan) -> bool:
        """
        Check if a plan can be preempted based on its current state and type.

        Args:
            plan_id: The ID of the plan
            plan: The action plan object

        Returns:
            True if plan can be preempted, False otherwise
        """
        # Plans in completed, failed, or cancelled states cannot be preempted
        if plan.status in [ActionPlanStatus.COMPLETED, ActionPlanStatus.FAILED, ActionPlanStatus.CANCELLED]:
            return False

        # Check if there are active steps that can be interrupted
        # In a real implementation, this would check the status of active ROS 2 goals
        return True

    def force_preemption(self, plan_id: str) -> bool:
        """
        Force preemption of a plan, potentially causing abrupt termination.

        Args:
            plan_id: The ID of the plan to force-preempt

        Returns:
            True if force preemption was successful, False otherwise
        """
        with self.lock:
            if plan_id not in self.active_preemptions:
                self.active_preemptions[plan_id] = {
                    "request_time": datetime.now(),
                    "reason": "force",
                    "status": "forced",
                    "callbacks": {}
                }

        try:
            # In a real implementation, this would send force-cancel signals to ROS 2
            # For now, simulate the force preemption
            self._simulate_force_preemption(plan_id)

            with self.lock:
                if plan_id in self.active_preemptions:
                    self.active_preemptions[plan_id]["status"] = "force_completed"
                    self.active_preemptions[plan_id]["completion_time"] = datetime.now()

            return True

        except Exception as e:
            with self.lock:
                if plan_id in self.active_preemptions:
                    self.active_preemptions[plan_id]["status"] = "force_failed"
                    self.active_preemptions[plan_id]["error"] = str(e)

            return False

    def _simulate_force_preemption(self, plan_id: str):
        """
        Simulate the force preemption process.
        """
        import time
        # Simulate time needed to force-cancel active actions
        time.sleep(0.2)

    def get_preemption_status(self, plan_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the current preemption status for a plan.

        Args:
            plan_id: The ID of the plan

        Returns:
            Preemption status information or None if no preemption requested
        """
        with self.lock:
            if plan_id not in self.active_preemptions:
                return None

            preemption_data = self.active_preemptions[plan_id]
            return {
                "plan_id": plan_id,
                "status": preemption_data["status"],
                "reason": preemption_data["reason"],
                "request_time": preemption_data["request_time"],
                "completion_time": preemption_data.get("completion_time"),
                "error": preemption_data.get("error")
            }

    def register_plan_for_preemption(self, plan: ActionPlan):
        """
        Register a plan so it can be preempted.

        Args:
            plan: The action plan to register
        """
        with self.lock:
            if plan.id not in self.active_preemptions:
                self.active_preemptions[plan.id] = {
                    "request_time": datetime.now(),
                    "reason": "registered",
                    "status": "registered",
                    "callbacks": {},
                    "original_plan": plan
                }

    def unregister_plan(self, plan_id: str) -> bool:
        """
        Unregister a plan from preemption tracking.

        Args:
            plan_id: The ID of the plan to unregister

        Returns:
            True if unregistered successfully, False if not found
        """
        with self.lock:
            if plan_id in self.active_preemptions:
                del self.active_preemptions[plan_id]
                if plan_id in self.preemption_callbacks:
                    del self.preemption_callbacks[plan_id]
                return True
            return False

    def _call_preemption_complete_callback(self, plan_id: str):
        """
        Call the preemption completion callback if it exists.
        """
        with self.lock:
            if plan_id in self.active_preemptions:
                callback = self.active_preemptions[plan_id]["callbacks"].get("on_complete")
                if callback:
                    try:
                        callback(plan_id, "preemption completed")
                    except Exception as e:
                        print(f"Error in preemption complete callback: {e}")

    def _call_preemption_failed_callback(self, plan_id: str, error: Exception):
        """
        Call the preemption failed callback if it exists.
        """
        with self.lock:
            if plan_id in self.active_preemptions:
                callback = self.active_preemptions[plan_id]["callbacks"].get("on_preemption_failed")
                if callback:
                    try:
                        callback(plan_id, error)
                    except Exception as e:
                        print(f"Error in preemption failed callback: {e}")

    def cancel_all_preemptions(self) -> int:
        """
        Cancel all pending preemptions.

        Returns:
            Number of preemptions cancelled
        """
        cancelled_count = 0
        with self.lock:
            for plan_id, preemption_data in list(self.active_preemptions.items()):
                if preemption_data["status"] in ["requested", "registered"]:
                    preemption_data["status"] = "cancelled"
                    cancelled_count += 1
                    # Call failed callback for cancelled preemptions
                    self._call_preemption_failed_callback(plan_id, PreemptionError("Preemption cancelled"))

        return cancelled_count

    def cleanup_completed_preemptions(self, max_age_minutes: int = 5):
        """
        Clean up completed or failed preemptions to free up resources.

        Args:
            max_age_minutes: Maximum age of preemption records to keep (in minutes)
        """
        with self.lock:
            cutoff_time = datetime.now()
            cutoff_time = cutoff_time.replace(minute=cutoff_time.minute - max_age_minutes)

            completed_preemptions = []
            for plan_id, preemption_data in self.active_preemptions.items():
                completion_time = preemption_data.get("completion_time")
                if completion_time and completion_time < cutoff_time:
                    completed_preemptions.append(plan_id)

            for plan_id in completed_preemptions:
                del self.active_preemptions[plan_id]
                if plan_id in self.preemption_callbacks:
                    del self.preemption_callbacks[plan_id]

    def get_all_preemption_requests(self) -> Dict[str, Dict[str, Any]]:
        """
        Get information about all preemption requests.

        Returns:
            Dictionary mapping plan IDs to preemption request information
        """
        with self.lock:
            return {
                plan_id: {
                    "status": data["status"],
                    "reason": data["reason"],
                    "request_time": data["request_time"],
                    "completion_time": data.get("completion_time"),
                    "error": data.get("error")
                }
                for plan_id, data in self.active_preemptions.items()
            }