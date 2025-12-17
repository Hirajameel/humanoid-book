import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ..utils.base_node import BaseVLANode
from ..utils.config_manager import ConfigManager
from ..utils.error_handlers import handle_vla_error, VLAErrorCode
from ..models.action_plan import ActionPlan, ActionPlanStatus
from ..models.robot_state import RobotState
from .action_mapper import ActionMapper
from .sequence_executor import SequenceExecutor
from .recovery_mechanisms import RecoveryMechanisms
from .monitoring import ExecutionMonitor
from .preemption_handler import PreemptionHandler
import json
import uuid
from datetime import datetime


class ActionExecutionNode(BaseVLANode):
    """
    ROS 2 node for executing action plans on the robot.
    """
    def __init__(self):
        # Load configuration
        self.config_manager = ConfigManager("config/vla_config.yaml")

        # Initialize base node
        super().__init__(
            node_name=self.config_manager.get('ros2.node_name', 'vla_action_execution') + "_execution",
            config_path="config/vla_config.yaml"
        )

        # Initialize services
        self.action_mapper = ActionMapper()
        self.recovery_mechanisms = RecoveryMechanisms()
        self.execution_monitor = ExecutionMonitor()
        self.preemption_handler = PreemptionHandler()
        self.sequence_executor = SequenceExecutor(self.action_mapper)

        # Setup publisher for execution status
        execution_status_topic = self.config_manager.get('ros2.execution_status_topic', '/vla/execution_status')
        self.execution_status_publisher = self.setup_publisher(String, execution_status_topic)

        # Setup subscriber for action plans
        action_plan_topic = self.config_manager.get('ros2.action_plan_topic', '/vla/action_plan')
        self.action_plan_subscriber = self.setup_subscriber(
            String, action_plan_topic, self._action_plan_callback
        )

        # Setup subscriber for preemption requests
        preemption_topic = self.config_manager.get('ros2.node_name', 'vla') + '/preempt'
        self.preemption_subscriber = self.setup_subscriber(
            String, preemption_topic, self._preemption_callback
        )

        self.active_plans = {}  # plan_id -> plan object
        self.logger.info("Action Execution Node initialized")

    def _action_plan_callback(self, msg: String):
        """
        Callback for receiving action plans and executing them.
        """
        try:
            plan_data = json.loads(msg.data)
            self.logger.info(f"Received action plan: {plan_data.get('id', 'unknown')}")

            # Create ActionPlan object from JSON data
            action_plan = self._create_action_plan_from_json(plan_data)

            if not action_plan:
                self.logger.error("Failed to create action plan from received data")
                return

            # Store the plan
            self.active_plans[action_plan.id] = action_plan

            # Register the plan for preemption
            self.preemption_handler.register_plan_for_preemption(action_plan)

            # Start monitoring the execution
            monitor_id = self.execution_monitor.start_monitoring(
                action_plan,
                on_status_update=self._on_execution_status_update,
                on_completion=self._on_execution_complete,
                on_error=self._on_execution_error
            )

            # Execute the plan
            execution_success = self.sequence_executor.execute_plan(
                action_plan,
                robot_state=None,  # Would get from robot state topic in real implementation
                on_step_complete=self._on_step_complete,
                on_plan_complete=lambda plan_id, plan: self._on_plan_complete(plan_id, plan, monitor_id),
                on_error=self._on_execution_error
            )

            if execution_success:
                self.logger.info(f"Started execution of plan: {action_plan.id}")
            else:
                self.logger.error(f"Failed to start execution of plan: {action_plan.id}")
                self._publish_execution_status(action_plan.id, "failed_to_start", "Execution failed to start")

        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding action plan JSON: {e}")
        except Exception as e:
            self.logger.error(f"Error processing action plan: {e}")
            self._publish_error_status(f"Error processing action plan: {str(e)}")

    def _create_action_plan_from_json(self, plan_data: dict) -> ActionPlan:
        """
        Create an ActionPlan object from JSON data.

        Args:
            plan_data: Dictionary containing plan data

        Returns:
            ActionPlan object or None if creation fails
        """
        from ..models.action_step import ActionStep, ActionType
        from datetime import datetime

        try:
            # Parse steps
            steps = []
            for step_data in plan_data.get("steps", []):
                step = ActionStep(
                    id=step_data["id"],
                    plan_id=step_data["plan_id"],
                    step_number=step_data["step_number"],
                    action_type=ActionType(step_data["action_type"]),
                    parameters=step_data.get("parameters", {}),
                    dependencies=step_data.get("dependencies", []),
                    timeout=step_data.get("timeout", 30),
                    retry_count=step_data.get("retry_count", 0)
                )
                steps.append(step)

            # Create the action plan
            action_plan = ActionPlan(
                id=plan_data["id"],
                command_id=plan_data["command_id"],
                steps=steps,
                created_at=datetime.fromisoformat(plan_data["created_at"]) if plan_data["created_at"] else datetime.now(),
                status=ActionPlanStatus(plan_data["status"]),
                updated_at=datetime.fromisoformat(plan_data["updated_at"]) if plan_data["updated_at"] else None,
                execution_context=plan_data.get("execution_context", {})
            )

            return action_plan

        except Exception as e:
            self.logger.error(f"Error creating action plan from JSON: {e}")
            return None

    def _on_execution_status_update(self, plan_id: str, status_info: dict):
        """
        Callback for execution status updates.
        """
        self._publish_execution_status(
            plan_id,
            status_info.get("status", "unknown"),
            f"Progress: {status_info.get('progress_percentage', 0):.1f}%",
            status_info
        )

    def _on_step_complete(self, plan_id: str, step):
        """
        Callback for step completion.
        """
        self.logger.info(f"Step completed in plan {plan_id}: {step.id}")
        # Update step status in monitoring
        self.execution_monitor.update_step_status(plan_id, step.id, "completed")

    def _on_plan_complete(self, plan_id: str, plan: ActionPlan, monitor_id: str):
        """
        Callback for plan completion.
        """
        self.logger.info(f"Plan completed: {plan_id}")
        # Update the plan status
        plan.status = ActionPlanStatus.COMPLETED
        # Stop monitoring
        self.execution_monitor.stop_monitoring(monitor_id)

    def _on_execution_complete(self, plan_id: str, plan: ActionPlan):
        """
        Callback when execution is complete.
        """
        self.logger.info(f"Execution complete for plan: {plan_id}")
        # Clean up active plans
        if plan_id in self.active_plans:
            del self.active_plans[plan_id]
        # Unregister from preemption
        self.preemption_handler.unregister_plan(plan_id)

    def _on_execution_error(self, plan_id: str, error):
        """
        Callback for execution errors.
        """
        self.logger.error(f"Execution error for plan {plan_id}: {error}")
        # Try to apply recovery mechanisms
        if plan_id in self.active_plans:
            plan = self.active_plans[plan_id]
            recovery_plan = self.recovery_mechanisms.handle_execution_failure(
                plan, error, robot_state=None
            )
            if recovery_plan:
                self.logger.info(f"Created recovery plan: {recovery_plan.id}")
                # Execute the recovery plan
                self.sequence_executor.execute_plan(
                    recovery_plan,
                    on_step_complete=self._on_step_complete,
                    on_plan_complete=self._on_plan_complete,
                    on_error=self._on_execution_error
                )

    def _preemption_callback(self, msg: String):
        """
        Callback for preemption requests.
        """
        try:
            request_data = json.loads(msg.data)
            plan_id = request_data.get("plan_id")
            reason = request_data.get("reason", "user_request")

            if not plan_id:
                self.logger.error("Preemption request missing plan_id")
                return

            self.logger.info(f"Received preemption request for plan {plan_id}, reason: {reason}")

            # Check if plan can be preempted
            if plan_id in self.active_plans:
                plan = self.active_plans[plan_id]
                if self.preemption_handler.can_preempt(plan_id, plan):
                    # Execute preemption
                    success = self.preemption_handler.execute_preemption(plan_id)
                    if success:
                        self.logger.info(f"Successfully preempted plan {plan_id}")
                        self._publish_execution_status(plan_id, "preempted", f"Plan preempted: {reason}")
                    else:
                        self.logger.error(f"Failed to preempt plan {plan_id}")
                        self._publish_execution_status(plan_id, "preemption_failed", "Preemption failed")
                else:
                    self.logger.warning(f"Plan {plan_id} cannot be preempted in its current state")
                    self._publish_execution_status(plan_id, "preemption_rejected", "Plan cannot be preempted")
            else:
                self.logger.warning(f"Preemption request for unknown plan: {plan_id}")

        except json.JSONDecodeError as e:
            self.logger.error(f"Error decoding preemption request JSON: {e}")
        except Exception as e:
            self.logger.error(f"Error processing preemption request: {e}")

    @handle_vla_error(VLAErrorCode.ACTION_EXECUTION_ERROR)
    def _publish_execution_status(self, plan_id: str, status: str, message: str, details: dict = None):
        """
        Publish execution status to the ROS 2 topic.
        """
        status_data = {
            "plan_id": plan_id,
            "status": status,
            "message": message,
            "timestamp": datetime.now().isoformat(),
            "details": details or {}
        }

        msg = String()
        msg.data = json.dumps(status_data)
        self.execution_status_publisher.publish(msg)
        self.logger.info(f"Published execution status: {status} for plan {plan_id}")

    def _publish_error_status(self, error_msg: str):
        """
        Publish an error status message.
        """
        error_data = {
            "error": True,
            "message": error_msg,
            "timestamp": datetime.now().isoformat()
        }

        msg = String()
        msg.data = json.dumps(error_data)
        self.execution_status_publisher.publish(msg)
        self.logger.error(f"Published error status: {error_msg}")

    def destroy_node(self):
        """
        Clean up resources when the node is destroyed.
        """
        # Cancel all active preemptions
        cancelled_count = self.preemption_handler.cancel_all_preemptions()
        self.logger.info(f"Cancelled {cancelled_count} preemption requests")

        # Clean up monitors
        self.execution_monitor.cleanup_inactive_monitors()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    action_execution_node = ActionExecutionNode()

    try:
        rclpy.spin(action_execution_node)
    except KeyboardInterrupt:
        pass
    finally:
        action_execution_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()