import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ..utils.base_node import BaseVLANode
from ..utils.config_manager import ConfigManager
from ..utils.error_handlers import handle_vla_error, VLAErrorCode
from ..models.voice_command import VoiceCommand
from ..models.action_plan import ActionPlan
from .llm_service import LLMService
from .plan_validator import PlanValidator
from .plan_generator import PlanGenerator
from .ambiguity_handler import AmbiguityHandler
import json
import uuid
from datetime import datetime


class PlanningNode(BaseVLANode):
    """
    ROS 2 node for cognitive planning - converts text commands into action plans.
    """
    def __init__(self):
        # Load configuration
        self.config_manager = ConfigManager("config/vla_config.yaml")

        # Initialize base node
        super().__init__(
            node_name=self.config_manager.get('ros2.node_name', 'vla_planning') + "_planning",
            config_path="config/vla_config.yaml"
        )

        # Initialize services
        self.llm_service = LLMService(self.config_manager)
        self.validator = PlanValidator()
        self.generator = PlanGenerator(self.llm_service, self.validator)
        self.ambiguity_handler = AmbiguityHandler(self.llm_service)

        # Setup publisher for action plans
        action_plan_topic = self.config_manager.get('ros2.action_plan_topic', '/vla/action_plan')
        self.action_plan_publisher = self.setup_publisher(String, action_plan_topic)

        # Setup subscriber for voice commands
        voice_topic = self.config_manager.get('ros2.voice_command_topic', '/vla/voice_command')
        self.voice_command_subscriber = self.setup_subscriber(
            String, voice_topic, self._voice_command_callback
        )

        self.logger.info("Planning Node initialized")

    def _voice_command_callback(self, msg: String):
        """
        Callback for receiving voice commands and generating action plans.
        """
        try:
            command_text = msg.data
            self.logger.info(f"Received voice command: {command_text}")

            # Create a temporary VoiceCommand object
            voice_command = VoiceCommand(
                id=str(uuid.uuid4()),
                text=command_text,
                timestamp=datetime.now(),
                confidence=0.9,  # Placeholder
                status="processing"
            )

            # Generate action plan from the command
            action_plan = self.generator.generate_plan(command_text)

            if action_plan:
                # Validate the plan
                is_valid, errors = self.validator.validate_plan(action_plan)
                if is_valid:
                    self.logger.info(f"Generated valid action plan with {len(action_plan.steps)} steps")
                    self._publish_action_plan(action_plan)
                else:
                    self.logger.error(f"Generated plan failed validation: {errors}")
            else:
                self.logger.error("Failed to generate action plan")
                # Publish a message indicating failure
                self._publish_error_message(f"Could not process command: {command_text}")

        except Exception as e:
            self.logger.error(f"Error processing voice command: {e}")
            self._publish_error_message(f"Error processing command: {str(e)}")

    @handle_vla_error(VLAErrorCode.LLM_PROCESSING_ERROR)
    def _publish_action_plan(self, action_plan: ActionPlan):
        """
        Publish an action plan to the ROS 2 topic.
        """
        # Convert action plan to JSON string
        plan_data = {
            "id": action_plan.id,
            "command_id": action_plan.command_id,
            "status": action_plan.status.value if hasattr(action_plan.status, 'value') else action_plan.status,
            "created_at": action_plan.created_at.isoformat() if hasattr(action_plan.created_at, 'isoformat') else str(action_plan.created_at),
            "updated_at": action_plan.updated_at.isoformat() if action_plan.updated_at and hasattr(action_plan.updated_at, 'isoformat') else None,
            "execution_context": action_plan.execution_context,
            "steps": []
        }

        for step in action_plan.steps:
            step_data = {
                "id": step.id,
                "plan_id": step.plan_id,
                "step_number": step.step_number,
                "action_type": step.action_type.value if hasattr(step.action_type, 'value') else step.action_type,
                "parameters": step.parameters,
                "dependencies": step.dependencies,
                "timeout": step.timeout,
                "retry_count": step.retry_count
            }
            plan_data["steps"].append(step_data)

        plan_json = json.dumps(plan_data)

        msg = String()
        msg.data = plan_json
        self.action_plan_publisher.publish(msg)
        self.logger.info(f"Published action plan: {action_plan.id}")

    def _publish_error_message(self, error_msg: str):
        """
        Publish an error message indicating plan generation failure.
        """
        error_data = {
            "error": True,
            "message": error_msg,
            "timestamp": datetime.now().isoformat()
        }

        msg = String()
        msg.data = json.dumps(error_data)
        self.action_plan_publisher.publish(msg)
        self.logger.warning(f"Published error message: {error_msg}")

    def generate_plan_for_command(self, command_text: str) -> Optional[ActionPlan]:
        """
        Generate a plan for a command directly (not through ROS messaging).

        Args:
            command_text: The command text to generate a plan for

        Returns:
            An ActionPlan or None if generation fails
        """
        try:
            return self.generator.generate_plan(command_text)
        except Exception as e:
            self.logger.error(f"Error generating plan for command '{command_text}': {e}")
            return None

    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    planning_node = PlanningNode()

    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()