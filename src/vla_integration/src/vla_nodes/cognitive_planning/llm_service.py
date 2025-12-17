import openai
import json
from typing import Dict, Any, List, Optional
from datetime import datetime
import logging
from ..utils.config_manager import ConfigManager
from ..utils.error_handlers import handle_vla_error, VLAErrorCode, retry_on_failure
from ..models.action_plan import ActionPlan, ActionStep, ActionPlanStatus
from ..models.action_step import ActionType


class LLMService:
    """
    Service for using LLM to interpret commands and generate action plans.
    """
    def __init__(self, config_manager: ConfigManager):
        self.config = config_manager
        api_key = self.config.get('openai.api_key')

        if not api_key:
            raise ValueError("OpenAI API key is required for LLMService")

        openai.api_key = api_key
        self.gpt_model = self.config.get('openai.gpt_model', 'gpt-4-turbo')
        self.temperature = self.config.get('openai.temperature', 0.3)

        # Define robot capabilities for reference
        self.robot_capabilities = {
            "navigation": ["move forward", "move backward", "turn left", "turn right", "go to", "navigate to"],
            "manipulation": ["pick up", "grasp", "release", "move arm", "place", "put down"],
            "perception": ["look at", "detect", "find", "scan", "identify", "recognize"],
            "communication": ["speak", "say", "tell", "announce"]
        }

    @handle_vla_error(VLAErrorCode.LLM_PROCESSING_ERROR, default_return=None)
    @retry_on_failure(max_retries=3, delay=1.0)
    def generate_action_plan(self, command_text: str, robot_state: Optional[Dict[str, Any]] = None) -> Optional[ActionPlan]:
        """
        Generate an action plan from a natural language command using LLM.

        Args:
            command_text: The natural language command
            robot_state: Optional current state of the robot for context

        Returns:
            An ActionPlan object or None if generation fails
        """
        # Create a prompt for the LLM to generate an action plan
        prompt = self._create_action_plan_prompt(command_text, robot_state)

        try:
            response = openai.chat.completions.create(
                model=self.gpt_model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that converts natural language commands into structured action plans for a humanoid robot. Respond only with valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                response_format={"type": "json_object"}
            )

            # Parse the response
            response_text = response.choices[0].message.content.strip()

            # Clean the response if it starts with a code block
            if response_text.startswith("```json"):
                response_text = response_text[7:]  # Remove ```json
            if response_text.endswith("```"):
                response_text = response_text[:-3]  # Remove ```

            response_data = json.loads(response_text)

            # Create ActionPlan from response
            action_plan = self._create_action_plan_from_response(response_data, command_text)
            return action_plan

        except json.JSONDecodeError as e:
            logging.error(f"Failed to parse LLM response as JSON: {e}")
            return None
        except Exception as e:
            logging.error(f"Error generating action plan: {e}")
            return None

    def _create_action_plan_prompt(self, command_text: str, robot_state: Optional[Dict[str, Any]] = None) -> str:
        """
        Create a prompt for the LLM to generate an action plan.
        """
        capabilities_str = json.dumps(self.robot_capabilities, indent=2)

        state_context = ""
        if robot_state:
            state_context = f"\nCurrent robot state: {json.dumps(robot_state, indent=2)}"

        prompt = f"""
        Convert the following natural language command into a structured action plan for a humanoid robot.

        Command: "{command_text}"

        {state_context}

        Robot capabilities:
        {capabilities_str}

        Provide your response as a JSON object with the following structure:
        {{
            "command_id": "<unique_id>",
            "description": "<brief description of the command>",
            "steps": [
                {{
                    "id": "<step_id>",
                    "step_number": <integer>,
                    "action_type": "<navigation|manipulation|perception|communication>",
                    "action": "<specific action to perform>",
                    "parameters": {{
                        "target": "<target object/location if applicable>",
                        "value": "<value if applicable>",
                        "duration": <duration in seconds if applicable>
                    }},
                    "dependencies": ["<list of step IDs that must complete before this step>"],
                    "timeout": <timeout in seconds>
                }}
            ]
        }}

        Make sure the action plan is feasible given the robot's capabilities.
        """

        return prompt

    def _create_action_plan_from_response(self, response_data: Dict[str, Any], original_command: str) -> ActionPlan:
        """
        Create an ActionPlan object from the LLM response data.
        """
        from uuid import uuid4

        # Create ActionStep objects from the response
        action_steps = []
        for step_data in response_data.get("steps", []):
            action_step = ActionStep(
                id=step_data.get("id", str(uuid4())),
                plan_id=response_data.get("command_id", str(uuid4())),  # This will be updated after ActionPlan creation
                step_number=step_data.get("step_number", len(action_steps)),
                action_type=step_data.get("action_type", "communication"),
                parameters=step_data.get("parameters", {}),
                dependencies=step_data.get("dependencies", []),
                timeout=step_data.get("timeout", 30),
                retry_count=0
            )
            action_steps.append(action_step)

        # Create the ActionPlan
        action_plan = ActionPlan(
            id=response_data.get("command_id", str(uuid4())),
            command_id=str(uuid4()),  # This should match the original voice command ID
            steps=action_steps,
            created_at=datetime.now(),
            status=ActionPlanStatus.PLANNED,
            execution_context={"original_command": original_command}
        )

        # Update the plan_id in all steps to match the action plan ID
        for step in action_plan.steps:
            step.plan_id = action_plan.id

        return action_plan

    @handle_vla_error(VLAErrorCode.LLM_PROCESSING_ERROR, default_return=None)
    @retry_on_failure(max_retries=3, delay=1.0)
    def parse_command_intent(self, command_text: str) -> Dict[str, Any]:
        """
        Parse the intent from a command without generating a full action plan.

        Args:
            command_text: The natural language command

        Returns:
            A dictionary with parsed intent information
        """
        prompt = f"""
        Parse the intent from the following command:
        "{command_text}"

        Provide your response as JSON with the following structure:
        {{
            "intent": "<main intent>",
            "action_type": "<navigation|manipulation|perception|communication|mixed>",
            "targets": ["<list of targets/objects>"],
            "locations": ["<list of locations if applicable>"],
            "confidence": <confidence score 0-1>
        }}
        """

        try:
            response = openai.chat.completions.create(
                model=self.gpt_model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that parses natural language commands into structured intent data. Respond only with valid JSON."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                response_format={"type": "json_object"}
            )

            response_text = response.choices[0].message.content.strip()

            # Clean the response if it starts with a code block
            if response_text.startswith("```json"):
                response_text = response_text[7:]  # Remove ```json
            if response_text.endswith("```"):
                response_text = response_text[:-3]  # Remove ```

            return json.loads(response_text)

        except json.JSONDecodeError as e:
            logging.error(f"Failed to parse LLM response as JSON: {e}")
            return {"intent": "unknown", "confidence": 0.0}
        except Exception as e:
            logging.error(f"Error parsing command intent: {e}")
            return {"intent": "unknown", "confidence": 0.0}

    @handle_vla_error(VLAErrorCode.LLM_PROCESSING_ERROR, default_return="")
    def clarify_command(self, command_text: str, context: Optional[Dict[str, Any]] = None) -> str:
        """
        Ask the LLM to clarify an ambiguous command.

        Args:
            command_text: The potentially ambiguous command
            context: Additional context that might help clarification

        Returns:
            A clarification request or interpretation of the command
        """
        context_str = json.dumps(context, indent=2) if context else "None"

        prompt = f"""
        The following command seems ambiguous: "{command_text}"

        Context: {context_str}

        Please provide a clarification request or your best interpretation of what the user might mean.
        Respond with your clarification request or interpretation.
        """

        try:
            response = openai.chat.completions.create(
                model=self.gpt_model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that clarifies ambiguous commands for a humanoid robot."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logging.error(f"Error clarifying command: {e}")
            return f"I'm not sure I understood your command: '{command_text}'. Could you please rephrase it?"