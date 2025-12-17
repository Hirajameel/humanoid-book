from typing import Optional, Callable, Any
import functools
import logging
from ..utils.error_handlers import VLAError, VLAErrorCode, handle_vla_error


def handle_llm_communication_error(default_return: Any = None):
    """
    Decorator to handle LLM communication errors specifically.
    """
    return handle_vla_error(VLAErrorCode.LLM_PROCESSING_ERROR, default_return)


def handle_plan_generation_error(default_return: Any = None):
    """
    Decorator to handle action plan generation errors specifically.
    """
    return handle_vla_error(VLAErrorCode.LLM_PROCESSING_ERROR, default_return)


def handle_plan_validation_error(default_return: Any = None):
    """
    Decorator to handle plan validation errors specifically.
    """
    return handle_vla_error(VLAErrorCode.PLAN_VALIDATION_ERROR, default_return)


class LLMCommunicationError(VLAError):
    """Exception raised for LLM communication errors."""
    pass


class PlanGenerationError(VLAError):
    """Exception raised for action plan generation errors."""
    pass


class PlanValidationError(VLAError):
    """Exception raised for action plan validation errors."""
    pass


class CognitivePlanningErrorHandler:
    """
    Error handler specifically for cognitive planning components.
    """
    @staticmethod
    def handle_llm_api_error(error: Exception) -> LLMCommunicationError:
        """Handle LLM API related errors."""
        return LLMCommunicationError(f"LLM API error: {str(error)}")

    @staticmethod
    def handle_command_parsing_error(error: Exception) -> PlanGenerationError:
        """Handle command parsing errors."""
        return PlanGenerationError(f"Command parsing error: {str(error)}")

    @staticmethod
    def handle_action_plan_generation_error(error: Exception) -> PlanGenerationError:
        """Handle action plan generation errors."""
        return PlanGenerationError(f"Action plan generation error: {str(error)}")

    @staticmethod
    def handle_plan_validation_error(error: Exception) -> PlanValidationError:
        """Handle plan validation errors."""
        return PlanValidationError(f"Plan validation error: {str(error)}")

    @staticmethod
    def handle_ambiguity_resolution_error(error: Exception) -> PlanGenerationError:
        """Handle ambiguity resolution errors."""
        return PlanGenerationError(f"Ambiguity resolution error: {str(error)}")

    @staticmethod
    def handle_openai_rate_limit_error(error: Exception) -> LLMCommunicationError:
        """Handle OpenAI rate limit errors."""
        return LLMCommunicationError(f"OpenAI rate limit exceeded: {str(error)}")

    @staticmethod
    def handle_openai_authentication_error(error: Exception) -> LLMCommunicationError:
        """Handle OpenAI authentication errors."""
        return LLMCommunicationError(f"OpenAI authentication failed: {str(error)}")