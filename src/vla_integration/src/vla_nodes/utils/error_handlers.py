from typing import Optional, Callable, Any
import functools
import logging
from enum import Enum


class VLAError(Exception):
    """Base exception for VLA system errors."""
    pass


class VLAErrorCode(Enum):
    """Error codes for VLA system errors."""
    AUDIO_CAPTURE_ERROR = "audio_capture_error"
    SPEECH_RECOGNITION_ERROR = "speech_recognition_error"
    LLM_PROCESSING_ERROR = "llm_processing_error"
    ACTION_EXECUTION_ERROR = "action_execution_error"
    PLAN_VALIDATION_ERROR = "plan_validation_error"
    CONFIGURATION_ERROR = "configuration_error"
    CONNECTION_ERROR = "connection_error"


class VLARetryError(VLAError):
    """Exception raised when retries are exhausted."""
    pass


def handle_vla_error(error_code: VLAErrorCode, default_return: Any = None):
    """
    Decorator to handle VLA system errors gracefully.
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                logging.error(f"VLA Error ({error_code.value}): {str(e)}")
                if default_return is not None:
                    return default_return
                else:
                    raise VLAError(f"VLA Error ({error_code.value}): {str(e)}") from e
        return wrapper
    return decorator


def retry_on_failure(max_retries: int = 3, delay: float = 1.0):
    """
    Decorator to retry a function on failure.
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_exception = None
            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    last_exception = e
                    if attempt < max_retries:
                        logging.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay}s...")
                        import time
                        time.sleep(delay)
                    else:
                        logging.error(f"All {max_retries} retries failed. Last error: {str(e)}")
                        raise VLARetryError(f"Function {func.__name__} failed after {max_retries} retries") from e
            return None
        return wrapper
    return decorator


class ErrorHandler:
    """
    Centralized error handling for the VLA system.
    """
    @staticmethod
    def handle_audio_error(error: Exception) -> VLAError:
        """Handle audio-related errors."""
        return VLAError(f"Audio error: {str(error)}")

    @staticmethod
    def handle_llm_error(error: Exception) -> VLAError:
        """Handle LLM-related errors."""
        return VLAError(f"LLM error: {str(error)}")

    @staticmethod
    def handle_action_error(error: Exception) -> VLAError:
        """Handle action execution errors."""
        return VLAError(f"Action execution error: {str(error)}")

    @staticmethod
    def handle_plan_error(error: Exception) -> VLAError:
        """Handle plan validation/creation errors."""
        return VLAError(f"Plan error: {str(error)}")