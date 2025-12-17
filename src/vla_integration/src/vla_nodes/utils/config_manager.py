import os
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
from dotenv import load_dotenv


class ConfigManager:
    """
    Manages configuration for the VLA system.
    Loads configuration from YAML files and environment variables.
    """

    def __init__(self, config_path: Optional[str] = None):
        # Load environment variables from .env file
        load_dotenv()

        # Load configuration from YAML file
        self.config = self._load_config(config_path)

    def _load_config(self, config_path: Optional[str] = None) -> Dict[str, Any]:
        """
        Load configuration from YAML file.
        Falls back to default configuration if file doesn't exist.
        """
        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
        else:
            # Default configuration
            config = {
                'openai': {
                    'api_key': os.getenv('OPENAI_API_KEY', ''),
                    'whisper_model': 'whisper-1',
                    'gpt_model': 'gpt-4-turbo',
                    'temperature': 0.3
                },
                'audio': {
                    'sample_rate': 16000,
                    'chunk_size': 1024,
                    'channels': 1,
                    'device_index': None
                },
                'ros2': {
                    'node_name': 'vla_system',
                    'voice_command_topic': '/vla/voice_command',
                    'action_plan_topic': '/vla/action_plan',
                    'execution_status_topic': '/vla/execution_status'
                },
                'execution': {
                    'timeout_seconds': 30,
                    'max_retries': 3,
                    'feedback_rate': 1.0
                }
            }

        # Override with environment variables if available
        self._apply_env_overrides(config)

        return config

    def _apply_env_overrides(self, config: Dict[str, Any]) -> None:
        """
        Override configuration values with environment variables.
        """
        # OpenAI settings
        if os.getenv('OPENAI_API_KEY'):
            config['openai']['api_key'] = os.getenv('OPENAI_API_KEY')

        if os.getenv('OPENAI_WHISPER_MODEL'):
            config['openai']['whisper_model'] = os.getenv('OPENAI_WHISPER_MODEL')

        if os.getenv('OPENAI_GPT_MODEL'):
            config['openai']['gpt_model'] = os.getenv('OPENAI_GPT_MODEL')

        # Audio settings
        if os.getenv('AUDIO_SAMPLE_RATE'):
            config['audio']['sample_rate'] = int(os.getenv('AUDIO_SAMPLE_RATE'))

        if os.getenv('AUDIO_CHUNK_SIZE'):
            config['audio']['chunk_size'] = int(os.getenv('AUDIO_CHUNK_SIZE'))

        # Execution settings
        if os.getenv('EXECUTION_TIMEOUT_SECONDS'):
            config['execution']['timeout_seconds'] = int(os.getenv('EXECUTION_TIMEOUT_SECONDS'))

        if os.getenv('EXECUTION_MAX_RETRIES'):
            config['execution']['max_retries'] = int(os.getenv('EXECUTION_MAX_RETRIES'))

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value using dot notation (e.g., 'openai.api_key').
        """
        keys = key.split('.')
        value = self.config

        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default

        return value

    def set(self, key: str, value: Any) -> None:
        """
        Set a configuration value using dot notation.
        """
        keys = key.split('.')
        config_ref = self.config

        for k in keys[:-1]:
            if k not in config_ref:
                config_ref[k] = {}
            config_ref = config_ref[k]

        config_ref[keys[-1]] = value

    def get_openai_config(self) -> Dict[str, Any]:
        """Get OpenAI-specific configuration."""
        return self.config.get('openai', {})

    def get_audio_config(self) -> Dict[str, Any]:
        """Get audio-specific configuration."""
        return self.config.get('audio', {})

    def get_ros2_config(self) -> Dict[str, Any]:
        """Get ROS 2-specific configuration."""
        return self.config.get('ros2', {})

    def get_execution_config(self) -> Dict[str, Any]:
        """Get execution-specific configuration."""
        return self.config.get('execution', {})