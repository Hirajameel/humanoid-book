import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import logging
import yaml
from pathlib import Path


class BaseVLANode(Node):
    """
    Base class for all VLA system nodes providing common functionality.
    """
    def __init__(self, node_name: str, config_path: str = None):
        super().__init__(node_name)

        # Setup logging
        self.logger = self.get_logger()

        # Load configuration
        self.config = self._load_config(config_path)

        # Setup default QoS profile
        self.qos_profile = QoSProfile(depth=10)

        self.logger.info(f"{node_name} initialized")

    def _load_config(self, config_path: str = None) -> dict:
        """
        Load configuration from YAML file or return default config.
        """
        if config_path and Path(config_path).exists():
            with open(config_path, 'r') as file:
                return yaml.safe_load(file)
        else:
            # Return default configuration
            return {
                'ros2': {
                    'node_name': self.get_name(),
                    'voice_command_topic': '/vla/voice_command',
                    'action_plan_topic': '/vla/action_plan',
                    'execution_status_topic': '/vla/execution_status'
                }
            }

    def setup_publisher(self, msg_type, topic_name: str, qos_profile=None):
        """
        Helper method to create a publisher with default QoS.
        """
        if qos_profile is None:
            qos_profile = self.qos_profile

        publisher = self.create_publisher(msg_type, topic_name, qos_profile)
        self.logger.info(f"Created publisher for topic: {topic_name}")
        return publisher

    def setup_subscriber(self, msg_type, topic_name: str, callback, qos_profile=None):
        """
        Helper method to create a subscriber with default QoS.
        """
        if qos_profile is None:
            qos_profile = self.qos_profile

        subscriber = self.create_subscription(
            msg_type, topic_name, callback, qos_profile
        )
        self.logger.info(f"Created subscriber for topic: {topic_name}")
        return subscriber