from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch description for the complete VLA (Vision-Language-Action) system.
    This brings up all necessary nodes for voice input, cognitive planning, and action execution.
    """
    # Get the package share directory
    pkg_dir = get_package_share_directory('vla_integration')

    # Configuration file path
    config_file = os.path.join(pkg_dir, 'config', 'vla_config.yaml')

    return LaunchDescription([
        # Voice Input Node - handles audio capture and speech-to-text
        Node(
            package='vla_integration',
            executable='voice_input_node',
            name='vla_voice_input',
            parameters=[config_file],
            output='screen'
        ),

        # Planning Node - processes voice commands and generates action plans
        Node(
            package='vla_integration',
            executable='planning_node',
            name='vla_planning',
            parameters=[config_file],
            output='screen'
        ),

        # Action Execution Node - executes action plans on the robot
        Node(
            package='vla_integration',
            executable='action_execution_node',
            name='vla_action_execution',
            parameters=[config_file],
            output='screen'
        )
    ])