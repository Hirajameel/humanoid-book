from setuptools import setup
import os
from glob import glob

package_name = 'vla_integration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[
        'src.vla_nodes.voice_input.voice_input_node',
        'src.vla_nodes.cognitive_planning.planning_node',
        'src.vla_nodes.action_execution.action_execution_node'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Book Project',
    maintainer_email='info@humanoid-book.com',
    description='Vision-Language-Action (VLA) Integration Package',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_input_node = src.vla_nodes.voice_input.voice_input_node:main',
            'planning_node = src.vla_nodes.cognitive_planning.planning_node:main',
            'action_execution_node = src.vla_nodes.action_execution.action_execution_node:main',
        ],
    },
)