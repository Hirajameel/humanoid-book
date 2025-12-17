from setuptools import setup, find_packages

setup(
    name='vla_integration',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'rclpy>=3.0.0',
        'openai>=1.0.0',
        'pyaudio>=0.2.11',
        'sounddevice>=0.4.6',
        'numpy>=1.21.0',
        'pyyaml>=6.0',
        'python-dotenv>=0.19.0',
    ],
    author='Humanoid Book Project',
    author_email='info@humanoid-book.com',
    description='Vision-Language-Action (VLA) Integration for Humanoid Robots',
    license='MIT',
    keywords='ros2 robot ai nlp',
    url='https://github.com/humanoid-book/vla-integration',
    entry_points={
        'console_scripts': [
            'vla-system = vla_integration.main:main',
        ],
    },
)