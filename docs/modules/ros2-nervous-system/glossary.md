# Glossary: ROS 2 Nervous System Module

## A

**Action**: A ROS 2 communication pattern for long-running tasks that provide feedback during execution and can be canceled. Used for complex robot behaviors like navigation or manipulation.

**AI Agent**: A software entity that perceives its environment and takes actions to achieve goals. In robotics, this often refers to software that processes sensor data and controls robot behavior.

## D

**DDS (Data Distribution Service)**: The middleware standard that ROS 2 uses by default for communication. Provides data-centric publish/subscribe functionality.

**Degree of Freedom**: The number of independent movements a robot joint or system can make. A humanoid joint typically has one or more degrees of freedom.

## J

**Joint**: In URDF, a connection between two links that defines how they can move relative to each other. Types include revolute, prismatic, and continuous joints.

## L

**Link**: In URDF, a rigid part of the robot with defined physical and visual properties. Examples include arms, legs, and torso segments.

## M

**Middleware**: Software that provides common services and capabilities to applications beyond what's offered by the operating system. In robotics, it enables communication between different software components.

**Node**: A basic compute unit in ROS 2 that performs computation. Nodes communicate with other nodes through topics, services, and actions.

## R

**Robot Operating System 2 (ROS 2)**: A flexible framework for writing robot software, providing libraries, tools, and conventions for robot development.

**rclpy**: The Python client library for ROS 2, providing Python API for ROS 2 concepts.

## S

**Service**: A ROS 2 communication pattern for synchronous request/response interactions between nodes.

## T

**Topic**: A ROS 2 communication channel for publish/subscribe messaging between nodes. Used for continuous data streams.

## U

**URDF (Unified Robot Description Format)**: An XML format for representing robot models, including their physical and visual properties.

## Q

**Quality of Service (QoS)**: Policies in ROS 2 that define communication behavior, including reliability, durability, and liveliness settings.