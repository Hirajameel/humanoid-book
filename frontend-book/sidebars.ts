import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            'modules/ros2-nervous-system/chapter-1-ros2-middleware',
            'modules/ros2-nervous-system/chapter-2-communication-model',
            'modules/ros2-nervous-system/chapter-3-software-hardware-bridge',
            'modules/ros2-nervous-system/glossary'
          ],
        },
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          items: [
            'modules/digital-twin-sim/chapter-1-digital-twins-robotics',
            'modules/digital-twin-sim/chapter-2-physics-simulation-gazebo',
            'modules/digital-twin-sim/chapter-3-sensor-simulation-visualization',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'modules/isaac-ai-brain/chapter-1-isaac-sim-overview',
            'modules/isaac-ai-brain/chapter-2-perception-navigation-isaac-ros',
            'modules/isaac-ai-brain/chapter-3-path-planning-nav2',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA) Integration',
          items: [
            'modules/vla-integration/index',
          ],
        },
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
