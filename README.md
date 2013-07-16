phantom_omni
============

ROS Node for Sensable Phantom Omni devices

Requires the omni_description package available at https://github.com/danepowell/omni_description

Forked from https://code.google.com/p/gt-ros-pkg/source/checkout?repo=hrl

Changes from the original version:
- Catkinized build system
- Compatibility with ROS Groovy
- General code cleanup
- Uses URDF description of Omni instead of hardcoded transforms.

To see it in action, simply:
roslaunch phantom_omni omni.launch