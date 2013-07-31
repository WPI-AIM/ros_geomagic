phantom_omni
============

ROS Node for Sensable Phantom Omni devices

Requires the [omni_description](https://github.com/danepowell/omni_description) package. 

Subscribes to the [same topics](http://www.ros.org/wiki/phantom_omni) as the [original node](https://code.google.com/p/gt-ros-pkg/source/checkout?repo=hrl). However, it does not publish the static end-effector pose. Rather, it publishes joint states and relies on robot_state_publisher to compute tfs.

Changes from the original version:
- Catkinized build system
- Compatibility with ROS Groovy
- General code cleanup
- Uses URDF description of Omni and the robot_state_publisher instead of hardcoded transforms.

To see it in action, simply:
roslaunch phantom_omni omni.launch
