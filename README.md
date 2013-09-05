phantom_omni
============

ROS Node for Sensable Phantom Omni devices

Requires the [omni_description](https://github.com/danepowell/omni_description) package. 

Parameters:
- ~omni_name (default: omni1)

Publishes:
- OMNI_NAME_joint_states (sensor_msgs/JointState): The state of each of the omni's joints.
- OMNI_NAME_button (phantom_omni/PhantomButtonEvent): Events for the grey and white buttons.

Subscribes:
- OMNI_NAME_force_feedback (geometry_msgs/Wrench): Force feedback to be displayed on the omni.

This is based on the [original phantom_omni package](http://www.ros.org/wiki/phantom_omni). However, it has several advantages:
- Catkinized build system
- Compatibility with ROS Groovy
- Uses URDF description of Omni and the robot_state_publisher instead of hardcoded transforms.
- Streamlined code and organization.

To see it in action, simply:
roslaunch phantom_omni omni.launch
