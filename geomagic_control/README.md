phantom_omni
============

ROS Node for Geomagic Touch ethernet devices.

Device name and publish rate can be adjusted in the geomagic_headless.launch file.

Publishes:
- OMNI_NAME_joint_states (sensor_msgs/JointState): The state of each of the omni's joints.
- OMNI_NAME_button (phantom_omni/PhantomButtonEvent): Events for the grey and white buttons.

Subscribes:
- OMNI_NAME_force_feedback (phantom_omni/OmniFeedback): Force feedback to be displayed on the omni. Takes a force and a position. If you simultaneously click the grey and white buttons, the omni will 'lock' to the position.

This is based on the [package of Dane Powell](https://github.com/danepowell/phantom_omni). However, it has several advantages:
- Compatible with Ubuntu 14.04LTS and ROS Indigo
- Uses the more beautiful URDF model from [Francisco](https://github.com/fsuarez6/phantom_omni/tree/hydro-devel/omni_description)

To see it in action, simply:
roslaunch geomagic_control geomagic.launch
