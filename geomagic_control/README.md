geomagic_phantom
============

ROS Node for Geomagic Touch ethernet devices.

Device name (the name of the device configured using geomagic_touch_setup) and publish rate
can be adjusted in the geomagic_headless.launch file. Defaults are used if not set.
You can also set the "prefix" in the launch file which would preceed all the topic names. Default is "/Geomagic"

Publishes:
- <prefix>/joint_states (sensor_msgs/JointState): The state of each of the geomagic's joints.
- <prefix>/button (geomagic_control/DeviceButtonEvent): Events for the grey and white buttons.
- <prefix>/pose (geometry_msgs/PoseStamped): End Effector Pose of Device.
- <prefix>/joint_states (sensor_msgs/JointState): Joint values/names of the device.

Subscribes:
- <prefix>/force_feedback (geomagic_control/DeviceFeedback): Force feedback to be displayed on the device. Takes a force and a position. If you simultaneously click the grey and white buttons, the geomagic will 'lock' to the position.

This is based on the [package of Dane Powell](https://github.com/danepowell/phantom_omni). However, it has several advantages:
- Compatible with Ubuntu 14.04LTS/16.04LTS and ROS Indigo & Kinetic
- Uses the more beautiful URDF model from [Francisco](https://github.com/fsuarez6/phantom_omni/tree/hydro-devel/omni_description)

To see it in action, simply:
```
roslaunch geomagic_control geomagic.launch
```

In case you have some troubles and your first 3 values are 0, you probably don't have English set as main language. Thats a problem coming from Open Haptics. Simply add this to your ~/.bashrc
```
export LC_ALL=en_US.UTF-8
```
