Description
---------

ROS Interface for Geomagic Touch Haptic device

## Original Author (Phantom Omni Interface)
Dane Powell
https://github.com/danepowell/phantom_omni.git

## Authors (Conversion from Phatom Omni to Geomagic Touch)
Hai Nguyen, Marc Killpack, Chi-Hung King, Sven Bock, Prof. Charlie Kemp
https://github.com/HumaRobotics/geomagic_touch

## Extended Author and Maintainer
Adnan Munawar


## Ubuntu 16.06, 18.04 and 20.04:
First, please install the Openhaptics SDK from this location:

https://github.com/jhu-cisst-external/phantom-omni-1394-drivers

### How to run
After you build the package with ROS, you can run it as

```bash
roslaunch geomagic_control geomagic_headless.launch
```
This should start streaming the ROS topics with the states of the device.

### Known Issues
For Phantom Omni, you may need admin priviliges to access the device. The simplest way is to run the following command in your terminal.
```
sudo chmod a+rw /dev/fw*
```
Then you can try rerunning the roslauch command above.

### Deprecated:
```
sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
sudo ln -s /usr/lib/libPHANToMIO.so.4 /usr/lib/libPHANToMIO.so
sudo ln -s /usr/lib/libPHANToMIO.so.4 /usr/lib/libPhantomIOLib42.so
```
