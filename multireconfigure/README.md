# Multireconfigure

A simple Python tool for dynamic reconfiguration of multiple nodes simulatanously.
Just specify the dynparam topics in the launchfile or as a list-of-strings ROS parameter for the script.
The script will attempt to create a client for each of these topics.
Then, it will create a dynparam server with each variable from these clients available.

*Note:* It will probably not work with enums since I didn't need it and implementing it seemed like too much hassle.

## Prequisites

To install the prequisites, just run the following command:
```
sudo apt install ros-$ROS_DISTRO-ddynamic-reconfigure ros-$ROS_DISTRO-ddynamic-reconfigure-python
```
