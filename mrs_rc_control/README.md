# MRS RC control

Package for controlling behavior of the UAV with the use of RC switches.

# RC trajectory tracking trigger
RC trajectory tracking trigger node enables to trigger sequence of services:

* go to trajectory start,
* start trajectory tracking,
* stop trajectory tracking,

by changing the position of switch on remote controller.

## Usage
* launch RC trajectory tracking node,
* load trajectory through topic /uav_name/control_manager/trajectory_reference,
* takeoff with the UAV,
* put the switch in HIGH position to call service /uav_name/control_manager/goto_trajectory_start,
* put the switch in LOW position to call service /uav_name/control_manager/start_trajectory_tracking,
* if needed, put the switch in HIGH position to call service /uav_name/control_manager/stop_trajectory_tracking.

The trigerring of services is allowed only if a trajectory is already loaded and the UAV is flying normally (according to control manager diagnostics).
The switch has to be in LOW position at the beginning of the control sequence. Otherwise its state and transitions are ignored until it is put back into LOW position.

## IMPORTANT
By default, RC trajectory tracking trigger uses the same channel as is used for activation of RC joystick control by control manager. For this reason, the RC Joystick control has to be always disabled when using RC trajectory tracking trigger in default configuration.
