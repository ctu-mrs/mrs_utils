# ROS TF connector

***See also:** [MRS system's frames of reference documentation](https://ctu-mrs.github.io/docs/system/frames_of_reference.html) (with illustrations!)*

This package serves to connect different transform trees in [ROS' tf2 transformation framework](http://wiki.ros.org/tf2/).
The *tf2* library only supports tree relations between coordinate frames to prevent ambivalence in calculation of transforms between two frames.
Therefore, it's impossible to directly connect inner nodes or leaves of two (or more) trees, because that would create a cycle in the transform graph.
This causes problems when the user wants to connect e.g. transform subtrees of two robots through frames that correspond to the same physical frame (such as a local GPS frame etc.).

This package solves this issue by connecting the roots of the corresponding trees via transforms that are dynamically recalculated so that the total transform between the selected inner nodes stays the same.

## Example use-case scenario

A configuration for a typical usage example is provided in the default config file `config/tf_connector.yaml`.
Two trees, each corresponding to a different UAV, are to be connected.
The roots of these trees are `uav1/fcu` and `uav2/fcu`, corresponding to the UAVs' Flight Control Units (as is typical in the [MRS system](https://ctu-mrs.github.io/docs/system/frames_of_reference.html)).
Both UAVs are localized in the same local GPS frame, although it corresponds to a different frame ID in the trees - `uav1/gps_origin` and `uav2/gps_origin`, respectively.

The frame IDs `uav1/fcu` and `uav2/fcu` are the **root frame IDs** of the UAV1's transform tree and UAV2's transform tree.
The frame IDs `uav1/gps_origin` and `uav2/gps_origin` are the **equal frame IDs** in the two transform trees.
The trees will be connected through a **common frame ID** called `common_origin` (it's name doesn't matter much, just make sure that it doesn't overlap with any existing frame IDs) through transforms from the **root frames**.
These transforms will be calculated and automatically updated by the *TF connector* so that the **equal frames** always correspond to the same frame.

You can test this by spawning two UAVs in the [MRS simulation](https://github.com/ctu-mrs/mrs_simulation) called UAV1 and UAV2 and running `roslaunch tf_connector tf_connector.launch`.

## Advanced functionality

### Offsets

If you need to specify offsets between the equal frames (technically making them no longer equal), you can do that in the config file - see the `config/tf_connector_offsets.yaml` example.
The offsets can be specified as intrinsic, extrinsic or both.
The intrinsic offset is applied in the **root frame** (typically the UAV's FCU frame, hence intrinsic) and the extrinsic in the **equal frame** (typically the static frame, hence extrinsic).

It's even possible to specify an array of offsets with time stamps instead of a single one.
Then, the *TF connector* will interpolate between these offsets based on their stamps.
This is especially useful e.g. for correcting GPS drift.
Take care to provide a sensible ROS time (e.g. run a rosbag with the `--clock` parameter and set the `use_sim_time` ROS parameter to `true`).

### Maximal update period

When working with rosbags and static transforms, it's sometimes useful to periodically republish the connecting transforms e.g. to force Rviz to update.
You can use the `max_update_period` parameter for this.
Also check the `ignore_older_messages` parameter if you only want to use the newest transforms message, which can sometimes prevent some jumps in the output.
