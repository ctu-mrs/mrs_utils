# ROS TF connector

***See also:** [MRS system's frames of reference documentation](https://ctu-mrs.github.io/docs/system/frames_of_reference.html) (with illustrations!)*

This package serves to connect different transform trees in [ROS 2's tf2 transformation framework](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html).
The *tf2* library only supports tree relations between coordinate frames to prevent ambivalence in calculation of transforms between two frames.
Therefore, it's impossible to directly connect inner nodes or leaves of two (or more) trees, because that would create a cycle in the transform graph.
This causes problems when the user wants to connect e.g. transform subtrees of two robots through frames that correspond to the same physical frame (such as a local GPS frame etc.).

This package solves this issue by connecting the roots of the corresponding trees via transforms that are dynamically recalculated so that the total transform between the selected inner nodes stays the same.

## Example use-case scenario

A configuration for a typical usage example is provided in the default config file `config/tf_connector.yaml`.
Two trees, each corresponding to a different UAV, are to be connected.
The roots of these trees are `uav1/fcu` and `uav2/fcu`, corresponding to the UAVs' Flight Control Units (as is typical in the [MRS system](https://ctu-mrs.github.io/docs/system/frames_of_reference.html)).
Both UAVs share the same world origin (set by the same `world_origin` in their world configs), although it corresponds to a different frame ID in the trees - `uav1/world_origin` and `uav2/world_origin`, respectively.

The frame IDs `uav1/fcu` and `uav2/fcu` are the **root frame IDs** of the UAV1's transform tree and UAV2's transform tree.
The frame IDs `uav1/world_origin` and `uav2/world_origin` are the **equal frame IDs** in the two transform trees.
The trees will be connected through a **common frame ID** called `common_origin` (it's name doesn't matter much, just make sure that it doesn't overlap with any existing frame IDs) through transforms from the **root frames**.
These transforms will be calculated and automatically updated by the *TF connector* so that the **equal frames** always correspond to the same frame.

You can test this by spawning two UAVs called `uav1` and `uav2` in the [MRS Gazebo simulator](https://github.com/ctu-mrs/mrs_uav_gazebo_simulator) and running `ros2 launch mrs_tf_connector tf_connector.launch.py`.
Don't use it with the [MRS multirotor simulator](https://github.com/ctu-mrs/mrs_multirotor_simulator) - it already connects the UAVs through its `simulator_origin` frame, and a second parent of `uavX/fcu` would break the tree.

## Advanced functionality

### Custom config

Instead of editing the default config, pass your own file to the launch file:

```bash
ros2 launch mrs_tf_connector tf_connector.launch.py custom_config:=./tf_connector.yaml
```

The path can be absolute or relative to the current working directory.
Single parameters (e.g. `max_update_period`) from the custom config override the defaults.
If the custom config contains `connections`, this list replaces the default one as a whole, e.g.:

```yaml
connections:
  - root_frame_id: uav1/fcu
    equal_frame_id: uav1/world_origin
  - root_frame_id: uav2/fcu
    equal_frame_id: uav2/world_origin
  - root_frame_id: uav3/fcu
    equal_frame_id: uav3/world_origin
```

### Offsets

If you need to specify offsets between the equal frames (technically making them no longer equal), you can do that in the config file - see the commented `offsets` example in `config/tf_connector.yaml`.
The offsets can be specified as intrinsic, extrinsic or both.
The intrinsic offset is applied in the **root frame** (typically the UAV's FCU frame, hence intrinsic) and the extrinsic in the **equal frame** (typically the static frame, hence extrinsic).

It's even possible to specify an array of offsets with time stamps instead of a single one.
Then, the *TF connector* will interpolate between these offsets based on their stamps.
This is especially useful e.g. for correcting GPS drift.
Take care to provide a sensible ROS time (e.g. play a rosbag with `ros2 bag play --clock` and set the `use_sim_time` ROS parameter to `true`).

### Maximal update period

When working with rosbags and static transforms, it's sometimes useful to periodically republish the connecting transforms e.g. to force RViz to update.
You can use the `max_update_period` parameter for this.
Also check the `ignore_older_messages` parameter if you only want to use the newest transforms message, which can sometimes prevent some jumps in the output.
