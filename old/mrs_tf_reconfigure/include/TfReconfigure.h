#ifndef TFRECONFIGURE_H
#define TFRECONFIGURE_H

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/transformer.h>

#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>

#include <mutex>
#include <mrs_lib/mutex.h>

namespace mrs_tf_reconfigure
{

/* class TfReconfigure //{ */

class TfReconfigure : public mrs_lib::Node {

public:
  TfReconfigure(rclcpp::NodeOptions options);

  bool is_initialized_ = false;

private:
  std::string frame_parent_          = "parent";
  std::string frame_child_           = "child";
  std::string frame_grandchild_      = "g_child";
  std::string frame_greatgrandchild_ = "g_g_child";

  bool modified_g_child_   = false;
  bool modified_g_g_child_ = false;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  std::mutex mutex_tf_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  // tf2_ros::TransformListener                     listener_;

  geometry_msgs::msg::TransformStamped t1_transform_;
  geometry_msgs::msg::TransformStamped t2_transform_;
  geometry_msgs::msg::TransformStamped t3_transform_;

  struct DynParams_t
  {
    // child
    double child_x;
    double child_y;
    double child_z;
    double child_yaw;
    double child_pitch;
    double child_roll;

    // g_child
    double g_child_x2;
    double g_child_y2;
    double g_child_z2;
    double g_child_yaw2;
    double g_child_pitch2;
    double g_child_roll2;

    // g_g_child
    double g_g_child_x3;
    double g_g_child_y3;
    double g_g_child_z3;
    double g_g_child_yaw3;
    double g_g_child_pitch3;
    double g_g_child_roll3;
  };

  std::mutex                            mutex_reconfigure_;
  std::shared_ptr<mrs_lib::DynparamMgr> reconfigure_server_;
  DynParams_t                           drs_params_;

  rclcpp::TimerBase::SharedPtr timer_tf_;

  double rate_timer_tf_ = 1.0;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  void timerTf();
  void callbackReconfigure();
  void broadcastTransforms();

  //}
};

} // namespace mrs_tf_reconfigure

#endif
