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
#include <mrs_lib/timer_handler.h>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

namespace mrs_tf_reconfigure
{

/* class TfReconfigure //{ */

class TfReconfigure : public mrs_lib::Node {

public:
  TfReconfigure(rclcpp::NodeOptions options);

  bool is_initialized_ = false;

private:
  std::string frame_parent_          = "frame_parent";
  std::string frame_child_           = "frame_child";
  std::string frame_grandchild_      = "frame_grandchild";
  std::string frame_greatgrandchild_ = "frame_greatgrandchild";

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
    double child_o_pitch;
    double child_o_roll;
    double child_o_yaw;
    double child_x;
    double child_y;
    double child_z;

    // g_child
    double g_child_o_pitch;
    double g_child_o_roll;
    double g_child_o_yaw;
    double g_child_x;
    double g_child_y;
    double g_child_z;

    // g_g_child
    double g_g_child_o_pitch;
    double g_g_child_o_roll;
    double g_g_child_o_yaw;
    double g_g_child_x;
    double g_g_child_y;
    double g_g_child_z;
  };

  std::mutex                            mutex_reconfigure_;
  std::shared_ptr<mrs_lib::DynparamMgr> reconfigure_server_;
  DynParams_t                           drs_params_;

  std::shared_ptr<TimerType> timer_tf_;
  bool                       params_dirty_ = false;

  double rate_timer_tf_ = 20.0;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  void timerTf();
  void callbackReconfigure([[maybe_unused]] const double &value);
  void broadcastTransforms();

  //}
};

} // namespace mrs_tf_reconfigure

#endif
