#ifndef TFRECONFIGURE_H
#define TFRECONFIGURE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include <mrs_lib/transformer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include <mrs_lib/param_loader.h>

#include <tf_reconfigure/tfConfig.h>

#include <mutex>

namespace tf_reconfigure
{

/* class TfReconfigure //{ */

class TfReconfigure : public nodelet::Nodelet {

public:
  virtual void onInit();

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

  tf::TransformBroadcaster br_;
  tf::TransformListener    listener_;

  tf::Transform t1_transform_;
  tf::Transform t2_transform_;
  tf::Transform t3_transform_;

  boost::recursive_mutex mutex_reconfigure_;

  boost::shared_ptr<dynamic_reconfigure::Server<tf_reconfigure::tfConfig>> reconfigure_server_;

  ros::Timer      timer_tf_;

  double rate_timer_tf_ = 1.0;

  void timerTf(const ros::TimerEvent& event);
  void callbackReconfigure([[maybe_unused]] tf_reconfigure::tfConfig& config, [[maybe_unused]] uint32_t level);
  void broadcastTransforms();

  //}
};

}  // namespace tf_reconfigure

#endif
