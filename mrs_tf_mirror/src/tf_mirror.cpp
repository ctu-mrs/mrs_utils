/* includes //{ */

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/node.h>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/static_transform_broadcaster.hpp>

//}

namespace mrs_tf_mirror
{

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

/* structs //{ */

typedef struct
{

  std::string tf_name;

  bool tf_source_enabled;
  bool odom_source_enabled;

  double      tf_poll_rate;
  std::string odom_topic;

  std::string in_tf_from;
  std::string in_tf_to;

  bool invert;

  bool static_tf;

  std::string out_tf_from;
  std::string out_tf_to;

  std::shared_ptr<rclcpp::TimerBase::SharedPtr> timer;

} Params_t;

//}

/* class MrsTfMirror //{ */

class TfMirror : public mrs_lib::Node {

public:
  TfMirror(rclcpp::NodeOptions options);

  bool is_initialized_ = false;

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  std::map<std::string, std::shared_ptr<TimerType>> tf_timers_;

  void timerTf(const Params_t params);

  std::vector<std::string> _tfs_;

  std::string _uav_name_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ------------------------ callbacks ----------------------- |

  std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>> sh_odoms_;

  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const Params_t params);

  // | ------------------ static tf broadcaster ----------------- |

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::mutex                                           mutex_tf_static_broadcaster_;

  // | ----------------- dynamic tf broadcaster ----------------- |

  std::shared_ptr<mrs_lib::TransformBroadcaster> tf_dynamic_broadcaster_;
  std::mutex                                     mutex_tf_dynamic_broadcaster_;

  // | ------------------------ routines ------------------------ |

  tf2::Transform tf2FromPose(const geometry_msgs::msg::Pose &pose_in);

  geometry_msgs::msg::Transform msgFromTf2(const tf2::Transform &tf_in);

  tf2::Transform tf2FromMsg(const geometry_msgs::msg::Transform &tf_in);

  geometry_msgs::msg::Pose poseFromTf2(const tf2::Transform &tf_in);

  geometry_msgs::msg::Vector3 pointToVector3(const geometry_msgs::msg::Point &point_in);
};

//}

/* onInit() //{ */

TfMirror::TfMirror(rclcpp::NodeOptions options) : Node("TfMirror", options) {

  node_                  = this_node_ptr();
  clock_                 = node_->get_clock();
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

  RCLCPP_INFO(node_->get_logger(), "Initializing");

  // | --------------------- tf broadcasters -------------------- |

  tf_dynamic_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>();

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  // | --------------------- subscriber opts -------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = node_;
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;

  // | --------------------- timer opts -------------------- |

  mrs_lib::TimerHandlerOptions thopts;
  thopts.node      = node_;
  thopts.autostart = true;

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(node_);
  std::string          custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("tfs", _tfs_);

  for (auto &tf : _tfs_) {

    Params_t params;

    params.tf_name = tf;

    param_loader.loadParam(tf + "/source/tf/enabled", params.tf_source_enabled, false);
    param_loader.loadParam(tf + "/source/odometry/enabled", params.odom_source_enabled, false);

    if (params.tf_source_enabled && params.odom_source_enabled) {
      RCLCPP_ERROR(node_->get_logger(), "both TF and Odom source are enabled for '%s'", tf.c_str());
      std::exit(1);
      rclcpp::shutdown();
    }

    if (!params.tf_source_enabled && !params.odom_source_enabled) {
      RCLCPP_ERROR(node_->get_logger(), "neither TF and Odom source are enabled for '%s'", tf.c_str());
      std::exit(1);
      rclcpp::shutdown();
    }

    bool source_prepand_uav_name;
    param_loader.loadParam(tf + "/source/prepand_uav_name", source_prepand_uav_name);

    if (params.tf_source_enabled) {

      param_loader.loadParam(tf + "/source/tf/from", params.in_tf_from);
      param_loader.loadParam(tf + "/source/tf/to", params.in_tf_to);

      if (source_prepand_uav_name) {
        params.in_tf_from = _uav_name_ + "/" + params.in_tf_from;
        params.in_tf_to   = _uav_name_ + "/" + params.in_tf_to;
      }

      param_loader.loadParam(tf + "/source/tf/poll_rate", params.tf_poll_rate);
    }

    if (params.odom_source_enabled) {

      param_loader.loadParam(tf + "/source/odometry/topic", params.odom_topic);

      if (source_prepand_uav_name) {
        params.odom_topic = "/" + _uav_name_ + "/" + params.odom_topic;
      }
    }

    param_loader.loadParam(tf + "/result/invert", params.invert);

    param_loader.loadParam(tf + "/result/static_tf", params.static_tf);

    bool result_prepand_uav_name;
    param_loader.loadParam(tf + "/result/prepand_uav_name", result_prepand_uav_name);

    param_loader.loadParam(tf + "/result/parent_frame", params.out_tf_from);
    param_loader.loadParam(tf + "/result/child_frame", params.out_tf_to);

    if (result_prepand_uav_name) {
      params.out_tf_from = _uav_name_ + "/" + params.out_tf_from;
      params.out_tf_to   = _uav_name_ + "/" + params.out_tf_to;
    }

    if (!param_loader.loadedSuccessfully()) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load parameters");
      std::exit(1);
      rclcpp::shutdown();
    }

    // | ------------------- create a subscriber ------------------ |

    if (params.odom_source_enabled) {
      sh_odoms_.push_back(mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, params.odom_topic,
                                                                              std::bind(&TfMirror::callbackOdometry, this, std::placeholders::_1, params)));
    }

    if (params.tf_source_enabled) {

      tf_timers_[tf] = std::make_shared<TimerType>(thopts, rclcpp::Rate(params.tf_poll_rate, clock_), std::bind(&TfMirror::timerTf, this, params));
    }
  }

  // | ------------------------- timers ------------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOdometry() //{ */

void TfMirror::callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const Params_t params) {

  if (!is_initialized_) {
    return;
  }

  tf2::Transform tf = tf2FromPose(msg->pose.pose);

  if (params.invert) {
    tf = tf.inverse();
  }

  const geometry_msgs::msg::Pose pose = poseFromTf2(tf);

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.transform.translation = pointToVector3(pose.position);
  tf_msg.transform.rotation    = pose.orientation;

  tf_msg.header.stamp    = msg->header.stamp;
  tf_msg.header.frame_id = params.out_tf_from;
  tf_msg.child_frame_id  = params.out_tf_to;

  {
    std::scoped_lock lock(mutex_tf_dynamic_broadcaster_);

    tf_dynamic_broadcaster_->sendTransform(tf_msg);
  }
}

//}

/* timerTf() //{ */

void TfMirror::timerTf(const Params_t params) {

  if (!is_initialized_) {
    return;
  }

  auto tf_in = transformer_->getTransform(params.in_tf_from, params.in_tf_to);

  if (!tf_in) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not find tf from '%s' to '%s'", params.in_tf_from.c_str(), params.in_tf_to.c_str());
    return;
  }

  tf2::Transform tf = tf2FromMsg(tf_in->transform);

  if (!params.invert) {
    tf = tf.inverse();
  }

  const geometry_msgs::msg::Pose pose = poseFromTf2(tf);

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.transform.translation = pointToVector3(pose.position);
  tf_msg.transform.rotation    = pose.orientation;

  tf_msg.header.stamp    = tf_in->header.stamp;
  tf_msg.header.frame_id = params.out_tf_from;
  tf_msg.child_frame_id  = params.out_tf_to;

  if (params.static_tf) {

    {
      std::scoped_lock lock(mutex_tf_static_broadcaster_);

      tf_static_broadcaster_->sendTransform(tf_msg);
    }

    RCLCPP_INFO(node_->get_logger(), "stopping timer for static tf '%s'", params.tf_name.c_str());

    tf_timers_.at(params.tf_name)->stop();

  } else {

    {
      std::scoped_lock lock(mutex_tf_dynamic_broadcaster_);

      tf_dynamic_broadcaster_->sendTransform(tf_msg);
    }
  }
}

//}

// | ------------------------ routines ------------------------ |

/* tf2FromPose() //{ */

tf2::Transform TfMirror::tf2FromPose(const geometry_msgs::msg::Pose &pose_in) {

  tf2::Vector3 position(pose_in.position.x, pose_in.position.y, pose_in.position.z);

  tf2::Quaternion q;
  tf2::fromMsg(pose_in.orientation, q);

  tf2::Transform tf_out(q, position);

  return tf_out;
}

//}

/* msgFromTf2() //{ */

geometry_msgs::msg::Transform TfMirror::msgFromTf2(const tf2::Transform &tf_in) {

  geometry_msgs::msg::Transform tf_out;

  tf_out.translation.x = tf_in.getOrigin().getX();
  tf_out.translation.y = tf_in.getOrigin().getY();
  tf_out.translation.z = tf_in.getOrigin().getZ();

  tf_out.rotation = tf2::toMsg(tf_in.getRotation());

  return tf_out;
}

//}

/* tf2FromMsg() //{ */

tf2::Transform TfMirror::tf2FromMsg(const geometry_msgs::msg::Transform &tf_in) {

  tf2::Transform tf_out;

  tf_out.setOrigin(tf2::Vector3(tf_in.translation.x, tf_in.translation.y, tf_in.translation.z));
  tf_out.setRotation(tf2::Quaternion(tf_in.rotation.x, tf_in.rotation.y, tf_in.rotation.z, tf_in.rotation.w));

  return tf_out;
}

//}

/* poseFromTf2() //{ */

geometry_msgs::msg::Pose TfMirror::poseFromTf2(const tf2::Transform &tf_in) {

  geometry_msgs::msg::Pose pose_out;

  pose_out.position.x = tf_in.getOrigin().getX();
  pose_out.position.y = tf_in.getOrigin().getY();
  pose_out.position.z = tf_in.getOrigin().getZ();

  pose_out.orientation = tf2::toMsg(tf_in.getRotation());

  return pose_out;
}

//}

/* pointToVector3() //{ */

geometry_msgs::msg::Vector3 TfMirror::pointToVector3(const geometry_msgs::msg::Point &point_in) {

  geometry_msgs::msg::Vector3 vec_out;

  vec_out.x = point_in.x;
  vec_out.y = point_in.y;
  vec_out.z = point_in.z;

  return vec_out;
}

//}

} // namespace mrs_tf_mirror

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_tf_mirror::TfMirror)
