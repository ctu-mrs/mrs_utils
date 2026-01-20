/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/subscribe_handler.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen3/Eigen/Eigen>

//}

namespace mrs_tf_mirror
{

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

  std::shared_ptr<ros::Timer> timer;

} Params_t;

//}

/* class MrsTfMirror //{ */

class TfMirror : public nodelet::Nodelet {

public:
  virtual void onInit();

  bool is_initialized_ = false;

private:
  ros::NodeHandle nh_;

  std::map<std::string, ros::Timer> tf_timers_;

  void timerTf(const ros::TimerEvent& event, const Params_t params);

  std::vector<std::string> _tfs_;

  std::string _uav_name_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ------------------------ callbacks ----------------------- |

  std::vector<mrs_lib::SubscribeHandler<nav_msgs::Odometry>> sh_odoms_;

  void callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg, const Params_t params);

  // | ------------------ static tf broadcaster ----------------- |

  tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_;
  std::mutex                          mutex_tf_static_broadcaster_;

  // | ----------------- dynamic tf broadcaster ----------------- |

  std::shared_ptr<mrs_lib::TransformBroadcaster> tf_dynamic_broadcaster_;
  std::mutex                                     mutex_tf_dynamic_broadcaster_;

  // | ------------------------ routines ------------------------ |

  tf2::Transform tf2FromPose(const geometry_msgs::Pose& pose_in);

  geometry_msgs::Transform msgFromTf2(const tf2::Transform& tf_in);

  tf2::Transform tf2FromMsg(const geometry_msgs::Transform& tf_in);

  geometry_msgs::Pose poseFromTf2(const tf2::Transform& tf_in);

  geometry_msgs::Vector3 pointToVector3(const geometry_msgs::Point& point_in);
};

//}

/* onInit() //{ */

void TfMirror::onInit() {

  ROS_INFO("[TfMirror]: Initializing");

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // | --------------------- tf broadcasters -------------------- |

  tf_dynamic_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>();

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "TfMirror");
  transformer_->retryLookupNewest(true);

  // | --------------------- subscriber opts -------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = _uav_name_;
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "TfMirror");

  param_loader.loadParam("UAV_NAME", _uav_name_);

  param_loader.loadParam("tfs", _tfs_);

  for (auto& tf : _tfs_) {

    Params_t params;

    params.tf_name = tf;

    param_loader.loadParam(tf + "/source/tf/enabled", params.tf_source_enabled, false);
    param_loader.loadParam(tf + "/source/odometry/enabled", params.odom_source_enabled, false);

    if (params.tf_source_enabled && params.odom_source_enabled) {
      ROS_ERROR("[TfMirror]: both TF and Odom source are enabled for '%s'", tf.c_str());
      ros::shutdown();
    }

    if (!params.tf_source_enabled && !params.odom_source_enabled) {
      ROS_ERROR("[TfMirror]: neither TF and Odom source are enabled for '%s'", tf.c_str());
      ros::shutdown();
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
      ROS_ERROR("[TfMirror]: failed to load parameters");
      ros::shutdown();
    }

    // | ------------------- create a subscriber ------------------ |

    if (params.odom_source_enabled) {
      sh_odoms_.push_back(mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, params.odom_topic,
                                                                        std::bind(&TfMirror::callbackOdometry, this, std::placeholders::_1, params)));
    }

    if (params.tf_source_enabled) {

      tf_timers_[tf] = nh_.createTimer(ros::Duration(1.0 / params.tf_poll_rate), std::bind(&TfMirror::timerTf, this, std::placeholders::_1, params));
    }
  }

  // | ------------------------- timers ------------------------- |

  is_initialized_ = true;

  ROS_INFO("[TfMirror]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOdometry() //{ */

void TfMirror::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msg, const Params_t params) {

  if (!is_initialized_) {
    return;
  }

  tf2::Transform tf = tf2FromPose(msg->pose.pose);

  if (params.invert) {
    tf = tf.inverse();
  }

  const geometry_msgs::Pose pose = poseFromTf2(tf);

  geometry_msgs::TransformStamped tf_msg;

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

void TfMirror::timerTf(const ros::TimerEvent& event, const Params_t params) {

  if (!is_initialized_) {
    return;
  }

  auto tf_in = transformer_->getTransform(params.in_tf_from, params.in_tf_to);

  if (!tf_in) {
    ROS_WARN_THROTTLE(1.0, "[TfMirror]: could not find tf from '%s' to '%s'", params.in_tf_from.c_str(), params.in_tf_to.c_str());
    return;
  }

  tf2::Transform tf = tf2FromMsg(tf_in->transform);

  if (!params.invert) {
    tf = tf.inverse();
  }

  const geometry_msgs::Pose pose = poseFromTf2(tf);

  geometry_msgs::TransformStamped tf_msg;

  tf_msg.transform.translation = pointToVector3(pose.position);
  tf_msg.transform.rotation    = pose.orientation;

  tf_msg.header.stamp    = tf_in->header.stamp;
  tf_msg.header.frame_id = params.out_tf_from;
  tf_msg.child_frame_id  = params.out_tf_to;

  if (params.static_tf) {

    {
      std::scoped_lock lock(mutex_tf_static_broadcaster_);

      tf_static_broadcaster_.sendTransform(tf_msg);
    }

    ROS_INFO("[TfMirror]: stopping timer for static tf '%s'", params.tf_name.c_str());

    tf_timers_.at(params.tf_name).stop();

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

tf2::Transform TfMirror::tf2FromPose(const geometry_msgs::Pose& pose_in) {

  tf2::Vector3 position(pose_in.position.x, pose_in.position.y, pose_in.position.z);

  tf2::Quaternion q;
  tf2::fromMsg(pose_in.orientation, q);

  tf2::Transform tf_out(q, position);

  return tf_out;
}

//}

/* msgFromTf2() //{ */

geometry_msgs::Transform TfMirror::msgFromTf2(const tf2::Transform& tf_in) {

  geometry_msgs::Transform tf_out;

  tf_out.translation.x = tf_in.getOrigin().getX();
  tf_out.translation.y = tf_in.getOrigin().getY();
  tf_out.translation.z = tf_in.getOrigin().getZ();

  tf_out.rotation = tf2::toMsg(tf_in.getRotation());

  return tf_out;
}

//}

/* tf2FromMsg() //{ */

tf2::Transform TfMirror::tf2FromMsg(const geometry_msgs::Transform& tf_in) {

  tf2::Transform tf_out;

  tf_out.setOrigin(tf2::Vector3(tf_in.translation.x, tf_in.translation.y, tf_in.translation.z));
  tf_out.setRotation(tf2::Quaternion(tf_in.rotation.x, tf_in.rotation.y, tf_in.rotation.z, tf_in.rotation.w));

  return tf_out;
}

//}

/* poseFromTf2() //{ */

geometry_msgs::Pose TfMirror::poseFromTf2(const tf2::Transform& tf_in) {

  geometry_msgs::Pose pose_out;

  pose_out.position.x = tf_in.getOrigin().getX();
  pose_out.position.y = tf_in.getOrigin().getY();
  pose_out.position.z = tf_in.getOrigin().getZ();

  pose_out.orientation = tf2::toMsg(tf_in.getRotation());

  return pose_out;
}

//}

/* pointToVector3() //{ */

geometry_msgs::Vector3 TfMirror::pointToVector3(const geometry_msgs::Point& point_in) {

  geometry_msgs::Vector3 vec_out;

  vec_out.x = point_in.x;
  vec_out.y = point_in.y;
  vec_out.z = point_in.z;

  return vec_out;
}

//}

}  // namespace mrs_tf_mirror

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_tf_mirror::TfMirror, nodelet::Nodelet)
