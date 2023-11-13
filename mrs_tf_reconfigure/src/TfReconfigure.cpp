#include <TfReconfigure.h>

namespace tf_reconfigure
{

/* onInit() //{ */

void TfReconfigure::onInit() {

  ROS_INFO("[TfReconfigure]: Initializing");
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader pl(nh_);

  pl.loadParam("frame_parent", frame_parent_, std::string("parent"));
  pl.loadParam("frame_child", frame_child_, std::string("child"));
  pl.loadParam("frame_grandchild", frame_grandchild_, std::string("g_child"));
  pl.loadParam("frame_greatgrandchild", frame_greatgrandchild_, std::string("g_g_child"));


  timer_tf_ = nh_.createTimer(ros::Rate(rate_timer_tf_), &TfReconfigure::timerTf, this);

  t1_transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t1_transform_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  t2_transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t2_transform_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  t3_transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t3_transform_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  reconfigure_server_.reset(new dynamic_reconfigure::Server<tf_reconfigure::tfConfig>(mutex_reconfigure_, nh_));
  dynamic_reconfigure::Server<tf_reconfigure::tfConfig>::CallbackType f = boost::bind(&TfReconfigure::callbackReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  transformer_ = std::make_unique<mrs_lib::Transformer>("tf_reconfigure");
  transformer_->retryLookupNewest(true);

  is_initialized_ = true;

  ROS_INFO("[TfReconfigure]: initialized");
}

//}

/* broadcastTransforms() //{ */

void TfReconfigure::broadcastTransforms() {

  if (!is_initialized_) {
    return;
  }

  if (modified_g_g_child_) {
    std::scoped_lock lock(mutex_tf_);

    ros::Time br_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(t1_transform_, br_time, frame_parent_, frame_child_));
    listener_.waitForTransform(frame_parent_, frame_child_, br_time, ros::Duration(0.5));

    br_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(t2_transform_, br_time, frame_child_, frame_grandchild_));
    listener_.waitForTransform(frame_child_, frame_grandchild_, br_time, ros::Duration(0.5));

    br_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(t3_transform_, br_time, frame_grandchild_, frame_greatgrandchild_));
    listener_.waitForTransform(frame_grandchild_, frame_greatgrandchild_, br_time, ros::Duration(0.5));

  } else if (modified_g_child_) {
    std::scoped_lock lock(mutex_tf_);

    ros::Time br_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(t1_transform_, br_time, frame_parent_, frame_child_));
    listener_.waitForTransform(frame_parent_, frame_child_, br_time, ros::Duration(0.5));

    br_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(t2_transform_, br_time, frame_child_, frame_grandchild_));
    listener_.waitForTransform(frame_child_, frame_grandchild_, br_time, ros::Duration(0.5));

  } else {
    std::scoped_lock lock(mutex_tf_);

    ros::Time br_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(t1_transform_, br_time, frame_parent_, frame_child_));
    listener_.waitForTransform(frame_parent_, frame_child_, br_time, ros::Duration(0.5));
  }
}

//}

/* timerTf() //{ */

void TfReconfigure::timerTf(const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;
  broadcastTransforms();
}

//}

/* //{ callbackReconfigure() */
void TfReconfigure::callbackReconfigure([[maybe_unused]] tf_reconfigure::tfConfig& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_) {
    return;
  }

  modified_g_child_ |= (config.roll2 != 0.0) | (config.pitch2 != 0.0) | (config.yaw2 != 0.0) | (config.x2 != 0.0) | (config.y2 != 0.0) | (config.z2 != 0.0);
  modified_g_g_child_ |= (config.roll3 != 0.0) | (config.pitch3 != 0.0) | (config.yaw3 != 0.0) | (config.x3 != 0.0) | (config.y3 != 0.0) | (config.z3 != 0.0);

  tf::Quaternion q;
  q.setRPY(config.roll, config.pitch, config.yaw);
  q.normalize();

  tf::Quaternion q2;
  q2.setRPY(config.roll2, config.pitch2, config.yaw2);
  q2.normalize();

  tf::Quaternion q3;
  q3.setRPY(config.roll3, config.pitch3, config.yaw3);
  q3.normalize();

  /* ROS_INFO("[TfReconfigure]: quaternion: x: %f y: %f z: %f w: %f", q.getX(), q.getY(), q.getZ(), q.getW()); */

  {
    std::scoped_lock lock(mutex_tf_);
    t1_transform_.setOrigin(tf::Vector3(config.x, config.y, config.z));
    t1_transform_.setRotation(q);

    t2_transform_.setOrigin(tf::Vector3(config.x2, config.y2, config.z2));
    t2_transform_.setRotation(q2);

    t3_transform_.setOrigin(tf::Vector3(config.x3, config.y3, config.z3));
    t3_transform_.setRotation(q3);
  }

  broadcastTransforms();
  broadcastTransforms();

  geometry_msgs::TransformStamped tf;

  std::optional<geometry_msgs::TransformStamped> ret;

  if (modified_g_g_child_) {
    ret = transformer_->getTransform(frame_greatgrandchild_, frame_parent_, ros::Time::now());
  } else if (modified_g_child_) {
    ret = transformer_->getTransform(frame_grandchild_, frame_parent_, ros::Time::now());
  } else {
    ret = transformer_->getTransform(frame_child_, frame_parent_, ros::Time::now());
  }

  if (ret) {
    tf = ret.value();
  } else {
    ROS_ERROR_STREAM("[Tf Reconfigure]: Error in TF transforming!");
  }
  ROS_INFO_STREAM(" \n\n\n\n\n\n\n\n -------------------------------------------- \n");
  ROS_INFO_STREAM("TF:\n" << tf.transform);

  double         sim_yaw, sim_pitch, sim_roll;
  tf::Quaternion quaternion;
  quaternionMsgToTF(tf.transform.rotation, quaternion);
  tf::Matrix3x3 m(quaternion);

  m.getRPY(sim_roll, sim_pitch, sim_yaw);

  ROS_INFO_STREAM("Angles for gazebo:  R: " << sim_roll << "  P: " << sim_pitch << "  Y: " << sim_yaw);
  ROS_INFO_STREAM("Angles for tf_static:   " << sim_yaw << " " << sim_pitch << " " << sim_roll);
}
//}

}  // namespace tf_reconfigure

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tf_reconfigure::TfReconfigure, nodelet::Nodelet)
