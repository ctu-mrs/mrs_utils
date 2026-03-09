#include <TfReconfigure.h>

namespace mrs_tf_reconfigure
{

/* TfReconfigure() //{ */

TfReconfigure::TfReconfigure(rclcpp::NodeOptions options) : Node("TfReconfigure", options) {
  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "Initializing");
  mrs_lib::ParamLoader pl(node_);

  pl.loadParam("frame_parent", frame_parent_, std::string("parent"));
  pl.loadParam("frame_child", frame_child_, std::string("child"));
  pl.loadParam("frame_grandchild", frame_grandchild_, std::string("g_child"));
  pl.loadParam("frame_greatgrandchild", frame_greatgrandchild_, std::string("g_g_child"));

  timer_tf_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / rate_timer_tf_), std::bind(&TfReconfigure::timerTf, this));

  t1_transform_.transform.translation.x = 0.0;
  t1_transform_.transform.translation.y = 0.0;
  t1_transform_.transform.translation.z = 0.0;
  tf2::convert(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), t1_transform_.transform.rotation);

  t2_transform_.transform.translation.x = 0.0;
  t2_transform_.transform.translation.y = 0.0;
  t2_transform_.transform.translation.z = 0.0;
  tf2::convert(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), t2_transform_.transform.rotation);

  t3_transform_.transform.translation.x = 0.0;
  t3_transform_.transform.translation.y = 0.0;
  t3_transform_.transform.translation.z = 0.0;
  tf2::convert(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), t3_transform_.transform.rotation);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  reconfigure_server_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_reconfigure_);
  // reconfigure_server_->get_param_provider().copyYamls(pl.getParamProvider());

  // Create callback wrapper with matching signature
  std::function<void(const double &)> callback = std::bind(&TfReconfigure::callbackReconfigure, this, std::placeholders::_1);

  // child
  reconfigure_server_->register_param("child.x", &drs_params_.child_x, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("child.y", &drs_params_.child_y, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("child.z", &drs_params_.child_z, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("child.yaw", &drs_params_.child_yaw, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("child.pitch", &drs_params_.child_pitch, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("child.roll", &drs_params_.child_roll, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);

  // g_child
  reconfigure_server_->register_param("g_child.x2", &drs_params_.g_child_x2, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_child.y2", &drs_params_.g_child_y2, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_child.z2", &drs_params_.g_child_z2, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_child.yaw2", &drs_params_.g_child_yaw2, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_child.pitch2", &drs_params_.g_child_pitch2, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_child.roll2", &drs_params_.g_child_roll2, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);

  // g_g_child
  reconfigure_server_->register_param("g_g_child.x3", &drs_params_.g_g_child_x3, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_g_child.y3", &drs_params_.g_g_child_y3, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_g_child.z3", &drs_params_.g_g_child_z3, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_g_child.yaw3", &drs_params_.g_g_child_yaw3, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_g_child.pitch3", &drs_params_.g_g_child_pitch3, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_g_child.roll3", &drs_params_.g_g_child_roll3, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  br_             = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* broadcastTransforms() //{ */

void TfReconfigure::broadcastTransforms() {

  if (!is_initialized_) {
    return;
  }

  if (modified_g_g_child_) {
    std::scoped_lock lock(mutex_tf_);

    geometry_msgs::msg::TransformStamped ts1;
    ts1.header.stamp    = clock_->now();
    ts1.header.frame_id = frame_parent_;
    ts1.child_frame_id  = frame_child_;
    ts1.transform       = t1_transform_.transform;
    br_->sendTransform(ts1);

    geometry_msgs::msg::TransformStamped ts2;
    ts2.header.stamp    = clock_->now();
    ts2.header.frame_id = frame_child_;
    ts2.child_frame_id  = frame_grandchild_;
    ts2.transform       = t2_transform_.transform;
    br_->sendTransform(ts2);

    geometry_msgs::msg::TransformStamped ts3;
    ts3.header.stamp    = clock_->now();
    ts3.header.frame_id = frame_grandchild_;
    ts3.child_frame_id  = frame_greatgrandchild_;
    ts3.transform       = t3_transform_.transform;
    br_->sendTransform(ts3);

  } else if (modified_g_child_) {
    std::scoped_lock lock(mutex_tf_);

    geometry_msgs::msg::TransformStamped ts1;
    ts1.header.stamp    = clock_->now();
    ts1.header.frame_id = frame_parent_;
    ts1.child_frame_id  = frame_child_;
    ts1.transform       = t1_transform_.transform;
    br_->sendTransform(ts1);

    geometry_msgs::msg::TransformStamped ts2;
    ts2.header.stamp    = clock_->now();
    ts2.header.frame_id = frame_child_;
    ts2.child_frame_id  = frame_grandchild_;
    ts2.transform       = t2_transform_.transform;
    br_->sendTransform(ts2);

  } else {
    std::scoped_lock lock(mutex_tf_);

    geometry_msgs::msg::TransformStamped ts1;
    ts1.header.stamp    = clock_->now();
    ts1.header.frame_id = frame_parent_;
    ts1.child_frame_id  = frame_child_;
    ts1.transform       = t1_transform_.transform;
    br_->sendTransform(ts1);
  }
}

//}

/* timerTf() //{ */

void TfReconfigure::timerTf() {

  if (!is_initialized_)
    return;
  broadcastTransforms();
}

//}

/* //{ callbackReconfigure() */
void TfReconfigure::callbackReconfigure([[maybe_unused]] const double &dummy) {

  if (!is_initialized_) {
    return;
  }

  // auto drs_params = mrs_lib::get_mutexed(mutex_reconfigure_, drs_params_);

  modified_g_child_ |= (drs_params_.g_child_roll2 != 0.0) | (drs_params_.g_child_pitch2 != 0.0) | (drs_params_.g_child_yaw2 != 0.0) |
                       (drs_params_.g_child_x2 != 0.0) | (drs_params_.g_child_y2 != 0.0) | (drs_params_.g_child_z2 != 0.0);

  modified_g_g_child_ |= (drs_params_.g_g_child_roll3 != 0.0) | (drs_params_.g_g_child_pitch3 != 0.0) | (drs_params_.g_g_child_yaw3 != 0.0) |
                         (drs_params_.g_g_child_x3 != 0.0) | (drs_params_.g_g_child_y3 != 0.0) | (drs_params_.g_g_child_z3 != 0.0);

  tf2::Quaternion q;
  q.setRPY(drs_params_.child_roll, drs_params_.child_pitch, drs_params_.child_yaw);
  q.normalize();

  tf2::Quaternion q2;
  q2.setRPY(drs_params_.g_child_roll2, drs_params_.g_child_pitch2, drs_params_.g_child_yaw2);
  q2.normalize();

  tf2::Quaternion q3;
  q3.setRPY(drs_params_.g_g_child_roll3, drs_params_.g_g_child_pitch3, drs_params_.g_g_child_yaw3);
  q3.normalize();

  /* RCLCPP_INFO(node_->get_logger(), "quaternion: x: %f y: %f z: %f w: %f", q.getX(), q.getY(), q.getZ(), q.getW()); */

  {
    std::scoped_lock lock(mutex_tf_);
    t1_transform_.transform.translation.x = drs_params_.child_x;
    t1_transform_.transform.translation.y = drs_params_.child_y;
    t1_transform_.transform.translation.z = drs_params_.child_z;
    tf2::convert(q, t1_transform_.transform.rotation);

    t2_transform_.transform.translation.x = drs_params_.g_child_x2;
    t2_transform_.transform.translation.y = drs_params_.g_child_y2;
    t2_transform_.transform.translation.z = drs_params_.g_child_z2;
    tf2::convert(q2, t2_transform_.transform.rotation);

    t3_transform_.transform.translation.x = drs_params_.g_g_child_x3;
    t3_transform_.transform.translation.y = drs_params_.g_g_child_y3;
    t3_transform_.transform.translation.z = drs_params_.g_g_child_z3;
    tf2::convert(q3, t3_transform_.transform.rotation);
  }

  // broadcastTransforms();
  broadcastTransforms();

  geometry_msgs::msg::TransformStamped tf;

  std::optional<geometry_msgs::msg::TransformStamped> ret;

  if (modified_g_g_child_) {
    ret = transformer_->getTransform(frame_greatgrandchild_, frame_parent_, clock_->now());
  } else if (modified_g_child_) {
    ret = transformer_->getTransform(frame_grandchild_, frame_parent_, clock_->now());
  } else {
    ret = transformer_->getTransform(frame_child_, frame_parent_, clock_->now());
  }

  if (ret) {
    tf = ret.value();
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Error in TF transforming!");
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), " \n\n\n\n\n\n\n\n -------------------------------------------- \n");
  // RCLCPP_INFO_STREAM(node_->get_logger(), "TF:\n" << tf.transform);

  double          sim_yaw, sim_pitch, sim_roll;
  tf2::Quaternion quaternion;
  tf2::convert(tf.transform.rotation, quaternion);
  tf2::Matrix3x3 m(quaternion);

  m.getRPY(sim_roll, sim_pitch, sim_yaw);

  RCLCPP_INFO_STREAM(node_->get_logger(), "Angles for gazebo:  R: " << sim_roll << "  P: " << sim_pitch << "  Y: " << sim_yaw);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Angles for tf_static:   " << sim_yaw << " " << sim_pitch << " " << sim_roll);
}
//}

} // namespace mrs_tf_reconfigure

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_tf_reconfigure::TfReconfigure)
