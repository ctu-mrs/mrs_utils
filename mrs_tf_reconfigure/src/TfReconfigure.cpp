#include <TfReconfigure.h>

namespace mrs_tf_reconfigure
{

/* TfReconfigure() //{ */

TfReconfigure::TfReconfigure(rclcpp::NodeOptions options) : Node("TfReconfigure", options) {
  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "Initializing");
  mrs_lib::ParamLoader pl(node_);

  mrs_lib::TimerHandlerOptions timer_opts;
  timer_opts.node      = node_;
  timer_opts.autostart = true;

  timer_tf_ = std::make_shared<TimerType>(timer_opts, rclcpp::Rate(rate_timer_tf_, clock_), std::bind(&TfReconfigure::timerTf, this));

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
  reconfigure_server_->register_param("child.o_pitch", &drs_params_.child_o_pitch, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("child.o_roll", &drs_params_.child_o_roll, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("child.o_yaw", &drs_params_.child_o_yaw, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("child.x", &drs_params_.child_x, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("child.y", &drs_params_.child_y, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("child.z", &drs_params_.child_z, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);

  // g_child
  reconfigure_server_->register_param("g_child.o_pitch", &drs_params_.g_child_o_pitch, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_child.o_roll", &drs_params_.g_child_o_roll, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_child.o_yaw", &drs_params_.g_child_o_yaw, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_child.x", &drs_params_.g_child_x, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_child.y", &drs_params_.g_child_y, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_child.z", &drs_params_.g_child_z, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);

  // g_g_child
  reconfigure_server_->register_param("g_g_child.o_pitch", &drs_params_.g_g_child_o_pitch, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_g_child.o_roll", &drs_params_.g_g_child_o_roll, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_g_child.o_yaw", &drs_params_.g_g_child_o_yaw, 0.0, mrs_lib::DynparamMgr::range_t<double>(-3.14, 3.14), callback);
  reconfigure_server_->register_param("g_g_child.x", &drs_params_.g_g_child_x, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_g_child.y", &drs_params_.g_g_child_y, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);
  reconfigure_server_->register_param("g_g_child.z", &drs_params_.g_g_child_z, 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0), callback);

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

  if (params_dirty_) {
    auto drs_params = mrs_lib::get_mutexed(mutex_reconfigure_, drs_params_);

    modified_g_child_ |= (drs_params.g_child_o_roll != 0.0) || (drs_params.g_child_o_pitch != 0.0) || (drs_params.g_child_o_yaw != 0.0) ||
                         (drs_params.g_child_x != 0.0) || (drs_params.g_child_y != 0.0) || (drs_params.g_child_z != 0.0);

    modified_g_g_child_ |= (drs_params.g_g_child_o_roll != 0.0) || (drs_params.g_g_child_o_pitch != 0.0) || (drs_params.g_g_child_o_yaw != 0.0) ||
                           (drs_params.g_g_child_x != 0.0) || (drs_params.g_g_child_y != 0.0) || (drs_params.g_g_child_z != 0.0);

    tf2::Quaternion q;
    q.setRPY(drs_params.child_o_roll, drs_params.child_o_pitch, drs_params.child_o_yaw);
    q.normalize();

    tf2::Quaternion q2;
    q2.setRPY(drs_params.g_child_o_roll, drs_params.g_child_o_pitch, drs_params.g_child_o_yaw);
    q2.normalize();

    tf2::Quaternion q3;
    q3.setRPY(drs_params.g_g_child_o_roll, drs_params.g_g_child_o_pitch, drs_params.g_g_child_o_yaw);
    q3.normalize();

    /* RCLCPP_INFO(node_->get_logger(), "quaternion: x: %f y: %f z: %f w: %f", q.getX(), q.getY(), q.getZ(), q.getW()); */

    {
      std::scoped_lock lock(mutex_tf_);
      t1_transform_.transform.translation.x = drs_params.child_x;
      t1_transform_.transform.translation.y = drs_params.child_y;
      t1_transform_.transform.translation.z = drs_params.child_z;
      tf2::convert(q, t1_transform_.transform.rotation);

      t2_transform_.transform.translation.x = drs_params.g_child_x;
      t2_transform_.transform.translation.y = drs_params.g_child_y;
      t2_transform_.transform.translation.z = drs_params.g_child_z;
      tf2::convert(q2, t2_transform_.transform.rotation);

      t3_transform_.transform.translation.x = drs_params.g_g_child_x;
      t3_transform_.transform.translation.y = drs_params.g_g_child_y;
      t3_transform_.transform.translation.z = drs_params.g_g_child_z;
      tf2::convert(q3, t3_transform_.transform.rotation);
    }

    broadcastTransforms();
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

    params_dirty_ = false;
  } else
    broadcastTransforms();
}

//}

/* //{ callbackReconfigure() */

void TfReconfigure::callbackReconfigure([[maybe_unused]] const double &value) {
  if (!is_initialized_) {
    return;
  }

  params_dirty_ = true;
}

//}

} // namespace mrs_tf_reconfigure

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_tf_reconfigure::TfReconfigure)
