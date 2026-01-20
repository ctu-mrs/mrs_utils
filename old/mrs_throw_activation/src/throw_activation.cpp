/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/UavState.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <sensor_msgs/Imu.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

//}

namespace mrs_throw_activation
{

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

/* class ThrowActivation //{ */

class ThrowActivation : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  void timerMain(const ros::TimerEvent& event);

  bool disarm(void);
  bool activate(void);

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_midair_activation_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;

  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus> sh_hw_api_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavState>    sh_uav_state_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>      sh_imu_;

  ros::Timer timer_main_;

  // | ------------------------- params ------------------------- |

  double _acceleration_magnitude_;

  ros::Time time_disarmed_;
};

//}

/* onInit() //{ */

void ThrowActivation::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  time_disarmed_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "ThrowActivation");

  // load parameters from config file
  param_loader.loadParam("acceleration_magnitude", _acceleration_magnitude_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ThrowActivation]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ThrowActivation";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_status_ = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in");
  sh_uav_state_     = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in");
  sh_imu_           = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "imu_in");

  // | --------------------- service clients -------------------- |

  sch_arming_            = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "arming_out");
  sch_midair_activation_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "midair_activation_out");

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(100.0), &ThrowActivation::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[ThrowActivation]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void ThrowActivation::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if ((ros::Time::now() - time_disarmed_).toSec() < 1.0) {
    return;
  }

  if (!sh_hw_api_status_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[ThrowActivation]: missing HW API status");
    return;
  }

  // HW API IS RUNNING

  auto hw_api_status = sh_hw_api_status_.getMsg();

  if (!hw_api_status->armed) {

    ROS_INFO_THROTTLE(1.0, "[ThrowActivation]: waiting for arming");
    return;
  }

  // UAV IS ARMED

  ROS_INFO_THROTTLE(1.0, "[ThrowActivation]: UAV is armed");

  if (!sh_uav_state_.getMsg() || (ros::Time::now() - sh_uav_state_.lastMsgTime()).toSec() > 0.1) {

    ROS_ERROR("[ThrowActivation]: the UAV is armed but we are missing the UAV State");

    disarm();
    return;
  }

  // GOT UAV STATE

  if (!sh_imu_.getMsg() || (ros::Time::now() - sh_imu_.lastMsgTime()).toSec() > 0.1) {

    ROS_ERROR("[ThrowActivation]: the UAV is armed but we are missing the IMU data");

    disarm();
    return;
  }

  // GOT IMU

  auto imu = sh_imu_.getMsg();

  const double accel = std::hypot(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  if (accel > _acceleration_magnitude_) {

    ROS_INFO_THROTTLE(0.1, "[ThrowActivation]: waiting for free fall, acceleration is %.2f", accel);
    return;
  }

  // FREE FALL DETECTED

  bool success = activate();

  if (!success) {

    ROS_ERROR_THROTTLE(1.0, "[ThrowActivation]: failed to activate in midair");

    disarm();
  }

  // ACTIVATION SUCCEDED

  ROS_INFO("[ThrowActivation]: activation finished");

  timer_main_.stop();
}

//}

// | ------------------------ routines ------------------------ |

/* diarm() //{ */

bool ThrowActivation::disarm() {

  std_srvs::SetBool srv;
  srv.request.data = false;

  ROS_INFO_THROTTLE(1.0, "[ThrowActivation]: disarming!");

  bool success = sch_arming_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ThrowActivation]: service call for disarming trajectory failed: " << srv.response.message);
      return false;
    } else {
      return true;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[ThrowActivation]: service call for disarming");
    return false;
  }
}

//}

/* activate() //{ */

bool ThrowActivation::activate() {

  std_srvs::Trigger srv;

  ROS_INFO_THROTTLE(1.0, "[ThrowActivation]: activating in midair!");

  bool success = sch_midair_activation_.call(srv, 10.0, 0.1);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[ThrowActivation]: service call for midair activation failed: " << srv.response.message);
      return false;
    } else {

      time_disarmed_ = ros::Time::now();

      return true;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[ThrowActivation]: service call for midair activation");
    return false;
  }
}

//}

}  // namespace mrs_throw_activation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_throw_activation::ThrowActivation, nodelet::Nodelet)
