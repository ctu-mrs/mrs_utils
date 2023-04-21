#define VERSION "1.0.4.0"

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>

#include <mavros_msgs/RCIn.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/CameraInfo.h>

#include <topic_tools/shape_shifter.h>

//}

/* defines //{ */

#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

//}

namespace mrs_rc_control
{

namespace rc_trajectory_tracking_trigger
{

/* class RcTrajectoryTrackingTrigger //{ */

// state machine
typedef enum
{

  STATE_IDLE,
  STATE_TRAJECTORY_LOADED,
  STATE_FLYING_NORMALLY,
  STATE_FLYING_TO_TRAJECTORY_START,
  STATE_AT_TRAJECTORY_START,
  STATE_SETTING_CONSTRAINTS,
  STATE_TRAJECTORY_TRACKING,
  STATE_TRAJECTORY_TRACKING_STOPPED,
  STATE_FINISHED,

} States_t;

// state machine
typedef enum
{

  TRANSITION_UP,
  TRANSITION_DOWN,
  TRANSITION_NONE

} SwitchTransition_t;

const char* state_names[8] = {

    "IDLING", "TRAJECTORY LOADED", "FLYING NORMALLY", "FLYING TO TRAJECTORY START", "AT TRAJECTORY START", "TRAJECTORY TRACKING", "STOPPED", "FINISHED"};

class RcTrajectoryTrackingTrigger : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;
  std::string     _version_;

  std::string _uav_name_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_fly_to_trajectory_start_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_start_trajectory_tracking_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger> sch_stop_trajectory_tracking_;
  mrs_lib::ServiceClientHandler<mrs_msgs::String>  sch_set_constraints_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<mavros_msgs::RCIn>                   sh_rc_in_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_diagnostics_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>       sh_trajectory_reference_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
  double     _main_timer_rate_;

  // | -------------------- the transformer  -------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ---------------------- odometry ---------------------- |

  void               callbackOdometry(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp);
  nav_msgs::Odometry uav_odometry_;
  bool               got_odometry_ = false;
  std::mutex         mutex_odometry_;

  // | --------------- control manager diagnostics -------------- |

  void                                callbackControlManagerDiagnostics(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& wrp);
  std::mutex                          mutex_control_manager_diagnostics_;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_;
  bool                                got_control_manager_diagnostics_ = false;

  // | --------------- trajectory reference -------------- |

  void                          callbackTrajectoryReference(mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>& wrp);
  std::mutex                    mutex_trajectory_reference_;
  geometry_msgs::PoseArray trajectory_reference_;
  bool                          got_trajectory_reference_ = false;

  // | ---------------------- RC ---------------------- |

  void              callbackRC(mrs_lib::SubscribeHandler<mavros_msgs::RCIn>& wrp);
  bool              got_rc_in_ = false;
  mavros_msgs::RCIn rc_in_;
  std::mutex        mutex_rc_in_;

  // listening to the RC channels as told by pixhawk
  int  rc_channel_last_value_         = PWM_MIDDLE;
  bool rc_channel_was_low_            = false;
  bool rc_channel_values_initialized_ = false;
  int  _rc_channel_                   = 0;

  SwitchTransition_t switch_transition_ = TRANSITION_NONE;

  // translates the PWM raw value to a desired range
  double RCChannelToRange(double rc_value, double range, double deadband);

  // | ---------------------- other params ---------------------- |
  int         call_attempt_counter_ = 0;
  int         _service_call_n_attempts_;
  ros::Time   time_of_last_service_call_;
  bool        is_flying_normally_          = false;
  bool        _set_constraints_for_flight_ = false;
  std::string _constraints_for_flight_;
  bool        constraints_set_ = false;

  // | ---------------------- other methods ---------------------- |
  bool callTriggerService(mrs_lib::ServiceClientHandler<std_srvs::Trigger>& sch_trigger);
  void checkDistanceToTrajectoryStart();
  bool setConstraints(std::string constraints);

  // | ---------------------- state machine --------------------- |

  uint current_state_ = STATE_IDLE;
  void changeState(States_t new_state);
};

//}

/* onInit() //{ */

void RcTrajectoryTrackingTrigger::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "RcTrajectoryTrackingTrigger");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[RcTrajectoryTrackingTrigger]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION,
              _version_.c_str());
    ros::shutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("rc/channel_number", _rc_channel_);
  param_loader.loadParam("service_call_attempts", _service_call_n_attempts_);
  param_loader.loadParam("flight_constraints/type", _constraints_for_flight_);
  param_loader.loadParam("flight_constraints/use", _set_constraints_for_flight_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[RcTrajectoryTrackingTrigger]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ------------------ subscribe handlers ------------------ |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "RcTrajectoryTrackingTrigger";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_            = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odometry_in", &RcTrajectoryTrackingTrigger::callbackOdometry, this);
  sh_rc_in_               = mrs_lib::SubscribeHandler<mavros_msgs::RCIn>(shopts, "rc_in", &RcTrajectoryTrackingTrigger::callbackRC, this);
  sh_control_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(
      shopts, "control_manager_diagnostics_in", &RcTrajectoryTrackingTrigger::callbackControlManagerDiagnostics, this);
  sh_trajectory_reference_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>(shopts, "trajectory_reference_in",
                                                                                      &RcTrajectoryTrackingTrigger::callbackTrajectoryReference, this);

  // | ---------------- service client handlers ---------------- |

  sch_stop_trajectory_tracking_  = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "stop_trajectory_tracking_out");
  sch_start_trajectory_tracking_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "start_trajectory_tracking_out");
  sch_fly_to_trajectory_start_   = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "fly_to_trajectory_start_out");
  sch_set_constraints_           = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "set_constraints_out");

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "RcTrajectoryTrackingTrigger");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &RcTrajectoryTrackingTrigger::timerMain, this);

  time_of_last_service_call_ = ros::Time::now();

  is_initialized_ = true;

  ROS_INFO("[RcTrajectoryTrackingTrigger]: initialized, version %s", VERSION);
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackOdometry() //{ */

void RcTrajectoryTrackingTrigger::callbackOdometry(mrs_lib::SubscribeHandler<nav_msgs::Odometry>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[RcTrajectoryTrackingTrigger]: Got odometry msg");

  uav_odometry_ = mrs_lib::get_mutexed(mutex_odometry_, *wrp.getMsg());

  got_odometry_ = true;
}

//}

/* callbackControlManagerDiagnostics() //{ */

void RcTrajectoryTrackingTrigger::callbackControlManagerDiagnostics(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[RcTrajectoryTrackingTrigger]: getting control manager diagnostics");

  {
    std::scoped_lock lock(mutex_control_manager_diagnostics_);

    control_manager_diagnostics_ = *wrp.getMsg();

    got_control_manager_diagnostics_ = true;
  }
}

//}

/* //{ callbackRC() */

void RcTrajectoryTrackingTrigger::callbackRC(mrs_lib::SubscribeHandler<mavros_msgs::RCIn>& wrp) {

  if (!is_initialized_)
    return;

  mavros_msgs::RCInConstPtr rc = wrp.getMsg();

  ROS_INFO_ONCE("[RcTrajectoryTrackingTrigger]: getting RC channels");

  // | ------------------- rc joystic control ------------------- |

  // when the switch change its position

  if (_rc_channel_ >= int(rc->channels.size())) {

    ROS_ERROR_THROTTLE(1.0, "[RcTrajectoryTrackingTrigger]: RC channel number (%d) is out of range [0-%d]", _rc_channel_, int(rc->channels.size()));

  } else {

    bool channel_low  = rc->channels[_rc_channel_] < (PWM_MIDDLE - PWM_DEADBAND) ? true : false;
    bool channel_high = rc->channels[_rc_channel_] > (PWM_MIDDLE + PWM_DEADBAND) ? true : false;

    if (!rc_channel_values_initialized_ && channel_low && current_state_ >= STATE_FLYING_NORMALLY) {

      ROS_INFO("[RcTrajectoryTrackingTrigger]: RC channel values initialized.");
      rc_channel_values_initialized_ = true;
      rc_channel_was_low_            = true;
    }

    if (rc_channel_values_initialized_) {

      if (rc_channel_was_low_ && channel_high) {

        switch_transition_  = TRANSITION_UP;
        rc_channel_was_low_ = false;
        ROS_INFO("[RcTrajectoryTrackingTrigger]: Switch transition set to TRANSITION_UP.");

      } else if (!rc_channel_was_low_ && channel_low) {

        switch_transition_  = TRANSITION_DOWN;
        rc_channel_was_low_ = true;
        ROS_INFO("[RcTrajectoryTrackingTrigger]: Switch transition set to TRANSITION_DOWN.");
      }
    } else {
      ROS_WARN_THROTTLE(2.0, "[RcTrajectoryTrackingTrigger]: Cannot set RC switch transition. RC channel values not initialized.");
    }
  }
}

//}

/* callbackTrajectoryReference() //{ */

void RcTrajectoryTrackingTrigger::callbackTrajectoryReference(mrs_lib::SubscribeHandler<geometry_msgs::PoseArray>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[RcTrajectoryTrackingTrigger]: Got trajectory reference msg");

  std::scoped_lock lock(mutex_trajectory_reference_);

  if (wrp.getMsg()->poses.size() > 0) {

    trajectory_reference_     = *wrp.getMsg();
    got_trajectory_reference_ = true;

  } else {

    ROS_WARN("[RcTrajectoryTrackingTrigger]: Received empty trajectory reference.");
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void RcTrajectoryTrackingTrigger::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_control_manager_diagnostics_) {
    ROS_WARN_THROTTLE(5.0, "[RcTrajectoryTrackingTrigger]: waiting for data: ControlManager diagnostics");
    return;
  }

  auto control_manager_diagnostics = mrs_lib::get_mutexed(mutex_control_manager_diagnostics_, control_manager_diagnostics_);

  is_flying_normally_ = control_manager_diagnostics.flying_normally;

  switch (current_state_) {

    case STATE_IDLE: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Waiting for trajectory to be loaded.");

      if (got_trajectory_reference_) {
        changeState(STATE_TRAJECTORY_LOADED);
      }

      break;
    }

    case STATE_TRAJECTORY_LOADED: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Waiting for takeoff.");

      checkDistanceToTrajectoryStart();

      if (is_flying_normally_) {
        changeState(STATE_FLYING_NORMALLY);
      }

      break;
    }

    case STATE_FLYING_NORMALLY: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Flying normally. Waiting for fly to trajectory start service call.");

      checkDistanceToTrajectoryStart();

      if (control_manager_diagnostics.tracker_status.trajectory_length < 1) {
        ROS_ERROR_THROTTLE(1.0, "[RcTrajectoryTrackingTrigger]: Trajectory not loaded correctly.");
      }

      if (switch_transition_ == TRANSITION_UP) {
        changeState(STATE_FLYING_TO_TRAJECTORY_START);
      }

      break;
    }

    case STATE_FLYING_TO_TRAJECTORY_START: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Flying to trajectory start.");

      if (!control_manager_diagnostics.tracker_status.have_goal && (ros::Time::now() - time_of_last_service_call_).toSec() > 1.0) {
        changeState(STATE_AT_TRAJECTORY_START);
      }

      break;
    }

    case STATE_AT_TRAJECTORY_START: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Hovering at trajectory start. Waiting for start trajectory tracking service call.");

      if (_set_constraints_for_flight_) {
        changeState(STATE_SETTING_CONSTRAINTS);
      } else if (switch_transition_ == TRANSITION_DOWN) {
        changeState(STATE_TRAJECTORY_TRACKING);
      }

      break;
    }

    case STATE_SETTING_CONSTRAINTS: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Hovering at trajectory start. Waiting for start trajectory tracking service call.");

      if (!constraints_set_) {
        constraints_set_ = setConstraints(_constraints_for_flight_);

        if (!constraints_set_) {

          if (++call_attempt_counter_ < _service_call_n_attempts_) {

            ROS_WARN("[RcTrajectoryTrackingTrigger]: Failed to call set constraints service.");
            return;

          } else {

            changeState(STATE_FINISHED);
            ROS_ERROR("[RcTrajectoryTrackingTrigger]: Failed to call set constraints service for the %dth time, giving up", call_attempt_counter_);
          }
        }

        call_attempt_counter_ = 0;
      }

      if (constraints_set_ && switch_transition_ == TRANSITION_DOWN) {
        changeState(STATE_TRAJECTORY_TRACKING);
      }

      break;
    }

    case STATE_TRAJECTORY_TRACKING: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Tracking trajectory.");
      if (switch_transition_ == TRANSITION_UP) {
        changeState(STATE_TRAJECTORY_TRACKING_STOPPED);
      } else if (!control_manager_diagnostics.tracker_status.tracking_trajectory && (ros::Time::now() - time_of_last_service_call_).toSec() > 1.0) {
        changeState(STATE_FINISHED);
      }

      break;
    }

    case STATE_TRAJECTORY_TRACKING_STOPPED: {

      ROS_INFO_THROTTLE(3.0, "[RcTrajectoryTrackingTrigger]: Trajectory tracking stopped.");
      break;
    }

    case STATE_FINISHED: {

      ROS_INFO_THROTTLE(5.0, "[RcTrajectoryTrackingTrigger]: Trajectory tracking finished.");
      break;
    }
  }
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void RcTrajectoryTrackingTrigger::changeState(States_t new_state) {

  ROS_WARN_THROTTLE(1.0, "[RcTrajectoryTrackingTrigger]: switching states %s -> %s", state_names[current_state_], state_names[new_state]);

  States_t next_state = new_state;
  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TRAJECTORY_LOADED: {

      checkDistanceToTrajectoryStart();
      break;
    }

    case STATE_FLYING_NORMALLY: {

      break;
    }

    case STATE_FLYING_TO_TRAJECTORY_START: {

      bool res = callTriggerService(sch_fly_to_trajectory_start_);

      if (!res) {

        if (++call_attempt_counter_ < _service_call_n_attempts_) {

          ROS_WARN("[RcTrajectoryTrackingTrigger]: Failed to call fly to trajectory start service.");
          return;

        } else {
          next_state = STATE_FINISHED;
          ROS_ERROR("[RcTrajectoryTrackingTrigger]: Failed to call fly to trajectory start for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      break;
    }

    case STATE_AT_TRAJECTORY_START: {

      break;
    }

    case STATE_SETTING_CONSTRAINTS: {

      break;
    }

    case STATE_TRAJECTORY_TRACKING: {

      bool res = callTriggerService(sch_start_trajectory_tracking_);

      if (!res) {

        if (++call_attempt_counter_ < _service_call_n_attempts_) {

          ROS_WARN("[RcTrajectoryTrackingTrigger]: Failed to call trajectory tracking start service.");
          return;

        } else {

          next_state = STATE_FINISHED;
          ROS_ERROR("[RcTrajectoryTrackingTrigger]: Failed to call trajectory tracking start for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      break;
    }

    case STATE_TRAJECTORY_TRACKING_STOPPED: {

      bool res = callTriggerService(sch_stop_trajectory_tracking_);

      if (!res) {

        if (++call_attempt_counter_ < _service_call_n_attempts_) {

          ROS_WARN("[RcTrajectoryTrackingTrigger]: Failed to call stop trajectory tracking service.");
          return;

        } else {

          next_state = STATE_FINISHED;
          ROS_ERROR("[RcTrajectoryTrackingTrigger]: Failed to call stop trajectory tracking for the %dth time, giving up", call_attempt_counter_);
        }
      }

      call_attempt_counter_ = 0;

      break;
    }

    case STATE_FINISHED: {

      break;
    }
  }

  current_state_ = next_state;
}

//}

/* callTriggerService() //{ */

bool RcTrajectoryTrackingTrigger::callTriggerService(mrs_lib::ServiceClientHandler<std_srvs::Trigger>& sch_trigger) {

  std_srvs::Trigger srv;

  bool res = sch_trigger.call(srv);

  if (res) {

    ROS_WARN_COND(!srv.response.success, "[RcTrajectoryTrackingTrigger]: Service call returned: '%s'", srv.response.message.c_str());
    time_of_last_service_call_ = ros::Time::now();
    return srv.response.success;

  } else {

    ROS_ERROR("[RcTrajectoryTrackingTrigger]: Service call failed!");

    return false;
  }
}

//}

bool RcTrajectoryTrackingTrigger::setConstraints(std::string constraints) {
  mrs_msgs::String string_service;
  string_service.request.value = constraints;
  sch_set_constraints_.call(string_service);
  ROS_INFO("[RcTrajectoryTrackingTrigger]: Set constraints service called with success = %d, %s", string_service.response.success,
           string_service.response.message.c_str());
  return string_service.response.success;
}

/* checkDistanceToTrajectoryStart() //{ */

void RcTrajectoryTrackingTrigger::checkDistanceToTrajectoryStart() {

  if (got_odometry_ && got_trajectory_reference_) {

    std::scoped_lock lock(mutex_odometry_, mutex_trajectory_reference_);

    auto ret = transformer_->getTransform(uav_odometry_.header.frame_id, trajectory_reference_.header.frame_id, uav_odometry_.header.stamp);

    if (ret) {

      ROS_INFO_ONCE("[ControlManager]: got TFs, checking distance to trajectory start.");
      geometry_msgs::TransformStamped tf = ret.value();  // transformation from odometry frame to trajectory ref frame
      mrs_msgs::ReferenceStamped      temp_ref;

      temp_ref.header             = uav_odometry_.header;
      temp_ref.reference.position = uav_odometry_.pose.pose.position;

      if (auto ret = transformer_->transform(temp_ref, tf)) {

        geometry_msgs::Point uav_position           = ret.value().reference.position;
        geometry_msgs::Point trajectory_first_point = trajectory_reference_.poses[0].position;

        double dist = sqrt(pow(uav_position.x - trajectory_first_point.x, 2) + pow(uav_position.y - trajectory_first_point.y, 2) +
                           pow(uav_position.z - trajectory_first_point.z, 2));

        ROS_INFO_THROTTLE(5.0, "[RcTrajectoryTrackingTrigger]: Distance of UAV from trajectory start: %.2f (m).", dist);
      } else {

        ROS_WARN_THROTTLE(1.0, "[ControlManager]: TFs not found, cannot check distance to trajectory start.");
      }

    } else {

      ROS_WARN_THROTTLE(1.0, "[ControlManager]: TFs not found, cannot check distance to trajectory start.");
    }


  } else {

    ROS_WARN_THROTTLE(5.0, "[RcTrajectoryTrackingTrigger]: Distance to trajectory start is not known. Odometry: %s, trajectory: %s",
                      got_odometry_ ? "ok" : "missing", got_trajectory_reference_ ? "ok" : "missing");
  }
}

//}

/* RCChannelToRange() //{ */

double RcTrajectoryTrackingTrigger::RCChannelToRange(double rc_value, double range, double deadband) {

  double tmp_0_to_1    = (rc_value - double(PWM_MIN)) / (double(PWM_RANGE));
  double tmp_neg1_to_1 = (tmp_0_to_1 - 0.5) * 2.0;

  if (tmp_neg1_to_1 > 1.0) {
    tmp_neg1_to_1 = 1.0;
  } else if (tmp_neg1_to_1 < -1.0) {
    tmp_neg1_to_1 = -1.0;
  }

  // check the deadband
  if (tmp_neg1_to_1 < deadband && tmp_neg1_to_1 > -deadband) {
    return 0.0;
  }

  if (tmp_neg1_to_1 > 0) {

    double tmp = (tmp_neg1_to_1 - deadband) / (1.0 - deadband);

    return range * tmp;

  } else {

    double tmp = (-tmp_neg1_to_1 - deadband) / (1.0 - deadband);

    return -range * tmp;
  }

  return 0.0;
}

//}

}  // namespace rc_trajectory_tracking_trigger

}  // namespace mrs_rc_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rc_control::rc_trajectory_tracking_trigger::RcTrajectoryTrackingTrigger, nodelet::Nodelet)
