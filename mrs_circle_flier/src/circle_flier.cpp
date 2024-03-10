/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/geometry/cyclic.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_circle_flier/mrs_circle_flierConfig.h>

//}

namespace mrs_circle_flier
{

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

/* class CircleFlier //{ */

class CircleFlier : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  void timerMain(const ros::TimerEvent& event);

  bool setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory);

  ros::ServiceClient service_client_trajectory_;
  ros::Publisher     publisher_trajectory_;

  ros::Timer timer_main_;

  double main_timer_rate_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef mrs_circle_flier::mrs_circle_flierConfig      DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(mrs_circle_flier::mrs_circle_flierConfig& params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;

  // | ---------- keeping track of the current position --------- |

  double     angle_     = 0;
  double     cycloid_x_ = 0;
  double     heading_   = 0;
  std::mutex mutex_global_;
};

//}

/* onInit() //{ */

void CircleFlier::onInit(void) {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "CircleFlier");

  // load parameters from config file
  param_loader.loadParam("publisher/active", params_.publisher_active);
  param_loader.loadParam("publisher/rate", params_.publisher_rate);
  param_loader.loadParam("publisher/mode", params_.publisher_mode);

  param_loader.loadParam("center/frame_id", params_.frame_id);
  param_loader.loadParam("center/x", params_.center_x);
  param_loader.loadParam("center/y", params_.center_y);
  param_loader.loadParam("center/z", params_.center_z);

  param_loader.loadParam("radius", params_.radius);
  param_loader.loadParam("speed", params_.speed);
  param_loader.loadParam("n_points", params_.n_points);
  param_loader.loadParam("dt", params_.dt);
  param_loader.loadParam("direction", params_.direction);
  param_loader.loadParam("orientation", params_.orientation);

  param_loader.loadParam("fly_now", params_.fly_now);

  param_loader.loadParam("cycloid_speed", params_.cycloid_speed);

  param_loader.loadParam("heading/use_heading", params_.use_heading);
  param_loader.loadParam("heading/mode", params_.heading_mode);
  param_loader.loadParam("heading/default_heading", params_.heading);
  param_loader.loadParam("heading/default_heading_rate", params_.heading_rate);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[CircleFlier]: Could not load all parameters!");
    ros::shutdown();
  }

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&CircleFlier::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  service_client_trajectory_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");
  publisher_trajectory_      = nh_.advertise<mrs_msgs::TrajectoryReference>("trajectory_reference_out", 1, false);

  timer_main_ = nh_.createTimer(ros::Rate(params_.publisher_rate), &CircleFlier::timerMain, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[CircleFlier]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackDrs() */

void CircleFlier::callbackDrs(mrs_circle_flier::mrs_circle_flierConfig& params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  ros::Duration dur(1.0 / params.publisher_rate);

  timer_main_.setPeriod(dur, true);

  ROS_INFO("[CircleFlier]: DRS updated");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void CircleFlier::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  mrs_msgs::TrajectoryReference trajectory;

  trajectory.header.stamp    = ros::Time::now();
  trajectory.header.frame_id = params.frame_id;

  trajectory.fly_now     = params.fly_now;
  trajectory.dt          = params.dt;
  trajectory.use_heading = params.use_heading;

  double arc_step     = params.speed * params.dt;
  double angular_step = arc_step / params.radius;

  auto angle     = mrs_lib::get_mutexed(mutex_global_, angle_);
  auto cycloid_x = mrs_lib::get_mutexed(mutex_global_, cycloid_x_);
  auto heading   = mrs_lib::get_mutexed(mutex_global_, heading_);

  double direction = params.direction == 0 ? -1.0 : 1.0;

  // fill in the trajectory
  for (int i = 0; i < params.n_points; i++) {

    mrs_msgs::Reference reference;

    if (params.orientation == 0) {
      reference.position.x = cos(angle) * params.radius + params.center_x;
      reference.position.y = sin(angle) * params.radius + params.center_y;
      reference.position.z = params.center_z;
    } else if (params.orientation == 1) {
      reference.position.x = cos(angle) * params.radius + params.center_x;
      reference.position.y = params.center_y;
      reference.position.z = sin(angle) * params.radius + params.center_z;
    } else if (params.orientation == 2) {
      reference.position.x = cos(angle) * params.radius + params.center_x;
      reference.position.y = sin(angle) * params.radius + params.center_y;
      reference.position.z = sin(angle) * params.radius + params.center_z;
    } else if (params.orientation == 3) {
      reference.position.x = cos(angle) * params.radius + cycloid_x + params.center_x;
      reference.position.y = params.center_y;
      reference.position.z = sin(angle) * params.radius + params.center_z;
    }

    switch (params.heading_mode) {
      case 0: {
        reference.heading = params.heading;
        break;
      }
      case 1: {
        reference.heading = atan2(params.center_y - reference.position.y, params.center_x - reference.position.x) - direction * M_PI_2;
        break;
      }
      case 2: {
        reference.heading = atan2(params.center_y - reference.position.y, params.center_x - reference.position.x);
        break;
      }
      case 3: {
        reference.heading = heading;
        break;
      }
    }

    trajectory.points.push_back(reference);

    angle += direction * angular_step;
    cycloid_x += params.cycloid_speed * params.dt;

    heading += params.heading_rate * params.dt;

    angle = radians::wrap(angle);
  }

  if (params_.publisher_active) {
    switch (params_.publisher_mode) {
      case 0: {
        publisher_trajectory_.publish(trajectory);
        break;
      }
      case 1: {
        setTrajectorySrv(trajectory);
        break;
      }
    }
  }

  // do a step with the global angle_
  arc_step     = params.speed * (1.0 / params.publisher_rate);
  angular_step = arc_step / params.radius;
  angle        = mrs_lib::get_mutexed(mutex_global_, angle_);
  angle += direction * angular_step;
  angle = radians::wrap(angle);
  mrs_lib::set_mutexed(mutex_global_, angle, angle_);

  // do a step with the global heading_
  if (params.heading_mode == 3) {
    heading = mrs_lib::get_mutexed(mutex_global_, heading_);
    heading += params.heading_rate * (1.0 / params.publisher_rate);
    heading = radians::wrap(heading);
    mrs_lib::set_mutexed(mutex_global_, heading, heading_);
  }

  // do a step with the cycloid
  if (params.orientation == 3) {
    cycloid_x = mrs_lib::get_mutexed(mutex_global_, cycloid_x_);
    cycloid_x += params.cycloid_speed * (1.0 / params.publisher_rate);
    mrs_lib::set_mutexed(mutex_global_, cycloid_x, cycloid_x_);
  }

}  // namespace mrs_circle_flier

//}

// | ------------------------ routines ------------------------ |

/* setTrajectorySrv() //{ */

bool CircleFlier::setTrajectorySrv(const mrs_msgs::TrajectoryReference trajectory) {

  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = trajectory;

  bool success = service_client_trajectory_.call(srv);

  if (success) {

    if (!srv.response.success) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "[CircleFlier]: service call for setting trajectory failed: " << srv.response.message);
      return false;
    } else {
      return true;
    }

  } else {
    ROS_ERROR_THROTTLE(1.0, "[CircleFlier]: service call for setting trajectory failed");
    return false;
  }
}

//}

}  // namespace mrs_circle_flier

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_circle_flier::CircleFlier, nodelet::Nodelet)
