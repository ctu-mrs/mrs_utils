#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/mutex.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_euler_counter_example/mrs_euler_counter_exampleConfig.h>

#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Eigen>

namespace mrs_euler_counter_example
{

/* class EulerCounterExample //{ */

class EulerCounterExample : public nodelet::Nodelet {

public:
  virtual void onInit();

  bool is_initialized_ = false;

private:
  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent &event);

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_thrust_vector_;
  ros::Publisher pub_altered_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                             mutex_drs_;
  typedef mrs_euler_counter_example::mrs_euler_counter_exampleConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t>                   Drs_t;
  boost::shared_ptr<Drs_t>                                           drs_;
  void        callbackDrs(mrs_euler_counter_example::mrs_euler_counter_exampleConfig &params, uint32_t level);
  DrsParams_t params_;
  std::mutex  mutex_params_;
};

//}

/* onInit() //{ */

void EulerCounterExample::onInit() {

  ROS_INFO("[EulerCounterExample]: Initializing");

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(100.0), &EulerCounterExample::timerMain, this);

  // | ----------------------- publishers ----------------------- |

  pub_thrust_vector_ = nh_.advertise<geometry_msgs::PoseStamped>("thrust_vec_out", 1, true);
  pub_altered_       = nh_.advertise<geometry_msgs::PoseStamped>("altered_out", 1, true);

  // | --------------------------- drs -------------------------- |

  params_.type        = 6;
  params_.apply_angle = false;
  params_.thrust_x    = 0.2;
  params_.thrust_y    = 0.3;
  params_.thrust_z    = 1.0;
  params_.angle       = 0.0;
  params_.thrust_yaw  = 0.3;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&EulerCounterExample::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  is_initialized_ = true;

  ROS_INFO("[EulerCounterExample]: initialized");
}

//}

/* //{ callbackDrs() */

void EulerCounterExample::callbackDrs(mrs_euler_counter_example::mrs_euler_counter_exampleConfig &params, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_) {
    return;
  }

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  ROS_INFO("[EulerCounterExample]: DRS updated");
}

//}

/* timerMain() //{ */

void EulerCounterExample::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[EulerCounterExample]: timerMain() spinning");

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  Eigen::Vector3d thrust_yaw;
  thrust_yaw << cos(params.thrust_yaw), sin(params.thrust_yaw), 0;

  // | --------- calculate the thrust-based orientation --------- |

  Eigen::Vector3d body_z;
  body_z << params.thrust_x, params.thrust_y, params.thrust_z;

  body_z.normalize();

  Eigen::Matrix3d R1;

  R1.col(2) = body_z;
  R1.col(1) = R1.col(2).cross(thrust_yaw);
  R1.col(1).normalize();
  R1.col(0) = R1.col(1).cross(R1.col(2));
  R1.col(0).normalize();

  {
    geometry_msgs::PoseStamped thrust_pose;
    thrust_pose.header.frame_id = "map";
    thrust_pose.header.stamp    = ros::Time::now();

    thrust_pose.pose.position.x = 0;
    thrust_pose.pose.position.y = 0;
    thrust_pose.pose.position.z = 0;

    thrust_pose.pose.orientation = mrs_lib::AttitudeConverter(R1);

    pub_thrust_vector_.publish(thrust_pose);
  }

  switch (params.type) {

    // rpy yaw intrinsic
    case 0: {

      auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(R1).getIntrinsicRPY();

      ROS_INFO_THROTTLE(1.0, "[EulerCounterExample]: Intrinsic RPY: [%.2f, %.2f, %.2f]", roll, pitch, yaw);

      if (params.apply_angle) {
        yaw = params.angle;
      }

      R1 = mrs_lib::AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_INTRINSIC);

      break;
    }

      // rpy yaw extrinsic
    case 1: {

      auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(R1).getExtrinsicRPY();

      ROS_INFO_THROTTLE(1.0, "[EulerCounterExample]: Extrinsic RPY: [%.2f, %.2f, %.2f]", roll, pitch, yaw);

      if (params.apply_angle) {
        yaw = params.angle;
      }

      R1 = mrs_lib::AttitudeConverter(roll, pitch, yaw, mrs_lib::RPY_EXTRINSIC);

      break;
    }

      // ypr yaw intrinsic
    case 2: {

      Eigen::Vector3d eulers = R1.eulerAngles(2, 1, 0);

      double yaw   = eulers[0];
      double pitch = eulers[1];
      double roll  = eulers[2];

      ROS_INFO_THROTTLE(1.0, "[EulerCounterExample]: Intrinsic YPR: [%.2f, %.2f, %.2f]", yaw, pitch, roll);

      if (params.apply_angle) {
        yaw = params.angle;
      }

      R1 = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

      break;
    }

      // ypr yaw extrinsic
    case 3: {

      Eigen::Vector3d eulers = R1.eulerAngles(0, 1, 2);

      double yaw   = eulers[2];
      double pitch = eulers[1];
      double roll  = eulers[0];

      ROS_INFO_THROTTLE(1.0, "[EulerCounterExample]: Extrinsic YPR: [%.2f, %.2f, %.2f]", yaw, pitch, roll);

      if (params.apply_angle) {
        yaw = params.angle;
      }

      R1 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

      break;
    }

      // heading_oblique
    case 4: {

      if (params.apply_angle) {
        R1 = mrs_lib::AttitudeConverter(R1).setHeading(params.angle);
      }

      break;
    }

    // heading_ortho
    case 5: {

      Eigen::Vector3d heading_vec;

      if (params.apply_angle) {
        heading_vec << cos(params.angle), sin(params.angle), 0;
      } else {

        double current_heading = mrs_lib::AttitudeConverter(R1).getHeading();

        heading_vec << cos(current_heading), sin(current_heading), 0;
      }

      R1.col(2) = body_z;
      R1.col(1) = R1.col(2).cross(heading_vec);
      R1.col(1).normalize();
      R1.col(0) = R1.col(1).cross(R1.col(2));
      R1.col(0).normalize();

      break;
    }

      // none
    case 6: {

      break;
    }
  }

  {
    geometry_msgs::PoseStamped thrust_pose;
    thrust_pose.header.frame_id = "map";
    thrust_pose.header.stamp    = ros::Time::now();

    thrust_pose.pose.position.x = 0;
    thrust_pose.pose.position.y = 2;
    thrust_pose.pose.position.z = 0;

    thrust_pose.pose.orientation = mrs_lib::AttitudeConverter(R1);

    pub_altered_.publish(thrust_pose);
  }
}

//}

}  // namespace mrs_euler_counter_example

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_euler_counter_example::EulerCounterExample, nodelet::Nodelet)
