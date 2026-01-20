/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>

#include <mrs_modules_msgs/SensorInfo.h>

//}

namespace mrs_sensor_info
{

/* class SensorInfo //{ */

class SensorInfo : public nodelet::Nodelet {

public:
  virtual void onInit();

  bool is_initialized_ = false;

  mrs_lib::PublisherHandler<mrs_modules_msgs::SensorInfo> ph_sensor_info_;

private:
  ros::NodeHandle nh_;
};

//}

/* onInit() //{ */

void SensorInfo::onInit() {

  ROS_INFO("[SensorInfo]: Initializing");

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "SensorInfo");

  std::string name;
  std::string topic;
  int         type;
  double      expected_rate;

  param_loader.loadParam("name", name);
  param_loader.loadParam("topic", topic);
  param_loader.loadParam("expected_rate", expected_rate);
  param_loader.loadParam("type", type);

  // | ----------------------- publishers ----------------------- |

  ph_sensor_info_ = mrs_lib::PublisherHandler<mrs_modules_msgs::SensorInfo>(nh_, "sensor_info_out", 1, true);

  // | ------------------------- publish ------------------------ |

  mrs_modules_msgs::SensorInfo info;

  info.name          = name;
  info.topic         = topic;
  info.type          = type;
  info.expected_rate = expected_rate;

  ph_sensor_info_.publish(info);

  is_initialized_ = true;

  ROS_INFO("[SensorInfo]: initialized");
}

//}

}  // namespace mrs_sensor_info

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_sensor_info::SensorInfo, nodelet::Nodelet)
