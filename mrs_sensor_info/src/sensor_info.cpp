/* includes //{ */

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>

#include <mrs_modules_msgs/msg/sensor_info.hpp>

//}

namespace mrs_sensor_info
{

/* class SensorInfo //{ */

class SensorInfo : public mrs_lib::Node {

public:
  SensorInfo(const rclcpp::NodeOptions& options);

  bool is_initialized_ = false;

  mrs_lib::PublisherHandler<mrs_modules_msgs::msg::SensorInfo> ph_sensor_info_;

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
};

//}

/* SensorInfo() //{ */

SensorInfo::SensorInfo(const rclcpp::NodeOptions& options) : mrs_lib::Node("SensorInfo", options) {

  RCLCPP_INFO(node_->get_logger(), "Initializing");

  // | ------------------------- params ------------------------- |

  mrs_lib::ParamLoader param_loader(node_);

  std::string name;
  std::string topic;
  int         type;
  double      expected_rate;

  param_loader.loadParam("name", name);
  param_loader.loadParam("topic", topic);
  param_loader.loadParam("expected_rate", expected_rate);
  param_loader.loadParam("type", type);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandlerOptions phopts;
  phopts.node = node_;
  phopts.throttle_rate = 1;

  ph_sensor_info_ = mrs_lib::PublisherHandler<mrs_modules_msgs::msg::SensorInfo>(phopts, "sensor_info_out");

  // | ------------------------- publish ------------------------ |

  mrs_modules_msgs::msg::SensorInfo info;

  info.name          = name;
  info.topic         = topic;
  info.type          = type;
  info.expected_rate = expected_rate;

  ph_sensor_info_.publish(info);

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "Initialized");
}

//}

}  // namespace mrs_sensor_info

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_sensor_info::SensorInfo)
