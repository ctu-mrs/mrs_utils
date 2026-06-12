#include <geometry_msgs/msg/PointStamped.h>
#include <geometry_msgs/msg/PoseStamped.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/msg/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/msg/PoseWithCovarianceIdentified.h>
#include <nav_msgs/msg/Odometry.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace e = Eigen;

namespace odometry_collector
{
  class OdometryCollector : public mrs_lib::Node
  {
  public:
    OdometryCollector(const rclcpp::NodeOptions& options);

  private:
    bool is_initialized_ = false;

    rclcpp::Node::SharedPtr  node_;
    rclcpp::Clock::SharedPtr clock_;
    std::mutex               mtx_;

    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;

    std::string _origin_frame_ = "local_origin";
    std::string _odom_topic_name_;

    std::shared_ptr<mrs_lib::Transformer> transformer_;

    std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>> sh_odometry_;

    mrs_lib::PublisherHandler<mrs_msgs::msg::PoseWithCovarianceIdentified> ph_odometry_collection_;

    void                               cbTmPublisher();
    std::shared_ptr<mrs_lib::ROSTimer> tm_pub_;

    void                               cbTmDiscoverTopics();
    std::shared_ptr<mrs_lib::ROSTimer> tm_discvr_topics_;

    std::vector<std::string> findNewTopics(const std::string& topic_name);
  };
}  // namespace
   // odometry_collector

OdometryCollector::OdometryCollector(const rclcpp::NodeOptions& options)
  : mrs_lib::Node("OdometryCollector",
                  options)
{
  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Initializing", node_->get_name());

  // | ---------- loading ros parameters using mrs_lib ---------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: Loading parameters using ParamLoader", node_->get_name());

  mrs_lib::ParamLoader param_loader(node_, node_->get_name());

  param_loader.loadParam("odom_topic", _odom_topic_name_);
  param_loader.loadParam("origin_frame", _origin_frame_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: parameter loading failure", node_->get_name());
    rclcpp::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  // mrs_lib::SubscriberHandlerOptions shopts;
  // shopts.node                                = this_node_ptr();
  // shopts.no_message_timeout                  = mrs_lib::no_timeout;
  // shopts.threadsafe                          = true;
  // shopts.autostart                           = true;
  // shopts.subscription_options.callback_group = cbkgrp_subs_;
  //
  // sh_odometry_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts,
  //                                                                    "~/odometry_in",
  //                                                                    &OdometryCollector::callbackOdometry,
  //                                                                    this);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandlerOptions phopts;

  phopts.node             = node_;
  ph_odometry_collection_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(phopts, "~/odometry_out");

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

void OdometryCollector::cbTmDiscoverTopics()
{
  if (!is_initialized_) {
    return;
  }

  std::vector<std::string> topics = findNewTopics(_odom_topic_name_);

  std::scoped_lock lck(mutex_);

  for (int i = 0; i < topics.size(); ++i) {
    gps_data_.push_back(new swarm_utils::nodes::GpsData(_invalidate_time_, topics[i]));
    sub_aggr_topics_.push_back(nh_.subscribe(topics[i],
                                             1,
                                             &swarm_utils::nodes::GpsData::callbackGps,
                                             gps_data_.back(),
                                             ros::TransportHints().tcpNoDelay()));
    ROS_INFO_STREAM("[OdometryCollector] Subscribing to topic: " << topics[i]);
  }

  if (sub_aggr_topics_.size() == 0) {
    ROS_WARN_STREAM("[OdometryCollector] Topic " << _gps_topic_name_ << " does not exist");
  }
}

std::vector<std::string> OdometryCollector::findNewTopics(string            topic_name,
                                                          vector<GpsData*>& gps_data)
{
  vector<string>           new_topics;
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);

  for (auto it = topic_infos.begin(); it != topic_infos.end(); it++) {

    if (it->name.find(topic_name) != std::string::npos && it->name.find(_uav_name_) == std::string::npos) {

      bool        found = false;
      scoped_lock lck(mutex_gps_data_);

      for (int i = 0; i < gps_data.size(); ++i) {

        if (it->name == gps_data[i]->topic) {
          found = true;
          break;
        }
      }

      if (!found) {
        ROS_INFO_STREAM("[SharedGpsAggr] Found topic with desired name: " << it->name);
        new_topics.push_back(it->name);
      }
    }
  }

  return new_topics;
}
//}
