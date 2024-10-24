/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <image_transport/image_transport.h>

//}

namespace mrs_camera_republisher
{

/* class Throttle //{ */

class Throttle : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  ros::NodeHandle nh_;

  std::unique_ptr<image_transport::ImageTransport> it_;

  double _dt_;

  std::string _camera_name_;

  mrs_lib::PublisherHandler<sensor_msgs::CameraInfo> ph_camera_info;

  image_transport::Publisher publisher_image_;

  image_transport::Subscriber subscriber_image_;
  void                        callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  image_transport::Subscriber subscriber_camera_info_;
  void                        callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info_;

  ros::Timer timer_main_;

  std::atomic<bool> sh_image_on_    = true;
  std::atomic<bool> sh_cam_info_on_ = true;

  void timerMain(const ros::TimerEvent& event);

  ros::Time last_time_published_image_;
  ros::Time last_time_published_cam_info_;
};

//}

/* onInit() //{ */

void Throttle::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  last_time_published_image_    = ros::Time(0);
  last_time_published_cam_info_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh, "Throttle");

  double rate;

  param_loader.loadParam("rate", rate);
  param_loader.loadParam("camera_name", _camera_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[CameraThrottle_%s]: failed to load non-optional parameters!", _camera_name_.c_str());
    ros::shutdown();
  }

  ROS_INFO("[CameraThrottle_%s]: initializing", _camera_name_.c_str());

  if (rate < 1e-3) {
    ROS_ERROR("[CameraThrottle_%s]: provided rate is too low", _camera_name_.c_str());
    ros::shutdown();
  }

  _dt_ = 1.0 / rate;

  // | --------------------- image transport -------------------- |

  it_ = std::make_unique<image_transport::ImageTransport>(nh);

  // | ----------------------- subscribers ---------------------- |

  subscriber_image_ = it_->subscribe("image_in", 1, &Throttle::callbackImage, this);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "camera_republisher";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_camera_info_ = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, "camera_info_in", &Throttle::callbackCameraInfo, this);

  // | ----------------------- publishers ----------------------- |

  ph_camera_info = mrs_lib::PublisherHandler<sensor_msgs::CameraInfo>(nh, "camera_info_out", 1);

  publisher_image_ = it_->advertise("image_out", 1);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Duration(1.0), &Throttle::timerMain, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[CameraThrottle_%s]: initialized", _camera_name_.c_str());
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void Throttle::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (sh_image_on_) {

    if (publisher_image_.getNumSubscribers() == 0) {

      sh_image_on_ = false;
      subscriber_image_.shutdown();
      ROS_INFO("[CameraThrottle_%s]: no subscribers on images, shutting down image subscriber", _camera_name_.c_str());
    }

  } else {

    if (publisher_image_.getNumSubscribers() > 0) {

      sh_image_on_      = true;
      subscriber_image_ = it_->subscribe("image_in", 1, &Throttle::callbackImage, this);
      ROS_INFO("[CameraThrottle_%s]: re-enabling image subscriber", _camera_name_.c_str());
    }
  }

  if (sh_cam_info_on_) {

    if (ph_camera_info.getNumSubscribers() == 0) {

      sh_cam_info_on_ = false;
      sh_camera_info_.stop();
      ROS_INFO("[CameraThrottle_%s]: no subscribers on camera info, shutting down camera info subscriber", _camera_name_.c_str());
    }

  } else {

    if (ph_camera_info.getNumSubscribers() > 0) {

      sh_cam_info_on_ = true;
      sh_camera_info_.start();
      ROS_INFO("[CameraThrottle_%s]: re-enabling camera info subscriber", _camera_name_.c_str());
    }
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackImage() //{ */

void Throttle::callbackImage(const sensor_msgs::Image::ConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  if ((ros::Time::now() - last_time_published_image_).toSec() < _dt_) {
    return;
  }

  publisher_image_.publish(msg);

  last_time_published_image_ = ros::Time::now();

  ROS_INFO_ONCE("[CameraThrottle_%s]: republishing throttled images", _camera_name_.c_str());
}

//}

/* callbackCameraInfo() //{ */

void Throttle::callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  if ((ros::Time::now() - last_time_published_image_).toSec() < _dt_) {
    return;
  }

  ph_camera_info.publish(msg);

  last_time_published_cam_info_ = ros::Time::now();

  ROS_INFO_ONCE("[CameraThrottle_%s]: republishing throttled camera info", _camera_name_.c_str());
}

//}

}  // namespace mrs_camera_republisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_camera_republisher::Throttle, nodelet::Nodelet);
