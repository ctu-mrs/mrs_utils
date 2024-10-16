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

/* class Downsampler //{ */

class Downsampler : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  ros::NodeHandle nh_;

  double _dt_;

  mrs_lib::PublisherHandler<sensor_msgs::CameraInfo> ph_camera_info;

  image_transport::Publisher publisher_image_;

  image_transport::Subscriber subscriber_image_;
  void                        callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  image_transport::Subscriber subscriber_camera_info_;
  void                        callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info_;

  ros::Time last_time_published_image_;
  ros::Time last_time_published_cam_info_;
};

//}

/* onInit() //{ */

void Downsampler::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[Downsampler]: initializing");

  ros::Time::waitForValid();

  last_time_published_image_    = ros::Time(0);
  last_time_published_cam_info_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh, "Downsampler");

  double rate;

  param_loader.loadParam("rate", rate);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Downsampler]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  if (rate < 1e-3) {
    ROS_ERROR("[Downsampler]: provided rate is too low");
    ros::shutdown();
  }

  _dt_ = 1.0 / rate;

  // | --------------------- image transport -------------------- |

  image_transport::ImageTransport it(nh);

  // | ----------------------- subscribers ---------------------- |

  subscriber_image_ = it.subscribe("image_in", 1, &Downsampler::callbackImage, this);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "camera_republisher";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_camera_info_ = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, "camera_info_in", &Downsampler::callbackCameraInfo, this);

  // | ----------------------- publishers ----------------------- |

  ph_camera_info = mrs_lib::PublisherHandler<sensor_msgs::CameraInfo>(nh, "camera_info_out", 1);

  publisher_image_ = it.advertise("image_out", 1);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[Downsampler]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackImage() //{ */

void Downsampler::callbackImage(const sensor_msgs::Image::ConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  if ((ros::Time::now() - last_time_published_image_).toSec() < _dt_) {
    return;
  }

  sensor_msgs::Image image = *msg;

  publisher_image_.publish(image);

  last_time_published_image_ = ros::Time::now();

  ROS_INFO_ONCE("[Downsampler]: republishing downsampled images");
}

//}

/* callbackCameraInfo() //{ */

void Downsampler::callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  if ((ros::Time::now() - last_time_published_image_).toSec() < _dt_) {
    return;
  }

  sensor_msgs::CameraInfo info = *msg;

  ph_camera_info.publish(info);

  last_time_published_cam_info_ = ros::Time::now();

  ROS_INFO_ONCE("[Downsampler]: republishing downsampled camera info");
}

//}

}  // namespace mrs_camera_republisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_camera_republisher::Downsampler, nodelet::Nodelet);
