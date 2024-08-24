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

/* class FrameIdChanger //{ */

class FrameIdChanger : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  ros::NodeHandle nh_;

  std::string _new_frame_;

  mrs_lib::PublisherHandler<sensor_msgs::CameraInfo> ph_camera_info;

  image_transport::Publisher publisher_image_;

  image_transport::Subscriber subscriber_image_;
  void                        callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  image_transport::Subscriber subscriber_camera_info_;
  void                        callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg);

  mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> sh_camera_info_;
};

//}

/* onInit() //{ */

void FrameIdChanger::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[FrameIdChanger]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "FrameIdChanger");

  param_loader.loadParam("new_frame_id", _new_frame_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[FrameIdChanger]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- image transport -------------------- |

  image_transport::ImageTransport it(nh);

  // | ----------------------- subscribers ---------------------- |

  subscriber_image_ = it.subscribe("image_in", 1, &FrameIdChanger::callbackImage, this);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "camera_republisher";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_camera_info_ = mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo>(shopts, "camera_info_in", &FrameIdChanger::callbackCameraInfo, this);

  // | ----------------------- publishers ----------------------- |

  ph_camera_info = mrs_lib::PublisherHandler<sensor_msgs::CameraInfo>(nh, "camera_info_out", 1);

  publisher_image_ = it.advertise("image_out", 1);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[FrameIdChanger]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackImage() //{ */

void FrameIdChanger::callbackImage(const sensor_msgs::Image::ConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  sensor_msgs::Image image = *msg;

  image.header.frame_id = _new_frame_;

  publisher_image_.publish(image);

  ROS_INFO_ONCE("[FrameIdChanger]: republishing images");
}

//}

/* callbackCameraInfo() //{ */

void FrameIdChanger::callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  sensor_msgs::CameraInfo info = *msg;

  info.header.frame_id = _new_frame_;

  ph_camera_info.publish(info);

  ROS_INFO_ONCE("[FrameIdChanger]: republishing camera info");
}

//}

}  // namespace mrs_camera_republisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_camera_republisher::FrameIdChanger, nodelet::Nodelet);
