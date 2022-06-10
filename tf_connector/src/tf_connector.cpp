/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nodelet/nodelet.h>

// Msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// MRS stuff
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

// std
#include <string>
#include <mutex>
#include <numeric>

//}

namespace tf_connector
{

  class TFConnector : public nodelet::Nodelet
  {
  public:
    TFConnector() : m_node_name("TFConnector") {};
    std::string m_node_name;

  private:
    struct offset_keyframe_t
    {
      ros::Time start_stamp;
      tf2::Transform offset;
    };
    using offset_keyframes_t = std::vector<offset_keyframe_t>;

    struct frame_connection_t
    {
      std::string root_frame_id;
      std::string equal_frame_id;
      offset_keyframes_t offsets_in;
      offset_keyframes_t offsets_ex;
      bool same_frames;
      ros::Time last_update;
      ros::Time change_time;
    };

    std::mutex m_mtx;
    std::string m_connecting_frame_id;
    using connection_vec_t = std::vector<std::shared_ptr<frame_connection_t>>;
    connection_vec_t m_frame_connections;
    bool m_ignore_older_msgs;

    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

    ros::Subscriber m_sub_tf;
    ros::Publisher m_pub_tf;
    ros::Timer m_tim_tf;

    ros::Duration m_max_update_period = ros::Duration(0.1);

  public:

    /* tf_callback() method //{ */

    void tf_callback(tf2_msgs::TFMessageConstPtr msg_ptr)
    {
      std::scoped_lock lck(m_mtx);
      const ros::Time now = ros::Time::now();
      check_timejump(now);

      const tf2_msgs::TFMessage& tf_msg = *msg_ptr;
      connection_vec_t changed_connections;
      for (const geometry_msgs::TransformStamped& tf : tf_msg.transforms)
      {
        // check whether this frame id is of interest
        for (auto& con_ptr : m_frame_connections)
        {
          // skip connections that have the same equal and root frame
          // to avoid weird publish/callback loops
          if (con_ptr->same_frames)
            continue;

          // TODO: check all frames in the chain, not just the last frame
          const auto& trigger_frame_id = con_ptr->equal_frame_id;
          if (tf.child_frame_id == trigger_frame_id && (!m_ignore_older_msgs || tf.header.stamp > con_ptr->last_update))
          {
            con_ptr->change_time = tf.header.stamp;
            changed_connections.push_back(con_ptr);
          }
        }
      }
    
      // if no interesting frame was changed, ignore the message
      if (changed_connections.empty())
        return;
    
      update_tfs(changed_connections, now);
    }

    //}

    /* timer_callback() method //{ */
    void timer_callback([[maybe_unused]] const ros::TimerEvent&)
    {
      std::scoped_lock lck(m_mtx);
      const ros::Time now = ros::Time::now();
      const ros::Duration max_duration(0.99*m_max_update_period.toSec());
      check_timejump(now);
    
      connection_vec_t changed_connections;
      for (auto& con_ptr : m_frame_connections)
      {
        if (now - con_ptr->last_update >= max_duration)
        {
          con_ptr->change_time = now;
          changed_connections.push_back(con_ptr);
        }
      }
      update_tfs(changed_connections, now);
    }
    //}

    /* update_tfs() method //{ */

    void update_tfs(const connection_vec_t& changed_connections, const ros::Time& now)
    {
      // if changed_frame_its is empty, update all frames
      if (changed_connections.empty())
        return;

      // create and publish an updated TF for each changed frame
      tf2_msgs::TFMessage new_tf_msg;
      new_tf_msg.transforms.reserve(changed_connections.size());
      for (const auto& con_ptr : changed_connections)
      {
        const auto& root_frame_id = con_ptr->root_frame_id;
        const auto& equal_frame_id = con_ptr->equal_frame_id;
        geometry_msgs::TransformStamped new_tf;
        try
        {
          new_tf = m_tf_buffer.lookupTransform(equal_frame_id, root_frame_id, con_ptr->change_time);
        }
        catch (const tf2::TransformException& ex)
        {
          try
          {
            new_tf = m_tf_buffer.lookupTransform(equal_frame_id, root_frame_id, ros::Time(0));
          }
          catch (const tf2::TransformException& ex)
          {
            ROS_WARN_THROTTLE(1.0, "Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", root_frame_id.c_str(), equal_frame_id.c_str(), ex.what());
            continue;
          }
        }

        // handle weird edge-cases like static transforms and transforms from-to the same frame
        if (new_tf.header.stamp == ros::Time(0) || new_tf.child_frame_id == new_tf.header.frame_id)
          new_tf.header.stamp = now;

        new_tf.child_frame_id = root_frame_id;
        new_tf.header.frame_id = m_connecting_frame_id;

        // interpolate the offsets
        const tf2::Transform offset_in = interpolate_keypoints(con_ptr->offsets_in, new_tf.header.stamp);
        const tf2::Transform offset_ex = interpolate_keypoints(con_ptr->offsets_ex, new_tf.header.stamp);

        // apply the offset
        tf2::Transform tf;
        tf2::fromMsg(new_tf.transform, tf);
        tf = offset_ex * tf * offset_in;
        new_tf.transform = tf2::toMsg(tf);

        new_tf_msg.transforms.push_back(new_tf);
        con_ptr->last_update = now;
      }
    
      if (!new_tf_msg.transforms.empty())
      {
        ROS_INFO_THROTTLE(1.0, "[TFConnector]: Publishing updated transform connection.");
        m_pub_tf.publish(new_tf_msg);
      }
    }

    //}

    /* check_timejump() method //{ */
    void check_timejump(const ros::Time& now)
    {
      static ros::Time prev_now = now;
      if (now < prev_now)
      {
        ROS_WARN_THROTTLE(1.0, "[%s]: Detected a jump in time, resetting.", m_node_name.c_str());
        for (auto& con_ptr : m_frame_connections)
          con_ptr->last_update = now;
      }
      prev_now = now;
    }
    //}

    /* interpolate_keypoints() method //{ */
    tf2::Transform interpolate_keypoints(const offset_keyframes_t& keyframes, const ros::Time& to_time) const
    {
      // handle special cases
      if (keyframes.empty())
        return tf2::Transform::getIdentity();
      if (keyframes.size() == 1)
        return keyframes.front().offset;

      // find keyframe before and after to_time
      using it_t = offset_keyframes_t::const_iterator;
      it_t kfr_before = keyframes.begin();
      it_t kfr_after = keyframes.begin();
      for (auto it = std::cbegin(keyframes); it != std::cend(keyframes); ++it)
      {
        const auto& kfr = *it;
        if (kfr.start_stamp < to_time)
        {
          kfr_before = it;
          kfr_after = it;
        }
        if (kfr.start_stamp > to_time)
        {
          kfr_after = it;
          break;
        }
      }

      // handle another special case
      if (kfr_before->start_stamp == kfr_after->start_stamp)
        return kfr_before->offset;

      // finally do the interpolation
      const double coeff = (to_time - kfr_before->start_stamp).toSec()/(kfr_after->start_stamp - kfr_before->start_stamp).toSec();
      const tf2::Quaternion interp_quat = kfr_before->offset.getRotation().slerp(kfr_after->offset.getRotation(), coeff);
      const auto& orig_before = kfr_before->offset.getOrigin();
      const auto& orig_after = kfr_after->offset.getOrigin();
      const tf2::Vector3 interp_vec = orig_before + coeff*(orig_after - orig_before);
      const tf2::Transform interp(interp_quat, interp_vec);
      return interp;
    }
    //}

    /* parse_offset() method //{ */
    std::optional<offset_keyframe_t> parse_offset(const XmlRpc::XmlRpcValue& offset, const size_t it) const
    {
      if (offset.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[%s]: The %lu-th member of the 'offsets' array is not an array, skipping", m_node_name.c_str(), it);
        return std::nullopt;
      }
    
      switch (offset.size())
      {
        // x,y,z,yaw
        case 4:
          {
            const ros::Time stamp(0);
            const tf2::Vector3 translation(offset[0], offset[1], offset[2]);
            const Eigen::Quaterniond q(Eigen::AngleAxisd(offset[3], Eigen::Vector3d::UnitZ()));
            const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
            return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
          }

        // stamp,x,y,z,yaw
        case 5:
          {
            const ros::Time stamp(offset[0]);
            const tf2::Vector3 translation(offset[1], offset[2], offset[3]);
            const Eigen::Quaterniond q(Eigen::AngleAxisd(offset[4], Eigen::Vector3d::UnitZ()));
            const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
            return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
          }

        // x,y,z,qx,qy,qz,qw
        case 7:
          {
            const ros::Time stamp(0);
            const tf2::Vector3 translation(offset[0], offset[1], offset[2]);
            // Eigen expects parameters of the constructor to be w, x, y, z
            const Eigen::Quaterniond q = Eigen::Quaterniond(offset[6], offset[3], offset[4], offset[5]).normalized();
            if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all())
            {
              ROS_ERROR_STREAM("[" << m_node_name << "]: The member of the 'offsets' array at index " << it << " has an invalid rotation (" << q.coeffs().transpose() << "), skipping");
              return std::nullopt;
            }
            // tf2 expects parameters of the constructor to be x, y, z, w
            const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
            return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
          }

        // stamp,x,y,z,qx,qy,qz,qw
        case 8:
          {
            const ros::Time stamp(offset[0]);
            const tf2::Vector3 translation(offset[1], offset[2], offset[3]);
            // Eigen expects parameters of the constructor to be w, x, y, z
            const Eigen::Quaterniond q = Eigen::Quaterniond(offset[7], offset[4], offset[5], offset[6]).normalized();
            if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all())
            {
              ROS_ERROR_STREAM("[" << m_node_name << "]: The member of the 'offsets' array at index " << it << " has an invalid rotation (" << q.coeffs().transpose() << "), skipping");
              return std::nullopt;
            }
            // tf2 expects parameters of the constructor to be x, y, z, w
            const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
            return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
          }

        default:
          {
            ROS_ERROR("[%s]: The member of the 'offsets' array at index %lu has incorrect size (%d, has to be 4 or 7), skipping", m_node_name.c_str(), it, offset.size());
            return std::nullopt;
          }
      }
    }
    //}

    /* load_offsets() method //{ */
    std::vector<offset_keyframes_t> load_offsets(mrs_lib::ParamLoader& pl, const std::string& name) const
    {
      std::vector<offset_keyframes_t> ret;
      const auto offsets_xml = pl.loadParam2<XmlRpc::XmlRpcValue>(name);
      if (offsets_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[%s]: The 'offsets' parameter doesn't have a type array, cannot load", m_node_name.c_str());
        return ret;
      }
    
      for (size_t it = 0; it < offsets_xml.size(); it++)
      {
        const auto& member = offsets_xml[it];
        // first, check if it's another array of arrays - that'd indicate keypoints
        if (member.getType() == XmlRpc::XmlRpcValue::TypeArray && member.size() > 0 && member[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          offset_keyframes_t keyframes;
          keyframes.reserve(member.size());
          for (int el_it = 0; el_it < member.size(); el_it++)
          {
            const auto parsed = parse_offset(member[el_it], it);
            if (parsed.has_value())
              keyframes.push_back(parsed.value());
              
          }
          ret.push_back(keyframes);
        }
        // otherwise it's just a single offset
        else
        {
          const auto parsed = parse_offset(member, it);
          if (parsed.has_value())
            ret.push_back({parsed.value()});
        }
      }
    
      return ret;
    }
    //}

    /* onInit() method //{ */

    virtual void onInit() override
    {
      ROS_INFO("[%s]: Initializing", m_node_name.c_str());
      ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
      ros::Time::waitForValid();

      /* load parameters //{ */

      ROS_INFO("[%s]: LOADING STATIC PARAMETERS", m_node_name.c_str());
      mrs_lib::ParamLoader pl(nh, m_node_name);

      pl.loadParam("connecting_frame_id", m_connecting_frame_id);
      const auto root_frame_ids = pl.loadParam2<std::vector<std::string>>("root_frame_ids");
      const auto equal_frame_ids = pl.loadParam2<std::vector<std::string>>("equal_frame_ids");
      pl.loadParam("ignore_older_messages", m_ignore_older_msgs);
      pl.loadParam("max_update_period", m_max_update_period);

      const auto offsets_in = load_offsets(pl, "offsets/intrinsic");
      const auto offsets_ex = load_offsets(pl, "offsets/extrinsic");

      if (!pl.loadedSuccessfully())
      {
        ROS_ERROR("[%s]: Some compulsory parameters were not loaded successfully, ending the node", m_node_name.c_str());
        ros::shutdown();
        return;
      }

      if (root_frame_ids.size() != equal_frame_ids.size() || root_frame_ids.size() != offsets_in.size() || root_frame_ids.size() != offsets_ex.size())
      {
        ROS_ERROR("[%s]: Number of root frame ids (%lu) must equal the number of equal frame ids (%lu) and the number of offsets (%lu, %lu), ending the node", m_node_name.c_str(), root_frame_ids.size(), equal_frame_ids.size(), offsets_in.size(), offsets_ex.size());
        ros::shutdown();
        return;
      }

      //}

      const auto now = ros::Time::now();
      for (size_t it = 0; it < root_frame_ids.size(); it++)
      {
        auto new_con_ptr = std::make_shared<frame_connection_t>();
        new_con_ptr->root_frame_id = root_frame_ids.at(it);
        new_con_ptr->equal_frame_id = equal_frame_ids.at(it);
        new_con_ptr->same_frames = new_con_ptr->root_frame_id == new_con_ptr->equal_frame_id;
        new_con_ptr->offsets_in = offsets_in.at(it);
        new_con_ptr->offsets_ex = offsets_ex.at(it);
        new_con_ptr->last_update = now;
        m_frame_connections.push_back(std::move(new_con_ptr));
      }

      /* publishers //{ */

      m_pub_tf = nh.advertise<tf2_msgs::TFMessage>("tf_out", 10);

      //}

      /* subscribers //{ */

      m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
      m_sub_tf = nh.subscribe("tf_in", 10, &TFConnector::tf_callback, this);

      //}

      if (m_max_update_period > ros::Duration(0))
        m_tim_tf = nh.createTimer(m_max_update_period, &TFConnector::timer_callback, this);

      ROS_INFO("[%s]: Initialized", m_node_name.c_str());
    }

    //}

  };

}  // namespace tf_connector

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tf_connector::TFConnector, nodelet::Nodelet)
