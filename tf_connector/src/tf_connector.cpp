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
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

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
      bool same_frames;

      offset_keyframes_t offsets_in;
      offset_keyframes_t offsets_ex;

      bool override_in = false;
      double override_in_x = 0.0;
      double override_in_y = 0.0;
      double override_in_z = 0.0;
      double override_in_heading = 0.0;

      bool override_ex = false;
      double override_ex_x = 0.0;
      double override_ex_y = 0.0;
      double override_ex_z = 0.0;
      double override_ex_heading = 0.0;

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
    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> m_ddynrec;

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
        const tf2::Transform offset_in = con_ptr->override_in
          ? to_tf(con_ptr->override_in_x, con_ptr->override_in_y, con_ptr->override_in_z, con_ptr->override_in_heading)
          : interpolate_keypoints(con_ptr->offsets_in, new_tf.header.stamp);
        const tf2::Transform offset_ex = con_ptr->override_ex
          ? to_tf(con_ptr->override_ex_x, con_ptr->override_ex_y, con_ptr->override_ex_z, con_ptr->override_ex_heading)
          : interpolate_keypoints(con_ptr->offsets_ex, new_tf.header.stamp);

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

    /* to_tf() method //{ */
    tf2::Transform to_tf(const double x, const double y, const double z, const double heading) const
    {
      const tf2::Vector3 translation(x, y, z);
      const Eigen::Quaterniond q(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
      const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
      return tf2::Transform(rotation, translation);
    }
    //}

    /* parse_offset() method //{ */
    double num(const XmlRpc::XmlRpcValue& xml) const
    {
      switch (xml.getType())
      {
        case XmlRpc::XmlRpcValue::TypeInt: return (int)xml;
        case XmlRpc::XmlRpcValue::TypeDouble: return (double)xml;
        default: return std::numeric_limits<double>::quiet_NaN();
      }
    }

    std::optional<offset_keyframe_t> parse_single_offset(const XmlRpc::XmlRpcValue& offset, const size_t it) const
    {
      if (offset.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR_STREAM("[" << m_node_name << "]: An offset of the " << it << ". connection is not an array, skipping");
        return std::nullopt;
      }
    
      switch (offset.size())
      {
        // x,y,z,yaw
        case 4:
          {
            const ros::Time stamp(0);
            const tf2::Transform tf = to_tf(num(offset[0]), num(offset[1]), num(offset[2]), num(offset[3]));
            return offset_keyframe_t{stamp, tf};
          }

        // stamp,x,y,z,yaw
        case 5:
          {
            const ros::Time stamp(num(num(offset[0])));
            const tf2::Transform tf = to_tf(num(offset[1]), num(offset[2]), num(offset[3]), num(offset[4]));
            return offset_keyframe_t{stamp, tf};
          }

        // x,y,z,qx,qy,qz,qw
        case 7:
          {
            const ros::Time stamp(0);
            const tf2::Vector3 translation(num(offset[0]), num(offset[1]), num(offset[2]));
            // Eigen expects parameters of the constructor to be w, x, y, z
            const Eigen::Quaterniond q = Eigen::Quaterniond(num(offset[6]), num(offset[3]), num(offset[4]), num(offset[5])).normalized();
            if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all())
            {
              ROS_ERROR_STREAM("[" << m_node_name << "]: An offset of the " << it << ". connection has an invalid rotation (" << q.coeffs().transpose() << "), skipping");
              return std::nullopt;
            }
            // tf2 expects parameters of the constructor to be x, y, z, w
            const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
            return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
          }

        // stamp,x,y,z,qx,qy,qz,qw
        case 8:
          {
            const ros::Time stamp(num(offset[0]));
            const tf2::Vector3 translation(num(offset[1]), num(offset[2]), num(offset[3]));
            // Eigen expects parameters of the constructor to be w, x, y, z
            const Eigen::Quaterniond q = Eigen::Quaterniond(num(offset[7]), num(offset[4]), num(offset[5]), num(offset[6])).normalized();
            if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all())
            {
              ROS_ERROR_STREAM("[" << m_node_name << "]: An offset of the " << it << ". connection has an invalid rotation (" << q.coeffs().transpose() << "), skipping");
              return std::nullopt;
            }
            // tf2 expects parameters of the constructor to be x, y, z, w
            const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
            return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
          }

        default:
          {
            ROS_ERROR_STREAM("[" << m_node_name << "]: An offset of the " << it << ". connection has incorrect size (" << offset.size() << ", has to be 4, 5, 7 or 8), skipping");
            return std::nullopt;
          }
      }
    }
    //}

    /* parse_offset_keypoints() method //{ */
    std::optional<offset_keyframes_t> parse_offset_keypoints(const XmlRpc::XmlRpcValue& offsets_xml, const size_t it) const
    {
      std::optional<offset_keyframes_t> ret;
    
      // first, check if it's an array of arrays - that'd indicate keypoints
      if (offsets_xml.getType() == XmlRpc::XmlRpcValue::TypeArray && offsets_xml.size() > 0 && offsets_xml[0].getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        offset_keyframes_t keyframes;
        keyframes.reserve(offsets_xml.size());
        for (int el_it = 0; el_it < offsets_xml.size(); el_it++)
        {
          const auto parsed = parse_single_offset(offsets_xml[el_it], it);
          if (parsed.has_value())
            keyframes.push_back(parsed.value());
          else
            return std::nullopt;
        }
        ret = keyframes;
      }
      // otherwise it's just a single offset
      else
      {
        const auto parsed = parse_single_offset(offsets_xml, it);
        if (parsed.has_value())
          ret = {parsed.value()};
        else
          return std::nullopt;
      }
    
      return ret;
    }
    //}

    /* parse_offsets() method //{ */
    std::optional<std::pair<offset_keyframes_t, offset_keyframes_t>> parse_offsets(const XmlRpc::XmlRpcValue& offsets_xml, const size_t it) const
    {
      std::pair<offset_keyframes_t, offset_keyframes_t> ret;
    
      for (auto mem_it = std::cbegin(offsets_xml); mem_it != std::cend(offsets_xml); ++mem_it)
      {
        const auto& mem_name = mem_it->first;
        const auto& mem = mem_it->second;
        if (mem_name == "intrinsic")
        {
          const auto offsets_in_opt = parse_offset_keypoints(offsets_xml["intrinsic"], it);
          if (!offsets_in_opt.has_value())
            return std::nullopt;
          ret.first = offsets_in_opt.value();
        }
        else if (mem_name == "extrinsic")
        {
          const auto offsets_ex_opt = parse_offset_keypoints(offsets_xml["extrinsic"], it);
          if (!offsets_ex_opt.has_value())
            return std::nullopt;
          ret.second = offsets_ex_opt.value();
        }
        else
        {
          ROS_ERROR_STREAM("[" << m_node_name << "]: The " << it << ". member of 'connections' has an unexpected member '" << mem_name << "' of 'offsets'. Aborting parse.");
          return std::nullopt;
        }
      }
    
      return ret;
    }
    //}

    /* parse_connections() method //{ */
    std::optional<connection_vec_t> parse_connections(const XmlRpc::XmlRpcValue& xmlarr) const
    {
      const static std::string root_frame_xmlname = "root_frame_id";
      const static std::string equal_frame_xmlname = "equal_frame_id";
      const static std::string offsets_xmlname = "offsets";
      if (xmlarr.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[%s]: The 'connections' parameter has to be array, but it's not. Cannot parse.", m_node_name.c_str());
        return std::nullopt;
      }
    
      const auto now = ros::Time::now();
      connection_vec_t ret;
      ret.reserve(xmlarr.size());
    
      for (size_t it = 0; it < xmlarr.size(); it++)
      {
        const auto& conn_xml = xmlarr[it];
        if (conn_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_ERROR_STREAM("[" << m_node_name << "]: Invalid type of the " << it << ". member of 'connections'. Cannot parse.");
          return std::nullopt;
        }
    
        if (!conn_xml.hasMember(root_frame_xmlname) || !conn_xml.hasMember(equal_frame_xmlname))
        {
          ROS_ERROR_STREAM("[" << m_node_name << "]: The " << it << ". member of 'connections' is missing either the '" << root_frame_xmlname << "' or '" << equal_frame_xmlname << "' member. Cannot parse.");
          return std::nullopt;
        }
    
        auto new_con_ptr = std::make_shared<frame_connection_t>();
        bool parsed_root = false;
        bool parsed_equal = false;
    
        for (auto mem_it = std::cbegin(conn_xml); mem_it != std::cend(conn_xml); ++mem_it)
        {
          const auto& mem_name = mem_it->first;
          const auto& mem = mem_it->second;
          // firstly, check types
          if (
                 (mem_name == root_frame_xmlname && mem.getType() != XmlRpc::XmlRpcValue::TypeString)
              || (mem_name == equal_frame_xmlname && mem.getType() != XmlRpc::XmlRpcValue::TypeString)
              || (mem_name == offsets_xmlname && mem.getType() != XmlRpc::XmlRpcValue::TypeStruct)
             )
          {
            ROS_ERROR_STREAM("[" << m_node_name << "]: The " << it << ". member of 'connections' has a wrong type of the '" << mem_name << "' member. Cannot parse.");
            return std::nullopt;
          }
    
          if (mem_name == root_frame_xmlname)
          {
            new_con_ptr->root_frame_id = std::string(mem);
            parsed_root = true;
          }
          else if (mem_name == equal_frame_xmlname)
          {
            new_con_ptr->equal_frame_id = std::string(mem);
            parsed_equal = true;
          }
          else if (mem_name == offsets_xmlname)
          {
            const auto offsets_opt = parse_offsets(mem, it);
            if (!offsets_opt.has_value())
              return std::nullopt;
            const auto& offsets = offsets_opt.value();
            new_con_ptr->offsets_in = offsets.first;
            new_con_ptr->offsets_ex = offsets.second;
          }
          else
          {
            ROS_ERROR_STREAM("[" << m_node_name << "]: The " << it << ". member of 'connections' has an unexpected member '" << mem_name << "'. Aborting parse.");
            return std::nullopt;
          }
    
        }
    
        if (!parsed_root || !parsed_equal)
        {
          ROS_ERROR_STREAM("[" << m_node_name << "]: The " << it << ". member of 'connections' misses a compulsory member '" << root_frame_xmlname << "' or '" << equal_frame_xmlname << "'. Aborting parse.");
          return std::nullopt;
        }
    
        new_con_ptr->same_frames = new_con_ptr->root_frame_id == new_con_ptr->equal_frame_id;
        new_con_ptr->last_update = now;
    
        ret.push_back(new_con_ptr);
      }
    
      return ret;
    }
    //}

    /* initialize_ddynrec() method //{ */
    void initialize_ddynrec()
    {
      for (auto& el : m_frame_connections)
      {
        const std::string& frame_name = el->equal_frame_id;
    
        const std::string group_in = frame_name + "/intrinsic";
        m_ddynrec->registerVariable(group_in+"/override", &(el->override_in), "if true, overrides the values specified in the config file", false, true, frame_name);
        m_ddynrec->registerVariable(group_in+"/x", &(el->override_in_x), "override value for the intrinsic translation's x component", -100.0, 100.0, frame_name);
        m_ddynrec->registerVariable(group_in+"/y", &(el->override_in_y), "override value for the intrinsic translation's y component", -100.0, 100.0, frame_name);
        m_ddynrec->registerVariable(group_in+"/z", &(el->override_in_z), "override value for the intrinsic translation's z component", -100.0, 100.0, frame_name);
        m_ddynrec->registerVariable(group_in+"/heading", &(el->override_in_heading), "override value for the intrinsic rotations's heading component", -M_PI, M_PI, frame_name);
    
        const std::string group_ex = frame_name + "/extrinsic";
        m_ddynrec->registerVariable(group_ex+"/override", &(el->override_ex), "if true, overrides the values specified in the config file", false, true, frame_name);
        m_ddynrec->registerVariable(group_ex+"/x", &(el->override_ex_x), "override value for the extrinsic translation's x component", -100.0, 100.0, frame_name);
        m_ddynrec->registerVariable(group_ex+"/y", &(el->override_ex_y), "override value for the extrinsic translation's y component", -100.0, 100.0, frame_name);
        m_ddynrec->registerVariable(group_ex+"/z", &(el->override_ex_z), "override value for the extrinsic translation's z component", -100.0, 100.0, frame_name);
        m_ddynrec->registerVariable(group_ex+"/heading", &(el->override_ex_heading), "override value for the extrinsic rotations's heading component", -M_PI, M_PI, frame_name);
      }
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
      pl.loadParam("ignore_older_messages", m_ignore_older_msgs);
      pl.loadParam("max_update_period", m_max_update_period);
      const auto conns_xml = pl.loadParam2<XmlRpc::XmlRpcValue>("connections");
      const auto conns_opt = parse_connections(conns_xml);

      if (!pl.loadedSuccessfully() || !conns_opt.has_value())
      {
        ROS_ERROR("[%s]: Some compulsory parameters were not loaded or parsed successfully, ending the node", m_node_name.c_str());
        ros::shutdown();
        return;
      }

      m_frame_connections = conns_opt.value();

      //}

      /* publishers //{ */

      m_pub_tf = nh.advertise<tf2_msgs::TFMessage>("tf_out", 10);
      m_ddynrec = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>(nh);
      initialize_ddynrec();
      m_ddynrec->publishServicesTopics();

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
