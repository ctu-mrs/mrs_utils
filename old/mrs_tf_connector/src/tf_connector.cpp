/* includes //{ */

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Msgs
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// MRS stuff
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/dynparam_mgr.h>
#include <mrs_lib/node.h>

// YAML
#include <yaml-cpp/yaml.h>

// std
#include <string>
#include <mutex>
#include <numeric>

//}
//

namespace mrs_tf_connector
{

class TFConnector : public mrs_lib::Node {
public:
  TFConnector(const rclcpp::NodeOptions& options) : mrs_lib::Node("TFConnector", options) {
    onInit();
  }

private:
  struct offset_keyframe_t
  {
    rclcpp::Time      start_stamp;
    tf2::Transform offset;
  };
  using offset_keyframes_t = std::vector<offset_keyframe_t>;

  struct frame_connection_t
  {
    std::string root_frame_id;
    std::string equal_frame_id;
    bool        same_frames;

    offset_keyframes_t offsets_in;
    offset_keyframes_t offsets_ex;

    bool   override_in         = false;
    double override_in_x       = 0.0;
    double override_in_y       = 0.0;
    double override_in_z       = 0.0;
    double override_in_heading = 0.0;

    bool   override_ex         = false;
    double override_ex_x       = 0.0;
    double override_ex_y       = 0.0;
    double override_ex_z       = 0.0;
    double override_ex_heading = 0.0;

    rclcpp::Time last_update;
    rclcpp::Time change_time;
  };

  std::mutex  m_mtx;
  std::string m_connecting_frame_id;
  using connection_vec_t = std::vector<std::shared_ptr<frame_connection_t>>;
  connection_vec_t m_frame_connections;
  bool             m_ignore_older_msgs;

  std::unique_ptr<tf2_ros::Buffer>            m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

  mrs_lib::SubscriberHandler<tf2_msgs::msg::TFMessage> m_sub_tf;
  mrs_lib::PublisherHandler<tf2_msgs::msg::TFMessage>  m_pub_tf;
  rclcpp::TimerBase::SharedPtr                         m_tim_tf;
  std::shared_ptr<mrs_lib::DynparamMgr>                m_ddynrec;

  std::mutex                            mutex_drs_params_;
  //DynParams_t                           drs_params_;

  rclcpp::Node::SharedPtr      node_;
  rclcpp::Clock::SharedPtr     clock_;
  double m_max_update_period = 0.1;

public:
  /* tf_callback() method //{ */

  void tf_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr msg_ptr) {
    std::scoped_lock lck(m_mtx);
    const rclcpp::Time  now = clock_->now();
    check_timejump(now);

    const tf2_msgs::msg::TFMessage& tf_msg = *msg_ptr;
    connection_vec_t           changed_connections;
    for (const geometry_msgs::msg::TransformStamped& tf : tf_msg.transforms) {
      // check whether this frame id is of interest
      for (auto& con_ptr : m_frame_connections) {
        // skip connections that have the same equal and root frame
        // to avoid weird publish/callback loops
        if (con_ptr->same_frames)
          continue;

        // TODO: check all frames in the chain, not just the last frame
        const auto& trigger_frame_id = con_ptr->equal_frame_id;
        if (tf.child_frame_id == trigger_frame_id && (!m_ignore_older_msgs || rclcpp::Time(tf.header.stamp) > con_ptr->last_update)) {
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
  void timer_callback() {
    std::scoped_lock    lck(m_mtx);
    const rclcpp::Time     now = clock_->now();
    const rclcpp::Duration max_duration(rclcpp::Duration::from_seconds(0.99 * m_max_update_period));
    check_timejump(now);

    connection_vec_t changed_connections;
    for (auto& con_ptr : m_frame_connections) {
      if (now - con_ptr->last_update >= max_duration) {
        con_ptr->change_time = now;
        changed_connections.push_back(con_ptr);
      }
    }
    update_tfs(changed_connections, now);
  }
  //}

  /* update_tfs() method //{ */

  void update_tfs(const connection_vec_t& changed_connections, const rclcpp::Time& now) {
    // if changed_frame_its is empty, update all frames
    if (changed_connections.empty())
      return;

    // create and publish an updated TF for each changed frame
    tf2_msgs::msg::TFMessage new_tf_msg;
    new_tf_msg.transforms.reserve(changed_connections.size());
    for (const auto& con_ptr : changed_connections) {
      const auto&                     root_frame_id  = con_ptr->root_frame_id;
      const auto&                     equal_frame_id = con_ptr->equal_frame_id;
      geometry_msgs::msg::TransformStamped new_tf;
      try {
        new_tf = m_tf_buffer->lookupTransform(equal_frame_id, root_frame_id, con_ptr->change_time);
      }
      catch (const tf2::TransformException& ex) {
        try {
          new_tf = m_tf_buffer->lookupTransform(equal_frame_id, root_frame_id, rclcpp::Time(0));
        }
        catch (const tf2::TransformException& ex) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", root_frame_id.c_str(), equal_frame_id.c_str(),
                            ex.what());
          continue;
        }
      }

      // handle weird edge-cases like static transforms and transforms from-to the same frame
      if (rclcpp::Time(new_tf.header.stamp) == rclcpp::Time(0) || new_tf.child_frame_id == new_tf.header.frame_id)
        new_tf.header.stamp = now;

      new_tf.child_frame_id  = root_frame_id;
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
      tf               = offset_ex * tf * offset_in;
      new_tf.transform = tf2::toMsg(tf);

      new_tf_msg.transforms.push_back(new_tf);
      con_ptr->last_update = now;
    }

    if (!new_tf_msg.transforms.empty()) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "Publishing updated transform connection.");
      m_pub_tf.publish(new_tf_msg);
    }
  }

  //}

  /* check_timejump() method //{ */
  void check_timejump(const rclcpp::Time& now) {
    static rclcpp::Time prev_now = now;
    if (now < prev_now) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Detected a jump in time, resetting.");
      for (auto& con_ptr : m_frame_connections)
        con_ptr->last_update = now;
    }
    prev_now = now;
  }
  //}

  /* interpolate_keypoints() method //{ */
  tf2::Transform interpolate_keypoints(const offset_keyframes_t& keyframes, const rclcpp::Time& to_time) const {
    // handle special cases
    if (keyframes.empty())
      return tf2::Transform::getIdentity();
    if (keyframes.size() == 1)
      return keyframes.front().offset;

    // find keyframe before and after to_time
    using it_t      = offset_keyframes_t::const_iterator;
    it_t kfr_before = keyframes.begin();
    it_t kfr_after  = keyframes.begin();
    for (auto it = std::cbegin(keyframes); it != std::cend(keyframes); ++it) {
      const auto& kfr = *it;
      if (kfr.start_stamp < to_time) {
        kfr_before = it;
        kfr_after  = it;
      }
      if (kfr.start_stamp > to_time) {
        kfr_after = it;
        break;
      }
    }

    // handle another special case
    if (kfr_before->start_stamp == kfr_after->start_stamp)
      return kfr_before->offset;

    // finally do the interpolation
    const double          coeff       = (to_time - kfr_before->start_stamp).seconds() / (kfr_after->start_stamp - kfr_before->start_stamp).seconds();
    const tf2::Quaternion interp_quat = kfr_before->offset.getRotation().slerp(kfr_after->offset.getRotation(), coeff);
    const auto&           orig_before = kfr_before->offset.getOrigin();
    const auto&           orig_after  = kfr_after->offset.getOrigin();
    const tf2::Vector3    interp_vec  = orig_before + coeff * (orig_after - orig_before);
    const tf2::Transform  interp(interp_quat, interp_vec);
    return interp;
  }
  //}

  /* to_tf() method //{ */
  tf2::Transform to_tf(const double x, const double y, const double z, const double heading) const {
    const tf2::Vector3       translation(x, y, z);
    const Eigen::Quaterniond q(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    const tf2::Quaternion    rotation(q.x(), q.y(), q.z(), q.w());
    return tf2::Transform(rotation, translation);
  }
  //}

  /* parse_offset() method //{ */
  double num(const YAML::Node& node) const {
    try {
      if (node.IsScalar()) {
        return node.as<double>();
      }
      return std::numeric_limits<double>::quiet_NaN();
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(node_->get_logger(), "Failed to parse YAML number: %s", e.what());
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  std::optional<offset_keyframe_t> parse_single_offset(const YAML::Node& offset, const size_t it) const {
    if (!offset.IsSequence()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "An offset of the " << it << ". connection is not an array, skipping");
      return std::nullopt;
    }

    switch (offset.size()) {
      // x,y,z,yaw
      case 4: {
        const rclcpp::Time      stamp(0);
        const tf2::Transform tf = to_tf(num(offset[0]), num(offset[1]), num(offset[2]), num(offset[3]));
        return offset_keyframe_t{stamp, tf};
      }

      // stamp,x,y,z,yaw
      case 5: {
        const rclcpp::Time      stamp(static_cast<uint64_t>(num(offset[0])));
        const tf2::Transform tf = to_tf(num(offset[1]), num(offset[2]), num(offset[3]), num(offset[4]));
        return offset_keyframe_t{stamp, tf};
      }

      // x,y,z,qx,qy,qz,qw
      case 7: {
        const rclcpp::Time    stamp(0);
        const tf2::Vector3 translation(num(offset[0]), num(offset[1]), num(offset[2]));
        // Eigen expects parameters of the constructor to be w, x, y, z
        const Eigen::Quaterniond q = Eigen::Quaterniond(num(offset[6]), num(offset[3]), num(offset[4]), num(offset[5])).normalized();
        if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all()) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "An offset of the " << it << ". connection has an invalid rotation (" << q.coeffs().transpose()
                               << "), skipping");
          return std::nullopt;
        }
        // tf2 expects parameters of the constructor to be x, y, z, w
        const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
        return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
      }

      // stamp,x,y,z,qx,qy,qz,qw
      case 8: {
        const rclcpp::Time    stamp(static_cast<uint64_t>(num(offset[0])));
        const tf2::Vector3 translation(num(offset[1]), num(offset[2]), num(offset[3]));
        // Eigen expects parameters of the constructor to be w, x, y, z
        const Eigen::Quaterniond q = Eigen::Quaterniond(num(offset[7]), num(offset[4]), num(offset[5]), num(offset[6])).normalized();
        if (q.vec().hasNaN() || q.coeffs().array().cwiseEqual(0.0).all()) {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "An offset of the " << it << ". connection has an invalid rotation (" << q.coeffs().transpose()
                               << "), skipping");
          return std::nullopt;
        }
        // tf2 expects parameters of the constructor to be x, y, z, w
        const tf2::Quaternion rotation(q.x(), q.y(), q.z(), q.w());
        return offset_keyframe_t{stamp, tf2::Transform(rotation, translation)};
      }

      default: {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "An offset of the " << it << ". connection has incorrect size (" << offset.size()
                             << ", has to be 4, 5, 7 or 8), skipping");
        return std::nullopt;
      }
    }
  }
  //}

  /* parse_offset_keypoints() method //{ */
  std::optional<offset_keyframes_t> parse_offset_keypoints(const YAML::Node& offsets_yaml, const size_t it) const {
    std::optional<offset_keyframes_t> ret;

    // first, check if it's an array of arrays - that'd indicate keypoints
    if (offsets_yaml.IsSequence() && offsets_yaml.size() > 0 && offsets_yaml[0].IsSequence()) {
      offset_keyframes_t keyframes;
      keyframes.reserve(offsets_yaml.size());
      for (size_t el_it = 0; el_it < offsets_yaml.size(); el_it++) {
        const auto parsed = parse_single_offset(offsets_yaml[el_it], it);
        if (parsed.has_value()) {
          keyframes.push_back(parsed.value());
        } else {
          return std::nullopt;
        }
      }
      ret = keyframes;
    }
    // otherwise it's just a single offset
    else if (offsets_yaml.IsSequence()) {
      const auto parsed = parse_single_offset(offsets_yaml, it);
      if (parsed.has_value()) {
        ret = {parsed.value()};
      } else {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }

    return ret;
  }
  //}

  /* parse_offsets() method //{ */
  std::optional<std::pair<offset_keyframes_t, offset_keyframes_t>> parse_offsets(const YAML::Node& offsets_yaml, const size_t it) const {
    std::pair<offset_keyframes_t, offset_keyframes_t> ret;

    // Default to identity transforms if no offsets specified
    ret.first = {offset_keyframe_t{rclcpp::Time(0), tf2::Transform::getIdentity()}};
    ret.second = {offset_keyframe_t{rclcpp::Time(0), tf2::Transform::getIdentity()}};

    if (!offsets_yaml || !offsets_yaml.IsMap()) {
      return ret;  // Return identity transforms as defaults
    }

    if (offsets_yaml["intrinsic"]) {
      const auto offsets_in_opt = parse_offset_keypoints(offsets_yaml["intrinsic"], it);
      if (offsets_in_opt.has_value()) {
        ret.first = offsets_in_opt.value();
      }
    }

    if (offsets_yaml["extrinsic"]) {
      const auto offsets_ex_opt = parse_offset_keypoints(offsets_yaml["extrinsic"], it);
      if (offsets_ex_opt.has_value()) {
        ret.second = offsets_ex_opt.value();
      }
    }

    return ret;
  }
  //}

  /* parse_connections() method //{ */
  std::optional<connection_vec_t> parse_connections(const YAML::Node& config) const {
    if (!config["connections"] || !config["connections"].IsSequence()) {
      RCLCPP_ERROR(node_->get_logger(), "The 'connections' in YAML is not a valid sequence. Cannot parse.");
      return std::nullopt;
    }

    const auto& yaml_connections = config["connections"];
    if (yaml_connections.size() == 0) {
      RCLCPP_ERROR(node_->get_logger(), "The 'connections' parameter is empty. Cannot parse.");
      return std::nullopt;
    }

    connection_vec_t ret;
    ret.reserve(yaml_connections.size());
    const auto now = clock_->now();

    for (size_t it = 0; it < yaml_connections.size(); it++) {
      const auto& conn_yaml = yaml_connections[it];
      
      if (!conn_yaml.IsMap()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid type of the " << it << ". member of 'connections'. Cannot parse.");
        return std::nullopt;
      }

      if (!conn_yaml["root_frame_id"] || !conn_yaml["equal_frame_id"]) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "The " << it << ". member of 'connections' is missing either the 'root_frame_id' or 'equal_frame_id' member. Cannot parse.");
        return std::nullopt;
      }

      auto con_ptr = std::make_shared<frame_connection_t>();

      // Parse frame IDs from YAML
      con_ptr->root_frame_id = conn_yaml["root_frame_id"].as<std::string>();
      con_ptr->equal_frame_id = conn_yaml["equal_frame_id"].as<std::string>();
      con_ptr->same_frames = (con_ptr->root_frame_id == con_ptr->equal_frame_id);
      con_ptr->last_update = now;

      // Parse offsets from YAML if they exist
      if (conn_yaml["offsets"]) {
        const auto offsets_opt = parse_offsets(conn_yaml["offsets"], it);
        if (offsets_opt.has_value()) {
          const auto& offsets = offsets_opt.value();
          con_ptr->offsets_in = offsets.first;
          con_ptr->offsets_ex = offsets.second;
        }
      } else {
        // Use identity transforms as defaults
        con_ptr->offsets_in = {offset_keyframe_t{rclcpp::Time(0), tf2::Transform::getIdentity()}};
        con_ptr->offsets_ex = {offset_keyframe_t{rclcpp::Time(0), tf2::Transform::getIdentity()}};
      }

      ret.push_back(con_ptr);
    }

    return ret;
  }
  //}

  /* initialize_ddynrec() method //{ */
  void initialize_ddynrec() {
    for (auto& el : m_frame_connections) {
      const std::string& frame_name = el->equal_frame_id;

      const std::string group_in = frame_name + "/intrinsic";
      m_ddynrec->register_param(group_in + "/override", &(el->override_in));
      m_ddynrec->register_param(group_in + "/x", &(el->override_in_x), 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0));
      m_ddynrec->register_param(group_in + "/y", &(el->override_in_y), 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0));
      m_ddynrec->register_param(group_in + "/z", &(el->override_in_z), 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0));
      m_ddynrec->register_param(group_in + "/heading", &(el->override_in_heading), 0.0, mrs_lib::DynparamMgr::range_t<double>(-M_PI, M_PI));

      const std::string group_ex = frame_name + "/extrinsic";
      m_ddynrec->register_param(group_ex + "/override", &(el->override_ex));
      m_ddynrec->register_param(group_ex + "/x", &(el->override_ex_x), 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0));
      m_ddynrec->register_param(group_ex + "/y", &(el->override_ex_y), 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0));
      m_ddynrec->register_param(group_ex + "/z", &(el->override_ex_z), 0.0, mrs_lib::DynparamMgr::range_t<double>(-100.0, 100.0));
      m_ddynrec->register_param(group_ex + "/heading", &(el->override_ex_heading), 0.0, mrs_lib::DynparamMgr::range_t<double>(-M_PI, M_PI));
    }
  }
  //}

  /* onInit() method //{ */

  void onInit() {
    node_ = this_node_ptr();
    clock_ = node_->get_clock();

    /* load parameters //{ */
 
    RCLCPP_INFO(node_->get_logger(), "LOADING STATIC PARAMETERS");
    mrs_lib::ParamLoader pl(node_);

    std::string public_config_path;
    pl.loadParam("public_config", public_config_path);
    pl.addYamlFile(public_config_path);

    pl.loadParam("connecting_frame_id", m_connecting_frame_id);
    pl.loadParam("ignore_older_messages", m_ignore_older_msgs);
    pl.loadParam("max_update_period", m_max_update_period);

    // Load and parse connections directly from YAML
    YAML::Node config;
    try {
      config = YAML::LoadFile(public_config_path);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML config: %s", e.what());
      rclcpp::shutdown();
      exit(1);
    }

    const auto conns_opt = parse_connections(config);

    if (!pl.loadedSuccessfully() || !conns_opt.has_value()) {
      RCLCPP_ERROR(node_->get_logger(), "Some compulsory parameters were not loaded or parsed successfully, ending the node");
      rclcpp::shutdown();
      exit(1);
    }

    m_frame_connections = conns_opt.value();

    //}

    /* publishers //{ */ 

    mrs_lib::PublisherHandlerOptions phopts;
    phopts.node = node_;

    m_pub_tf  = mrs_lib::PublisherHandler<tf2_msgs::msg::TFMessage>(phopts, "tf_out");

    m_ddynrec = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_drs_params_);
    m_ddynrec->get_param_provider().copyYamls(pl.getParamProvider());
    initialize_ddynrec();
    //m_ddynrec->publishServicesTopics();

    //}

    /* subscribers //{ */

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node = node_;

    m_tf_buffer       = std::make_unique<tf2_ros::Buffer>(clock_);
    m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
    m_sub_tf          = mrs_lib::SubscriberHandler<tf2_msgs::msg::TFMessage>(shopts, "tf_in", &TFConnector::tf_callback, this);

    //}

    if (m_max_update_period > 0)
      m_tim_tf = node_->create_wall_timer(std::chrono::duration<double>(1.0 / m_max_update_period), std::bind(&TFConnector::timer_callback, this));

    RCLCPP_INFO(node_->get_logger(), "Initialized");
  }

  //}
};

}  // namespace mrs_tf_connector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_tf_connector::TFConnector)
