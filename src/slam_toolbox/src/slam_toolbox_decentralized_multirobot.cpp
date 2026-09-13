/*
 * Decentralized multirobot_slam_toolbox
 * Copyright Work Modifications (c) 2025, Achala Athukorala
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

#include "slam_toolbox/slam_toolbox_decentralized_multirobot.hpp"

#include <cmath>

namespace slam_toolbox
{

/*****************************************************************************/
DecentralizedMultiRobotSlamToolbox::DecentralizedMultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options)
/*****************************************************************************/
{
  /** For decentralized multi-robot slam, each robot runs a slam_toolbox instance
   *  Each slam_toolbox instace should be run with a unique namespace
   *  This namespace acts as the 'identity' of the particular slam_toolbox instance
   */
  host_ns_ = this->get_namespace(); 
  // Remove namespace leading slash
  if (!host_ns_.empty() && host_ns_.front() == '/') host_ns_.erase(0,1); 
  if (host_ns_.empty()) {
    RCLCPP_ERROR(get_logger(), "This node must run in a non-root namespace (e.g., /robot1).");
    throw std::runtime_error("Namespace required");
  }

  if (!this->has_parameter("scan_share_topic")) {
    this->declare_parameter("scan_share_topic", "/localized_scan");
  }
  localized_scan_topic_ = this->get_parameter("scan_share_topic").as_string();
  if (!this->has_parameter("publish_peer_transforms")) {
    this->declare_parameter("publish_peer_transforms", true);
  }
  publish_peer_transforms_ = this->get_parameter("publish_peer_transforms").as_bool();
  if (!this->has_parameter("use_known_initial_poses")) {
    this->declare_parameter("use_known_initial_poses", false);
  }
  use_known_initial_poses_ = this->get_parameter("use_known_initial_poses").as_bool();
  if (use_known_initial_poses_) {
    if (!this->has_parameter("robot_names")) {
      this->declare_parameter<std::vector<std::string>>("robot_names", std::vector<std::string>{});
    }
    if (!this->has_parameter("initial_poses_xy_yaw")) {
      this->declare_parameter<std::vector<double>>("initial_poses_xy_yaw", std::vector<double>{});
    }
    const auto robot_names = this->get_parameter("robot_names").as_string_array();
    const auto initial_poses = this->get_parameter("initial_poses_xy_yaw").as_double_array();
    if (robot_names.empty() || initial_poses.size() != robot_names.size() * 3) {
      throw std::runtime_error(
              "known initial poses require robot_names and three x/y/yaw values per robot");
    }
    for (std::size_t index = 0; index < robot_names.size(); ++index) {
      initial_poses_.emplace(
        robot_names[index],
        Pose2(
          initial_poses[index * 3], initial_poses[index * 3 + 1],
          initial_poses[index * 3 + 2]));
    }
    if (initial_poses_.find(host_ns_) == initial_poses_.end()) {
      throw std::runtime_error("host namespace is missing from robot_names");
    }
    const auto & host_pose = initial_poses_.at(host_ns_);
    RCLCPP_INFO(
      get_logger(), "Known start for %s: x=%.3f y=%.3f yaw=%.3f rad",
      host_ns_.c_str(), host_pose.GetX(), host_pose.GetY(), host_pose.GetHeading());
  }
  RCLCPP_INFO(get_logger(), "Sharing scans on:  %s topic", localized_scan_topic_.c_str());

  localized_scan_pub_ = this->create_publisher<slam_toolbox::msg::LocalizedLaserScan>(
    localized_scan_topic_, 10);
  localized_scan_sub_ = this->create_subscription<slam_toolbox::msg::LocalizedLaserScan>(
    localized_scan_topic_, 10, std::bind(
      &DecentralizedMultiRobotSlamToolbox::localizedScanCallback,
      this, std::placeholders::_1));
}

/*****************************************************************************/
void DecentralizedMultiRobotSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // store scan header
  scan_header = scan->header;
  // no odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder * laser = getLaser(scan);

  if (!laser) {
    RCLCPP_WARN(
      get_logger(), "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // Note: When paused, only host scan processing will be paused.
  // Scan data from peers will still be processed -> refer to localizedScanCallback
  if (shouldProcessScan(scan, pose)) {
    LocalizedRangeScan * range_scan = addScan(laser, scan, pose);
    if (range_scan != nullptr) {
      Matrix3 covariance;
      covariance.SetToIdentity();
      publishLocalizedScan(
        scan, laser->GetOffsetPose(),
        range_scan->GetOdometricPose(), covariance, scan->header.stamp);
    }
  }
}

/*****************************************************************************/
void DecentralizedMultiRobotSlamToolbox::localizedScanCallback(
  slam_toolbox::msg::LocalizedLaserScan::ConstSharedPtr localized_scan)
{
  // The shared subscription is constructed before this lifecycle node is
  // configured.  A peer may already be active and publish while laser_assistant_,
  // dataset_ and smapper_ are still null.  Ignore those early messages; a fresh
  // scan will arrive after activation.
  if (this->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return;
  }

  std::string scan_ns = localized_scan->scan.header.frame_id.substr(
    0, localized_scan->scan.header.frame_id.find('/'));
  if (scan_ns == host_ns_) {
    return;  // Ignore callbacks from ourself
  }

  sensor_msgs::msg::LaserScan::ConstSharedPtr scan =
    std::make_shared<sensor_msgs::msg::LaserScan>(localized_scan->scan);
  tf2::Quaternion quat_tf;
  tf2::convert(localized_scan->pose.pose.pose.orientation, quat_tf);
  Pose2 pose(localized_scan->pose.pose.pose.position.x,
    localized_scan->pose.pose.pose.position.y,
    tf2::getYaw(quat_tf));
  if (!transformPeerPoseToHost(scan_ns, pose)) {
    return;
  }

  LaserRangeFinder * laser = getLaser(localized_scan);
  if (!laser) {
    RCLCPP_WARN(
      get_logger(), "Failed to create device for received localizedScanner"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }
  LocalizedRangeScan * range_scan = addExternalScan(laser, scan, pose);

  if (range_scan != nullptr && publish_peer_transforms_) {
    // Publish transform
    pose = range_scan->GetCorrectedPose();
    tf2::Quaternion q(0., 0., 0., 1.0);
    geometry_msgs::msg::TransformStamped tf_msg;
    q.setRPY(0., 0., pose.GetHeading());
    tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
    tf2::toMsg(transform, tf_msg.transform);
    tf_msg.header.frame_id = map_frame_;
    tf_msg.header.stamp = localized_scan->pose.header.stamp;
    tf_msg.child_frame_id = localized_scan->scanner_offset.header.frame_id;
    tfB_->sendTransform(tf_msg);
  }
}

/*****************************************************************************/
bool DecentralizedMultiRobotSlamToolbox::transformPeerPoseToHost(
  const std::string & source_ns, Pose2 & pose) const
/*****************************************************************************/
{
  if (!use_known_initial_poses_) {
    return true;
  }
  const auto source_it = initial_poses_.find(source_ns);
  const auto host_it = initial_poses_.find(host_ns_);
  if (source_it == initial_poses_.end() || host_it == initial_poses_.end()) {
    RCLCPP_ERROR(
      get_logger(), "Dropping shared scan: no known initial pose for %s", source_ns.c_str());
    return false;
  }

  // Convert source-local odometry coordinates to the known site coordinates,
  // then into this host's local map coordinates. Each slam_toolbox instance
  // can therefore keep its own odom TF while sharing geometrically consistent scans.
  const Pose2 & source_start = source_it->second;
  const Pose2 & host_start = host_it->second;
  const double source_cos = std::cos(source_start.GetHeading());
  const double source_sin = std::sin(source_start.GetHeading());
  const double site_x = source_start.GetX() + source_cos * pose.GetX() - source_sin * pose.GetY();
  const double site_y = source_start.GetY() + source_sin * pose.GetX() + source_cos * pose.GetY();
  const double delta_x = site_x - host_start.GetX();
  const double delta_y = site_y - host_start.GetY();
  const double host_cos = std::cos(host_start.GetHeading());
  const double host_sin = std::sin(host_start.GetHeading());
  pose = Pose2(
    host_cos * delta_x + host_sin * delta_y,
    -host_sin * delta_x + host_cos * delta_y,
    math::NormalizeAngle(
      source_start.GetHeading() + pose.GetHeading() - host_start.GetHeading()));
  return true;
}

/*****************************************************************************/
LocalizedRangeScan * DecentralizedMultiRobotSlamToolbox::addExternalScan(
  LaserRangeFinder * laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  Pose2 & odom_pose)
/*****************************************************************************/
{
  // get our localized range scan
  LocalizedRangeScan * range_scan = getLocalizedRangeScan(
    laser, scan, odom_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  Matrix3 covariance;
  covariance.SetToIdentity();

  if (processor_type_ == PROCESS) {
    processed = smapper_->getMapper()->Process(range_scan, &covariance);
  } else if (processor_type_ == PROCESS_FIRST_NODE) {
    processed = smapper_->getMapper()->ProcessAtDock(range_scan, &covariance);
    processor_type_ = PROCESS;
    update_reprocessing_transform = true;
  } else if (processor_type_ == PROCESS_NEAR_REGION) {
    boost::mutex::scoped_lock l(pose_mutex_);
    if (!process_near_pose_) {
      RCLCPP_ERROR(
        get_logger(), "Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(
      range_scan, false, &covariance);
    update_reprocessing_transform = true;
    processor_type_ = PROCESS;
  } else {
    RCLCPP_FATAL(
      get_logger(), "SlamToolbox: No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  // and add our scan to storage
  if (processed) {
    if (enable_interactive_mode_) {
      scan_holder_->addScan(*scan);
    }
  } else {
    delete range_scan;
    range_scan = nullptr;
  }

  return range_scan;
}

/*****************************************************************************/
LaserRangeFinder * DecentralizedMultiRobotSlamToolbox::getLaser(
  const slam_toolbox::msg::LocalizedLaserScan::ConstSharedPtr localized_scan)
/*****************************************************************************/
{
  const std::string & frame = localized_scan->scan.header.frame_id;
  if (lasers_.find(frame) == lasers_.end()) {
    try {
      lasers_[frame] = laser_assistant_->toLaserMetadata(
        localized_scan->scan, localized_scan->scanner_offset);
      dataset_->Add(lasers_[frame].getLaser(), true);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(
        get_logger(), "Failed to compute laser pose[%s], "
        "aborting initialization (%s)", frame.c_str(), e.what());
      return nullptr;
    }
  }

  return lasers_[frame].getLaser();
}

/*****************************************************************************/
void DecentralizedMultiRobotSlamToolbox::publishLocalizedScan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  const Pose2 & offset,
  const Pose2 & pose,
  const Matrix3 & cov,
  const rclcpp::Time & t)
/*****************************************************************************/
{
  slam_toolbox::msg::LocalizedLaserScan scan_msg;

  scan_msg.scan = *scan;

  tf2::Quaternion q_offset(0., 0., 0., 1.0);
  q_offset.setRPY(0., 0., offset.GetHeading());
  tf2::Transform scanner_offset(q_offset, tf2::Vector3(offset.GetX(), offset.GetY(), 0.0));
  tf2::toMsg(scanner_offset, scan_msg.scanner_offset.transform);
  scan_msg.scanner_offset.header.stamp = t;

  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
  tf2::toMsg(transform, scan_msg.pose.pose.pose);

  scan_msg.pose.pose.covariance[0] = cov(0, 0) * position_covariance_scale_;  // x
  scan_msg.pose.pose.covariance[1] = cov(0, 1) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[6] = cov(1, 0) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[7] = cov(1, 1) * position_covariance_scale_;  // y
  scan_msg.pose.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;      // yaw
  scan_msg.pose.header.stamp = t;

  // Physical-robot integrations commonly prefix frame IDs before they reach
  // slam_toolbox. Preserve an existing host prefix instead of producing names
  // such as robot1/robot1/laser.
  const auto prefix_frame = [this](const std::string & frame) {
      std::string normalized = frame;
      while (!normalized.empty() && normalized.front() == '/') {
        normalized.erase(0, 1);
      }
      if (normalized == host_ns_ || normalized.rfind(host_ns_ + "/", 0) == 0) {
        return normalized;
      }
      return host_ns_ + "/" + normalized;
    };
  scan_msg.scan.header.frame_id = prefix_frame(scan->header.frame_id);
  scan_msg.pose.header.frame_id = prefix_frame(map_frame_);

  scan_msg.scanner_offset.child_frame_id = scan_msg.scan.header.frame_id;

  scan_msg.scanner_offset.header.frame_id = prefix_frame(base_frame_);

  localized_scan_pub_->publish(scan_msg);
}

/*****************************************************************************/
bool DecentralizedMultiRobotSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_WARN(
      get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::DecentralizedMultiRobotSlamToolbox)
