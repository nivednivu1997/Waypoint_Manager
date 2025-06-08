#include "cpp_node/waypoint.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <sstream>

WaypointManager::WaypointManager() : Node("waypoint_manager") {
  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&WaypointManager::amclCallback, this, std::placeholders::_1)
  );

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", 10);

  manual_wp_service_ = this->create_service<std_srvs::srv::Trigger>(
    "create_manual_waypoint",
    std::bind(&WaypointManager::handleManualWaypoint, this, std::placeholders::_1, std::placeholders::_2)
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&WaypointManager::timerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Waypoint manager initialized.");
}

void WaypointManager::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
  has_pose_ = true;
}

void WaypointManager::handleManualWaypoint(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  if (!has_pose_) {
    response->success = false;
    response->message = "No pose received yet.";
    return;
  }

  std::string id = "manual_" + std::to_string(waypoints_.size());
  createManualWaypoint(true, id);
  response->success = true;
  response->message = "Manual waypoint created.";
}

void WaypointManager::createManualWaypoint(bool is_manual, const std::string& id) {
double yaw = get_yaw(current_pose_.orientation);

  if (isNearExistingWaypoint(current_pose_.position.x, current_pose_.position.y)) return;

  Waypoint wp{ id.empty() ? ("manual_" + std::to_string(waypoints_.size())) : id,
               current_pose_.position.x,
               current_pose_.position.y,
               yaw,
               is_manual };
  waypoints_.push_back(wp);

  // Update last_pose_ so that auto waypoints consider this manual waypoint as last
  last_pose_ = current_pose_;

  saveWaypointsToYAML();
  publishMarkers();
}


void WaypointManager::createWaypoint(bool is_manual, const std::string& id) {
  //double yaw = get_yaw(last_pose_.orientation);
  double x = current_pose_.position.x;
  double y = current_pose_.position.y;
  double theta = get_yaw(current_pose_.orientation);

  if (isNearExistingWaypoint(last_pose_.position.x, last_pose_.position.y)) return;

  Waypoint wp{ id.empty() ? ("auto_" + std::to_string(waypoints_.size())) : id,
               last_pose_.position.x,
               last_pose_.position.y,
               theta,
               is_manual };
  waypoints_.push_back(wp);
  last_pose_ = current_pose_;

  saveWaypointsToYAML();
  publishMarkers();
}

void WaypointManager::timerCallback() {
  if (!has_pose_) return;

  // Calculate angular difference between last_pose_ and current_pose_
  double yaw_diff = calculateYawDifference(current_pose_, last_pose_);

  if (yaw_diff > angleThreshold_) {
    createWaypoint(false);  // auto waypoint
    last_pose_ = current_pose_;  // update last_pose_ only after creating waypoint
    return;
  }

  // Also check distance (optional if required)
  double dist = calculateDistance(current_pose_, last_pose_);
  if (dist > distanceThreshold_) {
    createWaypoint(false);
    last_pose_ = current_pose_;
  }
}


bool WaypointManager::isNearExistingWaypoint(double x, double y, double threshold) {
  for (const auto& wp : waypoints_) {
    double dx = wp.x - x;
    double dy = wp.y - y;
    if (std::sqrt(dx * dx + dy * dy) < threshold) return true;
  }
  return false;
}

double WaypointManager::calculateDistance(const geometry_msgs::msg::Pose& p1,
                                          const geometry_msgs::msg::Pose& p2) {
  return std::hypot(p1.position.x - p2.position.x, p1.position.y - p2.position.y);
}


double WaypointManager::calculateYawDifference(
    const geometry_msgs::msg::Pose& p1,
    const geometry_msgs::msg::Pose& p2) {
  double yaw1 = get_yaw(p1.orientation);
  double yaw2 = get_yaw(p2.orientation);

  double diff = std::fabs(yaw1 - yaw2);
  if (diff > M_PI)
    diff = 2 * M_PI - diff;
  return diff;
}


void WaypointManager::saveWaypointsToYAML() {
  YAML::Emitter out;
  out << YAML::BeginSeq;
  for (const auto& wp : waypoints_) {
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << wp.id;
    out << YAML::Key << "x" << YAML::Value << wp.x;
    out << YAML::Key << "y" << YAML::Value << wp.y;
    out << YAML::Key << "theta" << YAML::Value << wp.theta;
    out << YAML::Key << "type" << YAML::Value << (wp.is_manual ? "manual" : "auto");
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  std::ofstream fout("waypoints.yaml");
  fout << out.c_str();
}

void WaypointManager::publishMarkers() {
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.clear();
  int id = 0;

  for (const auto& wp : waypoints_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "waypoints";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = wp.x;
    marker.pose.position.y = wp.y;
    marker.pose.position.z = 0.1;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    if (wp.is_manual) {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

double WaypointManager::get_yaw(const geometry_msgs::msg::Quaternion & q)
{
  // Yaw (Z-axis rotation) from quaternion
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


