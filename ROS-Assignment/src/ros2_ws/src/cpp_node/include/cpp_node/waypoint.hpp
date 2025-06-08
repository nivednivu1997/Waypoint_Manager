#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  


struct Waypoint {
  std::string id;
  double x, y, theta;
  bool is_manual;
};

class WaypointManager : public rclcpp::Node {
public:
  WaypointManager();

private:
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_wp_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Waypoint> waypoints_;
  geometry_msgs::msg::Pose last_pose_;
  geometry_msgs::msg::Pose current_pose_;

  bool has_pose_ = false;

  double distanceThreshold_ = 2.0;
  double angleThreshold_ = M_PI / 4.0;

  void createWaypoint(bool is_manual, const std::string& id = "");
  void createManualWaypoint(bool is_manual, const std::string& id);
  void saveWaypointsToYAML();
  void publishMarkers();
  bool isNearExistingWaypoint(double x, double y, double threshold = 0.5);
  double calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);
  double calculateYawDifference(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);

  void timerCallback();
  void handleManualWaypoint(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  double get_yaw(const geometry_msgs::msg::Quaternion & q);
};

#endif
