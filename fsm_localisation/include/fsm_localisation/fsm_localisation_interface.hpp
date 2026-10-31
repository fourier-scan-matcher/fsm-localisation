// Copyright 2022 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef FSM_LOCALISATION__FSM_LOCALISATION_INTERFACE_HPP_
#define FSM_LOCALISATION__FSM_LOCALISATION_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.h>

#include "fsm_localisation/fsm_localisation.hpp"
#include "fsm_localisation/occupancy_grid_map.hpp"

namespace fsm_localisation
{
/**
 * @brief Which of the two things this node does.
 * One node, two modes, chosen once at start-up and never afterwards. Only the
 * topics, services and state the chosen mode uses are created, so what the node
 * offers is exactly what the mode means and nothing from the other one is
 * present to be called by mistake.
 */
enum class Mode
{
  amelioration,

  odometry
};

/**
 * @brief Parse a mode's name.
 * @return the matching enumerator, or `std::nullopt` when `name` is neither
 * `"odometry"` nor `"amelioration"`.
 */
std::optional<Mode> modeFromName(const std::string& name);

/**
 * @brief The name a mode was parsed from, for a log line or a message.
 */
std::string_view nameOfMode(const Mode mode);

/**
 * @brief Everything about this node that is a ROS concern.
 * Owns the parameters, the topics, the services and the transform broadcast,
 * and delegates every computation to Matcher or to Corrector.
 */
class Interface : public rclcpp::Node
{
public:
  explicit Interface(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void clearTrajectory(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void correctOnce(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief One correction, from the estimate given and whatever else is held.
   * @return an empty string when a correction was published, otherwise a
   * sentence saying why none was. The same route serves the estimate
   * subscription and the correct-once service, so the two cannot
   * disagree about what a correction is.
   */
  std::string correctFrom(
    const geometry_msgs::msg::PoseWithCovarianceStamped& estimate);

  void declareParameters();

  void estimateCallback(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr estimate);

  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);

  void publishCorrectedPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& estimate,
    const Pose& corrected_base);
  void publishCorrectionTransforms(
    const Pose& estimated_base, const Pose& corrected_base,
    const rclcpp::Time& stamp);
  /**
   * @brief Publish a cast into the map from the corrected sensor, in the scan's
   * own frame and at the scan's own angles, for laying over the scan.
   * @param upside_down whether the sensor is mounted upside down, in which case
   * `corrected_sensor` is the upright sensor its scan was handed over as.
   */
  void publishMapScan(const sensor_msgs::msg::LaserScan& real_scan,
    const Pose& corrected_sensor, const bool upside_down,
    const rclcpp::Time& stamp);
  void publishOdometry(const MatchResult& result, const rclcpp::Time& stamp,
    double interval);
  void publishPath(const MatchResult& result, const rclcpp::Time& stamp);
  void publishPose(const MatchResult& result, const rclcpp::Time& stamp);
  void publishTransform(const MatchResult& result, const rclcpp::Time& stamp);

  Mode readMode();
  Parameters readParameters(const Mode mode);

  geometry_msgs::msg::Pose retypePose(const Pose& pose) const;

  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  void setInitialPose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void setUpAmelioration(const Parameters& parameters);
  void setUpOdometry(const Parameters& parameters);

  void start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  Mode mode_{Mode::odometry};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  rclcpp::CallbackGroup::SharedPtr service_group_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::string global_frame_id_;
  std::string base_frame_id_;

  /** @brief Start and stop write it and callbacks read it on other threads. */
  std::atomic<bool> locked_{true};

  std::unique_ptr<Matcher> matcher_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_trajectory_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_initial_pose_service_;

  nav_msgs::msg::Path path_;

  std::string initial_pose_topic_;
  std::string lo_frame_id_;

  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    estimate_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    corrected_pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr map_scan_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr correct_once_service_;

  std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  Parameters parameters_;

  CastingMethod casting_method_{CastingMethod::ray_marching};
  int occupancy_threshold_{65};

  std::string correction_frame_id_;
  std::string estimate_frame_id_;
  std::string map_scan_topic_;
  std::string robot_frame_id_;

  rclcpp::Duration transform_timeout_{0, 0};

  rclcpp::Time previous_estimate_stamp_{0, 0, RCL_ROS_TIME};

  mutable std::mutex held_mutex_;
  std::shared_ptr<const Corrector> corrector_;
  std::shared_ptr<const OccupancyGridMap> map_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr latest_estimate_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
};

}

#endif
