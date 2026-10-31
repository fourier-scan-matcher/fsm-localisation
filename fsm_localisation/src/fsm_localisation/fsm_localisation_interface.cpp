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

#include "fsm_localisation/fsm_localisation_interface.hpp"

#include <array>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace fsm_localisation
{
namespace
{
constexpr int kSlowInterval = 10000;

Pose compose(const Pose& a, const Pose& b)
{
  const double cos_t = std::cos(a.t);
  const double sin_t = std::sin(a.t);

  return Pose{
    a.x + cos_t * b.x - sin_t * b.y,
    a.y + sin_t * b.x + cos_t * b.y,
    FSM::Utils::wrapAngle(a.t + b.t)};
}

bool isFinite(const double value)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);

  return ((bits >> 52) & 0x7FFU) != 0x7FFU;
}

Pose invert(const Pose& a)
{
  const double cos_t = std::cos(a.t);
  const double sin_t = std::sin(a.t);

  return Pose{
    -(cos_t * a.x + sin_t * a.y),
    -(-sin_t * a.x + cos_t * a.y),
    FSM::Utils::wrapAngle(-a.t)};
}

/**
 * @brief Whether a rotation turns the z axis of its frame to point down.
 * @details A sensor mounted upside down, rolled half a turn about its forward
 * axis, has its z axis down, and seen from above its rays sweep clockwise
 * where an upright sensor's sweep anticlockwise. Its x axis points the same
 * way as an upright one's would, so the heading `yawOf` reads from it is
 * right, and the heading alone cannot tell the two mountings apart.
 * @remark `orientation` is one `yawOf` has read a heading from, so it is
 * finite and has a length to be normalised by.
 */
bool isUpsideDown(const geometry_msgs::msg::Quaternion& orientation)
{
  const tf2::Matrix3x3 rotation(tf2::Quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w));

  return rotation.getColumn(2).z() < 0.0;
}

rclcpp::QoS scanQos(const std::string& reliability, int depth)
{
  rclcpp::QoS qos(static_cast<std::size_t>(depth));

  if (reliability == "best_effort")
    qos.best_effort();
  else
    qos.reliable();

  return qos;
}

/**
 * @brief A scan's ranges in the order an upright sensor reports them, and
 * where on the robot that upright sensor stands.
 */
struct UprightScan
{
  Pose sensor;
  std::vector<double> ranges;
};

/**
 * @brief `scan` as an upright sensor would have reported it, and where on the
 * robot that sensor stands.
 * @details An upright sensor's ray `k` lies at heading `h + a_k` on the robot,
 * where `h` is the heading of the sensor's x axis and
 * `a_k = angle_min + k * angle_increment`, and the corrector reads every scan
 * as an upright sensor's. A sensor whose z axis points down sweeps the other
 * way, so its ray `j` lies at `h - a_j`, and reading it as `h + a_j` reads the
 * mirror image of the room.
 * Reversing the ranges puts them back in anticlockwise order. Ray `k` of
 * the reversed scan is ray `n - 1 - k` of the scan, which lies at
 * `h - angle_min - (n - 1 - k) * angle_increment`, and that is
 * `(h - s) + a_k` with `s = 2 * angle_min + (n - 1) * angle_increment`, the
 * sum of the scan's first and last angles. The reversed scan is therefore
 * exactly what an upright sensor with the same angles reports from heading
 * `h - s`, and it is handed over as that sensor's. No ray is dropped or
 * interpolated, and the corrector reads its angles exactly as it reads an
 * upright scan's.
 * @remark `s` is nought for a scan symmetric about the sensor's heading, and
 * there the reversal alone is the mirror. It is minus one ray for a panoramic
 * scan beginning at minus pi, and a reversal without the turn would rotate the
 * answer by that ray.
 */
UprightScan uprightScanOf(const sensor_msgs::msg::LaserScan& scan,
  const Pose& sensor_on_robot, const bool upside_down)
{
  if (!upside_down)
  {
    return UprightScan{
      sensor_on_robot,
      std::vector<double>(scan.ranges.begin(), scan.ranges.end())};
  }

  const double rays = static_cast<double>(scan.ranges.size());
  const double skew =
    2.0 * scan.angle_min + (rays - 1.0) * scan.angle_increment;

  return UprightScan{
    Pose{
      sensor_on_robot.x,
      sensor_on_robot.y,
      FSM::Utils::wrapAngle(sensor_on_robot.t - skew)},
    std::vector<double>(scan.ranges.rbegin(), scan.ranges.rend())};
}

std::optional<double> yawOf(const geometry_msgs::msg::Quaternion& orientation)
{
  if (!isFinite(orientation.x) || !isFinite(orientation.y) ||
    !isFinite(orientation.z) || !isFinite(orientation.w))
  {
    return std::nullopt;
  }

  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;

  constexpr double kMinimumQuaternionNormSquared = 1e-9;

  if (norm_squared < kMinimumQuaternionNormSquared)
    return std::nullopt;

  const tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

  return FSM::Utils::wrapAngle(yaw);
}

/**
 * @brief What a scan covers, in radians.
 * @details Every ray occupies one increment, so the span is the ray count times
 * the increment rather than the angle between the first ray and the last. The
 * difference is one increment, and deriving it the other way makes a whole turn
 * come out at a ray more than the sensor carries whatever the sensor is.
 */
double apertureOf(const sensor_msgs::msg::LaserScan& scan)
{
  return static_cast<double>(scan.ranges.size()) * scan.angle_increment;
}

/**
 * @brief Whether a scan's angles are ones a correction can be made from.
 * @details A scan spanning less than a whole turn is accepted and the map is
 * read through the window that sensor could have seen. A scan spanning more
 * than a turn is refused, because a ray of it would be read twice. A scan whose
 * first ray does not sit within a ray of half its own span behind the heading
 * is refused too, as a scan of another convention, and so is one whose first
 * angle is not a number.
 * @return an empty string when the angles can be read, otherwise a sentence
 * naming the field and the value it carried.
 */
std::string anglesProblemOf(const sensor_msgs::msg::LaserScan& scan)
{
  const double rays = static_cast<double>(scan.ranges.size());
  const double increment = scan.angle_increment;
  const double slack = std::abs(increment) > 0.0
    ? std::abs(increment) : 2.0 * M_PI / std::max(1.0, rays);

  if (!(increment > 0.0))
  {
    return std::format("the scan carries angle_increment {}, which is not a "
      "spacing rays can be laid out at", increment);
  }

  const double aperture = rays * increment;

  if (aperture > 2.0 * M_PI + slack)
  {
    return std::format("the scan carries {} ranges at angle_increment {}, "
      "which spans {} radians, more than a full turn. A ray of such a scan "
      "would be read twice", scan.ranges.size(), increment, aperture);
  }

  const double expected_min = -0.5 * aperture;

  if (!std::isfinite(scan.angle_min) ||
    std::abs(FSM::Utils::wrapAngle(scan.angle_min - expected_min)) > slack)
  {
    return std::format("the scan carries angle_min {}, where a scan spanning "
      "{} radians has to be centred on the sensor's heading and so begin "
      "within a ray of {}", scan.angle_min, aperture, expected_min);
  }

  return {};
}

std::string sentenceOf(const CorrectionError error)
{
  switch (error)
  {
    case CorrectionError::scan_too_short:
      return "the scan carries fewer ranges than size_scan asks to match at";
    case CorrectionError::scan_entirely_invalid:
      return "no range in the scan is usable";
    case CorrectionError::estimate_not_in_free_space:
      return "the estimate places the sensor outside the map or on a blocked "
        "cell";
    case CorrectionError::first_cast_entirely_at_max_range:
      return "every ray cast from the estimate leaves the map, so the map "
        "knows nothing about anywhere the sensor can see from there";
    case CorrectionError::recovery_budget_exhausted:
      return "the match spent its whole recovery budget without settling, "
        "which is what an estimate in unmapped space looks like";
    case CorrectionError::aperture_not_a_sensor:
      return "the scan's aperture is not something a sensor can cover: it must "
        "be more than nothing and no more than one whole turn";
  }

  return "the correction failed for an unrecognised reason";
}

}

std::optional<Mode> modeFromName(const std::string& name)
{
  if (name == "amelioration")
    return Mode::amelioration;

  if (name == "odometry")
    return Mode::odometry;

  return std::nullopt;
}

std::string_view nameOfMode(const Mode mode)
{
  switch (mode)
  {
    case Mode::amelioration:
      return "amelioration";
    case Mode::odometry:
      return "odometry";
  }

  assert(false && "nameOfMode: unrecognised Mode");
  return "";
}

Interface::Interface(const rclcpp::NodeOptions& options)
: rclcpp::Node("fsm_localisation", options)
{
  declareParameters();

  mode_ = readMode();

  parameters_ = readParameters(mode_);

  if (const std::string problem = validate(parameters_); !problem.empty())
  {
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  setDiagnosticSink(
    [this](const std::string& message)
    {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    });

  global_frame_id_ = get_parameter("global_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    get_parameter("scan_topic").as_string(),
    scanQos(get_parameter("scan_qos_reliability").as_string(),
      static_cast<int>(get_parameter("scan_qos_depth").as_int())),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
    {
      scanCallback(std::move(scan));
    });

  service_group_ =
    create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  start_service_ = create_service<std_srvs::srv::Trigger>(
    "~/start",
    std::bind(&Interface::start, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "~/stop",
    std::bind(&Interface::stop, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (mode_ == Mode::amelioration)
    setUpAmelioration(parameters_);
  else
    setUpOdometry(parameters_);

  RCLCPP_INFO(get_logger(), "Initialised in %s mode.",
    std::string(nameOfMode(mode_)).c_str());
  RCLCPP_INFO(get_logger(),
    "To start issue ros2 service call %s/start std_srvs/srv/Trigger",
    get_name());
}

void Interface::clearTrajectory(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Clearing trajectory ...");

  const bool was_locked = locked_;
  locked_ = true;
  matcher_->clearTrajectory();
  path_.poses.clear();
  locked_ = was_locked;

  response->success = true;
  response->message = "estimated trajectory cleared";
}

void Interface::correctOnce(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (locked_)
  {
    response->success = false;
    response->message = "the node is stopped; call start first";
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr estimate;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    estimate = latest_estimate_;
  }

  if (!estimate)
  {
    response->success = false;
    response->message = "no pose estimate has arrived on "
      + get_parameter("pose_estimate_input_topic").as_string();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const std::string problem = correctFrom(*estimate);

  response->success = problem.empty();
  response->message = problem.empty() ? "corrected" : problem;
}

std::string Interface::correctFrom(
  const geometry_msgs::msg::PoseWithCovarianceStamped& estimate)
{
  std::shared_ptr<const Corrector> corrector;
  std::shared_ptr<const OccupancyGridMap> map;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    corrector = corrector_;
    map = map_;
    scan = latest_scan_;
  }

  if (!corrector)
    return "no map has arrived on " + get_parameter("map_topic").as_string();

  if (!scan)
    return "no scan has arrived on " + get_parameter("scan_topic").as_string();

  if (const std::string problem = anglesProblemOf(*scan); !problem.empty())
    return problem;

  if (estimate.header.frame_id != global_frame_id_)
  {
    return "the estimate names frame \"" + estimate.header.frame_id
      + "\" but the map is in \"" + global_frame_id_ + "\"";
  }

  if (!isFinite(estimate.pose.pose.position.x) ||
    !isFinite(estimate.pose.pose.position.y))
  {
    return "the estimate's position is not a finite point";
  }

  const std::optional<double> estimate_yaw =
    yawOf(estimate.pose.pose.orientation);

  if (!estimate_yaw.has_value())
  {
    return "the estimate's orientation carries no heading: a quaternion must "
      "be finite and must have a length to be normalised by";
  }

  if (estimate.header.stamp.sec < 0)
    return "the estimate is stamped before zero, which a ROS time cannot hold";

  const Pose estimated_base{
    estimate.pose.pose.position.x,
    estimate.pose.pose.position.y,
    *estimate_yaw};

  geometry_msgs::msg::TransformStamped placement;

  try
  {
    placement = transform_buffer_->lookupTransform(
      robot_frame_id_, scan->header.frame_id,
      rclcpp::Time(estimate.header.stamp), transform_timeout_);
  }
  catch (const tf2::TransformException& exception)
  {
    return "the sensor's placement on the robot could not be read from the "
      "transform tree within " + std::to_string(transform_timeout_.seconds())
      + " s: " + exception.what();
  }

  const std::optional<double> placement_yaw =
    yawOf(placement.transform.rotation);

  if (!placement_yaw.has_value())
    return "the sensor's placement on the robot carries no heading";

  const Pose sensor_on_robot{
    placement.transform.translation.x,
    placement.transform.translation.y,
    *placement_yaw};

  const bool upside_down = isUpsideDown(placement.transform.rotation);

  const UprightScan upright =
    uprightScanOf(*scan, sensor_on_robot, upside_down);

  const Pose estimated_sensor = compose(estimated_base, upright.sensor);

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector->correct(upright.ranges, estimated_sensor, scan->range_max,
      apertureOf(*scan), scan->angle_min);

  if (!corrected.has_value())
    return sentenceOf(corrected.error());

  const rclcpp::Time stamp(estimate.header.stamp);

  double interval = 0.0;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    interval = (stamp - previous_estimate_stamp_).seconds();
    previous_estimate_stamp_ = stamp;
  }

  if (interval > 0.0 && corrected->execution_time > interval)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "A correction takes %.0f ms but estimates arrive every %.0f ms, so "
      "estimates are going uncorrected. Matching is at %zu rays; setting "
      "size_scan below that matches fewer and takes less time over it.",
      1000.0 * corrected->execution_time, 1000.0 * interval,
      upright.ranges.size());
  }

  if (corrected->num_recoveries > 0)
  {
    RCLCPP_WARN(get_logger(),
      "Correction needed %u recovery attempts; it is not reproducible across "
      "builds", corrected->num_recoveries);
  }

  const Pose corrected_base =
    compose(corrected->corrected, invert(upright.sensor));

  publishCorrectedPose(estimate, corrected_base);
  publishCorrectionTransforms(estimated_base, corrected_base, stamp);

  if (map_scan_publisher_)
    publishMapScan(*scan, corrected->corrected, upside_down, stamp);

  return {};
}

void Interface::declareParameters()
{
  const std::string name = get_name();

  declare_parameter("mode", "");

  declare_parameter("scan_topic", "/base_scan");
  declare_parameter("pose_estimate_topic", name + "/pose_estimate");

  declare_parameter("global_frame_id", "map");
  declare_parameter("base_frame_id", "base_laser_link");

  declare_parameter("scan_qos_reliability", "reliable");
  declare_parameter("scan_qos_depth", 1);

  declare_parameter("size_scan", 0);
  declare_parameter("min_magnification_size", 0);
  declare_parameter("max_magnification_size", 3);
  declare_parameter("num_iterations", 2);
  declare_parameter("xy_bound", 0.2);
  declare_parameter("t_bound", M_PI / 4);
  declare_parameter("max_counter", 200);
  declare_parameter("max_recoveries", 10);
  declare_parameter("rng_seed", 0);
  declare_parameter("ray_search", "angular");

  declare_parameter("enforce_early_gearup", false);
  declare_parameter("enforce_translation_bound", false);
  declare_parameter("recover_on_failed_sift", false);

  declare_parameter("initial_pose_topic", name + "/initial_pose");
  declare_parameter("path_estimate_topic", name + "/path_estimate");
  declare_parameter("lo_topic", name + "/lo");
  declare_parameter("lo_frame_id", "lo");

  declare_parameter("map_topic", "/map");
  declare_parameter("pose_estimate_input_topic", "/amcl_pose");
  declare_parameter("map_scan_topic", name + "/map_scan");
  declare_parameter("map_scan_method", "ray_marching");
  declare_parameter("occupancy_threshold", 65);
  declare_parameter("robot_frame_id", "base_footprint");
  declare_parameter("estimate_frame_id", name + "/estimate");
  declare_parameter("correction_frame_id", name + "/corrected");
  declare_parameter("tf_lookup_timeout", 0.1);
}

void Interface::estimateCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr estimate)
{
  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    latest_estimate_ = estimate;
  }

  if (locked_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "Will not correct this estimate: the node is stopped");
    return;
  }

  if (const std::string problem = correctFrom(*estimate); !problem.empty())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "No correction from this estimate: %s", problem.c_str());
  }
}

void Interface::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
  const std::expected<std::shared_ptr<const OccupancyGridMap>, std::string> built =
    OccupancyGridMap::build(*map, occupancy_threshold_, casting_method_);

  if (!built.has_value())
  {
    RCLCPP_ERROR(get_logger(), "Ignoring this map: %s", built.error().c_str());
    return;
  }

  const auto corrector = std::make_shared<const Corrector>(parameters_, *built);

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    map_ = *built;
    corrector_ = corrector;
  }

  RCLCPP_INFO(get_logger(),
    "Map taken: %u by %u cells at %.3f m, cast by %s.",
    map->info.width, map->info.height, map->info.resolution,
    std::string(nameOfCastingMethod(casting_method_)).c_str());
}

void Interface::publishCorrectedPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped& estimate,
  const Pose& corrected_base)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.stamp = estimate.header.stamp;
  message.header.frame_id = global_frame_id_;
  message.pose.pose = retypePose(corrected_base);
  message.pose.covariance = estimate.pose.covariance;

  corrected_pose_publisher_->publish(message);
}

void Interface::publishCorrectionTransforms(
  const Pose& estimated_base, const Pose& corrected_base,
  const rclcpp::Time& stamp)
{
  const auto asTransform =
    [this, &stamp](const std::string& parent, const std::string& child,
      const Pose& pose)
    {
      geometry_msgs::msg::TransformStamped message;
      message.header.stamp = stamp;
      message.header.frame_id = parent;
      message.child_frame_id = child;

      message.transform.translation.x = pose.x;
      message.transform.translation.y = pose.y;
      message.transform.translation.z = 0.0;

      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, pose.t);
      quaternion.normalize();
      message.transform.rotation.x = quaternion.x();
      message.transform.rotation.y = quaternion.y();
      message.transform.rotation.z = quaternion.z();
      message.transform.rotation.w = quaternion.w();

      transform_broadcaster_->sendTransform(message);
    };

  asTransform(global_frame_id_, estimate_frame_id_, estimated_base);
  asTransform(estimate_frame_id_, correction_frame_id_,
    compose(invert(estimated_base), corrected_base));
}

/**
 * @details Ray `k` of the cast is cast on its own, along the direction the
 * sensor's ray `k` points in, so the cast carries the scan's own first angle,
 * spacing and ray count and lies over the scan ray for ray whatever the
 * sensor covers. One cast of the scan's ray count spread over a whole turn
 * would lie over the scan only where the scan is a whole turn beginning half
 * a turn behind the heading.
 * For a sensor mounted upside down `corrected_sensor` is the upright sensor
 * its scan was handed over as, whose ray `n - 1 - k` points where the mounted
 * sensor's ray `k` does, so ray `k` is cast along that upright sensor's ray
 * `n - 1 - k`.
 * The map casts a single ray half a turn behind the pose it is given, so each
 * ray is cast from a pose turned half a turn past the ray's own direction.
 */
void Interface::publishMapScan(const sensor_msgs::msg::LaserScan& real_scan,
  const Pose& corrected_sensor, const bool upside_down,
  const rclcpp::Time& stamp)
{
  std::shared_ptr<const OccupancyGridMap> map;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    map = map_;
  }

  if (!map)
    return;

  const std::size_t rays = real_scan.ranges.size();

  sensor_msgs::msg::LaserScan message;
  message.header.stamp = stamp;
  message.header.frame_id = real_scan.header.frame_id;
  message.angle_min = real_scan.angle_min;
  message.angle_max = real_scan.angle_max;
  message.angle_increment = real_scan.angle_increment;
  message.range_min = real_scan.range_min;
  message.range_max = real_scan.range_max;
  message.ranges.reserve(rays);

  for (std::size_t ray = 0; ray < rays; ++ray)
  {
    const std::size_t upright_ray = upside_down ? rays - 1 - ray : ray;
    const double heading = corrected_sensor.t + real_scan.angle_min
      + static_cast<double>(upright_ray) * real_scan.angle_increment;

    message.ranges.push_back(static_cast<float>(map->ranges(
      Pose{corrected_sensor.x, corrected_sensor.y, heading + M_PI}, 1U,
      real_scan.range_max).front()));
  }

  map_scan_publisher_->publish(message);
}

void Interface::publishOdometry(const MatchResult& result,
  const rclcpp::Time& stamp, double interval)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = lo_frame_id_;
  message.child_frame_id = base_frame_id_;

  message.pose.pose = retypePose(result.increment);

  const std::array<double, 36> covariance{};
  message.pose.covariance = covariance;
  message.twist.covariance = covariance;

  if (interval > 0.0)
  {
    message.twist.twist.linear.x = result.increment.x / interval;
    message.twist.twist.linear.y = result.increment.y / interval;
    message.twist.twist.angular.z = result.increment.t / interval;
  }
  else
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "Scans %.9f seconds apart carry no velocity between them, so this "
      "odometry message reports none. The displacement it reports stands.",
      interval);
  }

  odometry_publisher_->publish(message);
}

void Interface::publishPath(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = global_frame_id_;
  pose.pose = retypePose(result.accumulated);

  path_.header.stamp = stamp;
  path_.header.frame_id = global_frame_id_;
  path_.poses.push_back(pose);

  path_publisher_->publish(path_);
}

void Interface::publishPose(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_id_;
  message.pose = retypePose(result.accumulated);

  pose_publisher_->publish(message);
}

void Interface::publishTransform(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = lo_frame_id_;
  message.child_frame_id = base_frame_id_;

  message.transform.translation.x = result.increment.x;
  message.transform.translation.y = result.increment.y;
  message.transform.translation.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, result.increment.t);
  message.transform.rotation.x = quaternion.x();
  message.transform.rotation.y = quaternion.y();
  message.transform.rotation.z = quaternion.z();
  message.transform.rotation.w = quaternion.w();

  transform_broadcaster_->sendTransform(message);
}

Mode Interface::readMode()
{
  const std::string name = get_parameter("mode").as_string();

  const std::optional<Mode> mode = modeFromName(name);

  if (!mode.has_value())
  {
    const std::string problem = "mode must be \"odometry\" or "
      "\"amelioration\", got \"" + name + "\"";
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  return *mode;
}

Parameters Interface::readParameters(const Mode mode)
{
  Parameters parameters;
  parameters.size_scan =
    static_cast<std::size_t>(get_parameter("size_scan").as_int());
  parameters.num_iterations =
    static_cast<unsigned int>(get_parameter("num_iterations").as_int());
  parameters.xy_bound = get_parameter("xy_bound").as_double();
  parameters.t_bound = get_parameter("t_bound").as_double();
  parameters.max_counter =
    static_cast<unsigned int>(get_parameter("max_counter").as_int());
  parameters.min_magnification_size =
    static_cast<unsigned int>(get_parameter("min_magnification_size").as_int());
  parameters.max_magnification_size =
    static_cast<unsigned int>(get_parameter("max_magnification_size").as_int());
  parameters.max_recoveries =
    static_cast<unsigned int>(get_parameter("max_recoveries").as_int());
  parameters.rng_seed =
    static_cast<unsigned int>(get_parameter("rng_seed").as_int());
  parameters.ray_search = get_parameter("ray_search").as_string();

  parameters.enforce_early_gearup =
    get_parameter("enforce_early_gearup").as_bool();
  parameters.enforce_translation_bound =
    get_parameter("enforce_translation_bound").as_bool();
  parameters.recover_on_failed_sift =
    get_parameter("recover_on_failed_sift").as_bool();

  if (mode == Mode::amelioration)
  {
    parameters.occupancy_threshold =
      static_cast<int>(get_parameter("occupancy_threshold").as_int());
    parameters.map_scan_method = get_parameter("map_scan_method").as_string();
  }

  return parameters;
}

geometry_msgs::msg::Pose Interface::retypePose(const Pose& pose) const
{
  geometry_msgs::msg::Pose message;

  message.position.x = pose.x;
  message.position.y = pose.y;
  message.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, pose.t);
  quaternion.normalize();
  message.orientation.x = quaternion.x();
  message.orientation.y = quaternion.y();
  message.orientation.z = quaternion.z();
  message.orientation.w = quaternion.w();

  return message;
}

void Interface::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  if (mode_ == Mode::amelioration)
  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    latest_scan_ = std::move(scan);
    return;
  }

  if (locked_)
  {
    RCLCPP_WARN(get_logger(), "Will not process this scan");
    return;
  }

  if (scan->header.stamp.sec < 0)
  {
    RCLCPP_WARN(get_logger(),
      "Scan stamped before zero, which a ROS time cannot hold; ignoring it");
    return;
  }

  const rclcpp::Time stamp(scan->header.stamp);

  const std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());

  const std::expected<MatchResult, MatchError> result =
    matcher_->process(ranges);

  if (!result.has_value())
  {
    if (result.error() == MatchError::scan_too_short)
    {
      RCLCPP_WARN(get_logger(),
        "Scan carries %zu ranges but size_scan is %zu; ignoring it",
        ranges.size(), matcher_->parameters().size_scan);
    }

    previous_stamp_ = stamp;
    return;
  }

  const double interval = (stamp - previous_stamp_).seconds();
  previous_stamp_ = stamp;

  RCLCPP_INFO(get_logger(), "FSM executed in %.1f ms",
    1000.0 * result->execution_time);

  if (result->execution_time > interval)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "A match takes %.0f ms but scans arrive every %.0f ms, so scans are "
      "being dropped. Matching is at %zu rays; setting size_scan below that "
      "matches fewer and takes less time over it.",
      1000.0 * result->execution_time, 1000.0 * interval,
      matcher_->matchSize());
  }

  if (result->num_recoveries > 0)
  {
    RCLCPP_WARN(get_logger(),
      "Match needed %u recovery attempts; this scan pair is not reproducible "
      "across builds", result->num_recoveries);
  }

  publishTransform(*result, stamp);
  publishOdometry(*result, stamp, interval);
  publishPose(*result, stamp);
  publishPath(*result, stamp);
}

void Interface::setInitialPose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;

  const auto listener = std::make_shared<rclcpp::Node>(
    std::string(get_name()) + "_initial_pose_listener");

  const bool arrived =
    rclcpp::wait_for_message(message, listener, initial_pose_topic_);

  if (!arrived)
  {
    response->success = false;
    response->message = "no message on " + initial_pose_topic_;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!isFinite(message.pose.pose.position.x) ||
    !isFinite(message.pose.pose.position.y))
  {
    response->success = false;
    response->message = "initial pose position is not a finite point";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const std::optional<double> yaw = yawOf(message.pose.pose.orientation);

  if (!yaw.has_value())
  {
    response->success = false;
    response->message =
      "initial pose orientation carries no heading: a quaternion must be "
      "finite and must have a length to be normalised by, and four zeros "
      "have neither";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const Pose pose{
    message.pose.pose.position.x,
    message.pose.pose.position.y,
    *yaw};

  matcher_->setInitialPose(pose);

  response->success = true;
  response->message = "initial pose set";
  RCLCPP_INFO(get_logger(), "Setting initial pose to (%.2f,%.2f,%.2f)",
    pose.x, pose.y, pose.t);
}

void Interface::setUpAmelioration(const Parameters& parameters)
{
  const std::optional<CastingMethod> method =
    castingMethodFromName(parameters.map_scan_method);

  if (!method.has_value())
  {
    const std::string problem = "map_scan_method must be \"ray_marching\", "
      "\"bresenham\" or \"cddt\", got \"" + parameters.map_scan_method + "\"";
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  casting_method_ = *method;
  occupancy_threshold_ = parameters.occupancy_threshold;

  const double timeout = get_parameter("tf_lookup_timeout").as_double();

  if (!isFinite(timeout) || timeout < 0.0)
  {
    const std::string problem =
      "tf_lookup_timeout must be a finite number and must not be negative, "
      "got " + std::to_string(timeout);
    RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
    throw std::invalid_argument(problem);
  }

  transform_timeout_ = rclcpp::Duration::from_seconds(timeout);

  const std::string map_topic = get_parameter("map_topic").as_string();
  const std::string estimate_topic =
    get_parameter("pose_estimate_input_topic").as_string();
  const std::string corrected_topic =
    get_parameter("pose_estimate_topic").as_string();

  for (const auto& [name, value] : {
      std::pair<std::string, std::string>{"map_topic", map_topic},
      {"pose_estimate_input_topic", estimate_topic},
      {"pose_estimate_topic", corrected_topic},
      {"scan_topic", get_parameter("scan_topic").as_string()}})
  {
    if (value.empty())
    {
      const std::string problem = name + " must name a topic";
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
      throw std::invalid_argument(problem);
    }
  }

  robot_frame_id_ = get_parameter("robot_frame_id").as_string();
  estimate_frame_id_ = get_parameter("estimate_frame_id").as_string();
  correction_frame_id_ = get_parameter("correction_frame_id").as_string();
  map_scan_topic_ = get_parameter("map_scan_topic").as_string();

  for (const auto& [name, value] : {
      std::pair<std::string, std::string>{"global_frame_id", global_frame_id_},
      {"robot_frame_id", robot_frame_id_},
      {"estimate_frame_id", estimate_frame_id_},
      {"correction_frame_id", correction_frame_id_}})
  {
    if (value.empty() || value.front() == '/')
    {
      const std::string problem = name + " must name a frame and must not "
        "begin with a slash, got \"" + value + "\"";
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
      throw std::invalid_argument(problem);
    }
  }

  transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  transform_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*transform_buffer_, this);

  corrected_pose_publisher_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      corrected_topic, 1);

  if (!map_scan_topic_.empty())
  {
    map_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      map_scan_topic_, 1);
  }

  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
    {
      mapCallback(std::move(map));
    });

  estimate_subscription_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      estimate_topic, 1,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr estimate)
      {
        estimateCallback(std::move(estimate));
      });

  correct_once_service_ = create_service<std_srvs::srv::Trigger>(
    "~/correct_once",
    std::bind(&Interface::correctOnce, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);
}

void Interface::setUpOdometry(const Parameters& parameters)
{
  matcher_ = std::make_unique<Matcher>(parameters);

  lo_frame_id_ = get_parameter("lo_frame_id").as_string();
  initial_pose_topic_ = get_parameter("initial_pose_topic").as_string();

  for (const auto& [name, value] : {
      std::pair<std::string, std::string>{"scan_topic",
        get_parameter("scan_topic").as_string()},
      {"initial_pose_topic", initial_pose_topic_},
      {"pose_estimate_topic", get_parameter("pose_estimate_topic").as_string()},
      {"path_estimate_topic", get_parameter("path_estimate_topic").as_string()},
      {"lo_topic", get_parameter("lo_topic").as_string()}})
  {
    if (value.empty())
    {
      const std::string problem = name + " must name a topic";
      RCLCPP_FATAL(get_logger(), "Refusing to start: %s", problem.c_str());
      throw std::invalid_argument(problem);
    }
  }

  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    get_parameter("lo_topic").as_string(), 1);
  pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    get_parameter("pose_estimate_topic").as_string(), 1);
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(
    get_parameter("path_estimate_topic").as_string(), 1);

  clear_trajectory_service_ = create_service<std_srvs::srv::Trigger>(
    "~/clear_estimated_trajectory",
    std::bind(&Interface::clearTrajectory, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  set_initial_pose_service_ = create_service<std_srvs::srv::Trigger>(
    "~/set_initial_pose",
    std::bind(&Interface::setInitialPose, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  path_.header.frame_id = global_frame_id_;
}

void Interface::start(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "%s is now available.",
    std::string(nameOfMode(mode_)).c_str());
  locked_ = false;

  response->success = true;
  response->message = std::string(nameOfMode(mode_)) + " started";
}

void Interface::stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "%s is shut down.",
    std::string(nameOfMode(mode_)).c_str());
  locked_ = true;

  response->success = true;
  response->message = std::string(nameOfMode(mode_)) + " stopped";
}

}
