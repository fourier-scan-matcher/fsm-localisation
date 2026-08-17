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
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace fsm_localisation
{

namespace
{

/* How often a repeated complaint is allowed to say itself, in milliseconds. */
constexpr int kSlowInterval = 10000;

/*
 * `b` expressed in the frame `a` is expressed in. Used to put the sensor on
 * the map from the robot's estimated pose and the sensor's placement on the
 * robot, and to put the correction back on the robot afterwards.
 */
Pose compose(const Pose& a, const Pose& b)
{
  const double cos_t = std::cos(a.t);
  const double sin_t = std::sin(a.t);

  return Pose{
    a.x + cos_t * b.x - sin_t * b.y,
    a.y + sin_t * b.x + cos_t * b.y,
    FSM::Utils::wrapAngle(a.t + b.t)};
}

/*
 * The exponent is inspected directly rather than asking std::isfinite,
 * following the reasoning set out over isValidRange in fsm_localisation.cpp:
 * this package ships under fast arithmetic, and a guard the compiler is
 * allowed to fold away is no guard at all.
 */
bool isFinite(const double value)
{
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);

  return ((bits >> 52) & 0x7FFU) != 0x7FFU;
}

/* The frame `a` is expressed in, expressed in `a`. */
Pose invert(const Pose& a)
{
  const double cos_t = std::cos(a.t);
  const double sin_t = std::sin(a.t);

  return Pose{
    -(cos_t * a.x + sin_t * a.y),
    -(-sin_t * a.x + cos_t * a.y),
    FSM::Utils::wrapAngle(-a.t)};
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

/*
 * Reading a heading out of a quaternion divides by the quaternion's own
 * length, so one of length zero has no heading to give. Four zeros is the
 * commonest way to arrive at one: it is what a bridge carrying ROS 1 traffic
 * delivers for an orientation nobody set, and what anybody assembling a pose
 * by hand produces on forgetting the fourth component. What the division then
 * yields is up to the compiler, and neither answer is usable. A value that is
 * not a number accumulates into every pose the node goes on to publish and no
 * later scan clears it; a heading of zero is a bearing nobody asked for,
 * quietly wrong for the rest of the run. Nothing is returned instead, and the
 * caller says so.
 *
 * This mirrors the refusal the occupancy grid reader already makes of the same
 * quaternion, so a pose and a map are held to one standard.
 */
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

/* What a correction could not get past, as the sentence it is reported with. */
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
  }

  return "the correction failed for an unrecognised reason";
}

}  // namespace

/*******************************************************************************
*/
std::optional<Mode> modeFromName(const std::string& name)
{
  if (name == "amelioration")
    return Mode::amelioration;

  if (name == "odometry")
    return Mode::odometry;

  return std::nullopt;
}

/*******************************************************************************
*/
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

/*******************************************************************************
*/
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

  /*
   * The core reports in plain strings and does not say how serious any of them
   * is, so none is invented here. Nearly everything it has to say is stage
   * timing that an ordinary build compiles out entirely; what reaches this in
   * an ordinary build is rare and worth seeing, which is why it goes out at
   * info rather than debug.
   */
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

/*******************************************************************************
*/
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

/*******************************************************************************
 * One correction on demand, from what the node is holding right now. The
 * outcome goes into the response rather than only into the log, since a caller
 * that asked for one correction is owed the answer to that question.
 */
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

  geometry_msgs::msg::PoseWithCovarianceStamped estimate;

  /*
   * The listener is a throwaway node rather than this one, for the reason set
   * out over setInitialPose: waiting on a subscription belonging to a node an
   * executor is already spinning puts it into two wait sets at once.
   */
  const auto listener = std::make_shared<rclcpp::Node>(
    std::string(get_name()) + "_estimate_listener");

  const bool arrived = rclcpp::wait_for_message(estimate, listener,
    get_parameter("pose_estimate_input_topic").as_string());

  if (!arrived)
  {
    response->success = false;
    response->message = "no pose estimate has arrived on "
      + get_parameter("pose_estimate_input_topic").as_string();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const std::string problem = correctFrom(estimate);

  response->success = problem.empty();
  response->message = problem.empty() ? "corrected" : problem;
}

/*******************************************************************************
 * Specification FR-4, steps 1 to 7.
 */
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

  const Pose estimated_base{
    estimate.pose.pose.position.x,
    estimate.pose.pose.position.y,
    *estimate_yaw};

  /*
   * Where the sensor sits on the robot. The robot's own frame is named by a
   * parameter and the sensor's is read off the scan, which always carries it,
   * so nothing here has to be told twice what the scan already says.
   */
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

  const Pose estimated_sensor = compose(estimated_base, sensor_on_robot);

  const std::vector<double> ranges(scan->ranges.begin(), scan->ranges.end());

  const std::expected<CorrectionResult, CorrectionError> corrected =
    corrector->correct(ranges, estimated_sensor, scan->range_max);

  if (!corrected.has_value())
    return sentenceOf(corrected.error());

  const rclcpp::Time stamp(estimate.header.stamp);

  /*
   * Under the same lock as the map and the scan, because a correction can be
   * driven either by an estimate arriving or by the service, and the service
   * group is reentrant.
   */
  double interval = 0.0;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    interval = (stamp - previous_estimate_stamp_).seconds();
    previous_estimate_stamp_ = stamp;
  }

  /*
   * The same complaint the odometry mode makes about scans, for the same
   * reason and with the same remedy: a correction that outlasts the gap
   * between estimates means estimates are going uncorrected, and the setting
   * that buys the time back is the one that matches fewer rays.
   */
  if (interval > 0.0 && corrected->execution_time > interval)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "A correction takes %.0f ms but estimates arrive every %.0f ms, so "
      "estimates are going uncorrected. Matching is at %zu rays; setting "
      "size_scan below that matches fewer and takes less time over it.",
      1000.0 * corrected->execution_time, 1000.0 * interval, ranges.size());
  }

  if (corrected->num_recoveries > 0)
  {
    RCLCPP_WARN(get_logger(),
      "Correction needed %u recovery attempts; it is not reproducible across "
      "builds", corrected->num_recoveries);
  }

  const Pose corrected_base =
    compose(corrected->corrected, invert(sensor_on_robot));

  publishCorrectedPose(estimate, corrected_base);
  publishCorrectionTransforms(estimated_base, corrected_base, stamp);

  if (map_scan_publisher_)
    publishMapScan(*scan, corrected->corrected, stamp);

  return {};
}

/*******************************************************************************
*/
void Interface::declareParameters()
{
  const std::string name = get_name();

  /* Mode. No default: a node that does not know which of the two it is has
   * nothing sensible to fall back on. */
  declare_parameter("mode", "");

  /* Shared. */
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

  /* Odometry only. */
  declare_parameter("initial_pose_topic", name + "/initial_pose");
  declare_parameter("path_estimate_topic", name + "/path_estimate");
  declare_parameter("lo_topic", name + "/lo");
  declare_parameter("lo_frame_id", "lo");

  /* Amelioration only. */
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

/*******************************************************************************
 * An estimate is what drives a correction, so this is the trigger rather than
 * the scan.
 */
void Interface::estimateCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr estimate)
{
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

/*******************************************************************************
 * A map replaces whatever came before it. A correction already running holds
 * its own reference and finishes against the map it started with.
 */
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

/*******************************************************************************
 * The corrected robot pose. The covariance is the input's, copied through
 * untouched: this node measures a pose and has nothing to say about how
 * certain the sender was of the one it sent.
 */
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

/*******************************************************************************
 * Two transforms to two frames that exist only for this purpose, so no
 * existing tree gains a second parent. The first carries the estimate as it
 * arrived; the second carries the correction alone, so anything wanting only
 * how far the estimate was out reads that one and nothing else.
 */
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

/*******************************************************************************
 * The scan the corrected pose would see, for laying over the one the sensor
 * did see. Costs one extra cast per correction, and nothing in the matching
 * depends on it.
 *
 * The angular fields are the real scan's, verbatim, which is what makes the
 * two comparable. That is only meaningful where the real scan is a full turn,
 * which is the shape this matcher takes everywhere.
 */
void Interface::publishMapScan(const sensor_msgs::msg::LaserScan& real_scan,
  const Pose& corrected_sensor, const rclcpp::Time& stamp)
{
  std::shared_ptr<const OccupancyGridMap> map;

  {
    const std::lock_guard<std::mutex> lock(held_mutex_);
    map = map_;
  }

  if (!map)
    return;

  const std::vector<double> ranges = map->ranges(corrected_sensor,
    static_cast<unsigned int>(real_scan.ranges.size()), real_scan.range_max);

  sensor_msgs::msg::LaserScan message;
  message.header.stamp = stamp;
  message.header.frame_id = real_scan.header.frame_id;
  message.angle_min = real_scan.angle_min;
  message.angle_max = real_scan.angle_max;
  message.angle_increment = real_scan.angle_increment;
  message.range_min = real_scan.range_min;
  message.range_max = real_scan.range_max;

  message.ranges.assign(ranges.begin(), ranges.end());

  map_scan_publisher_->publish(message);
}

/*******************************************************************************
 * Construct the lo_frame_id <- base_frame_id odometry message and publish it
 *
 * Two consecutive scans can carry the same stamp, and it is not a fault when
 * they do: a replayed recording is free to repeat one, and a driver that
 * stamps on publication rather than on acquisition can emit two inside a
 * single clock tick. A recording that does it is more likely to be replayed
 * than fixed, so this is an ordinary path and not an impossible one.
 *
 * The displacement between such a pair is measured and is published as
 * measured, since it is the node's whole product and dropping the message to
 * protect a field derived from it would cost more than it saved. The velocity
 * is left at zero. A displacement over no elapsed time is not a large rate, it
 * is no rate at all, and dividing anyway yields an infinity that any consumer
 * folding it into a filter carries for the rest of the run with no way back. A
 * zero is a claim about one sample that the next scan corrects. The same holds
 * for a stamp that goes backwards, which is why the interval is required to be
 * positive rather than merely different from zero.
 */
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

/*******************************************************************************
 * The total path estimate with respect to the global frame
 */
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

/*******************************************************************************
 * The current pose estimate with respect to the global frame
 */
void Interface::publishPose(const MatchResult& result,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = global_frame_id_;
  message.pose = retypePose(result.accumulated);

  pose_publisher_->publish(message);
}

/*******************************************************************************
 * Construct the lo_frame_id <- base_frame_id transform and publish it
 */
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

/*******************************************************************************
 * The mode is read before anything else, because everything else is read
 * differently depending on it.
 */
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

/*******************************************************************************
 * Only what the chosen mode uses is read, and therefore only what the chosen
 * mode uses is validated. A configuration file carrying a nonsense setting for
 * the mode that is not running starts the node, which is what lets one file
 * serve both modes.
 */
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

/*******************************************************************************
*/
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

/*******************************************************************************
 * In the odometry mode a scan is the whole event. In the amelioration mode it
 * is held until an estimate asks for it, and only the newest one is kept: a
 * correction is answered from where the robot is now, not from where it was
 * when some earlier scan arrived.
 */
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

  /*
   * Execution time rises faster than the ray count does, so a scan matched at
   * a high resolution can take longer than the sensor takes to produce the
   * next one. Every scan arriving while this one is still being matched is
   * dropped, the odometry thins out, and nothing so far says why. Say why, and
   * name the setting that fixes it. Throttled, because a node that cannot keep
   * up would otherwise say so on every scan it does manage.
   */
  if (result->execution_time > interval)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kSlowInterval,
      "A match takes %.0f ms but scans arrive every %.0f ms, so scans are "
      "being dropped. Matching is at %zu rays; setting size_scan below that "
      "matches fewer and takes less time over it.",
      1000.0 * result->execution_time, 1000.0 * interval,
      matcher_->matchSize());
  }

  /*
   * Recovery draws a pose at random, so a match that needed it is not
   * reproducible across two builds even from the same seed: the standard
   * library does not specify how a distribution turns random bits into a
   * number. Any comparison of this node against another build is only as good
   * as this line staying quiet.
   */
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

/*******************************************************************************
 * If there is an initial pose then set it
 */
void Interface::setInitialPose(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  geometry_msgs::msg::PoseWithCovarianceStamped message;

  /*
   * The listener is a throwaway node rather than this one. Waiting on a
   * subscription belonging to a node an executor is already spinning puts that
   * subscription into two wait sets at once, which throws and takes the whole
   * process down.
   */
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

  /*
   * A pose the node cannot use is refused outright rather than repaired.
   * Whatever is put in place of it would be reported as the operator's own
   * starting point and carried through every pose the node publishes, and
   * there is no way to tell afterwards that it was invented here.
   */
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

/*******************************************************************************
*/
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

  /*
   * tf2 refuses a frame whose name begins with a slash, and the refusal
   * surfaces later, from inside a lookup, as a message about a frame that does
   * not exist. Said here instead, once, about the setting that caused it.
   */
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

  /* An empty topic disables the inspection scan, which is the one thing here
   * that exists to be looked at rather than used. */
  if (!map_scan_topic_.empty())
  {
    map_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      map_scan_topic_, 1);
  }

  /*
   * Transient local with a depth of one, so a map latched before this node
   * started is delivered on subscription rather than waited for forever.
   */
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

/*******************************************************************************
*/
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

/*******************************************************************************
*/
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

/*******************************************************************************
*/
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

}  // namespace fsm_localisation
