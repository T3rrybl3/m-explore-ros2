/*********************************************************************
 *
 * Direct Nav2 frontier explorer for ROS 2 live-SLAM maps.
 *
 * This node intentionally does not route goals through any proxy action,
 * bounded subgoal filter, or chained subgoal executor. It computes frontiers
 * from /map and sends one NavigateToPose goal at a time directly to Nav2.
 *
 *********************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace explore
{
namespace
{
constexpr int8_t kUnknownCell = -1;
constexpr double kProgressEpsilonM = 0.05;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double distance2d(const geometry_msgs::msg::Point& a,
                  const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::string formatPoint(const geometry_msgs::msg::Point& point)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << "(" << point.x << "," << point.y << ")";
  return stream.str();
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();
  return msg;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  const double siny_cosp =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

const char* resultCodeName(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char* computePathErrorName(uint16_t error_code)
{
  using Result = nav2_msgs::action::ComputePathToPose::Result;
  switch (error_code) {
    case Result::NONE:
      return "none";
    case Result::UNKNOWN:
      return "unknown";
    case Result::INVALID_PLANNER:
      return "invalid_planner";
    case Result::TF_ERROR:
      return "tf_error";
    case Result::START_OUTSIDE_MAP:
      return "start_outside_map";
    case Result::GOAL_OUTSIDE_MAP:
      return "goal_outside_map";
    case Result::START_OCCUPIED:
      return "start_occupied";
    case Result::GOAL_OCCUPIED:
      return "goal_occupied";
    case Result::TIMEOUT:
      return "timeout";
    case Result::NO_VALID_PATH:
      return "no_valid_path";
    default:
      return "unknown_error_code";
  }
}
}  // namespace

enum class ExplorerState
{
  WAITING_FOR_MAP,
  WAITING_FOR_MAP_MATURITY,
  WAITING_FOR_TF,
  WAITING_FOR_NAV2,
  SELECTING_FRONTIER,
  NAVIGATING_TO_FRONTIER,
  RECOVERING_AFTER_FAILURE,
  PAUSED,
  EXPLORATION_COMPLETE,
  EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS,
  EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS
};

struct CandidateStats
{
  size_t raw_generated = 0;
  size_t duplicate_candidates = 0;
  size_t unique_candidates = 0;
  size_t accepted_clearance = 0;
  size_t rejected_unknown = 0;
  size_t rejected_occupied = 0;
  size_t rejected_low_clearance = 0;
  size_t rejected_blacklisted = 0;
  size_t suppressed_by_area_memory = 0;
  size_t rejection_logs_emitted = 0;
};

struct FrontierObservationRecord
{
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point anchor;
  geometry_msgs::msg::Point boundary_anchor;
  double size_m = 0.0;
  std::deque<size_t> observed_cycles;
};

struct FrontierAreaRecord
{
  int area_id = 0;
  geometry_msgs::msg::Point anchor;
  geometry_msgs::msg::Point last_centroid;
  geometry_msgs::msg::Point last_target;
  geometry_msgs::msg::Point last_boundary_anchor;
  double last_size_m = 0.0;
  std::string reason;
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;
  rclcpp::Time expires_at;
  int failure_count = 0;
  int success_count = 0;
  double radius_m = 0.0;
};

struct FrontierGroup
{
  struct GoalCandidate
  {
    geometry_msgs::msg::Point target;
    uint32_t mx = 0;
    uint32_t my = 0;
    double clearance_m = std::numeric_limits<double>::infinity();
    double frontier_distance_m = 0.0;
    double robot_distance_m = 0.0;
    double heading_delta = 0.0;
    double score = 0.0;
  };

  std::vector<geometry_msgs::msg::Point> points;
  std::vector<GoalCandidate> candidates;
  CandidateStats candidate_stats;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point target;
  geometry_msgs::msg::Point blacklist_anchor;
  uint32_t target_mx = 0;
  uint32_t target_my = 0;
  double target_clearance_m = std::numeric_limits<double>::infinity();
  double goal_yaw = 0.0;
  double size_m = 0.0;
  double distance_m = 0.0;
  double heading_delta = 0.0;
  double score = 0.0;
  bool target_valid = false;
  bool blacklist_anchor_valid = false;
  bool cluster_blacklisted = false;
  bool stability_confirmed = false;
  bool unstable_suppressed = false;
  std::string blacklist_reason;
  int area_id = -1;
  double area_memory_penalty = 0.0;
  std::string area_memory_reason;
  std::string residual_classification;
  size_t stability_observations = 0;
};

struct BlacklistEntry
{
  geometry_msgs::msg::Point target;
  rclcpp::Time expires_at;
  int failures = 0;
  std::string reason;
};

struct MapMaturity
{
  bool valid = false;
  bool size_ok = false;
  bool known_ok = false;
  bool initial_wait_elapsed = false;
  uint32_t width = 0;
  uint32_t height = 0;
  size_t known_cell_count = 0;
  double known_ratio = 0.0;
};

struct SelectionResult
{
  enum class Outcome
  {
    SELECTED,
    COMPLETE,
    BLOCKED
  };

  Outcome outcome = Outcome::COMPLETE;
  std::vector<FrontierGroup> frontiers;
  FrontierGroup selected;
  size_t too_close_count = 0;
  size_t blacklisted_count = 0;
  size_t invalid_target_count = 0;
  size_t clearance_rejected_count = 0;
  size_t precheck_rejected_count = 0;
  size_t area_suppressed_count = 0;
  size_t stable_sendable_count = 0;
  size_t unstable_frontier_count = 0;
  size_t no_clear_frontier_count = 0;
  size_t mostly_unknown_frontier_count = 0;
  size_t mostly_occupied_frontier_count = 0;
  size_t low_clearance_frontier_count = 0;
  size_t hard_suppressed_count = 0;
  double total_frontier_size_m = 0.0;
};

struct ComputePathCheckResult
{
  bool reachable = false;
  bool timed_out = false;
  bool goal_rejected = false;
  uint16_t error_code = nav2_msgs::action::ComputePathToPose::Result::UNKNOWN;
  size_t path_poses = 0;
  std::string reason = "not_checked";
};

class DirectNav2Explorer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ComputePathGoalHandle =
      rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  DirectNav2Explorer()
  : Node("explore_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    readParameters();

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(map_mutex_);
          latest_map_ = msg;
          ++map_revision_;
        });

    frontier_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/explore/frontiers", 10);

    rclcpp::QoS status_qos(10);
    status_qos.transient_local();
    status_publisher_ =
        this->create_publisher<explore_lite_msgs::msg::ExploreStatus>(
            "/explore/status", status_qos);

    resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "/explore/resume", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          handleResume(msg->data);
        });

    nav_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, navigate_action_name_);
    compute_path_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    compute_path_client_ =
        rclcpp_action::create_client<ComputePathToPose>(
            this, compute_path_action_name_, compute_path_callback_group_);
    next_rescan_at_ = this->now();
    blocked_retry_at_ = this->now();
    nav2_wait_started_at_ = this->now();
    first_meaningful_map_time_ = this->now();
    exploration_started_at_ = this->now();
    last_useful_frontier_success_at_ = this->now();

    const double safe_frequency = std::max(0.01, planner_frequency_);
    const auto timer_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / safe_frequency));
    planner_timer_ = this->create_wall_timer(
        timer_period, [this]() { onPlannerTimer(); });
    active_goal_monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() { onActiveGoalMonitorTimer(); });

    RCLCPP_INFO(
        this->get_logger(),
        "direct_nav2_explorer_started map_topic=%s global_frame=%s "
        "robot_base_frame=%s navigate_action_name=%s planner_frequency=%.3f",
        map_topic_.c_str(), global_frame_.c_str(), robot_base_frame_.c_str(),
        navigate_action_name_.c_str(), planner_frequency_);
    RCLCPP_INFO(
        this->get_logger(),
        "explore_frontier_area_memory_params enabled=%s radius_m=%.3f "
        "success_radius_m=%.3f failure_radius_m=%.3f memory_timeout_sec=%.3f "
        "success_timeout_sec=%.3f failure_timeout_sec=%.3f "
        "size_growth_ratio=%.3f size_growth_min_m=%.3f "
        "anchor_change_radius_m=%.3f diversity_penalty_enabled=%s "
        "recent_area_penalty=%.3f recent_area_history_size=%d",
        enable_frontier_area_memory_ ? "true" : "false",
        frontier_area_memory_radius_m_, frontier_success_suppress_radius_m_,
        frontier_failure_suppress_radius_m_, frontier_area_memory_timeout_sec_,
        frontier_success_suppress_timeout_sec_,
        frontier_failure_suppress_timeout_sec_,
        frontier_area_size_growth_ratio_, frontier_area_size_growth_min_m_,
        frontier_area_anchor_change_radius_m_,
        enable_frontier_diversity_penalty_ ? "true" : "false",
        recent_area_penalty_, recent_area_history_size_);
    RCLCPP_INFO(
        this->get_logger(),
        "explore_tolerance_completion_params enabled=%s "
        "goal_reached_confirm_count=%d goal_reached_hold_sec=%.3f "
        "max_wait_after_goal_reached_sec=%.3f "
        "cancel_nav2_on_explorer_tolerance_success=%s "
        "nav2_cancel_after_tolerance_timeout_sec=%.3f",
        enable_explorer_tolerance_completion_ ? "true" : "false",
        goal_reached_confirm_count_, goal_reached_hold_sec_,
        max_wait_after_goal_reached_sec_,
        cancel_nav2_on_explorer_tolerance_success_ ? "true" : "false",
        nav2_cancel_after_tolerance_timeout_sec_);
    RCLCPP_INFO(
        this->get_logger(),
        "explore_frontier_stability_params enabled=%s "
        "required_observations=%d window=%d unstable_timeout_sec=%.3f "
        "unstable_radius_m=%.3f unknown_ratio_threshold=%.3f "
        "occupied_ratio_threshold=%.3f",
        enable_frontier_stability_filter_ ? "true" : "false",
        frontier_stability_required_observations_,
        frontier_stability_window_, unstable_frontier_suppress_timeout_sec_,
        unstable_frontier_area_radius_m_, unstable_unknown_ratio_threshold_,
        unstable_occupied_ratio_threshold_);
    RCLCPP_INFO(
        this->get_logger(),
        "explore_slam_instability_diagnostic mode=explorer_side "
        "map_flicker_guard=%s residual_exhaustion=%s "
        "slam_nav2_robot_tuning=unchanged",
        enable_frontier_stability_filter_ ? "enabled" : "disabled",
        enable_residual_frontier_exhaustion_ ? "enabled" : "disabled");
    setState(ExplorerState::WAITING_FOR_MAP, "startup");
  }

private:
  template <typename ParameterT>
  void declareParameterIfNeeded(const std::string& name,
                                const ParameterT& default_value)
  {
    if (this->has_parameter(name)) {
      return;
    }
    this->declare_parameter<ParameterT>(name, default_value);
  }

  void declareParameters()
  {
    declareParameterIfNeeded<std::string>("map_topic", "/map");
    declareParameterIfNeeded<std::string>("robot_base_frame", "base_footprint");
    declareParameterIfNeeded<std::string>("global_frame", "map");
    declareParameterIfNeeded<std::string>(
        "navigate_action_name", "/navigate_to_pose");
    declareParameterIfNeeded<bool>("use_sim_time", true);
    declareParameterIfNeeded<double>("planner_frequency", 0.2);
    declareParameterIfNeeded<double>("progress_timeout", 30.0);
    declareParameterIfNeeded<bool>("visualize", true);
    declareParameterIfNeeded<double>("potential_scale", 5.0);
    declareParameterIfNeeded<double>("orientation_scale", 0.2);
    declareParameterIfNeeded<double>("gain_scale", 0.2);
    declareParameterIfNeeded<double>("min_frontier_size", 0.30);
    declareParameterIfNeeded<double>("initial_map_wait_sec", 6.0);
    declareParameterIfNeeded<int>("min_map_width_cells", 50);
    declareParameterIfNeeded<int>("min_map_height_cells", 50);
    declareParameterIfNeeded<int>("min_known_cell_count", 1200);
    declareParameterIfNeeded<double>("min_known_cell_ratio", 0.01);
    declareParameterIfNeeded<double>("min_goal_distance_m", 0.85);
    declareParameterIfNeeded<double>("goal_reached_distance_m", 0.60);
    declareParameterIfNeeded<bool>("enable_explorer_tolerance_completion", true);
    declareParameterIfNeeded<int>("goal_reached_confirm_count", 2);
    declareParameterIfNeeded<double>("goal_reached_hold_sec", 0.5);
    declareParameterIfNeeded<double>("max_wait_after_goal_reached_sec", 2.0);
    declareParameterIfNeeded<bool>(
        "cancel_nav2_on_explorer_tolerance_success", true);
    declareParameterIfNeeded<double>(
        "nav2_cancel_after_tolerance_timeout_sec", 2.0);
    declareParameterIfNeeded<double>("same_goal_distance_m", 0.35);
    declareParameterIfNeeded<double>("blacklist_radius", 0.75);
    declareParameterIfNeeded<double>("blacklist_timeout_sec", 90.0);
    declareParameterIfNeeded<int>("max_consecutive_failures_per_frontier", 2);
    declareParameterIfNeeded<double>("blocked_retry_timeout_sec", 30.0);
    declareParameterIfNeeded<double>("nav2_action_wait_timeout_sec", 30.0);
    declareParameterIfNeeded<double>("tf_wait_timeout_sec", 1.0);
    declareParameterIfNeeded<double>("rescan_delay_sec", 1.0);
    declareParameterIfNeeded<int>("occupied_threshold", 50);
    declareParameterIfNeeded<bool>("log_candidate_rejections_debug", false);
    declareParameterIfNeeded<int>("max_candidate_rejection_logs_per_cycle", 10);
    declareParameterIfNeeded<int>("max_candidate_rejection_logs_per_cluster", 3);
    declareParameterIfNeeded<double>("blocked_no_reachable_backoff_sec", 15.0);
    declareParameterIfNeeded<bool>(
        "require_frontier_set_change_for_blocked_retry", false);
    declareParameterIfNeeded<bool>("enable_frontier_area_memory", true);
    declareParameterIfNeeded<double>("frontier_area_memory_radius_m", 1.25);
    declareParameterIfNeeded<double>(
        "frontier_success_suppress_radius_m", 1.25);
    declareParameterIfNeeded<double>(
        "frontier_failure_suppress_radius_m", 1.25);
    declareParameterIfNeeded<double>("frontier_area_memory_timeout_sec", 90.0);
    declareParameterIfNeeded<double>(
        "frontier_success_suppress_timeout_sec", 60.0);
    declareParameterIfNeeded<double>(
        "frontier_failure_suppress_timeout_sec", 90.0);
    declareParameterIfNeeded<double>("frontier_area_size_growth_ratio", 1.5);
    declareParameterIfNeeded<double>("frontier_area_size_growth_min_m", 1.0);
    declareParameterIfNeeded<double>(
        "frontier_area_anchor_change_radius_m", 1.25);
    declareParameterIfNeeded<bool>("enable_success_area_suppression", true);
    declareParameterIfNeeded<bool>("enable_frontier_diversity_penalty", true);
    declareParameterIfNeeded<double>("recent_area_penalty", 3.0);
    declareParameterIfNeeded<int>("recent_area_history_size", 20);
    declareParameterIfNeeded<bool>("enable_goal_clearance_filter", true);
    declareParameterIfNeeded<double>("min_goal_clearance_m", 0.45);
    declareParameterIfNeeded<double>("frontier_goal_search_radius_m", 1.0);
    declareParameterIfNeeded<int>("max_frontier_goal_candidates", 80);
    declareParameterIfNeeded<bool>("reject_unknown_goal_cells", true);
    declareParameterIfNeeded<bool>("reject_occupied_goal_cells", true);
    declareParameterIfNeeded<bool>("enable_compute_path_precheck", true);
    declareParameterIfNeeded<std::string>(
        "compute_path_action_name", "/compute_path_to_pose");
    declareParameterIfNeeded<double>("compute_path_timeout_sec", 2.0);
    declareParameterIfNeeded<int>("max_precheck_candidates_per_cycle", 8);
    declareParameterIfNeeded<int>("min_precheck_path_poses", 2);
    declareParameterIfNeeded<bool>("enable_frontier_stability_filter", true);
    declareParameterIfNeeded<int>("frontier_stability_required_observations", 2);
    declareParameterIfNeeded<int>("frontier_stability_window", 3);
    declareParameterIfNeeded<double>(
        "unstable_frontier_suppress_timeout_sec", 30.0);
    declareParameterIfNeeded<double>("unstable_frontier_area_radius_m", 1.25);
    declareParameterIfNeeded<double>("unstable_unknown_ratio_threshold", 0.80);
    declareParameterIfNeeded<double>("unstable_occupied_ratio_threshold", 0.50);
    declareParameterIfNeeded<int>(
        "no_clear_area_hard_suppress_after_failures", 3);
    declareParameterIfNeeded<double>("no_clear_area_long_timeout_sec", 180.0);
    declareParameterIfNeeded<bool>(
        "no_clear_area_reuse_requires_clear_candidate", true);
    declareParameterIfNeeded<bool>(
        "failed_area_reuse_requires_compute_path_success", true);
    declareParameterIfNeeded<double>("frontier_area_failure_hard_radius_m", 1.50);
    declareParameterIfNeeded<bool>("enable_residual_frontier_exhaustion", true);
    declareParameterIfNeeded<int>("residual_exhaustion_blocked_cycles", 3);
    declareParameterIfNeeded<double>("residual_exhaustion_min_runtime_sec", 60.0);
    declareParameterIfNeeded<double>(
        "residual_exhaustion_no_success_window_sec", 60.0);
    declareParameterIfNeeded<bool>(
        "residual_exhaustion_require_no_stable_sendable_frontiers", true);
    declareParameterIfNeeded<int>(
        "successful_area_hard_suppress_after_successes", 2);
    declareParameterIfNeeded<double>("successful_area_long_timeout_sec", 120.0);
    declareParameterIfNeeded<bool>(
        "successful_area_reuse_requires_frontier_growth", true);
  }

  void readParameters()
  {
    this->get_parameter("map_topic", map_topic_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("global_frame", global_frame_);
    this->get_parameter("navigate_action_name", navigate_action_name_);
    this->get_parameter("planner_frequency", planner_frequency_);
    this->get_parameter("progress_timeout", progress_timeout_);
    this->get_parameter("visualize", visualize_);
    this->get_parameter("potential_scale", potential_scale_);
    this->get_parameter("orientation_scale", orientation_scale_);
    this->get_parameter("gain_scale", gain_scale_);
    this->get_parameter("min_frontier_size", min_frontier_size_);
    this->get_parameter("initial_map_wait_sec", initial_map_wait_sec_);
    this->get_parameter("min_map_width_cells", min_map_width_cells_);
    this->get_parameter("min_map_height_cells", min_map_height_cells_);
    this->get_parameter("min_known_cell_count", min_known_cell_count_);
    this->get_parameter("min_known_cell_ratio", min_known_cell_ratio_);
    this->get_parameter("min_goal_distance_m", min_goal_distance_m_);
    this->get_parameter("goal_reached_distance_m", goal_reached_distance_m_);
    this->get_parameter("enable_explorer_tolerance_completion",
                        enable_explorer_tolerance_completion_);
    this->get_parameter("goal_reached_confirm_count",
                        goal_reached_confirm_count_);
    this->get_parameter("goal_reached_hold_sec", goal_reached_hold_sec_);
    this->get_parameter("max_wait_after_goal_reached_sec",
                        max_wait_after_goal_reached_sec_);
    this->get_parameter("cancel_nav2_on_explorer_tolerance_success",
                        cancel_nav2_on_explorer_tolerance_success_);
    this->get_parameter("nav2_cancel_after_tolerance_timeout_sec",
                        nav2_cancel_after_tolerance_timeout_sec_);
    this->get_parameter("same_goal_distance_m", same_goal_distance_m_);
    this->get_parameter("blacklist_radius", blacklist_radius_);
    this->get_parameter("blacklist_timeout_sec", blacklist_timeout_sec_);
    this->get_parameter("max_consecutive_failures_per_frontier",
                        max_consecutive_failures_per_frontier_);
    this->get_parameter("blocked_retry_timeout_sec",
                        blocked_retry_timeout_sec_);
    this->get_parameter("nav2_action_wait_timeout_sec",
                        nav2_action_wait_timeout_sec_);
    this->get_parameter("tf_wait_timeout_sec", tf_wait_timeout_sec_);
    this->get_parameter("rescan_delay_sec", rescan_delay_sec_);
    this->get_parameter("occupied_threshold", occupied_threshold_);
    this->get_parameter("log_candidate_rejections_debug",
                        log_candidate_rejections_debug_);
    this->get_parameter("max_candidate_rejection_logs_per_cycle",
                        max_candidate_rejection_logs_per_cycle_);
    this->get_parameter("max_candidate_rejection_logs_per_cluster",
                        max_candidate_rejection_logs_per_cluster_);
    this->get_parameter("blocked_no_reachable_backoff_sec",
                        blocked_no_reachable_backoff_sec_);
    this->get_parameter("require_frontier_set_change_for_blocked_retry",
                        require_frontier_set_change_for_blocked_retry_);
    this->get_parameter("enable_frontier_area_memory",
                        enable_frontier_area_memory_);
    this->get_parameter("frontier_area_memory_radius_m",
                        frontier_area_memory_radius_m_);
    this->get_parameter("frontier_success_suppress_radius_m",
                        frontier_success_suppress_radius_m_);
    this->get_parameter("frontier_failure_suppress_radius_m",
                        frontier_failure_suppress_radius_m_);
    this->get_parameter("frontier_area_memory_timeout_sec",
                        frontier_area_memory_timeout_sec_);
    this->get_parameter("frontier_success_suppress_timeout_sec",
                        frontier_success_suppress_timeout_sec_);
    this->get_parameter("frontier_failure_suppress_timeout_sec",
                        frontier_failure_suppress_timeout_sec_);
    this->get_parameter("frontier_area_size_growth_ratio",
                        frontier_area_size_growth_ratio_);
    this->get_parameter("frontier_area_size_growth_min_m",
                        frontier_area_size_growth_min_m_);
    this->get_parameter("frontier_area_anchor_change_radius_m",
                        frontier_area_anchor_change_radius_m_);
    this->get_parameter("enable_success_area_suppression",
                        enable_success_area_suppression_);
    this->get_parameter("enable_frontier_diversity_penalty",
                        enable_frontier_diversity_penalty_);
    this->get_parameter("recent_area_penalty", recent_area_penalty_);
    this->get_parameter("recent_area_history_size", recent_area_history_size_);
    this->get_parameter("enable_goal_clearance_filter",
                        enable_goal_clearance_filter_);
    this->get_parameter("min_goal_clearance_m", min_goal_clearance_m_);
    this->get_parameter("frontier_goal_search_radius_m",
                        frontier_goal_search_radius_m_);
    this->get_parameter("max_frontier_goal_candidates",
                        max_frontier_goal_candidates_);
    this->get_parameter("reject_unknown_goal_cells",
                        reject_unknown_goal_cells_);
    this->get_parameter("reject_occupied_goal_cells",
                        reject_occupied_goal_cells_);
    this->get_parameter("enable_compute_path_precheck",
                        enable_compute_path_precheck_);
    this->get_parameter("compute_path_action_name", compute_path_action_name_);
    this->get_parameter("compute_path_timeout_sec",
                        compute_path_timeout_sec_);
    this->get_parameter("max_precheck_candidates_per_cycle",
                        max_precheck_candidates_per_cycle_);
    this->get_parameter("min_precheck_path_poses",
                        min_precheck_path_poses_);
    this->get_parameter("enable_frontier_stability_filter",
                        enable_frontier_stability_filter_);
    this->get_parameter("frontier_stability_required_observations",
                        frontier_stability_required_observations_);
    this->get_parameter("frontier_stability_window",
                        frontier_stability_window_);
    this->get_parameter("unstable_frontier_suppress_timeout_sec",
                        unstable_frontier_suppress_timeout_sec_);
    this->get_parameter("unstable_frontier_area_radius_m",
                        unstable_frontier_area_radius_m_);
    this->get_parameter("unstable_unknown_ratio_threshold",
                        unstable_unknown_ratio_threshold_);
    this->get_parameter("unstable_occupied_ratio_threshold",
                        unstable_occupied_ratio_threshold_);
    this->get_parameter("no_clear_area_hard_suppress_after_failures",
                        no_clear_area_hard_suppress_after_failures_);
    this->get_parameter("no_clear_area_long_timeout_sec",
                        no_clear_area_long_timeout_sec_);
    this->get_parameter("no_clear_area_reuse_requires_clear_candidate",
                        no_clear_area_reuse_requires_clear_candidate_);
    this->get_parameter("failed_area_reuse_requires_compute_path_success",
                        failed_area_reuse_requires_compute_path_success_);
    this->get_parameter("frontier_area_failure_hard_radius_m",
                        frontier_area_failure_hard_radius_m_);
    this->get_parameter("enable_residual_frontier_exhaustion",
                        enable_residual_frontier_exhaustion_);
    this->get_parameter("residual_exhaustion_blocked_cycles",
                        residual_exhaustion_blocked_cycles_);
    this->get_parameter("residual_exhaustion_min_runtime_sec",
                        residual_exhaustion_min_runtime_sec_);
    this->get_parameter("residual_exhaustion_no_success_window_sec",
                        residual_exhaustion_no_success_window_sec_);
    this->get_parameter(
        "residual_exhaustion_require_no_stable_sendable_frontiers",
        residual_exhaustion_require_no_stable_sendable_frontiers_);
    this->get_parameter("successful_area_hard_suppress_after_successes",
                        successful_area_hard_suppress_after_successes_);
    this->get_parameter("successful_area_long_timeout_sec",
                        successful_area_long_timeout_sec_);
    this->get_parameter("successful_area_reuse_requires_frontier_growth",
                        successful_area_reuse_requires_frontier_growth_);

    planner_frequency_ = std::max(0.01, planner_frequency_);
    progress_timeout_ = std::max(1.0, progress_timeout_);
    min_frontier_size_ = std::max(0.0, min_frontier_size_);
    initial_map_wait_sec_ = std::max(0.0, initial_map_wait_sec_);
    min_map_width_cells_ = std::max(1, min_map_width_cells_);
    min_map_height_cells_ = std::max(1, min_map_height_cells_);
    min_known_cell_count_ = std::max(0, min_known_cell_count_);
    min_known_cell_ratio_ = std::max(0.0, min_known_cell_ratio_);
    min_goal_distance_m_ = std::max(0.0, min_goal_distance_m_);
    goal_reached_distance_m_ = std::max(0.0, goal_reached_distance_m_);
    goal_reached_confirm_count_ = std::max(1, goal_reached_confirm_count_);
    goal_reached_hold_sec_ = std::max(0.0, goal_reached_hold_sec_);
    max_wait_after_goal_reached_sec_ =
        std::max(goal_reached_hold_sec_, max_wait_after_goal_reached_sec_);
    nav2_cancel_after_tolerance_timeout_sec_ =
        std::max(0.0, nav2_cancel_after_tolerance_timeout_sec_);
    same_goal_distance_m_ = std::max(0.01, same_goal_distance_m_);
    blacklist_radius_ = std::max(0.01, blacklist_radius_);
    blacklist_timeout_sec_ = std::max(1.0, blacklist_timeout_sec_);
    max_consecutive_failures_per_frontier_ =
        std::max(1, max_consecutive_failures_per_frontier_);
    blocked_retry_timeout_sec_ = std::max(1.0, blocked_retry_timeout_sec_);
    nav2_action_wait_timeout_sec_ =
        std::max(0.1, nav2_action_wait_timeout_sec_);
    tf_wait_timeout_sec_ = std::max(0.0, tf_wait_timeout_sec_);
    rescan_delay_sec_ = std::max(0.0, rescan_delay_sec_);
    occupied_threshold_ = std::max(1, occupied_threshold_);
    max_candidate_rejection_logs_per_cycle_ =
        std::max(0, max_candidate_rejection_logs_per_cycle_);
    max_candidate_rejection_logs_per_cluster_ =
        std::max(0, max_candidate_rejection_logs_per_cluster_);
    blocked_no_reachable_backoff_sec_ =
        std::max(1.0, blocked_no_reachable_backoff_sec_);
    frontier_area_memory_radius_m_ =
        std::max(0.01, frontier_area_memory_radius_m_);
    frontier_success_suppress_radius_m_ =
        std::max(0.01, frontier_success_suppress_radius_m_);
    frontier_failure_suppress_radius_m_ =
        std::max(0.01, frontier_failure_suppress_radius_m_);
    frontier_area_memory_timeout_sec_ =
        std::max(1.0, frontier_area_memory_timeout_sec_);
    frontier_success_suppress_timeout_sec_ =
        std::max(1.0, frontier_success_suppress_timeout_sec_);
    frontier_failure_suppress_timeout_sec_ =
        std::max(1.0, frontier_failure_suppress_timeout_sec_);
    frontier_area_size_growth_ratio_ =
        std::max(1.0, frontier_area_size_growth_ratio_);
    frontier_area_size_growth_min_m_ =
        std::max(0.0, frontier_area_size_growth_min_m_);
    frontier_area_anchor_change_radius_m_ =
        std::max(0.01, frontier_area_anchor_change_radius_m_);
    recent_area_penalty_ = std::max(0.0, recent_area_penalty_);
    recent_area_history_size_ = std::max(1, recent_area_history_size_);
    min_goal_clearance_m_ = std::max(0.0, min_goal_clearance_m_);
    frontier_goal_search_radius_m_ =
        std::max(0.0, frontier_goal_search_radius_m_);
    max_frontier_goal_candidates_ =
        std::max(1, max_frontier_goal_candidates_);
    compute_path_timeout_sec_ = std::max(0.1, compute_path_timeout_sec_);
    max_precheck_candidates_per_cycle_ =
        std::max(1, max_precheck_candidates_per_cycle_);
    min_precheck_path_poses_ = std::max(1, min_precheck_path_poses_);
    frontier_stability_required_observations_ =
        std::max(1, frontier_stability_required_observations_);
    frontier_stability_window_ = std::max(1, frontier_stability_window_);
    frontier_stability_required_observations_ =
        std::min(frontier_stability_required_observations_,
                 frontier_stability_window_);
    unstable_frontier_suppress_timeout_sec_ =
        std::max(1.0, unstable_frontier_suppress_timeout_sec_);
    unstable_frontier_area_radius_m_ =
        std::max(0.01, unstable_frontier_area_radius_m_);
    unstable_unknown_ratio_threshold_ =
        std::max(0.0, std::min(1.0, unstable_unknown_ratio_threshold_));
    unstable_occupied_ratio_threshold_ =
        std::max(0.0, std::min(1.0, unstable_occupied_ratio_threshold_));
    no_clear_area_hard_suppress_after_failures_ =
        std::max(1, no_clear_area_hard_suppress_after_failures_);
    no_clear_area_long_timeout_sec_ =
        std::max(frontier_failure_suppress_timeout_sec_,
                 no_clear_area_long_timeout_sec_);
    frontier_area_failure_hard_radius_m_ =
        std::max(frontier_failure_suppress_radius_m_,
                 frontier_area_failure_hard_radius_m_);
    residual_exhaustion_blocked_cycles_ =
        std::max(1, residual_exhaustion_blocked_cycles_);
    residual_exhaustion_min_runtime_sec_ =
        std::max(0.0, residual_exhaustion_min_runtime_sec_);
    residual_exhaustion_no_success_window_sec_ =
        std::max(0.0, residual_exhaustion_no_success_window_sec_);
    successful_area_hard_suppress_after_successes_ =
        std::max(1, successful_area_hard_suppress_after_successes_);
    successful_area_long_timeout_sec_ =
        std::max(frontier_success_suppress_timeout_sec_,
                 successful_area_long_timeout_sec_);
  }

  void onPlannerTimer()
  {
    if (state_ == ExplorerState::PAUSED ||
        state_ == ExplorerState::EXPLORATION_COMPLETE ||
        state_ == ExplorerState::EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (state_ ==
            ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS &&
        now < blocked_retry_at_ && !hasExpiredBlacklist(now) &&
        !hasExpiredFrontierAreaRecord(now)) {
      return;
    }
    if (state_ ==
            ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS &&
        now >= blocked_retry_at_) {
      setState(ExplorerState::SELECTING_FRONTIER,
               "blocked_no_reachable_backoff_elapsed");
    }
    if (state_ ==
            ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS &&
        hasExpiredBlacklist(now)) {
      setState(ExplorerState::SELECTING_FRONTIER,
               "frontier_blacklist_timeout_elapsed");
    }
    if (state_ ==
            ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS &&
        hasExpiredFrontierAreaRecord(now)) {
      setState(ExplorerState::SELECTING_FRONTIER,
               "frontier_area_memory_timeout_elapsed");
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map = latestMap();
    if (!map) {
      setState(ExplorerState::WAITING_FOR_MAP, "no_map_received");
      return;
    }

    const MapMaturity maturity = evaluateMapMaturity(*map);
    if (!maturity.valid || !maturity.size_ok || !maturity.known_ok ||
        !maturity.initial_wait_elapsed) {
      setState(ExplorerState::WAITING_FOR_MAP_MATURITY,
               "map_not_mature");
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "explore_waiting_for_map_maturity valid=%s width=%u height=%u "
          "min_width=%d min_height=%d known_cells=%lu "
          "min_known_cells=%d known_ratio=%.5f min_known_ratio=%.5f "
          "initial_wait_elapsed=%s initial_map_wait_sec=%.3f",
          maturity.valid ? "true" : "false", maturity.width,
          maturity.height, min_map_width_cells_, min_map_height_cells_,
          maturity.known_cell_count, min_known_cell_count_,
          maturity.known_ratio, min_known_cell_ratio_,
          maturity.initial_wait_elapsed ? "true" : "false",
          initial_map_wait_sec_);
      return;
    }

    geometry_msgs::msg::Pose robot_pose;
    if (!lookupRobotPose(robot_pose)) {
      setState(ExplorerState::WAITING_FOR_TF, "tf_unavailable");
      return;
    }

    if (!navClientReady()) {
      setState(ExplorerState::WAITING_FOR_NAV2, "nav2_action_unavailable");
      return;
    }

    if (active_goal_) {
      monitorActiveGoal(robot_pose);
      return;
    }

    if (now < next_rescan_at_) {
      return;
    }

    setState(ExplorerState::SELECTING_FRONTIER, "ready_to_select");
    pruneBlacklist();
    ++frontier_scan_cycle_;

    SelectionResult selection = selectFrontier(*map, robot_pose);
    pruneFrontierAreaRecords();
    pruneFrontierObservations();
    publishFrontierMarkers(selection.frontiers,
                           selection.outcome == SelectionResult::Outcome::SELECTED ?
                               &selection.selected : nullptr);

    if (selection.outcome == SelectionResult::Outcome::COMPLETE) {
      RCLCPP_INFO(this->get_logger(),
                  "exploration_complete reason=no_valid_frontier_groups");
      setState(ExplorerState::EXPLORATION_COMPLETE, "no_frontiers");
      return;
    }

    if (selection.outcome == SelectionResult::Outcome::BLOCKED) {
      ++blocked_no_reachable_cycles_;
      RCLCPP_WARN(
          this->get_logger(),
          "explore_no_valid_frontiers_after_filter frontiers=%lu "
          "too_close=%lu blacklisted=%lu invalid_target=%lu "
          "clearance_rejected=%lu precheck_rejected=%lu "
          "area_suppressed=%lu unstable=%lu hard_suppressed=%lu",
          selection.frontiers.size(), selection.too_close_count,
          selection.blacklisted_count, selection.invalid_target_count,
          selection.clearance_rejected_count,
          selection.precheck_rejected_count, selection.area_suppressed_count,
          selection.unstable_frontier_count,
          selection.hard_suppressed_count);
      if (shouldExhaustResidualFrontiers(selection, now)) {
        RCLCPP_WARN(
            this->get_logger(),
            "exploration_exhausted_residual_frontiers blocked_cycles=%d "
            "frontiers=%lu stable_sendable_frontiers=%lu "
            "unstable_frontiers=%lu no_clear_frontiers=%lu "
            "mostly_unknown_frontiers=%lu mostly_occupied_frontiers=%lu "
            "low_clearance_frontiers=%lu hard_suppressed_frontiers=%lu",
            blocked_no_reachable_cycles_, selection.frontiers.size(),
            selection.stable_sendable_count, selection.unstable_frontier_count,
            selection.no_clear_frontier_count,
            selection.mostly_unknown_frontier_count,
            selection.mostly_occupied_frontier_count,
            selection.low_clearance_frontier_count,
            selection.hard_suppressed_count);
        setState(ExplorerState::EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS,
                 "residual_frontiers_exhausted");
        return;
      }
      RCLCPP_WARN(
          this->get_logger(),
          "exploration_blocked_no_reachable_frontiers frontiers=%lu "
          "blocked_no_reachable_backoff_sec=%.3f",
          selection.frontiers.size(), blocked_no_reachable_backoff_sec_);
      blocked_retry_at_ =
          now + rclcpp::Duration::from_seconds(blocked_no_reachable_backoff_sec_);
      blocked_map_revision_ = latestMapRevision();
      setState(ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS,
               "frontiers_exist_but_none_sendable");
      return;
    }

    blocked_no_reachable_cycles_ = 0;
    sendNavigationGoal(selection.selected, robot_pose);
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr latestMap()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return latest_map_;
  }

  size_t latestMapRevision()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return map_revision_;
  }

  bool frontierSetChangedSignificantly(const SelectionResult& selection)
  {
    bool changed = false;
    if (!blocked_frontier_signature_initialized_) {
      changed = true;
      blocked_frontier_signature_initialized_ = true;
    } else if (selection.frontiers.size() != last_blocked_frontier_count_) {
      changed = true;
    } else if (std::fabs(selection.total_frontier_size_m -
                        last_blocked_total_frontier_size_m_) >
               frontier_area_size_growth_min_m_) {
      changed = true;
    } else if (selection.stable_sendable_count !=
               last_blocked_stable_sendable_count_) {
      changed = true;
    }

    last_blocked_frontier_count_ = selection.frontiers.size();
    last_blocked_total_frontier_size_m_ = selection.total_frontier_size_m;
    last_blocked_stable_sendable_count_ = selection.stable_sendable_count;
    return changed;
  }

  bool shouldExhaustResidualFrontiers(const SelectionResult& selection,
                                      const rclcpp::Time& now)
  {
    const bool frontier_set_changed = frontierSetChangedSignificantly(selection);
    const double runtime_sec = (now - exploration_started_at_).seconds();
    const double no_success_sec =
        (now - last_useful_frontier_success_at_).seconds();
    RCLCPP_WARN(
        this->get_logger(),
        "explore_residual_frontier_summary total_frontiers=%lu "
        "stable_sendable_frontiers=%lu unstable_frontiers=%lu "
        "no_clear_frontiers=%lu mostly_unknown_frontiers=%lu "
        "mostly_occupied_frontiers=%lu low_clearance_frontiers=%lu "
        "hard_suppressed_frontiers=%lu blocked_cycles=%d "
        "runtime_sec=%.3f no_success_sec=%.3f frontier_set_changed=%s",
        selection.frontiers.size(), selection.stable_sendable_count,
        selection.unstable_frontier_count, selection.no_clear_frontier_count,
        selection.mostly_unknown_frontier_count,
        selection.mostly_occupied_frontier_count,
        selection.low_clearance_frontier_count,
        selection.hard_suppressed_count, blocked_no_reachable_cycles_,
        runtime_sec, no_success_sec, frontier_set_changed ? "true" : "false");

    if (!enable_residual_frontier_exhaustion_) {
      return false;
    }
    if (blocked_no_reachable_cycles_ < residual_exhaustion_blocked_cycles_) {
      return false;
    }
    if (runtime_sec < residual_exhaustion_min_runtime_sec_) {
      return false;
    }
    if (no_success_sec < residual_exhaustion_no_success_window_sec_) {
      return false;
    }
    if (residual_exhaustion_require_no_stable_sendable_frontiers_ &&
        selection.stable_sendable_count > 0) {
      return false;
    }
    if (frontier_set_changed) {
      return false;
    }
    return selection.frontiers.size() > 0;
  }

  MapMaturity evaluateMapMaturity(const nav_msgs::msg::OccupancyGrid& map)
  {
    MapMaturity maturity;
    maturity.width = map.info.width;
    maturity.height = map.info.height;
    maturity.valid = map.info.width > 0 && map.info.height > 0 &&
        map.info.resolution > 0.0 &&
        map.data.size() >=
            static_cast<size_t>(map.info.width) *
                static_cast<size_t>(map.info.height);
    if (!maturity.valid) {
      first_meaningful_map_seen_ = false;
      return maturity;
    }

    maturity.size_ok =
        static_cast<int>(map.info.width) >= min_map_width_cells_ &&
        static_cast<int>(map.info.height) >= min_map_height_cells_;

    const size_t total_cells =
        static_cast<size_t>(map.info.width) * static_cast<size_t>(map.info.height);
    for (size_t i = 0; i < total_cells; ++i) {
      if (map.data[i] != kUnknownCell) {
        ++maturity.known_cell_count;
      }
    }
    maturity.known_ratio = total_cells == 0 ?
        0.0 : static_cast<double>(maturity.known_cell_count) /
            static_cast<double>(total_cells);
    const bool count_gate =
        min_known_cell_count_ > 0 &&
        maturity.known_cell_count >= static_cast<size_t>(min_known_cell_count_);
    const bool ratio_gate =
        min_known_cell_ratio_ > 0.0 &&
        maturity.known_ratio >= min_known_cell_ratio_;
    maturity.known_ok =
        (min_known_cell_count_ <= 0 && min_known_cell_ratio_ <= 0.0) ||
        count_gate || ratio_gate;

    if (maturity.size_ok && maturity.known_ok) {
      if (!first_meaningful_map_seen_) {
        first_meaningful_map_seen_ = true;
        first_meaningful_map_time_ = this->now();
        RCLCPP_INFO(
            this->get_logger(),
            "explore_first_meaningful_map width=%u height=%u "
            "known_cells=%lu known_ratio=%.5f initial_map_wait_sec=%.3f",
            maturity.width, maturity.height, maturity.known_cell_count,
            maturity.known_ratio, initial_map_wait_sec_);
      }
      maturity.initial_wait_elapsed =
          (this->now() - first_meaningful_map_time_).seconds() >=
          initial_map_wait_sec_;
    } else {
      first_meaningful_map_seen_ = false;
    }
    return maturity;
  }

  bool lookupRobotPose(geometry_msgs::msg::Pose& pose)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
          global_frame_, robot_base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(tf_wait_timeout_sec_));
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "explore_waiting_for_tf global_frame=%s robot_base_frame=%s "
          "error='%s'",
          global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool navClientReady()
  {
    if (nav_client_->action_server_is_ready()) {
      if (!enable_compute_path_precheck_ ||
          compute_path_client_->action_server_is_ready()) {
        nav2_wait_started_ = false;
        return true;
      }
    }

    if (!nav2_wait_started_) {
      nav2_wait_started_ = true;
      nav2_wait_started_at_ = this->now();
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "explore_waiting_for_nav2 action=%s compute_path_action=%s "
        "navigate_ready=%s compute_path_ready=%s elapsed_sec=%.3f "
        "warn_timeout_sec=%.3f",
        navigate_action_name_.c_str(), compute_path_action_name_.c_str(),
        nav_client_->action_server_is_ready() ? "true" : "false",
        (!enable_compute_path_precheck_ ||
         compute_path_client_->action_server_is_ready()) ? "true" : "false",
        (this->now() - nav2_wait_started_at_).seconds(),
        nav2_action_wait_timeout_sec_);
    return false;
  }

  void onActiveGoalMonitorTimer()
  {
    if (!active_goal_ || state_ == ExplorerState::PAUSED) {
      return;
    }

    geometry_msgs::msg::Pose robot_pose;
    if (!lookupRobotPose(robot_pose)) {
      return;
    }
    monitorActiveGoal(robot_pose);
  }

  void monitorActiveGoal(const geometry_msgs::msg::Pose& robot_pose)
  {
    const double distance_m =
        distance2d(robot_pose.position, active_goal_target_);
    if (distance_m < active_goal_best_distance_m_ - kProgressEpsilonM) {
      active_goal_best_distance_m_ = distance_m;
      active_goal_last_progress_at_ = this->now();
    }

    if (distance_m <= goal_reached_distance_m_) {
      if (!enable_explorer_tolerance_completion_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "explore_goal_reached_distance_waiting_for_nav2_result target=%s "
            "distance_m=%.3f goal_reached_distance_m=%.3f",
            formatPoint(active_goal_target_).c_str(), distance_m,
            goal_reached_distance_m_);
        return;
      }

      const rclcpp::Time now = this->now();
      if (!active_goal_tolerance_started_) {
        active_goal_tolerance_started_ = true;
        active_goal_tolerance_first_at_ = now;
        active_goal_tolerance_confirm_count_ = 0;
      }
      active_goal_tolerance_last_at_ = now;
      ++active_goal_tolerance_confirm_count_;
      const double hold_sec =
          (now - active_goal_tolerance_first_at_).seconds();
      const bool confirmed =
          active_goal_tolerance_confirm_count_ >= goal_reached_confirm_count_ &&
          hold_sec >= goal_reached_hold_sec_;
      const bool max_wait_elapsed =
          hold_sec >= max_wait_after_goal_reached_sec_;
      const bool cancel_timeout_elapsed =
          hold_sec >= nav2_cancel_after_tolerance_timeout_sec_;
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "explore_goal_reached_tolerance_confirming target=%s "
          "distance_m=%.3f goal_reached_distance_m=%.3f "
          "confirm_count=%d required_confirm_count=%d hold_sec=%.3f "
          "required_hold_sec=%.3f max_wait_after_goal_reached_sec=%.3f "
          "nav2_cancel_after_tolerance_timeout_sec=%.3f",
          formatPoint(active_goal_target_).c_str(), distance_m,
          goal_reached_distance_m_, active_goal_tolerance_confirm_count_,
          goal_reached_confirm_count_, hold_sec, goal_reached_hold_sec_,
          max_wait_after_goal_reached_sec_,
          nav2_cancel_after_tolerance_timeout_sec_);
      if (confirmed || max_wait_elapsed || cancel_timeout_elapsed) {
        const std::string reason = confirmed ? "confirmed" :
            (max_wait_elapsed ? "max_wait_elapsed" :
             "cancel_timeout_elapsed");
        acceptExplorerToleranceSuccess(distance_m, reason);
      }
      return;
    }

    resetActiveGoalToleranceProgress();

    const double progress_age_sec =
        (this->now() - active_goal_last_progress_at_).seconds();
    if (progress_age_sec < progress_timeout_) {
      return;
    }

    RCLCPP_WARN(
        this->get_logger(),
        "explore_nav2_goal_progress_timeout target=%s distance_m=%.3f "
        "best_distance_m=%.3f progress_age_sec=%.3f "
        "progress_timeout=%.3f",
        formatPoint(active_goal_target_).c_str(), distance_m,
        active_goal_best_distance_m_, progress_age_sec, progress_timeout_);

    if (active_goal_handle_) {
      nav_client_->async_cancel_goal(active_goal_handle_);
    }
    if (active_goal_frontier_valid_) {
      suppressFrontierArea(active_goal_frontier_, "progress_timeout");
    } else {
      addOrUpdateBlacklist(active_goal_target_, "progress_timeout");
    }
    clearActiveGoal("progress_timeout");
    next_rescan_at_ = this->now() + rclcpp::Duration::from_seconds(rescan_delay_sec_);
    setState(ExplorerState::RECOVERING_AFTER_FAILURE, "progress_timeout");
  }

  void resetActiveGoalToleranceProgress()
  {
    active_goal_tolerance_started_ = false;
    active_goal_tolerance_confirm_count_ = 0;
  }

  void recordExplorerToleranceCompletedSequence(uint64_t sequence)
  {
    std::lock_guard<std::mutex> lock(completed_goal_mutex_);
    explorer_tolerance_completed_sequences_.push_back(sequence);
    while (explorer_tolerance_completed_sequences_.size() > 32U) {
      explorer_tolerance_completed_sequences_.erase(
          explorer_tolerance_completed_sequences_.begin());
    }
  }

  bool wasCompletedByExplorerTolerance(uint64_t sequence) const
  {
    std::lock_guard<std::mutex> lock(completed_goal_mutex_);
    return std::find(explorer_tolerance_completed_sequences_.begin(),
                     explorer_tolerance_completed_sequences_.end(),
                     sequence) != explorer_tolerance_completed_sequences_.end();
  }

  void requestActiveNav2Cancel(uint64_t sequence,
                               const geometry_msgs::msg::Point& target,
                               const std::string& reason)
  {
    if (!active_goal_handle_) {
      RCLCPP_INFO(
          this->get_logger(),
          "explore_nav2_cancel_requested reason=%s target=%s sequence=%lu "
          "status=skipped_no_goal_handle",
          reason.c_str(), formatPoint(target).c_str(), sequence);
      return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "explore_nav2_cancel_requested reason=%s target=%s sequence=%lu",
        reason.c_str(), formatPoint(target).c_str(), sequence);
    try {
      nav_client_->async_cancel_goal(
          active_goal_handle_,
          [this, sequence, target, reason](auto response) {
            if (!response) {
              RCLCPP_WARN(
                  this->get_logger(),
                  "explore_nav2_cancel_result reason=%s target=%s "
                  "sequence=%lu status=missing_response",
                  reason.c_str(), formatPoint(target).c_str(), sequence);
              return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "explore_nav2_cancel_result reason=%s target=%s sequence=%lu "
                "return_code=%u goals_canceling=%lu",
                reason.c_str(), formatPoint(target).c_str(), sequence,
                static_cast<unsigned int>(response->return_code),
                response->goals_canceling.size());
          });
    } catch (const std::exception& ex) {
      RCLCPP_WARN(
          this->get_logger(),
          "explore_nav2_cancel_result reason=%s target=%s sequence=%lu "
          "status=exception error='%s'",
          reason.c_str(), formatPoint(target).c_str(), sequence, ex.what());
    }
  }

  void acceptExplorerToleranceSuccess(double distance_m,
                                      const std::string& accept_reason)
  {
    if (!active_goal_) {
      return;
    }

    const uint64_t sequence = active_goal_sequence_;
    const geometry_msgs::msg::Point target = active_goal_target_;
    const FrontierGroup frontier = active_goal_frontier_;
    const bool frontier_valid = active_goal_frontier_valid_;
    recordExplorerToleranceCompletedSequence(sequence);

    int area_id = -1;
    if (frontier_valid) {
      area_id = suppressFrontierArea(frontier, "succeeded");
    }
    last_useful_frontier_success_at_ = this->now();
    blocked_no_reachable_cycles_ = 0;
    blocked_frontier_signature_initialized_ = false;

    RCLCPP_INFO(
        this->get_logger(),
        "explore_goal_reached_tolerance_accept target=%s distance_m=%.3f "
        "goal_reached_distance_m=%.3f accept_reason=%s sequence=%lu "
        "area_id=%d",
        formatPoint(target).c_str(), distance_m, goal_reached_distance_m_,
        accept_reason.c_str(), sequence, area_id);

    if (cancel_nav2_on_explorer_tolerance_success_) {
      requestActiveNav2Cancel(sequence, target, "explorer_tolerance_success");
    }

    clearActiveGoal("explorer_tolerance_success");
    next_rescan_at_ = this->now();
    setState(ExplorerState::SELECTING_FRONTIER,
             "explorer_tolerance_success");
  }

  SelectionResult selectFrontier(const nav_msgs::msg::OccupancyGrid& map,
                                 const geometry_msgs::msg::Pose& robot_pose)
  {
    SelectionResult result;
    candidate_rejection_logs_this_cycle_ = 0;
    result.frontiers = extractFrontiers(map, robot_pose);
    if (result.frontiers.empty()) {
      result.outcome = SelectionResult::Outcome::COMPLETE;
      return result;
    }
    for (const auto& frontier : result.frontiers) {
      result.total_frontier_size_m += frontier.size_m;
      if (frontier.stability_confirmed && !frontier.candidates.empty() &&
          !frontier.cluster_blacklisted) {
        ++result.stable_sendable_count;
      }
      if (frontier.unstable_suppressed ||
          frontier.area_memory_reason == "unstable_frontier") {
        ++result.unstable_frontier_count;
      }
      if (frontier.area_memory_reason == "no_clear_free_goal" ||
          frontier.residual_classification == "no_clear_free_goal") {
        ++result.no_clear_frontier_count;
      }
      if (frontier.residual_classification == "mostly_unknown") {
        ++result.mostly_unknown_frontier_count;
      }
      if (frontier.residual_classification == "mostly_occupied") {
        ++result.mostly_occupied_frontier_count;
      }
      if (frontier.residual_classification == "low_clearance") {
        ++result.low_clearance_frontier_count;
      }
      if (frontier.blacklist_reason == "hard_suppressed" ||
          frontier.area_memory_reason.find("hard") != std::string::npos) {
        ++result.hard_suppressed_count;
      }
    }

    std::sort(result.frontiers.begin(), result.frontiers.end(),
              [](const FrontierGroup& a, const FrontierGroup& b) {
                return a.score < b.score;
              });

    int precheck_count = 0;
    for (const auto& frontier : result.frontiers) {
      if (frontier.cluster_blacklisted) {
        ++result.blacklisted_count;
        if (frontier.area_id >= 0 ||
            frontier.candidate_stats.suppressed_by_area_memory > 0) {
          ++result.area_suppressed_count;
        }
        continue;
      }

      if (frontier.candidates.empty()) {
        ++result.invalid_target_count;
        ++result.clearance_rejected_count;
        if (!(frontier.area_id >= 0 &&
              frontier.area_memory_reason == "no_clear_free_goal")) {
          suppressFrontierArea(frontier, "no_clear_free_goal");
        }
        continue;
      }

      for (const auto& candidate : frontier.candidates) {
        FrontierGroup candidate_frontier = frontier;
        applyCandidate(candidate_frontier, candidate);

        if (candidate_frontier.distance_m < min_goal_distance_m_) {
          ++result.too_close_count;
          RCLCPP_INFO(
              this->get_logger(),
              "explore_frontier_too_close_deferred target=%s distance_m=%.3f "
              "min_goal_distance_m=%.3f size_m=%.3f",
              formatPoint(candidate_frontier.target).c_str(),
              candidate_frontier.distance_m, min_goal_distance_m_,
              candidate_frontier.size_m);
          suppressFrontierArea(candidate_frontier, "too_close");
          continue;
        }

        if (!enable_frontier_area_memory_ &&
            isBlacklisted(candidate_frontier.target)) {
          ++result.blacklisted_count;
          std::string reason;
          isBlacklisted(candidate_frontier.target, &reason);
          RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "explore_frontier_blacklist_skip reason=%s target=%s "
              "radius_m=%.3f",
              reason.c_str(), formatPoint(candidate_frontier.target).c_str(),
              blacklist_radius_);
          continue;
        }

        if (enable_compute_path_precheck_) {
          if (precheck_count >= max_precheck_candidates_per_cycle_) {
            RCLCPP_INFO(
                this->get_logger(),
                "explore_compute_path_precheck_limit_reached checked=%d "
                "max_precheck_candidates_per_cycle=%d",
                precheck_count, max_precheck_candidates_per_cycle_);
            result.outcome = SelectionResult::Outcome::BLOCKED;
            return result;
          }
          ++precheck_count;
          const auto precheck = computePathPrecheck(candidate_frontier);
          if (!precheck.reachable) {
            ++result.precheck_rejected_count;
            RCLCPP_WARN(
                this->get_logger(),
                "explore_frontier_rejected_compute_path target=%s "
                "error_code=%u reason=%s path_poses=%lu",
                formatPoint(candidate_frontier.target).c_str(),
                static_cast<unsigned int>(precheck.error_code),
                precheck.reason.c_str(), precheck.path_poses);
            suppressFrontierArea(candidate_frontier,
                                 "compute_path_" + precheck.reason);
            continue;
          }
        }

        const int area_id =
            suppressFrontierArea(candidate_frontier, "selected_recently");
        candidate_frontier.area_id = area_id;
        candidate_frontier.area_memory_reason = "selected_recently";
        result.selected = candidate_frontier;
        result.outcome = SelectionResult::Outcome::SELECTED;
        markFrontierStableByPrecheck(candidate_frontier);
        RCLCPP_INFO(
            this->get_logger(),
            "explore_frontier_selected target=%s centroid=%s distance_m=%.3f "
            "clearance_m=%.3f size_m=%.3f score=%.6f area_id=%d "
            "area_anchor=%s area_reason=%s area_penalty=%.3f",
            formatPoint(candidate_frontier.target).c_str(),
            formatPoint(candidate_frontier.centroid).c_str(),
            candidate_frontier.distance_m, candidate_frontier.target_clearance_m,
            candidate_frontier.size_m, candidate_frontier.score,
            candidate_frontier.area_id,
            formatPoint(candidate_frontier.blacklist_anchor).c_str(),
            candidate_frontier.area_memory_reason.c_str(),
            candidate_frontier.area_memory_penalty);
        return result;
      }
    }

    result.outcome = SelectionResult::Outcome::BLOCKED;
    return result;
  }

  std::vector<FrontierGroup> extractFrontiers(
      const nav_msgs::msg::OccupancyGrid& map,
      const geometry_msgs::msg::Pose& robot_pose)
  {
    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    const size_t total_cells = static_cast<size_t>(width) * height;
    std::vector<bool> frontier_flag(total_cells, false);
    std::vector<FrontierGroup> frontiers;

    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const size_t idx = index(x, y, width);
        if (frontier_flag[idx] || !isFrontierCell(map, x, y)) {
          continue;
        }

        FrontierGroup frontier =
            buildFrontierGroup(map, x, y, frontier_flag, robot_pose);
        if (frontier.size_m >= min_frontier_size_) {
          frontiers.push_back(frontier);
        }
      }
    }
    return frontiers;
  }

  bool isFailureSuppressionReason(const std::string& reason) const
  {
    return reason == "no_clear_free_goal" || reason.find("compute_path") == 0 ||
        reason == "nav2_failed" || reason == "nav2_canceled" ||
        reason == "goal_rejected" || reason == "progress_timeout" ||
        reason == "unstable_frontier" || reason == "too_close";
  }

  double areaRadiusForReason(const std::string& reason) const
  {
    if (reason == "unstable_frontier") {
      return unstable_frontier_area_radius_m_;
    }
    if (reason == "succeeded" || reason == "selected_recently") {
      return frontier_success_suppress_radius_m_;
    }
    if (isFailureSuppressionReason(reason)) {
      return frontier_failure_suppress_radius_m_;
    }
    return frontier_area_memory_radius_m_;
  }

  double areaTimeoutForReason(const std::string& reason) const
  {
    if (reason == "unstable_frontier") {
      return unstable_frontier_suppress_timeout_sec_;
    }
    if (reason == "succeeded" || reason == "selected_recently") {
      return frontier_success_suppress_timeout_sec_;
    }
    if (isFailureSuppressionReason(reason)) {
      return frontier_failure_suppress_timeout_sec_;
    }
    return frontier_area_memory_timeout_sec_;
  }

  bool recordMatchesFrontier(const FrontierAreaRecord& record,
                             const FrontierGroup& frontier) const
  {
    if (distance2d(record.anchor, frontier.blacklist_anchor) <= record.radius_m ||
        distance2d(record.last_centroid, frontier.centroid) <= record.radius_m ||
        distance2d(record.last_boundary_anchor, frontier.blacklist_anchor) <=
            record.radius_m) {
      return true;
    }
    if (frontier.target_valid &&
        distance2d(record.last_target, frontier.target) <= record.radius_m) {
      return true;
    }
    return false;
  }

  bool frontierChangedEnoughForReuse(const FrontierAreaRecord& record,
                                     const FrontierGroup& frontier,
                                     std::string& reason) const
  {
    const rclcpp::Time now = this->now();
    if (now >= record.expires_at) {
      reason = "timeout_expired";
      return true;
    }
    if (distance2d(record.last_centroid, frontier.centroid) > record.radius_m) {
      reason = "centroid_moved";
      return true;
    }
    if (record.last_size_m > 0.0 &&
        (frontier.size_m > record.last_size_m * frontier_area_size_growth_ratio_ ||
         frontier.size_m - record.last_size_m > frontier_area_size_growth_min_m_)) {
      reason = "frontier_size_increased";
      return true;
    }
    if (distance2d(record.last_boundary_anchor, frontier.blacklist_anchor) >
        frontier_area_anchor_change_radius_m_) {
      reason = "boundary_anchor_changed";
      return true;
    }
    reason = "unchanged_area";
    return false;
  }

  bool reasonRequiresClearCandidateForReuse(
      const FrontierAreaRecord& record) const
  {
    if (record.reason == "no_clear_free_goal" ||
        record.reason == "unstable_frontier" ||
        record.reason == "too_close") {
      return no_clear_area_reuse_requires_clear_candidate_;
    }
    if (isFailureSuppressionReason(record.reason)) {
      return failed_area_reuse_requires_compute_path_success_;
    }
    if (record.reason == "succeeded" &&
        record.success_count >= successful_area_hard_suppress_after_successes_) {
      return successful_area_reuse_requires_frontier_growth_;
    }
    return false;
  }

  bool isHardSuppressedRecord(const FrontierAreaRecord& record) const
  {
    if ((record.reason == "no_clear_free_goal" ||
         record.reason == "unstable_frontier" ||
         isFailureSuppressionReason(record.reason)) &&
        record.failure_count >= no_clear_area_hard_suppress_after_failures_) {
      return true;
    }
    if (record.reason == "succeeded" &&
        record.success_count >= successful_area_hard_suppress_after_successes_) {
      return true;
    }
    return false;
  }

  bool cheapGoalCellClear(const nav_msgs::msg::OccupancyGrid& map,
                          uint32_t mx, uint32_t my) const
  {
    if (!inBounds(static_cast<int>(mx), static_cast<int>(my),
                  map.info.width, map.info.height)) {
      return false;
    }
    const int8_t value = map.data[index(mx, my, map.info.width)];
    if (value == kUnknownCell && reject_unknown_goal_cells_) {
      return false;
    }
    if (value >= occupied_threshold_ && reject_occupied_goal_cells_) {
      return false;
    }
    if (!isFree(value)) {
      return false;
    }
    return !enable_goal_clearance_filter_ ||
        nearestOccupiedDistance(map, mx, my) >= min_goal_clearance_m_;
  }

  bool hasClearCandidateEvidence(
      const nav_msgs::msg::OccupancyGrid& map,
      const FrontierGroup& frontier,
      const std::vector<size_t>& free_boundary_cells) const
  {
    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    const double resolution = map.info.resolution;
    const int radius_cells =
        static_cast<int>(std::ceil(frontier_goal_search_radius_m_ / resolution));
    const size_t total_cells =
        static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<bool> seen(total_cells, false);

    auto test_cell = [&](int mx, int my) -> bool {
      if (!inBounds(mx, my, width, height)) {
        return false;
      }
      const size_t idx = index(mx, my, width);
      if (seen[idx]) {
        return false;
      }
      seen[idx] = true;
      return cheapGoalCellClear(map, static_cast<uint32_t>(mx),
                                static_cast<uint32_t>(my));
    };

    std::vector<size_t> sorted_boundary_cells = free_boundary_cells;
    std::sort(sorted_boundary_cells.begin(), sorted_boundary_cells.end(),
              [this, &map, &frontier](size_t a, size_t b) {
                uint32_t ax = 0;
                uint32_t ay = 0;
                uint32_t bx = 0;
                uint32_t by = 0;
                cellsFromIndex(a, map.info.width, ax, ay);
                cellsFromIndex(b, map.info.width, bx, by);
                return distance2d(cellToWorld(map, ax, ay),
                                  frontier.centroid) <
                    distance2d(cellToWorld(map, bx, by), frontier.centroid);
              });

    size_t checked = 0;
    const size_t max_checks =
        std::max(static_cast<size_t>(max_frontier_goal_candidates_),
                 static_cast<size_t>(max_frontier_goal_candidates_) * 4U);
    for (const size_t seed_idx : sorted_boundary_cells) {
      uint32_t sx = 0;
      uint32_t sy = 0;
      cellsFromIndex(seed_idx, width, sx, sy);
      if (test_cell(static_cast<int>(sx), static_cast<int>(sy))) {
        return true;
      }
      if (++checked >= max_checks) {
        return false;
      }
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          if (dx * dx + dy * dy > radius_cells * radius_cells) {
            continue;
          }
          if (test_cell(static_cast<int>(sx) + dx,
                        static_cast<int>(sy) + dy)) {
            return true;
          }
          if (++checked >= max_checks) {
            return false;
          }
        }
      }
    }
    return false;
  }

  FrontierAreaRecord* matchingAreaRecord(
      FrontierGroup& frontier,
      const nav_msgs::msg::OccupancyGrid& map,
      const std::vector<size_t>& free_boundary_cells,
      std::string& reuse_reason)
  {
    if (!enable_frontier_area_memory_) {
      return nullptr;
    }
    for (auto& record : frontier_area_records_) {
      if (!recordMatchesFrontier(record, frontier)) {
        continue;
      }
      frontier.area_id = record.area_id;
      frontier.area_memory_reason = record.reason;
      if (record.reason == "succeeded" && !enable_success_area_suppression_) {
        return nullptr;
      }
      if (frontierChangedEnoughForReuse(record, frontier, reuse_reason)) {
        const bool clear_candidate_found =
            hasClearCandidateEvidence(map, frontier, free_boundary_cells);
        if (reasonRequiresClearCandidateForReuse(record) &&
            !clear_candidate_found &&
            reuse_reason != "timeout_expired") {
          record.last_seen = this->now();
          frontier.blacklist_reason = isHardSuppressedRecord(record) ?
              "hard_suppressed" : record.reason;
          RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "explore_frontier_area_reuse_blocked "
              "reason=no_clear_requires_clear_candidate area_id=%d "
              "suppression_reason=%s centroid=%s anchor=%s "
              "record_anchor=%s reuse_reason=%s failure_count=%d "
              "success_count=%d",
              record.area_id, record.reason.c_str(),
              formatPoint(frontier.centroid).c_str(),
              formatPoint(frontier.blacklist_anchor).c_str(),
              formatPoint(record.anchor).c_str(), reuse_reason.c_str(),
              record.failure_count, record.success_count);
          if (isHardSuppressedRecord(record)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "explore_frontier_area_hard_suppressed area_id=%d reason=%s "
                "anchor=%s radius_m=%.3f failure_count=%d success_count=%d",
                record.area_id, record.reason.c_str(),
                formatPoint(record.anchor).c_str(), record.radius_m,
                record.failure_count, record.success_count);
          }
          return &record;
        }
        if (reuse_reason == "timeout_expired") {
          RCLCPP_INFO(
              this->get_logger(),
              "explore_frontier_area_reused_after_timeout area_id=%d "
              "reason=%s centroid=%s anchor=%s record_anchor=%s "
              "radius_m=%.3f size_m=%.3f last_size_m=%.3f",
              record.area_id, record.reason.c_str(),
              formatPoint(frontier.centroid).c_str(),
              formatPoint(frontier.blacklist_anchor).c_str(),
              formatPoint(record.anchor).c_str(), record.radius_m,
              frontier.size_m, record.last_size_m);
          RCLCPP_INFO(
              this->get_logger(),
              "explore_frontier_area_reuse_allowed reason=timeout_expired "
              "area_id=%d suppression_reason=%s centroid=%s anchor=%s",
              record.area_id, record.reason.c_str(),
              formatPoint(frontier.centroid).c_str(),
              formatPoint(frontier.blacklist_anchor).c_str());
        } else {
          RCLCPP_INFO(
              this->get_logger(),
              "explore_frontier_area_reused_after_map_change area_id=%d "
              "reuse_reason=%s suppression_reason=%s centroid=%s anchor=%s "
              "record_anchor=%s radius_m=%.3f size_m=%.3f last_size_m=%.3f",
              record.area_id, reuse_reason.c_str(), record.reason.c_str(),
              formatPoint(frontier.centroid).c_str(),
              formatPoint(frontier.blacklist_anchor).c_str(),
              formatPoint(record.anchor).c_str(), record.radius_m,
              frontier.size_m, record.last_size_m);
          if (clear_candidate_found &&
              reasonRequiresClearCandidateForReuse(record)) {
            RCLCPP_INFO(
                this->get_logger(),
                "explore_frontier_area_reuse_allowed "
                "reason=clear_candidate_found area_id=%d suppression_reason=%s "
                "centroid=%s anchor=%s",
                record.area_id, record.reason.c_str(),
                formatPoint(frontier.centroid).c_str(),
                formatPoint(frontier.blacklist_anchor).c_str());
          }
        }
        return nullptr;
      }
      record.last_seen = this->now();
      frontier.blacklist_reason = isHardSuppressedRecord(record) ?
          "hard_suppressed" : record.reason;
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "explore_frontier_area_skip area_id=%d reason=%s centroid=%s "
          "anchor=%s record_anchor=%s radius_m=%.3f size_m=%.3f "
          "last_size_m=%.3f",
          record.area_id, record.reason.c_str(),
          formatPoint(frontier.centroid).c_str(),
          formatPoint(frontier.blacklist_anchor).c_str(),
          formatPoint(record.anchor).c_str(), record.radius_m,
          frontier.size_m, record.last_size_m);
      if (record.reason == "unstable_frontier") {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "explore_frontier_stability_skip area_id=%d centroid=%s "
            "anchor=%s observations=%lu required_observations=%d",
            record.area_id, formatPoint(frontier.centroid).c_str(),
            formatPoint(frontier.blacklist_anchor).c_str(),
            frontier.stability_observations,
            frontier_stability_required_observations_);
      }
      return &record;
    }
    return nullptr;
  }

  size_t recordFrontierObservation(const FrontierGroup& frontier)
  {
    if (!enable_frontier_stability_filter_) {
      return static_cast<size_t>(frontier_stability_required_observations_);
    }
    FrontierObservationRecord* match = nullptr;
    for (auto& record : frontier_observations_) {
      if (distance2d(record.centroid, frontier.centroid) <=
              unstable_frontier_area_radius_m_ ||
          distance2d(record.anchor, frontier.blacklist_anchor) <=
              unstable_frontier_area_radius_m_ ||
          distance2d(record.boundary_anchor, frontier.blacklist_anchor) <=
              unstable_frontier_area_radius_m_) {
        match = &record;
        break;
      }
    }
    if (match == nullptr) {
      frontier_observations_.push_back(FrontierObservationRecord());
      match = &frontier_observations_.back();
    }
    match->centroid = frontier.centroid;
    match->anchor = frontier.blacklist_anchor;
    match->boundary_anchor = frontier.blacklist_anchor;
    match->size_m = frontier.size_m;
    if (match->observed_cycles.empty() ||
        match->observed_cycles.back() != frontier_scan_cycle_) {
      match->observed_cycles.push_back(frontier_scan_cycle_);
    }
    const size_t min_cycle = frontier_scan_cycle_ >
            static_cast<size_t>(frontier_stability_window_) ?
        frontier_scan_cycle_ - static_cast<size_t>(frontier_stability_window_) + 1U :
        0U;
    while (!match->observed_cycles.empty() &&
           match->observed_cycles.front() < min_cycle) {
      match->observed_cycles.pop_front();
    }
    return match->observed_cycles.size();
  }

  void markFrontierStableByPrecheck(const FrontierGroup& frontier)
  {
    if (!enable_frontier_stability_filter_) {
      return;
    }
    for (auto& record : frontier_observations_) {
      if (distance2d(record.centroid, frontier.centroid) <=
              unstable_frontier_area_radius_m_ ||
          distance2d(record.anchor, frontier.blacklist_anchor) <=
              unstable_frontier_area_radius_m_) {
        while (record.observed_cycles.size() <
               static_cast<size_t>(frontier_stability_required_observations_)) {
          record.observed_cycles.push_back(frontier_scan_cycle_);
        }
        RCLCPP_INFO(
            this->get_logger(),
            "explore_frontier_stable_confirmed centroid=%s anchor=%s "
            "reason=compute_path_success observations=%lu",
            formatPoint(frontier.centroid).c_str(),
            formatPoint(frontier.blacklist_anchor).c_str(),
            record.observed_cycles.size());
        return;
      }
    }
  }

  void pruneFrontierObservations()
  {
    if (!enable_frontier_stability_filter_) {
      return;
    }
    const size_t min_cycle = frontier_scan_cycle_ >
            static_cast<size_t>(frontier_stability_window_) ?
        frontier_scan_cycle_ - static_cast<size_t>(frontier_stability_window_) + 1U :
        0U;
    std::vector<FrontierObservationRecord> retained;
    retained.reserve(frontier_observations_.size());
    for (auto record : frontier_observations_) {
      while (!record.observed_cycles.empty() &&
             record.observed_cycles.front() < min_cycle) {
        record.observed_cycles.pop_front();
      }
      if (!record.observed_cycles.empty()) {
        retained.push_back(record);
      }
    }
    frontier_observations_.swap(retained);
  }

  std::string classifyResidualFrontier(const FrontierGroup& frontier) const
  {
    const auto& stats = frontier.candidate_stats;
    if (stats.unique_candidates == 0) {
      return "no_clear_free_goal";
    }
    const double unknown_ratio =
        static_cast<double>(stats.rejected_unknown) /
        static_cast<double>(stats.unique_candidates);
    const double occupied_ratio =
        static_cast<double>(stats.rejected_occupied) /
        static_cast<double>(stats.unique_candidates);
    const double low_clearance_ratio =
        static_cast<double>(stats.rejected_low_clearance) /
        static_cast<double>(stats.unique_candidates);
    if (unknown_ratio >= unstable_unknown_ratio_threshold_) {
      return "mostly_unknown";
    }
    if (occupied_ratio >= unstable_occupied_ratio_threshold_) {
      return "mostly_occupied";
    }
    if (low_clearance_ratio >= unstable_occupied_ratio_threshold_) {
      return "low_clearance";
    }
    return "no_clear_free_goal";
  }

  bool isNoisyResidualFrontier(const FrontierGroup& frontier) const
  {
    return frontier.residual_classification == "mostly_unknown" ||
        frontier.residual_classification == "mostly_occupied" ||
        frontier.residual_classification == "low_clearance" ||
        frontier.residual_classification == "no_clear_free_goal";
  }

  void applyDiversityPenalty(FrontierGroup& frontier)
  {
    if (!enable_frontier_area_memory_ ||
        !enable_frontier_diversity_penalty_) {
      return;
    }

    size_t count = 0;
    for (auto it = recent_area_history_.rbegin();
         it != recent_area_history_.rend() &&
         count < static_cast<size_t>(recent_area_history_size_);
         ++it, ++count) {
      if (distance2d(frontier.blacklist_anchor, *it) <=
          frontier_area_memory_radius_m_) {
        frontier.area_memory_penalty += recent_area_penalty_;
      }
    }
    frontier.score += frontier.area_memory_penalty;
  }

  FrontierGroup buildFrontierGroup(
      const nav_msgs::msg::OccupancyGrid& map, uint32_t start_x,
      uint32_t start_y, std::vector<bool>& frontier_flag,
      const geometry_msgs::msg::Pose& robot_pose)
  {
    FrontierGroup frontier;
    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    std::queue<std::pair<uint32_t, uint32_t> > queue;
    std::vector<size_t> free_boundary_cells;
    std::vector<bool> free_boundary_flag(
        static_cast<size_t>(width) * static_cast<size_t>(height), false);

    frontier_flag[index(start_x, start_y, width)] = true;
    queue.push(std::make_pair(start_x, start_y));

    while (!queue.empty()) {
      const std::pair<uint32_t, uint32_t> cell = queue.front();
      queue.pop();
      const uint32_t x = cell.first;
      const uint32_t y = cell.second;

      frontier.points.push_back(cellToWorld(map, x, y));

      const int dx4[4] = {1, -1, 0, 0};
      const int dy4[4] = {0, 0, 1, -1};
      for (int i = 0; i < 4; ++i) {
        const int nx = static_cast<int>(x) + dx4[i];
        const int ny = static_cast<int>(y) + dy4[i];
        if (!inBounds(nx, ny, width, height) ||
            !isFree(map.data[index(nx, ny, width)])) {
          continue;
        }
        const size_t free_idx = index(nx, ny, width);
        if (!free_boundary_flag[free_idx]) {
          free_boundary_flag[free_idx] = true;
          free_boundary_cells.push_back(free_idx);
        }
      }

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (!inBounds(nx, ny, width, height)) {
            continue;
          }
          const size_t nidx = index(nx, ny, width);
          if (frontier_flag[nidx] || !isFrontierCell(map, nx, ny)) {
            continue;
          }
          frontier_flag[nidx] = true;
          queue.push(std::make_pair(static_cast<uint32_t>(nx),
                                    static_cast<uint32_t>(ny)));
        }
      }
    }

    for (const auto& point : frontier.points) {
      frontier.centroid.x += point.x;
      frontier.centroid.y += point.y;
    }
    if (!frontier.points.empty()) {
      frontier.centroid.x /= static_cast<double>(frontier.points.size());
      frontier.centroid.y /= static_cast<double>(frontier.points.size());
    }
    frontier.size_m =
        static_cast<double>(frontier.points.size()) * map.info.resolution;

    frontier.blacklist_anchor =
        clusterAnchor(map, frontier, free_boundary_cells);
    frontier.blacklist_anchor_valid = true;
    frontier.stability_observations = recordFrontierObservation(frontier);
    frontier.stability_confirmed =
        !enable_frontier_stability_filter_ ||
        frontier.stability_observations >=
            static_cast<size_t>(frontier_stability_required_observations_);
    if (frontier.stability_confirmed) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "explore_frontier_stable_confirmed centroid=%s anchor=%s "
          "observations=%lu required_observations=%d",
          formatPoint(frontier.centroid).c_str(),
          formatPoint(frontier.blacklist_anchor).c_str(),
          frontier.stability_observations,
          frontier_stability_required_observations_);
    }

    std::string area_reuse_reason;
    FrontierAreaRecord* area_record =
        matchingAreaRecord(frontier, map, free_boundary_cells,
                           area_reuse_reason);
    if (area_record != nullptr) {
      frontier.cluster_blacklisted = true;
      frontier.blacklist_reason = area_record->reason;
      frontier.area_id = area_record->area_id;
      frontier.area_memory_reason = area_record->reason;
      frontier.candidate_stats.suppressed_by_area_memory = 1;
      frontier.score = std::numeric_limits<double>::infinity();
      return frontier;
    }

    std::string blacklist_reason;
    if (!enable_frontier_area_memory_ &&
        isBlacklisted(frontier.blacklist_anchor, &blacklist_reason)) {
      frontier.cluster_blacklisted = true;
      frontier.blacklist_reason = blacklist_reason;
      frontier.score = std::numeric_limits<double>::infinity();
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "explore_frontier_blacklist_skip reason=%s anchor=%s "
          "radius_m=%.3f",
          blacklist_reason.c_str(),
          formatPoint(frontier.blacklist_anchor).c_str(), blacklist_radius_);
      return frontier;
    }

    buildGoalCandidates(map, frontier, free_boundary_cells, robot_pose);

    if (!frontier.candidates.empty()) {
      applyCandidate(frontier, frontier.candidates.front());
      applyDiversityPenalty(frontier);
    } else {
      frontier.score = std::numeric_limits<double>::infinity();
      frontier.residual_classification = classifyResidualFrontier(frontier);
      RCLCPP_INFO(
          this->get_logger(),
          "explore_frontier_residual_classification centroid=%s anchor=%s "
          "classification=%s unique_candidates=%lu rejected_unknown=%lu "
          "rejected_occupied=%lu rejected_low_clearance=%lu",
          formatPoint(frontier.centroid).c_str(),
          formatPoint(frontier.blacklist_anchor).c_str(),
          frontier.residual_classification.c_str(),
          frontier.candidate_stats.unique_candidates,
          frontier.candidate_stats.rejected_unknown,
          frontier.candidate_stats.rejected_occupied,
          frontier.candidate_stats.rejected_low_clearance);
      const bool unstable_residual =
          enable_frontier_stability_filter_ &&
          !frontier.stability_confirmed &&
          isNoisyResidualFrontier(frontier);
      if (unstable_residual) {
        frontier.unstable_suppressed = true;
        frontier.area_id = suppressFrontierArea(frontier, "unstable_frontier");
        frontier.area_memory_reason = "unstable_frontier";
        RCLCPP_INFO(
            this->get_logger(),
            "explore_frontier_unstable_suppressed centroid=%s anchor=%s "
            "classification=%s observations=%lu required_observations=%d "
            "timeout_sec=%.3f",
            formatPoint(frontier.centroid).c_str(),
            formatPoint(frontier.blacklist_anchor).c_str(),
            frontier.residual_classification.c_str(),
            frontier.stability_observations,
            frontier_stability_required_observations_,
            unstable_frontier_suppress_timeout_sec_);
      } else {
        frontier.area_id = suppressFrontierArea(frontier, "no_clear_free_goal");
        frontier.area_memory_reason = "no_clear_free_goal";
      }
    }

    return frontier;
  }

  geometry_msgs::msg::Point clusterAnchor(
      const nav_msgs::msg::OccupancyGrid& map,
      const FrontierGroup& frontier,
      const std::vector<size_t>& free_boundary_cells) const
  {
    if (free_boundary_cells.empty()) {
      return frontier.centroid;
    }

    geometry_msgs::msg::Point best = frontier.centroid;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const size_t cell_idx : free_boundary_cells) {
      uint32_t mx = 0;
      uint32_t my = 0;
      cellsFromIndex(cell_idx, map.info.width, mx, my);
      const auto point = cellToWorld(map, mx, my);
      const double distance = distance2d(point, frontier.centroid);
      if (distance < best_distance) {
        best_distance = distance;
        best = point;
      }
    }
    return best;
  }

  void buildGoalCandidates(
      const nav_msgs::msg::OccupancyGrid& map, FrontierGroup& frontier,
      const std::vector<size_t>& free_boundary_cells,
      const geometry_msgs::msg::Pose& robot_pose)
  {
    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    const double resolution = map.info.resolution;
    const int radius_cells =
        static_cast<int>(std::ceil(frontier_goal_search_radius_m_ / resolution));
    const size_t total_cells =
        static_cast<size_t>(width) * static_cast<size_t>(height);
    CandidateStats& stats = frontier.candidate_stats;

    struct RawCandidate {
      size_t idx = 0;
      double frontier_distance_m = 0.0;
    };
    std::vector<RawCandidate> raw_candidates;
    std::vector<bool> seen(total_cells, false);
    const size_t max_unique_raw_candidates =
        std::max(static_cast<size_t>(max_frontier_goal_candidates_),
                 static_cast<size_t>(max_frontier_goal_candidates_) * 4U);

    auto add_cell = [&](int mx, int my) -> bool {
      ++stats.raw_generated;
      if (!inBounds(mx, my, width, height)) {
        return true;
      }
      const size_t candidate_idx = index(mx, my, width);
      if (seen[candidate_idx]) {
        ++stats.duplicate_candidates;
        return true;
      }
      seen[candidate_idx] = true;
      if (raw_candidates.size() >= max_unique_raw_candidates) {
        return false;
      }
      const auto point = cellToWorld(map, static_cast<uint32_t>(mx),
                                     static_cast<uint32_t>(my));
      RawCandidate raw;
      raw.idx = candidate_idx;
      raw.frontier_distance_m = distance2d(point, frontier.centroid);
      raw_candidates.push_back(raw);
      return raw_candidates.size() < max_unique_raw_candidates;
    };

    std::vector<size_t> sorted_boundary_cells = free_boundary_cells;
    std::sort(sorted_boundary_cells.begin(), sorted_boundary_cells.end(),
              [this, &map, &frontier](size_t a, size_t b) {
                uint32_t ax = 0;
                uint32_t ay = 0;
                uint32_t bx = 0;
                uint32_t by = 0;
                cellsFromIndex(a, map.info.width, ax, ay);
                cellsFromIndex(b, map.info.width, bx, by);
                return distance2d(cellToWorld(map, ax, ay),
                                  frontier.centroid) <
                    distance2d(cellToWorld(map, bx, by), frontier.centroid);
              });

    bool raw_limit_reached = false;
    for (const size_t seed_idx : sorted_boundary_cells) {
      if (raw_limit_reached) {
        break;
      }
      uint32_t sx = 0;
      uint32_t sy = 0;
      cellsFromIndex(seed_idx, width, sx, sy);
      if (!add_cell(static_cast<int>(sx), static_cast<int>(sy))) {
        raw_limit_reached = true;
        break;
      }
      for (int dy = -radius_cells; dy <= radius_cells && !raw_limit_reached;
           ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          if (dx * dx + dy * dy > radius_cells * radius_cells) {
            continue;
          }
          if (!add_cell(static_cast<int>(sx) + dx,
                        static_cast<int>(sy) + dy)) {
            raw_limit_reached = true;
            break;
          }
        }
      }
    }
    stats.unique_candidates = raw_candidates.size();

    std::sort(raw_candidates.begin(), raw_candidates.end(),
              [](const RawCandidate& a, const RawCandidate& b) {
                return a.frontier_distance_m < b.frontier_distance_m;
              });

    for (const auto& raw : raw_candidates) {
      if (frontier.candidates.size() >=
          static_cast<size_t>(max_frontier_goal_candidates_)) {
        break;
      }

      uint32_t mx = 0;
      uint32_t my = 0;
      cellsFromIndex(raw.idx, width, mx, my);
      FrontierGroup::GoalCandidate candidate;
      if (!validateGoalCandidate(map, mx, my, frontier.centroid, robot_pose,
                                 stats, candidate)) {
        continue;
      }
      frontier.candidates.push_back(candidate);
    }

    std::sort(frontier.candidates.begin(), frontier.candidates.end(),
              [](const FrontierGroup::GoalCandidate& a,
                 const FrontierGroup::GoalCandidate& b) {
                return a.score < b.score;
              });

    RCLCPP_INFO(
        this->get_logger(),
        "explore_frontier_candidate_summary centroid=%s size_m=%.3f "
        "raw_generated=%lu duplicate_candidates=%lu unique_candidates=%lu "
        "accepted_clearance=%lu rejected_unknown=%lu rejected_occupied=%lu "
        "rejected_low_clearance=%lu rejected_blacklisted=%lu "
        "suppressed_by_area_memory=%lu",
        formatPoint(frontier.centroid).c_str(), frontier.size_m,
        stats.raw_generated, stats.duplicate_candidates,
        stats.unique_candidates, stats.accepted_clearance,
        stats.rejected_unknown, stats.rejected_occupied,
        stats.rejected_low_clearance, stats.rejected_blacklisted,
        stats.suppressed_by_area_memory);

    if (frontier.candidates.empty()) {
      RCLCPP_INFO(
          this->get_logger(),
          "explore_frontier_rejected_no_clear_free_goal centroid=%s "
          "size_m=%.3f raw_generated=%lu unique_candidates=%lu "
          "rejected_unknown=%lu rejected_occupied=%lu "
          "rejected_low_clearance=%lu rejected_blacklisted=%lu "
          "suppressed_by_area_memory=%lu min_goal_clearance_m=%.3f",
          formatPoint(frontier.centroid).c_str(), frontier.size_m,
          stats.raw_generated, stats.unique_candidates,
          stats.rejected_unknown, stats.rejected_occupied,
          stats.rejected_low_clearance, stats.rejected_blacklisted,
          stats.suppressed_by_area_memory, min_goal_clearance_m_);
    }
  }

  void maybeLogCandidateDetail(
      CandidateStats& stats, const std::string& log_key,
      const geometry_msgs::msg::Point& target, uint32_t mx, uint32_t my,
      const std::string& detail)
  {
    if (!log_candidate_rejections_debug_) {
      return;
    }
    if (candidate_rejection_logs_this_cycle_ >=
            static_cast<size_t>(max_candidate_rejection_logs_per_cycle_) ||
        stats.rejection_logs_emitted >=
            static_cast<size_t>(max_candidate_rejection_logs_per_cluster_)) {
      return;
    }
    ++candidate_rejection_logs_this_cycle_;
    ++stats.rejection_logs_emitted;
    RCLCPP_DEBUG(
        this->get_logger(),
        "%s target=%s cell=(%u,%u) %s",
        log_key.c_str(), formatPoint(target).c_str(), mx, my,
        detail.c_str());
  }

  bool validateGoalCandidate(
      const nav_msgs::msg::OccupancyGrid& map, uint32_t mx, uint32_t my,
      const geometry_msgs::msg::Point& frontier_centroid,
      const geometry_msgs::msg::Pose& robot_pose,
      CandidateStats& stats,
      FrontierGroup::GoalCandidate& candidate)
  {
    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    if (!inBounds(static_cast<int>(mx), static_cast<int>(my), width, height)) {
      return false;
    }

    const int8_t value = map.data[index(mx, my, width)];
    const auto target = cellToWorld(map, mx, my);
    if (value == kUnknownCell && reject_unknown_goal_cells_) {
      ++stats.rejected_unknown;
      maybeLogCandidateDetail(
          stats, "explore_frontier_rejected_unknown", target, mx, my,
          "reason=unknown");
      return false;
    }
    if (value >= occupied_threshold_ && reject_occupied_goal_cells_) {
      ++stats.rejected_occupied;
      std::ostringstream detail;
      detail << "reason=occupied occupancy=" << static_cast<int>(value)
             << " occupied_threshold=" << occupied_threshold_;
      maybeLogCandidateDetail(
          stats, "explore_frontier_rejected_occupied", target, mx, my,
          detail.str());
      return false;
    }
    if (!isFree(value)) {
      ++stats.rejected_occupied;
      return false;
    }

    const double clearance_m = nearestOccupiedDistance(map, mx, my);
    if (enable_goal_clearance_filter_ &&
        clearance_m < min_goal_clearance_m_) {
      ++stats.rejected_low_clearance;
      std::ostringstream detail;
      detail.setf(std::ios::fixed);
      detail.precision(3);
      detail << "reason=low_clearance clearance_m=" << clearance_m
             << " min_goal_clearance_m=" << min_goal_clearance_m_;
      maybeLogCandidateDetail(
          stats, "explore_frontier_rejected_low_clearance", target, mx, my,
          detail.str());
      return false;
    }

    if (!enable_frontier_area_memory_ && isBlacklisted(target)) {
      ++stats.rejected_blacklisted;
      std::string reason;
      isBlacklisted(target, &reason);
      maybeLogCandidateDetail(
          stats, "explore_frontier_blacklist_skip", target, mx, my,
          "reason=" + reason);
      return false;
    }

    const double robot_yaw = yawFromQuaternion(robot_pose.orientation);
    const double target_heading = std::atan2(
        target.y - robot_pose.position.y, target.x - robot_pose.position.x);
    candidate.target = target;
    candidate.mx = mx;
    candidate.my = my;
    candidate.clearance_m = clearance_m;
    candidate.frontier_distance_m = distance2d(target, frontier_centroid);
    candidate.robot_distance_m = distance2d(robot_pose.position, target);
    candidate.heading_delta =
        std::fabs(normalizeAngle(target_heading - robot_yaw));
    const double finite_clearance =
        std::isfinite(clearance_m) ? clearance_m :
        min_goal_clearance_m_ + map.info.resolution;
    candidate.score = candidate.frontier_distance_m +
        0.05 * candidate.robot_distance_m +
        0.05 * candidate.heading_delta -
        0.25 * std::min(finite_clearance,
                        min_goal_clearance_m_ + map.info.resolution);
    ++stats.accepted_clearance;
    std::ostringstream detail;
    detail.setf(std::ios::fixed);
    detail.precision(3);
    detail << "clearance_m=" << candidate.clearance_m
           << " frontier_distance_m=" << candidate.frontier_distance_m
           << " robot_distance_m=" << candidate.robot_distance_m
           << " score=" << candidate.score;
    maybeLogCandidateDetail(
        stats, "explore_frontier_candidate_goal", candidate.target, mx, my,
        detail.str());
    return true;
  }

  double nearestOccupiedDistance(const nav_msgs::msg::OccupancyGrid& map,
                                 uint32_t mx, uint32_t my) const
  {
    if (!enable_goal_clearance_filter_) {
      return std::numeric_limits<double>::infinity();
    }
    const double resolution = map.info.resolution;
    const int radius_cells =
        static_cast<int>(std::ceil(min_goal_clearance_m_ / resolution));
    double nearest_m = std::numeric_limits<double>::infinity();
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int nx = static_cast<int>(mx) + dx;
        const int ny = static_cast<int>(my) + dy;
        if (!inBounds(nx, ny, map.info.width, map.info.height)) {
          continue;
        }
        const int8_t value = map.data[index(nx, ny, map.info.width)];
        if (value < occupied_threshold_) {
          continue;
        }
        const double distance_m =
            std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution;
        nearest_m = std::min(nearest_m, distance_m);
      }
    }
    return nearest_m;
  }

  void applyCandidate(FrontierGroup& frontier,
                      const FrontierGroup::GoalCandidate& candidate) const
  {
    frontier.target = candidate.target;
    frontier.target_mx = candidate.mx;
    frontier.target_my = candidate.my;
    frontier.target_clearance_m = candidate.clearance_m;
    frontier.distance_m = candidate.robot_distance_m;
    frontier.heading_delta = candidate.heading_delta;
    frontier.goal_yaw = std::atan2(
        frontier.centroid.y - frontier.target.y,
        frontier.centroid.x - frontier.target.x);
    frontier.score = potential_scale_ * frontier.distance_m +
        orientation_scale_ * frontier.heading_delta -
        gain_scale_ * frontier.size_m +
        candidate.score;
    frontier.target_valid = true;
  }

  ComputePathCheckResult computePathPrecheck(const FrontierGroup& frontier)
  {
    ComputePathCheckResult check;
    if (!enable_compute_path_precheck_) {
      check.reachable = true;
      check.reason = "disabled";
      check.error_code = ComputePathToPose::Result::NONE;
      return check;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "explore_compute_path_precheck_started action=%s target=%s "
        "timeout_sec=%.3f",
        compute_path_action_name_.c_str(),
        formatPoint(frontier.target).c_str(), compute_path_timeout_sec_);

    if (!compute_path_client_ ||
        !compute_path_client_->action_server_is_ready()) {
      check.reason = "server_unavailable";
      return check;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(compute_path_timeout_sec_));
    auto remaining_timeout = [&deadline]() {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        return std::chrono::steady_clock::duration::zero();
      }
      return deadline - now;
    };

    ComputePathToPose::Goal goal;
    goal.goal.header.frame_id = global_frame_;
    goal.goal.header.stamp = this->now();
    goal.goal.pose.position = frontier.target;
    goal.goal.pose.orientation = quaternionFromYaw(frontier.goal_yaw);
    goal.planner_id = "";
    goal.use_start = false;

    auto send_future = compute_path_client_->async_send_goal(goal);
    if (send_future.wait_for(remaining_timeout()) != std::future_status::ready) {
      check.timed_out = true;
      check.reason = "goal_response_timeout";
      return check;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle) {
      check.goal_rejected = true;
      check.reason = "goal_rejected";
      return check;
    }

    auto result_future = compute_path_client_->async_get_result(goal_handle);
    if (result_future.wait_for(remaining_timeout()) != std::future_status::ready) {
      compute_path_client_->async_cancel_goal(goal_handle);
      check.timed_out = true;
      check.reason = "result_timeout";
      return check;
    }

    auto wrapped_result = result_future.get();
    if (!wrapped_result.result) {
      check.reason = "missing_result";
      return check;
    }

    check.path_poses = wrapped_result.result->path.poses.size();
    check.error_code = wrapped_result.result->error_code;
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->error_code == ComputePathToPose::Result::NONE &&
        check.path_poses >= static_cast<size_t>(min_precheck_path_poses_)) {
      check.reachable = true;
      check.reason = "ok";
      RCLCPP_INFO(
          this->get_logger(),
          "explore_compute_path_precheck_success target=%s path_poses=%lu",
          formatPoint(frontier.target).c_str(), check.path_poses);
      return check;
    }

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      check.reason = std::string("action_") + resultCodeName(wrapped_result.code);
    } else if (wrapped_result.result->error_code !=
               ComputePathToPose::Result::NONE) {
      check.reason = computePathErrorName(wrapped_result.result->error_code);
    } else {
      check.reason = "empty_path";
    }
    return check;
  }

  bool isFrontierCell(const nav_msgs::msg::OccupancyGrid& map,
                      int x, int y) const
  {
    const uint32_t width = map.info.width;
    const uint32_t height = map.info.height;
    if (!inBounds(x, y, width, height) ||
        map.data[index(x, y, width)] != kUnknownCell) {
      return false;
    }

    const int dx4[4] = {1, -1, 0, 0};
    const int dy4[4] = {0, 0, 1, -1};
    for (int i = 0; i < 4; ++i) {
      const int nx = x + dx4[i];
      const int ny = y + dy4[i];
      if (inBounds(nx, ny, width, height) &&
          isFree(map.data[index(nx, ny, width)])) {
        return true;
      }
    }
    return false;
  }

  bool inBounds(int x, int y, uint32_t width, uint32_t height) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(width) &&
        y < static_cast<int>(height);
  }

  size_t index(int x, int y, uint32_t width) const
  {
    return static_cast<size_t>(y) * static_cast<size_t>(width) +
        static_cast<size_t>(x);
  }

  void cellsFromIndex(size_t idx, uint32_t width,
                      uint32_t& x, uint32_t& y) const
  {
    x = static_cast<uint32_t>(idx % width);
    y = static_cast<uint32_t>(idx / width);
  }

  bool isFree(int8_t value) const
  {
    return value >= 0 && value < occupied_threshold_;
  }

  geometry_msgs::msg::Point cellToWorld(
      const nav_msgs::msg::OccupancyGrid& map, uint32_t x, uint32_t y) const
  {
    const double local_x =
        (static_cast<double>(x) + 0.5) * map.info.resolution;
    const double local_y =
        (static_cast<double>(y) + 0.5) * map.info.resolution;
    const double yaw = yawFromQuaternion(map.info.origin.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    geometry_msgs::msg::Point point;
    point.x = map.info.origin.position.x + local_x * cos_yaw -
        local_y * sin_yaw;
    point.y = map.info.origin.position.y + local_x * sin_yaw +
        local_y * cos_yaw;
    point.z = 0.0;
    return point;
  }

  void sendNavigationGoal(const FrontierGroup& frontier,
                          const geometry_msgs::msg::Pose& robot_pose)
  {
    if (active_goal_ &&
        distance2d(active_goal_target_, frontier.target) <
            same_goal_distance_m_) {
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position = frontier.target;
    goal.pose.pose.orientation = quaternionFromYaw(frontier.goal_yaw);

    active_goal_ = true;
    active_goal_sequence_++;
    active_goal_target_ = frontier.target;
    active_goal_frontier_ = frontier;
    active_goal_frontier_valid_ = true;
    active_goal_best_distance_m_ =
        distance2d(robot_pose.position, frontier.target);
    active_goal_last_progress_at_ = this->now();
    active_goal_sent_at_ = this->now();
    resetActiveGoalToleranceProgress();

    const uint64_t sequence = active_goal_sequence_;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this, sequence, frontier](
            const NavigateGoalHandle::SharedPtr& goal_handle) {
          if (sequence != active_goal_sequence_ || !active_goal_) {
            return;
          }
          if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(),
                        "explore_nav2_goal_rejected target=%s area_id=%d",
                        formatPoint(frontier.target).c_str(), frontier.area_id);
            suppressFrontierArea(frontier, "goal_rejected");
            clearActiveGoal("goal_rejected");
            next_rescan_at_ =
                this->now() + rclcpp::Duration::from_seconds(rescan_delay_sec_);
            setState(ExplorerState::RECOVERING_AFTER_FAILURE,
                     "goal_rejected");
            return;
          }
          active_goal_handle_ = goal_handle;
          RCLCPP_INFO(this->get_logger(),
                      "explore_nav2_goal_accepted target=%s area_id=%d",
                      formatPoint(frontier.target).c_str(), frontier.area_id);
        };
    options.result_callback =
        [this, sequence, frontier](
            const NavigateGoalHandle::WrappedResult& result) {
          handleNavigationResult(sequence, frontier, result);
        };

    RCLCPP_INFO(
        this->get_logger(),
        "explore_nav2_goal_sent action=%s target=%s centroid=%s "
        "distance_m=%.3f size_m=%.3f score=%.6f area_id=%d "
        "area_anchor=%s area_reason=%s",
        navigate_action_name_.c_str(), formatPoint(frontier.target).c_str(),
        formatPoint(frontier.centroid).c_str(), frontier.distance_m,
        frontier.size_m, frontier.score, frontier.area_id,
        formatPoint(frontier.blacklist_anchor).c_str(),
        frontier.area_memory_reason.c_str());
    setState(ExplorerState::NAVIGATING_TO_FRONTIER, "goal_sent");
    nav_client_->async_send_goal(goal, options);
  }

  void handleNavigationResult(
      uint64_t sequence, const FrontierGroup& frontier,
      const NavigateGoalHandle::WrappedResult& result)
  {
    if (wasCompletedByExplorerTolerance(sequence)) {
      const uint16_t error_code = result.result ? result.result->error_code : 0;
      const std::string error_msg =
          result.result ? result.result->error_msg : std::string("");
      RCLCPP_INFO(
          this->get_logger(),
          "explore_nav2_result_ignored_after_explorer_tolerance_success "
          "target=%s result_code=%s error_code=%u error_msg='%s' "
          "sequence=%lu area_id=%d",
          formatPoint(frontier.target).c_str(), resultCodeName(result.code),
          static_cast<unsigned int>(error_code), error_msg.c_str(),
          sequence, frontier.area_id);
      return;
    }

    if (sequence != active_goal_sequence_) {
      RCLCPP_INFO(this->get_logger(),
                  "explore_nav2_stale_result_ignored target=%s result_code=%s",
                  formatPoint(frontier.target).c_str(),
                  resultCodeName(result.code));
      return;
    }

    const uint16_t error_code = result.result ? result.result->error_code : 0;
    const std::string error_msg =
        result.result ? result.result->error_msg : std::string("");
    RCLCPP_INFO(
        this->get_logger(),
        "explore_nav2_result target=%s result_code=%s error_code=%u "
        "error_msg='%s' area_id=%d",
        formatPoint(frontier.target).c_str(), resultCodeName(result.code),
        static_cast<unsigned int>(error_code), error_msg.c_str(),
        frontier.area_id);

    const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result &&
        result.result->error_code == NavigateToPose::Result::NONE;
    const int area_id = suppressFrontierArea(
        frontier, succeeded ? "succeeded" :
            (result.code == rclcpp_action::ResultCode::CANCELED ?
             "nav2_canceled" : "nav2_failed"));
    clearActiveGoal(succeeded ? "nav2_succeeded" : "nav2_terminal_result");
    next_rescan_at_ =
        this->now() + rclcpp::Duration::from_seconds(rescan_delay_sec_);

    if (succeeded) {
      last_useful_frontier_success_at_ = this->now();
      blocked_no_reachable_cycles_ = 0;
      blocked_frontier_signature_initialized_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "explore_nav2_goal_succeeded target=%s area_id=%d "
                  "centroid=%s area_anchor=%s",
                  formatPoint(frontier.target).c_str(), area_id,
                  formatPoint(frontier.centroid).c_str(),
                  formatPoint(frontier.blacklist_anchor).c_str());
      setState(ExplorerState::SELECTING_FRONTIER,
               "nav2_goal_succeeded_rescan");
      return;
    }

    setState(ExplorerState::RECOVERING_AFTER_FAILURE,
             "nav2_goal_failed");
  }

  void handleResume(bool resume)
  {
    if (!resume) {
      if (active_goal_ && active_goal_handle_) {
        nav_client_->async_cancel_goal(active_goal_handle_);
      }
      clearActiveGoal("paused");
      setState(ExplorerState::PAUSED, "resume_false");
      RCLCPP_INFO(this->get_logger(), "explore_pause_applied");
      return;
    }

    if (state_ == ExplorerState::PAUSED ||
        state_ == ExplorerState::EXPLORATION_COMPLETE ||
        state_ ==
            ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS ||
        state_ ==
            ExplorerState::EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS) {
      next_rescan_at_ = this->now();
      blocked_retry_at_ = this->now();
      blocked_no_reachable_cycles_ = 0;
      blocked_frontier_signature_initialized_ = false;
      if (state_ ==
          ExplorerState::EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS) {
        frontier_area_records_.clear();
        blacklist_.clear();
        RCLCPP_INFO(this->get_logger(),
                    "explore_residual_exhaustion_manual_retry_reset");
      }
      setState(ExplorerState::SELECTING_FRONTIER, "resume_true");
      RCLCPP_INFO(this->get_logger(), "explore_resume_applied");
    }
  }

  void clearActiveGoal(const std::string& reason)
  {
    if (active_goal_) {
      RCLCPP_INFO(this->get_logger(),
                  "explore_active_goal_cleared reason=%s target=%s",
                  reason.c_str(), formatPoint(active_goal_target_).c_str());
    }
    active_goal_ = false;
    active_goal_frontier_valid_ = false;
    resetActiveGoalToleranceProgress();
    active_goal_handle_.reset();
    active_goal_sequence_++;
  }

  geometry_msgs::msg::Point areaAnchorForFrontier(
      const FrontierGroup& frontier) const
  {
    if (frontier.target_valid) {
      return frontier.target;
    }
    if (frontier.blacklist_anchor_valid) {
      return frontier.blacklist_anchor;
    }
    return frontier.centroid;
  }

  FrontierAreaRecord* findAreaRecordForUpdate(
      const FrontierGroup& frontier, const geometry_msgs::msg::Point& anchor,
      double radius_m)
  {
    FrontierAreaRecord* best_record = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    for (auto& record : frontier_area_records_) {
      const double match_radius = std::max(radius_m, record.radius_m);
      double match_distance = distance2d(record.anchor, anchor);
      if (frontier.blacklist_anchor_valid) {
        match_distance = std::min(
            match_distance, distance2d(record.last_boundary_anchor,
                                       frontier.blacklist_anchor));
      }
      match_distance =
          std::min(match_distance, distance2d(record.last_centroid,
                                             frontier.centroid));
      if (frontier.target_valid) {
        match_distance =
            std::min(match_distance,
                     distance2d(record.last_target, frontier.target));
      }
      if (match_distance <= match_radius && match_distance < best_distance) {
        best_distance = match_distance;
        best_record = &record;
      }
    }
    return best_record;
  }

  void pushRecentArea(const geometry_msgs::msg::Point& anchor)
  {
    if (!enable_frontier_area_memory_ ||
        !enable_frontier_diversity_penalty_) {
      return;
    }
    recent_area_history_.push_back(anchor);
    while (recent_area_history_.size() >
           static_cast<size_t>(recent_area_history_size_)) {
      recent_area_history_.erase(recent_area_history_.begin());
    }
  }

  int suppressFrontierArea(const FrontierGroup& frontier,
                           const std::string& reason)
  {
    if (!enable_frontier_area_memory_) {
      addOrUpdateBlacklist(areaAnchorForFrontier(frontier), reason);
      return -1;
    }

    const rclcpp::Time now = this->now();
    const geometry_msgs::msg::Point anchor = areaAnchorForFrontier(frontier);
    const geometry_msgs::msg::Point boundary_anchor =
        frontier.blacklist_anchor_valid ? frontier.blacklist_anchor : anchor;
    const geometry_msgs::msg::Point target =
        frontier.target_valid ? frontier.target : anchor;
    double radius_m = areaRadiusForReason(reason);
    double timeout_sec = areaTimeoutForReason(reason);

    FrontierAreaRecord* record =
        findAreaRecordForUpdate(frontier, anchor, radius_m);
    const bool created = record == nullptr;
    if (created) {
      frontier_area_records_.push_back(FrontierAreaRecord());
      record = &frontier_area_records_.back();
      record->area_id = next_frontier_area_id_++;
      record->first_seen = now;
    }

    if (isFailureSuppressionReason(reason)) {
      ++record->failure_count;
      if (record->failure_count >= max_consecutive_failures_per_frontier_) {
        timeout_sec = std::max(timeout_sec, blacklist_timeout_sec_ * 2.0);
      }
      if (record->failure_count >= no_clear_area_hard_suppress_after_failures_) {
        timeout_sec = std::max(timeout_sec, no_clear_area_long_timeout_sec_);
        radius_m = std::max(radius_m, frontier_area_failure_hard_radius_m_);
      }
    }
    if (reason == "succeeded") {
      ++record->success_count;
      if (record->success_count >=
          successful_area_hard_suppress_after_successes_) {
        timeout_sec = std::max(timeout_sec, successful_area_long_timeout_sec_);
      }
    }
    if (reason == "succeeded" || reason == "selected_recently") {
      pushRecentArea(anchor);
    }

    record->anchor = anchor;
    record->last_centroid = frontier.centroid;
    record->last_target = target;
    record->last_boundary_anchor = boundary_anchor;
    record->last_size_m = frontier.size_m;
    record->reason = reason;
    record->last_seen = now;
    record->expires_at = now + rclcpp::Duration::from_seconds(timeout_sec);
    record->radius_m = radius_m;

    RCLCPP_WARN(
        this->get_logger(),
        "explore_frontier_area_suppressed action=%s area_id=%d reason=%s "
        "anchor=%s centroid=%s target=%s boundary_anchor=%s radius_m=%.3f "
        "timeout_sec=%.3f failure_count=%d success_count=%d size_m=%.3f",
        created ? "added" : "updated", record->area_id, reason.c_str(),
        formatPoint(record->anchor).c_str(),
        formatPoint(record->last_centroid).c_str(),
        formatPoint(record->last_target).c_str(),
        formatPoint(record->last_boundary_anchor).c_str(), record->radius_m,
        timeout_sec, record->failure_count, record->success_count,
        record->last_size_m);
    if ((isFailureSuppressionReason(reason) &&
         record->failure_count >= no_clear_area_hard_suppress_after_failures_) ||
        (reason == "succeeded" &&
         record->success_count >=
             successful_area_hard_suppress_after_successes_)) {
      RCLCPP_WARN(
          this->get_logger(),
          "explore_frontier_area_hard_suppressed area_id=%d reason=%s "
          "anchor=%s radius_m=%.3f timeout_sec=%.3f failure_count=%d "
          "success_count=%d",
          record->area_id, reason.c_str(),
          formatPoint(record->anchor).c_str(), record->radius_m,
          timeout_sec, record->failure_count, record->success_count);
    }
    return record->area_id;
  }

  bool hasExpiredFrontierAreaRecord(const rclcpp::Time& now) const
  {
    for (const auto& record : frontier_area_records_) {
      if (now >= record.expires_at) {
        return true;
      }
    }
    return false;
  }

  void pruneFrontierAreaRecords()
  {
    if (!enable_frontier_area_memory_) {
      return;
    }
    const rclcpp::Time now = this->now();
    std::vector<FrontierAreaRecord> retained;
    retained.reserve(frontier_area_records_.size());
    for (const auto& record : frontier_area_records_) {
      if (now >= record.expires_at) {
        RCLCPP_INFO(
            this->get_logger(),
            "explore_frontier_area_expired area_id=%d reason=%s anchor=%s "
            "failure_count=%d success_count=%d radius_m=%.3f",
            record.area_id, record.reason.c_str(),
            formatPoint(record.anchor).c_str(), record.failure_count,
            record.success_count, record.radius_m);
      } else {
        retained.push_back(record);
      }
    }
    frontier_area_records_.swap(retained);
  }

  void addOrUpdateBlacklist(const geometry_msgs::msg::Point& target,
                            const std::string& reason)
  {
    const rclcpp::Time expires_at =
        this->now() + rclcpp::Duration::from_seconds(blacklist_timeout_sec_);
    for (auto& entry : blacklist_) {
      if (distance2d(entry.target, target) <= blacklist_radius_) {
        entry.target = target;
        entry.expires_at = expires_at;
        entry.reason = reason;
        ++entry.failures;
        RCLCPP_WARN(
            this->get_logger(),
            "explore_frontier_blacklisted action=updated reason=%s anchor=%s "
            "failures=%d max_consecutive_failures_per_frontier=%d "
            "timeout_sec=%.3f radius_m=%.3f",
            reason.c_str(), formatPoint(target).c_str(), entry.failures,
            max_consecutive_failures_per_frontier_, blacklist_timeout_sec_,
            blacklist_radius_);
        return;
      }
    }

    BlacklistEntry entry;
    entry.target = target;
    entry.expires_at = expires_at;
    entry.reason = reason;
    entry.failures = 1;
    blacklist_.push_back(entry);
    RCLCPP_WARN(
        this->get_logger(),
        "explore_frontier_blacklisted action=added reason=%s anchor=%s "
        "failures=%d timeout_sec=%.3f radius_m=%.3f",
        reason.c_str(), formatPoint(target).c_str(), entry.failures,
        blacklist_timeout_sec_, blacklist_radius_);
  }

  bool isBlacklisted(const geometry_msgs::msg::Point& target,
                     std::string* reason = nullptr) const
  {
    const rclcpp::Time now = this->now();
    for (const auto& entry : blacklist_) {
      if (now < entry.expires_at &&
          distance2d(entry.target, target) <= blacklist_radius_) {
        if (reason != nullptr) {
          *reason = entry.reason;
        }
        return true;
      }
    }
    return false;
  }

  bool hasExpiredBlacklist(const rclcpp::Time& now) const
  {
    for (const auto& entry : blacklist_) {
      if (now >= entry.expires_at) {
        return true;
      }
    }
    return false;
  }

  void pruneBlacklist()
  {
    const rclcpp::Time now = this->now();
    std::vector<BlacklistEntry> retained;
    retained.reserve(blacklist_.size());
    for (const auto& entry : blacklist_) {
      if (now >= entry.expires_at) {
        RCLCPP_INFO(
            this->get_logger(),
            "explore_frontier_blacklist_expired target=%s reason=%s "
            "failures=%d",
            formatPoint(entry.target).c_str(), entry.reason.c_str(),
            entry.failures);
      } else {
        retained.push_back(entry);
      }
    }
    blacklist_.swap(retained);
  }

  void publishFrontierMarkers(
      const std::vector<FrontierGroup>& frontiers,
      const FrontierGroup* selected_frontier)
  {
    if (!visualize_ || !frontier_publisher_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->now();
    marker.ns = "direct_nav2_frontiers";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker.pose.orientation.w = 1.0;
    markers.markers.push_back(marker);

    int id = 0;
    for (const auto& frontier : frontiers) {
      marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = global_frame_;
      marker.header.stamp = this->now();
      marker.ns = "direct_nav2_frontier_points";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.color.b = 1.0;
      marker.color.a = 0.65;
      marker.points = frontier.points;
      markers.markers.push_back(marker);

      marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = global_frame_;
      marker.header.stamp = this->now();
      marker.ns = "direct_nav2_frontier_targets";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      if (!frontier.target_valid) {
        continue;
      }
      marker.pose.orientation.w = 1.0;
      marker.pose.position = frontier.target;
      const bool selected = selected_frontier != nullptr &&
          distance2d(frontier.target, selected_frontier->target) <
              same_goal_distance_m_;
      marker.scale.x = selected ? 0.45 : 0.25;
      marker.scale.y = selected ? 0.45 : 0.25;
      marker.scale.z = selected ? 0.45 : 0.25;
      if (isBlacklisted(frontier.target)) {
        marker.color.r = 1.0;
        marker.color.a = 0.75;
      } else if (frontier.distance_m < min_goal_distance_m_) {
        marker.color.r = 1.0;
        marker.color.g = 0.55;
        marker.color.a = 0.75;
      } else if (selected) {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.a = 0.9;
      } else {
        marker.color.g = 1.0;
        marker.color.a = 0.75;
      }
      markers.markers.push_back(marker);
    }

    frontier_publisher_->publish(markers);
  }

  void setState(ExplorerState state, const std::string& reason)
  {
    if (state_initialized_ && state_ == state) {
      return;
    }

    const ExplorerState previous = state_;
    state_ = state;
    state_initialized_ = true;

    explore_lite_msgs::msg::ExploreStatus status;
    status.status = statusForState(state);
    status_publisher_->publish(status);

    RCLCPP_INFO(this->get_logger(),
                "explore_status_transition from=%s to=%s status=%s reason=%s",
                stateName(previous), stateName(state), status.status.c_str(),
                reason.c_str());
  }

  const char* stateName(ExplorerState state) const
  {
    switch (state) {
      case ExplorerState::WAITING_FOR_MAP:
        return "WAITING_FOR_MAP";
      case ExplorerState::WAITING_FOR_MAP_MATURITY:
        return "WAITING_FOR_MAP_MATURITY";
      case ExplorerState::WAITING_FOR_TF:
        return "WAITING_FOR_TF";
      case ExplorerState::WAITING_FOR_NAV2:
        return "WAITING_FOR_NAV2";
      case ExplorerState::SELECTING_FRONTIER:
        return "SELECTING_FRONTIER";
      case ExplorerState::NAVIGATING_TO_FRONTIER:
        return "NAVIGATING_TO_FRONTIER";
      case ExplorerState::RECOVERING_AFTER_FAILURE:
        return "RECOVERING_AFTER_FAILURE";
      case ExplorerState::PAUSED:
        return "PAUSED";
      case ExplorerState::EXPLORATION_COMPLETE:
        return "EXPLORATION_COMPLETE";
      case ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS:
        return "EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS";
      case ExplorerState::EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS:
        return "EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS";
    }
    return "UNKNOWN";
  }

  std::string statusForState(ExplorerState state) const
  {
    switch (state) {
      case ExplorerState::WAITING_FOR_MAP:
        return "waiting_for_map";
      case ExplorerState::WAITING_FOR_MAP_MATURITY:
        return "waiting_for_map_maturity";
      case ExplorerState::WAITING_FOR_TF:
        return "waiting_for_tf";
      case ExplorerState::WAITING_FOR_NAV2:
        return "waiting_for_nav2";
      case ExplorerState::SELECTING_FRONTIER:
        return "selecting_frontier";
      case ExplorerState::NAVIGATING_TO_FRONTIER:
        return "navigating_to_frontier";
      case ExplorerState::RECOVERING_AFTER_FAILURE:
        return "recovering_after_failure";
      case ExplorerState::PAUSED:
        return explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
      case ExplorerState::EXPLORATION_COMPLETE:
        return explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
      case ExplorerState::EXPLORATION_BLOCKED_NO_REACHABLE_FRONTIERS:
        return "exploration_blocked_no_reachable_frontiers";
      case ExplorerState::EXPLORATION_EXHAUSTED_RESIDUAL_FRONTIERS:
        return "exploration_exhausted_residual_frontiers";
    }
    return "unknown";
  }

  std::string map_topic_;
  std::string robot_base_frame_;
  std::string global_frame_;
  std::string navigate_action_name_;
  double planner_frequency_ = 0.2;
  double progress_timeout_ = 30.0;
  bool visualize_ = true;
  double potential_scale_ = 5.0;
  double orientation_scale_ = 0.2;
  double gain_scale_ = 0.2;
  double min_frontier_size_ = 0.30;
  double initial_map_wait_sec_ = 6.0;
  int min_map_width_cells_ = 50;
  int min_map_height_cells_ = 50;
  int min_known_cell_count_ = 1200;
  double min_known_cell_ratio_ = 0.01;
  double min_goal_distance_m_ = 0.85;
  double goal_reached_distance_m_ = 0.60;
  bool enable_explorer_tolerance_completion_ = true;
  int goal_reached_confirm_count_ = 2;
  double goal_reached_hold_sec_ = 0.5;
  double max_wait_after_goal_reached_sec_ = 2.0;
  bool cancel_nav2_on_explorer_tolerance_success_ = true;
  double nav2_cancel_after_tolerance_timeout_sec_ = 2.0;
  double same_goal_distance_m_ = 0.35;
  double blacklist_radius_ = 0.75;
  double blacklist_timeout_sec_ = 90.0;
  int max_consecutive_failures_per_frontier_ = 2;
  double blocked_retry_timeout_sec_ = 30.0;
  double nav2_action_wait_timeout_sec_ = 30.0;
  double tf_wait_timeout_sec_ = 1.0;
  double rescan_delay_sec_ = 1.0;
  int occupied_threshold_ = 50;
  bool log_candidate_rejections_debug_ = false;
  int max_candidate_rejection_logs_per_cycle_ = 10;
  int max_candidate_rejection_logs_per_cluster_ = 3;
  double blocked_no_reachable_backoff_sec_ = 15.0;
  bool require_frontier_set_change_for_blocked_retry_ = false;
  bool enable_frontier_area_memory_ = true;
  double frontier_area_memory_radius_m_ = 1.25;
  double frontier_success_suppress_radius_m_ = 1.25;
  double frontier_failure_suppress_radius_m_ = 1.25;
  double frontier_area_memory_timeout_sec_ = 90.0;
  double frontier_success_suppress_timeout_sec_ = 60.0;
  double frontier_failure_suppress_timeout_sec_ = 90.0;
  double frontier_area_size_growth_ratio_ = 1.5;
  double frontier_area_size_growth_min_m_ = 1.0;
  double frontier_area_anchor_change_radius_m_ = 1.25;
  bool enable_success_area_suppression_ = true;
  bool enable_frontier_diversity_penalty_ = true;
  double recent_area_penalty_ = 3.0;
  int recent_area_history_size_ = 20;
  bool enable_goal_clearance_filter_ = true;
  double min_goal_clearance_m_ = 0.45;
  double frontier_goal_search_radius_m_ = 1.0;
  int max_frontier_goal_candidates_ = 80;
  bool reject_unknown_goal_cells_ = true;
  bool reject_occupied_goal_cells_ = true;
  bool enable_compute_path_precheck_ = true;
  std::string compute_path_action_name_ = "/compute_path_to_pose";
  double compute_path_timeout_sec_ = 2.0;
  int max_precheck_candidates_per_cycle_ = 8;
  int min_precheck_path_poses_ = 2;
  bool enable_frontier_stability_filter_ = true;
  int frontier_stability_required_observations_ = 2;
  int frontier_stability_window_ = 3;
  double unstable_frontier_suppress_timeout_sec_ = 30.0;
  double unstable_frontier_area_radius_m_ = 1.25;
  double unstable_unknown_ratio_threshold_ = 0.80;
  double unstable_occupied_ratio_threshold_ = 0.50;
  int no_clear_area_hard_suppress_after_failures_ = 3;
  double no_clear_area_long_timeout_sec_ = 180.0;
  bool no_clear_area_reuse_requires_clear_candidate_ = true;
  bool failed_area_reuse_requires_compute_path_success_ = true;
  double frontier_area_failure_hard_radius_m_ = 1.50;
  bool enable_residual_frontier_exhaustion_ = true;
  int residual_exhaustion_blocked_cycles_ = 3;
  double residual_exhaustion_min_runtime_sec_ = 60.0;
  double residual_exhaustion_no_success_window_sec_ = 60.0;
  bool residual_exhaustion_require_no_stable_sendable_frontiers_ = true;
  int successful_area_hard_suppress_after_successes_ = 2;
  double successful_area_long_timeout_sec_ = 120.0;
  bool successful_area_reuse_requires_frontier_growth_ = true;

  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  size_t map_revision_ = 0;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      map_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      frontier_publisher_;
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr
      status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr active_goal_monitor_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::CallbackGroup::SharedPtr compute_path_callback_group_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;

  ExplorerState state_ = ExplorerState::WAITING_FOR_MAP;
  bool state_initialized_ = false;
  bool first_meaningful_map_seen_ = false;
  rclcpp::Time first_meaningful_map_time_;
  bool nav2_wait_started_ = false;
  rclcpp::Time nav2_wait_started_at_;

  bool active_goal_ = false;
  uint64_t active_goal_sequence_ = 0;
  NavigateGoalHandle::SharedPtr active_goal_handle_;
  geometry_msgs::msg::Point active_goal_target_;
  FrontierGroup active_goal_frontier_;
  bool active_goal_frontier_valid_ = false;
  double active_goal_best_distance_m_ = std::numeric_limits<double>::infinity();
  rclcpp::Time active_goal_last_progress_at_;
  rclcpp::Time active_goal_sent_at_;
  bool active_goal_tolerance_started_ = false;
  rclcpp::Time active_goal_tolerance_first_at_;
  rclcpp::Time active_goal_tolerance_last_at_;
  int active_goal_tolerance_confirm_count_ = 0;

  rclcpp::Time next_rescan_at_;
  rclcpp::Time blocked_retry_at_;
  rclcpp::Time exploration_started_at_;
  rclcpp::Time last_useful_frontier_success_at_;
  size_t blocked_map_revision_ = 0;
  int blocked_no_reachable_cycles_ = 0;
  bool blocked_frontier_signature_initialized_ = false;
  size_t last_blocked_frontier_count_ = 0;
  size_t last_blocked_stable_sendable_count_ = 0;
  double last_blocked_total_frontier_size_m_ = 0.0;
  size_t candidate_rejection_logs_this_cycle_ = 0;
  std::vector<BlacklistEntry> blacklist_;
  std::vector<FrontierAreaRecord> frontier_area_records_;
  std::vector<FrontierObservationRecord> frontier_observations_;
  std::vector<geometry_msgs::msg::Point> recent_area_history_;
  mutable std::mutex completed_goal_mutex_;
  std::vector<uint64_t> explorer_tolerance_completed_sequences_;
  int next_frontier_area_id_ = 1;
  size_t frontier_scan_cycle_ = 0;
};
}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<explore::DirectNav2Explorer>();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
