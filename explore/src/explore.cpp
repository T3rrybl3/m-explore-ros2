/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

inline static const char* result_code_name(rclcpp_action::ResultCode code)
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

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<std::string>("goal_stamp_mode", std::string("now"));
  this->declare_parameter<bool>("debug_frontier_search", false);
  this->declare_parameter<bool>("publish_debug_frontiers", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("goal_stamp_mode", goal_stamp_mode_);
  this->get_parameter("debug_frontier_search", debug_frontier_search_);
  this->get_parameter("publish_debug_frontiers", publish_debug_frontiers_);

  progress_timeout_ = timeout;
  min_frontier_size_ = min_frontier_size;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                    "s",
                                                                     10);
  }
  if (publish_debug_frontiers_) {
    raw_frontier_marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "explore/frontiers_raw", 10);
    filtered_frontier_marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "explore/frontiers_filtered", 10);
    blacklisted_frontier_marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "explore/frontiers_blacklisted", 10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::visualizeDebugFrontiers(
    const frontier_exploration::FrontierSearchResult& search_result,
    const std::vector<frontier_exploration::Frontier>& blacklisted_frontiers,
    const frontier_exploration::Frontier* selected_frontier)
{
  const auto orange = std_msgs::msg::ColorRGBA().set__r(1.0).set__g(0.55).set__a(0.65);
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.65);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.75);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.75);
  publishFrontierDebugMarkers(raw_frontier_marker_publisher_,
                              search_result.raw_frontiers, "frontiers_raw",
                              orange, orange);
  publishFrontierDebugMarkers(filtered_frontier_marker_publisher_,
                              search_result.filtered_frontiers,
                              "frontiers_filtered", blue, green,
                              selected_frontier);
  publishFrontierDebugMarkers(blacklisted_frontier_marker_publisher_,
                              blacklisted_frontiers, "frontiers_blacklisted",
                              red, red);
}

void Explore::publishFrontierDebugMarkers(
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const std::vector<frontier_exploration::Frontier>& frontiers,
    const std::string& marker_namespace,
    const std_msgs::msg::ColorRGBA& point_color,
    const std_msgs::msg::ColorRGBA& centroid_color,
    const frontier_exploration::Frontier* selected_frontier)
{
  if (!publisher) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers_msg;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = marker_namespace;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  m.pose.orientation.w = 1.0;
  markers_msg.markers.push_back(m);

  size_t id = 0;
  for (const auto& frontier : frontiers) {
    if (!frontier.points.empty()) {
      m = visualization_msgs::msg::Marker();
      m.header.frame_id = costmap_client_.getGlobalFrameID();
      m.header.stamp = this->now();
      m.ns = marker_namespace;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.frame_locked = true;
      m.type = visualization_msgs::msg::Marker::POINTS;
      m.id = int(id++);
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.08;
      m.scale.y = 0.08;
      m.scale.z = 0.08;
      m.color = point_color;
      m.points = frontier.points;
      markers_msg.markers.push_back(m);
    }

    m = visualization_msgs::msg::Marker();
    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = this->now();
    m.ns = marker_namespace;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.frame_locked = true;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id++);
    m.pose.orientation.w = 1.0;
    m.pose.position = frontier.centroid;
    bool selected = selected_frontier != nullptr &&
        same_point(frontier.centroid, selected_frontier->centroid);
    m.scale.x = selected ? 0.45 : 0.25;
    m.scale.y = selected ? 0.45 : 0.25;
    m.scale.z = selected ? 0.45 : 0.25;
    m.color = selected ?
        std_msgs::msg::ColorRGBA().set__r(1.0).set__g(1.0).set__a(0.9) :
        centroid_color;
    markers_msg.markers.push_back(m);
  }
  publisher->publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto search_result = search_.searchFromWithDiagnostics(pose.position);
  auto frontiers = search_result.filtered_frontiers;
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }
  if (debug_frontier_search_) {
    logMinSizeRejectedFrontiers(search_result);
    if (!search_result.valid) {
      RCLCPP_INFO(logger_,
                  "explore_frontier_debug_invalid reason=%s",
                  search_result.invalid_reason.c_str());
    }
  }

  if (frontiers.empty()) {
    std::string stop_reason = "map_unavailable_or_invalid";
    if (search_result.valid) {
      if (search_result.raw_frontiers.empty()) {
        stop_reason = search_result.unknown_cell_count > 0 ?
            "unknown_not_adjacent_to_free" : "no_raw_frontiers";
      } else {
        stop_reason = "all_filtered_by_min_size";
      }
    }
    if (debug_frontier_search_) {
      logFrontierDebug(search_result, 0, 0, nullptr, stop_reason);
    }
    if (publish_debug_frontiers_) {
      visualizeDebugFrontiers(search_result, {}, nullptr);
    }
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  std::vector<frontier_exploration::Frontier> blacklisted_frontiers;
  std::copy_if(frontiers.begin(), frontiers.end(),
               std::back_inserter(blacklisted_frontiers),
               [this](const frontier_exploration::Frontier& f) {
                 return goalOnBlacklist(f.centroid);
               });
  const size_t blacklist_filtered = blacklisted_frontiers.size();
  const size_t remaining = frontiers.size() - blacklist_filtered;
  if (frontier == frontiers.end()) {
    if (debug_frontier_search_) {
      logFrontierDebug(search_result, blacklist_filtered, remaining, nullptr,
                       "all_blacklisted");
      logAllBlacklistedDetails(frontiers);
    }
    if (publish_debug_frontiers_) {
      visualizeDebugFrontiers(search_result, blacklisted_frontiers, nullptr);
    }
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  if (debug_frontier_search_) {
    logFrontierDebug(search_result, blacklist_filtered, remaining, &(*frontier),
                     "remaining_frontiers_available");
  }
  if (publish_debug_frontiers_) {
    visualizeDebugFrontiers(search_result, blacklisted_frontiers, &(*frontier));
  }
  geometry_msgs::msg::Point target_position = frontier->centroid;

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if ((this->now() - last_progress_ >
      tf2::durationFromSec(progress_timeout_)) && !resuming_) {
    frontier_blacklist_.push_back(target_position);
    logBlacklistAdded(target_position, "progress_timeout");
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  if (goal_stamp_mode_ == "latest") {
    goal.pose.header.stamp =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  } else {
    goal.pose.header.stamp = this->now();
  }

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this, target_position](NavigationGoalHandle::SharedPtr goal_handle) {
        if (goal_handle) {
          return;
        }
        prev_goal_.x = std::numeric_limits<double>::quiet_NaN();
        prev_goal_.y = std::numeric_limits<double>::quiet_NaN();
        prev_goal_.z = std::numeric_limits<double>::quiet_NaN();
        RCLCPP_INFO(logger_, "explore_nav2_goal_rejected target=%s",
                    formatPoint(target_position).c_str());
        RCLCPP_INFO(logger_,
                    "explore_frontier_blacklist_skipped "
                    "reason=proxy_soft_abort target=%s nav2_error_code=0 "
                    "nav2_error_msg='goal rejected by proxy before execution'",
                    formatPoint(target_position).c_str());
      };
  // send_goal_options.feedback_callback =
  //   std::bind(&Explore::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  if (goal_stamp_mode_ == "latest") {
    goal.pose.header.stamp =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  } else {
    goal.pose.header.stamp = this->now();
  }

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

std::string Explore::formatPoint(const geometry_msgs::msg::Point& point) const
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "(" << point.x << "," << point.y << ")";
  return stream.str();
}

void Explore::logFrontierDebug(
    const frontier_exploration::FrontierSearchResult& search_result,
    size_t blacklist_filtered, size_t remaining,
    const frontier_exploration::Frontier* selected_frontier,
    const std::string& stop_reason)
{
  std::string selected = selected_frontier == nullptr ?
      "none" : formatPoint(selected_frontier->centroid);
  std::string robot_cell = "n/a";
  std::string robot_occupancy = "n/a";
  if (search_result.robot_cell_valid) {
    std::ostringstream cell_stream;
    cell_stream << "(" << search_result.robot_mx << ","
                << search_result.robot_my << ")";
    robot_cell = cell_stream.str();
    robot_occupancy = std::to_string(search_result.robot_occupancy);
  }
  RCLCPP_INFO(
      logger_,
      "explore_frontier_debug raw_frontiers=%lu min_size_filtered=%lu "
      "sorted_frontiers=%lu blacklist_filtered=%lu remaining=%lu "
      "blacklist_size=%lu selected=%s stop_reason=%s resolution=%.3f "
      "width=%u height=%u robot_cell=%s robot_occupancy=%s",
      search_result.raw_frontiers.size(),
      search_result.min_size_rejected_frontiers.size(),
      search_result.filtered_frontiers.size(), blacklist_filtered, remaining,
      frontier_blacklist_.size(), selected.c_str(), stop_reason.c_str(),
      search_result.resolution, search_result.width, search_result.height,
      robot_cell.c_str(), robot_occupancy.c_str());
}

void Explore::logMinSizeRejectedFrontiers(
    const frontier_exploration::FrontierSearchResult& search_result)
{
  for (size_t i = 0; i < search_result.min_size_rejected_frontiers.size(); ++i) {
    const auto& frontier = search_result.min_size_rejected_frontiers[i];
    double size_m = frontier.size * search_result.resolution;
    RCLCPP_INFO(
        logger_,
        "explore_frontier_min_size_filtered index=%lu centroid=%s "
        "size_cells=%u size_m=%.3f min_frontier_size=%.3f",
        i, formatPoint(frontier.centroid).c_str(), frontier.size, size_m,
        min_frontier_size_);
  }
}

void Explore::logAllBlacklistedDetails(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  constexpr static size_t tolerance_cells = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
  double resolution = costmap2d->getResolution();
  double tolerance_m = tolerance_cells * resolution;
  size_t limit = std::min<size_t>(frontiers.size(), 5);
  for (size_t i = 0; i < limit; ++i) {
    const auto& candidate = frontiers[i].centroid;
    double best_distance = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Point best_match;
    bool found = false;
    for (const auto& blacklisted : frontier_blacklist_) {
      double dx = candidate.x - blacklisted.x;
      double dy = candidate.y - blacklisted.y;
      double distance = sqrt(dx * dx + dy * dy);
      if (distance < best_distance) {
        best_distance = distance;
        best_match = blacklisted;
        found = true;
      }
    }
    std::string nearest = found ? formatPoint(best_match) : "none";
    RCLCPP_INFO(
        logger_,
        "explore_frontier_all_blacklisted_detail index=%lu candidate=%s "
        "nearest_blacklist=%s nearest_distance_m=%.3f tolerance_cells=%lu "
        "tolerance_m=%.3f",
        i, formatPoint(candidate).c_str(), nearest.c_str(), best_distance,
        tolerance_cells, tolerance_m);
  }
}

void Explore::logBlacklistAdded(const geometry_msgs::msg::Point& goal,
                                const std::string& reason)
{
  if (!debug_frontier_search_) {
    return;
  }
  RCLCPP_INFO(logger_,
              "explore_frontier_blacklist_added reason=%s target=%s "
              "blacklist_size=%lu",
              reason.c_str(), formatPoint(goal).c_str(),
              frontier_blacklist_.size());
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerance = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();
  double resolution = costmap2d->getResolution();
  double tolerance_m = tolerance * resolution;

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerance_m && y_diff < tolerance_m) {
      if (debug_frontier_search_) {
        RCLCPP_INFO(
            logger_,
            "explore_frontier_blacklist_match candidate=%s blacklist=%s "
            "resolution=%.3f tolerance_cells=%lu tolerance_m=%.3f "
            "x_diff_m=%.3f y_diff_m=%.3f",
            formatPoint(goal).c_str(), formatPoint(frontier_goal).c_str(),
            resolution, tolerance, tolerance_m, x_diff, y_diff);
      }
      return true;
    }
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal)
{
  const unsigned int nav2_error_code = result.result ?
      static_cast<unsigned int>(result.result->error_code) : 0U;
  const std::string nav2_error_msg = result.result ?
      result.result->error_msg : "";
  const bool blocked_by_preemption_guard =
      nav2_error_msg.find("goal blocked by preemption guard") !=
      std::string::npos;
  const bool preempted_by_newer_goal =
      nav2_error_msg.find("goal preempted by newer m-explore goal") !=
      std::string::npos;
  const bool proxy_soft_abort =
      blocked_by_preemption_guard || preempted_by_newer_goal;

  RCLCPP_INFO(logger_,
              "explore_nav2_result target=%s result_code=%s "
              "nav2_error_code=%u nav2_error_msg='%s'",
              formatPoint(frontier_goal).c_str(),
              result_code_name(result.code), nav2_error_code,
              nav2_error_msg.c_str());

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_DEBUG(logger_, "Goal was aborted");
      if (proxy_soft_abort) {
        if (blocked_by_preemption_guard) {
          prev_goal_.x = std::numeric_limits<double>::quiet_NaN();
          prev_goal_.y = std::numeric_limits<double>::quiet_NaN();
          prev_goal_.z = std::numeric_limits<double>::quiet_NaN();
        }
        RCLCPP_INFO(logger_,
                    "explore_frontier_blacklist_skipped "
                    "reason=proxy_soft_abort target=%s nav2_error_code=%u "
                    "nav2_error_msg='%s'",
                    formatPoint(frontier_goal).c_str(), nav2_error_code,
                    nav2_error_msg.c_str());
        return;
      }
      frontier_blacklist_.push_back(frontier_goal);
      logBlacklistAdded(frontier_goal, "nav2_aborted");
      RCLCPP_DEBUG(logger_, "Adding current goal to black list");
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
