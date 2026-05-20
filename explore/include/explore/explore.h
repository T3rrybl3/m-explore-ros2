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
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using ComputePathGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;

private:
  struct ReachabilityResult {
    bool reachable = false;
    bool definitive = false;
    size_t path_poses = 0;
    std::string reason = "not_checked";
  };

  struct CachedReachabilityRejection {
    geometry_msgs::msg::Point target;
    uint64_t map_sequence = 0;
    rclcpp::Time inserted_at;
    size_t path_poses = 0;
    std::string reason;
  };

  /**
   * @brief  Make a global plan
   */
  void makePlan();

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);
  void visualizeDebugFrontiers(
      const frontier_exploration::FrontierSearchResult& search_result,
      const std::vector<frontier_exploration::Frontier>& blacklisted_frontiers,
      const frontier_exploration::Frontier* selected_frontier);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      raw_frontier_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      filtered_frontier_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      blacklisted_frontier_marker_publisher_;

  /**
    * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
    */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  rclcpp::CallbackGroup::SharedPtr reachability_callback_group_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr
      compute_path_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  rclcpp::TimerBase::SharedPtr unknown_robot_cell_retry_timer_;
  rclcpp::TimerBase::SharedPtr stale_map_blacklist_guard_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  double min_frontier_size_;
  bool visualize_;
  bool return_to_init_;
  bool debug_frontier_search_;
  bool publish_debug_frontiers_;
  bool enable_unknown_robot_cell_frontier_retry_;
  double unknown_robot_cell_retry_timeout_sec_;
  int unknown_robot_cell_retry_min_map_updates_;
  bool enable_reachability_biased_top_k_;
  int reachability_top_k_;
  int reachability_max_candidate_checks_;
  double reachability_precheck_timeout_sec_;
  int reachability_min_path_poses_;
  bool debug_reachability_selection_;
  double reachability_rejection_cache_timeout_sec_ = 10.0;
  bool enable_stale_map_blacklist_guard_;
  int stale_map_blacklist_guard_min_remaining_frontiers_;
  int stale_map_blacklist_guard_max_blacklists_per_map_update_;
  int stale_map_blacklist_guard_min_map_updates_;
  double stale_map_blacklist_guard_timeout_sec_;
  std::string robot_base_frame_;
  std::string goal_stamp_mode_;
  bool resuming_ = false;
  bool unknown_robot_cell_retry_active_ = false;
  uint64_t unknown_robot_cell_retry_start_sequence_ = 0;
  rclcpp::Time unknown_robot_cell_retry_start_time_;
  bool stale_map_blacklist_guard_active_ = false;
  uint64_t stale_map_blacklist_guard_start_sequence_ = 0;
  rclcpp::Time stale_map_blacklist_guard_start_time_;
  uint64_t blacklist_guard_current_sequence_ = 0;
  int blacklist_guard_allowed_on_current_sequence_ = 0;
  size_t last_plan_remaining_frontiers_ = 0;
  std::vector<CachedReachabilityRejection> reachability_rejection_cache_;

  void logFrontierDebug(
      const frontier_exploration::FrontierSearchResult& search_result,
      size_t blacklist_filtered, size_t remaining,
      const frontier_exploration::Frontier* selected_frontier,
      const std::string& stop_reason, int selected_rank = -1,
      double selected_distance_m = std::numeric_limits<double>::quiet_NaN(),
      size_t reachability_checked = 0, size_t reachability_rejected = 0);
  void logMinSizeRejectedFrontiers(
      const frontier_exploration::FrontierSearchResult& search_result);
  void logAllBlacklistedDetails(
      const std::vector<frontier_exploration::Frontier>& frontiers);
  void logBlacklistAdded(const geometry_msgs::msg::Point& goal,
                         const std::string& reason);
  void publishFrontierDebugMarkers(
      const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
      const std::vector<frontier_exploration::Frontier>& frontiers,
      const std::string& marker_namespace,
      const std_msgs::msg::ColorRGBA& point_color,
      const std_msgs::msg::ColorRGBA& centroid_color,
      const frontier_exploration::Frontier* selected_frontier = nullptr);
  bool shouldDeferUnknownRobotCellTerminalDecision(
      const frontier_exploration::FrontierSearchResult& search_result,
      const std::string& stop_reason);
  bool unknownRobotCellRetryReady(
      const Costmap2DClient::MapUpdateMetadata& metadata);
  uint64_t unknownRobotCellRetryMapUpdates(
      const Costmap2DClient::MapUpdateMetadata& metadata) const;
  void ensureUnknownRobotCellRetryTimer();
  void clearUnknownRobotCellRetry(const std::string& reason);
  bool shouldDeferBlacklist(const geometry_msgs::msg::Point& target,
                            const std::string& reason,
                            size_t remaining_frontiers);
  ReachabilityResult checkFrontierReachability(
      const frontier_exploration::Frontier& frontier,
      size_t candidate_rank);
  void logReachabilityCheck(
      const frontier_exploration::Frontier& frontier, size_t candidate_rank,
      const ReachabilityResult& result);
  void logReachabilitySelected(const frontier_exploration::Frontier& frontier,
                               size_t selected_rank);
  void logReachabilityCachedRejection(
      const frontier_exploration::Frontier& frontier, size_t candidate_rank,
      const CachedReachabilityRejection& cached_rejection);
  void logReachabilityFallback(const std::string& reason,
                               size_t candidate_rank = 0);
  void logReachabilityExpand(size_t initial_k, size_t max_checks,
                             size_t remaining);
  void logReachabilityNoReachable(size_t checked, size_t remaining,
                                  uint64_t map_sequence);
  void cacheReachabilityRejection(
      const frontier_exploration::Frontier& frontier,
      const ReachabilityResult& result, uint64_t map_sequence);
  bool cachedReachabilityRejection(
      const frontier_exploration::Frontier& frontier, uint64_t map_sequence,
      CachedReachabilityRejection& cached_rejection);
  void pruneReachabilityRejectionCache(uint64_t map_sequence);
  double distanceFromRobot(const geometry_msgs::msg::Point& point);
  void recordBlacklistAllowedForCurrentMap();
  bool staleMapBlacklistGuardReady(
      const Costmap2DClient::MapUpdateMetadata& metadata) const;
  void ensureStaleMapBlacklistGuardTimer();
  void clearStaleMapBlacklistGuard(const std::string& reason);
  std::string formatPoint(const geometry_msgs::msg::Point& point) const;
};
}  // namespace explore

#endif
