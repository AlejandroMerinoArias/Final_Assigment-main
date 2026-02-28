#include "exploring/exploration_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <random>
#include <unordered_map>
#include <fstream>
#include <chrono>
#include <iomanip>

namespace planning {

// ===========================================================================
// Constructor
// ===========================================================================

ExplorationManager::ExplorationManager()
    : Node("exploration_manager"), min_frontier_size_(5),
      blacklist_radius_(1.5), min_z_(0.3), max_z_(50.0),
      max_step_distance_(1.0), cave_entrance_x_(-320.0),
      exploration_inflation_radius_(0.7), min_goal_distance_(3.0),
      num_candidates_(20), downsample_grid_(1.0), drone_speed_(1.0),
      vertical_penalty_weight_(2.0), vertical_penalty_deadband_(1.0),
      max_goal_z_delta_(2.0), max_goal_z_delta_stuck_bonus_(2.0),
      frontier_search_radius_(25.0),
      request_count_(0), consecutive_failures_(0),
      fail_stat_blacklist_(0.0), fail_stat_obstacle_(0.0), fail_stat_distance_(0.0),
      fail_stat_cave_(0.0), fail_stat_los_(0.0),
      min_stuck_failures_(10), stuck_fraction_threshold_(0.5), stuck_alpha_(0.25),
      los_short_range_threshold_(6.0),
      branch_commitment_weight_(0.35), max_branch_bonus_(0.35),
      goal_history_max_size_(60), recent_goal_hard_reject_radius_(2.2),
      recent_goal_hard_reject_count_(12), revisit_penalty_radius_(4.5),
      revisit_penalty_weight_(0.85),
      backtrack_reject_distance_(2.5),
      backtrack_penalty_factor_(0.55),
      unknown_volume_weight_(0.35),
      unknown_volume_radius_(3.5),
      unknown_sample_step_multiplier_(2.0),
      unknown_min_cave_depth_margin_(0.5),
      heading_update_alpha_(0.35),
      failed_region_merge_radius_(3.0),
      failed_region_base_reject_radius_(2.0),
      failed_region_reject_radius_gain_(0.7),
      failed_region_max_hits_(6) {
  // Declare and read parameters
  this->declare_parameter("min_frontier_size", min_frontier_size_);
  this->declare_parameter("blacklist_radius", blacklist_radius_);
  this->declare_parameter("min_z", min_z_);
  this->declare_parameter("max_z", max_z_);
  this->declare_parameter("max_step_distance", max_step_distance_);
  this->declare_parameter("cave_entrance_x", cave_entrance_x_);
  this->declare_parameter("exploration_inflation_radius", exploration_inflation_radius_);
  this->declare_parameter("min_goal_distance", min_goal_distance_);
  // Dai-Lite params
  this->declare_parameter("num_candidates", num_candidates_);
  this->declare_parameter("downsample_grid", downsample_grid_);
  this->declare_parameter("drone_speed", drone_speed_);
  this->declare_parameter("vertical_penalty_weight", vertical_penalty_weight_);
  this->declare_parameter("vertical_penalty_deadband", vertical_penalty_deadband_);
  this->declare_parameter("max_goal_z_delta", max_goal_z_delta_);
  this->declare_parameter("max_goal_z_delta_stuck_bonus", max_goal_z_delta_stuck_bonus_);
  this->declare_parameter("frontier_search_radius", frontier_search_radius_);
  // Stuck-mode parameters
  this->declare_parameter("min_stuck_failures", min_stuck_failures_);
  this->declare_parameter("stuck_fraction_threshold", stuck_fraction_threshold_);
  this->declare_parameter("stuck_alpha", stuck_alpha_);
  this->declare_parameter("los_short_range_threshold", los_short_range_threshold_);
  this->declare_parameter("branch_commitment_weight", branch_commitment_weight_);
  this->declare_parameter("max_branch_bonus", max_branch_bonus_);
  this->declare_parameter("goal_history_max_size", goal_history_max_size_);
  this->declare_parameter("recent_goal_hard_reject_radius", recent_goal_hard_reject_radius_);
  this->declare_parameter("recent_goal_hard_reject_count", recent_goal_hard_reject_count_);
  this->declare_parameter("revisit_penalty_radius", revisit_penalty_radius_);
  this->declare_parameter("revisit_penalty_weight", revisit_penalty_weight_);
  this->declare_parameter("backtrack_reject_distance", backtrack_reject_distance_);
  this->declare_parameter("backtrack_penalty_factor", backtrack_penalty_factor_);
  this->declare_parameter("unknown_volume_weight", unknown_volume_weight_);
  this->declare_parameter("unknown_volume_radius", unknown_volume_radius_);
  this->declare_parameter("unknown_sample_step_multiplier", unknown_sample_step_multiplier_);
  this->declare_parameter("unknown_min_cave_depth_margin", unknown_min_cave_depth_margin_);
  this->declare_parameter("heading_update_alpha", heading_update_alpha_);
  this->declare_parameter("failed_region_merge_radius", failed_region_merge_radius_);
  this->declare_parameter("failed_region_base_reject_radius", failed_region_base_reject_radius_);
  this->declare_parameter("failed_region_reject_radius_gain", failed_region_reject_radius_gain_);
  this->declare_parameter("failed_region_max_hits", failed_region_max_hits_);

  min_frontier_size_ = this->get_parameter("min_frontier_size").as_int();
  blacklist_radius_ = this->get_parameter("blacklist_radius").as_double();
  min_z_ = this->get_parameter("min_z").as_double();
  max_z_ = this->get_parameter("max_z").as_double();
  max_step_distance_ = this->get_parameter("max_step_distance").as_double();
  cave_entrance_x_ = this->get_parameter("cave_entrance_x").as_double();
  exploration_inflation_radius_ = this->get_parameter("exploration_inflation_radius").as_double();
  min_goal_distance_ = this->get_parameter("min_goal_distance").as_double();
  num_candidates_ = this->get_parameter("num_candidates").as_int();
  downsample_grid_ = this->get_parameter("downsample_grid").as_double();
  drone_speed_ = this->get_parameter("drone_speed").as_double();
  vertical_penalty_weight_ = this->get_parameter("vertical_penalty_weight").as_double();
  vertical_penalty_deadband_ = this->get_parameter("vertical_penalty_deadband").as_double();
  max_goal_z_delta_ = this->get_parameter("max_goal_z_delta").as_double();
  max_goal_z_delta_stuck_bonus_ = this->get_parameter("max_goal_z_delta_stuck_bonus").as_double();
  frontier_search_radius_ = this->get_parameter("frontier_search_radius").as_double();
  min_stuck_failures_ = this->get_parameter("min_stuck_failures").as_int();
  stuck_fraction_threshold_ = this->get_parameter("stuck_fraction_threshold").as_double();
  stuck_alpha_ = this->get_parameter("stuck_alpha").as_double();
  los_short_range_threshold_ = this->get_parameter("los_short_range_threshold").as_double();
  branch_commitment_weight_ = this->get_parameter("branch_commitment_weight").as_double();
  max_branch_bonus_ = this->get_parameter("max_branch_bonus").as_double();
  goal_history_max_size_ = this->get_parameter("goal_history_max_size").as_int();
  recent_goal_hard_reject_radius_ = this->get_parameter("recent_goal_hard_reject_radius").as_double();
  recent_goal_hard_reject_count_ = this->get_parameter("recent_goal_hard_reject_count").as_int();
  revisit_penalty_radius_ = this->get_parameter("revisit_penalty_radius").as_double();
  revisit_penalty_weight_ = this->get_parameter("revisit_penalty_weight").as_double();
  backtrack_reject_distance_ = this->get_parameter("backtrack_reject_distance").as_double();
  backtrack_penalty_factor_ = this->get_parameter("backtrack_penalty_factor").as_double();
  unknown_volume_weight_ = this->get_parameter("unknown_volume_weight").as_double();
  unknown_volume_radius_ = this->get_parameter("unknown_volume_radius").as_double();
  unknown_sample_step_multiplier_ = this->get_parameter("unknown_sample_step_multiplier").as_double();
  unknown_min_cave_depth_margin_ = this->get_parameter("unknown_min_cave_depth_margin").as_double();
  heading_update_alpha_ = this->get_parameter("heading_update_alpha").as_double();
  failed_region_merge_radius_ = this->get_parameter("failed_region_merge_radius").as_double();
  failed_region_base_reject_radius_ = this->get_parameter("failed_region_base_reject_radius").as_double();
  failed_region_reject_radius_gain_ = this->get_parameter("failed_region_reject_radius_gain").as_double();
  failed_region_max_hits_ = this->get_parameter("failed_region_max_hits").as_int();
  
  // Store base thresholds for stuck-mode relaxation
  base_min_goal_distance_ = min_goal_distance_;
  base_blacklist_radius_ = blacklist_radius_;
  base_obstacle_clearance_ = exploration_inflation_radius_;

  RCLCPP_INFO(this->get_logger(), "=== Dai-Lite Sampling Explorer Started ===");
  RCLCPP_INFO(this->get_logger(), "  num_candidates : %d", num_candidates_);
  RCLCPP_INFO(this->get_logger(), "  downsample_grid : %.2f m", downsample_grid_);
  RCLCPP_INFO(this->get_logger(), "  frontier_search_radius : %.1f m", frontier_search_radius_);
  RCLCPP_INFO(this->get_logger(), "  min_goal_distance : %.1f m", min_goal_distance_);
  RCLCPP_INFO(this->get_logger(), "  max_step_distance : %.1f m", max_step_distance_);
  RCLCPP_INFO(this->get_logger(), "  max_goal_z_delta : %.1f m (+%.1f m in stuck mode)",
              max_goal_z_delta_, max_goal_z_delta_stuck_bonus_);
  RCLCPP_INFO(this->get_logger(), "  inflation_radius : %.2f m", exploration_inflation_radius_);
  RCLCPP_INFO(this->get_logger(), "  cave_entrance_x : %.1f m", cave_entrance_x_);
  RCLCPP_INFO(this->get_logger(), "  Z range : %.1f - %.1f m", min_z_, max_z_);
  RCLCPP_INFO(this->get_logger(), "  branch_commitment_weight : %.2f", branch_commitment_weight_);
  RCLCPP_INFO(this->get_logger(), "  max_branch_bonus : %.2f", max_branch_bonus_);
  RCLCPP_INFO(this->get_logger(), "  recent_goal_hard_reject_radius : %.2f m", recent_goal_hard_reject_radius_);
  RCLCPP_INFO(this->get_logger(), "  revisit_penalty_radius : %.2f m", revisit_penalty_radius_);
  RCLCPP_INFO(this->get_logger(), "  backtrack_reject_distance : %.2f m", backtrack_reject_distance_);
  RCLCPP_INFO(this->get_logger(), "  unknown_volume_weight : %.2f", unknown_volume_weight_);
  RCLCPP_INFO(this->get_logger(), "  unknown_volume_radius : %.2f m", unknown_volume_radius_);
  RCLCPP_INFO(this->get_logger(), "  unknown_sample_step_multiplier : %.2f", unknown_sample_step_multiplier_);
  RCLCPP_INFO(this->get_logger(), "  unknown_min_cave_depth_margin : %.2f m", unknown_min_cave_depth_margin_);
  
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscribe to 3D OctoMap
  map_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ExplorationManager::map_callback, this,
                std::placeholders::_1));

  // Subscribe to blacklisted goals (optional, published by FSM)
  blacklist_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/exploration/blacklist_goal", 10,
      std::bind(&ExplorationManager::blacklist_callback, this,
                std::placeholders::_1));

  // Service server
  goal_service_ = this->create_service<exploring::srv::GetExplorationGoal>(
      "/exploration/get_goal",
      std::bind(&ExplorationManager::handle_get_goal, this,
                std::placeholders::_1, std::placeholders::_2));

  // Visualization / status publishers
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/exploration/frontiers_viz", 10);
  // Slice pub removed (not 2.5D anymore), but keeping map_ready
  map_ready_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/exploration/map_ready", 1);
  
  // Initialize performance logging CSV file
  perf_log_file_.open("/tmp/exploration_performance.csv");
  if (perf_log_file_.is_open()) {
    perf_log_file_ << "timestamp,request_id,total_time_ms,num_voxels_found,num_downsampled,"
                   << "num_candidates_evaluated,selected_utility,"
                   << "drone_x,drone_y,drone_z,goal_x,goal_y,goal_z\n";
    perf_log_file_.flush();
    RCLCPP_INFO(this->get_logger(), "Performance logging enabled: /tmp/exploration_performance.csv");
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to open performance log file");
  }
}

// ===========================================================================
// Callbacks
// ===========================================================================

void ExplorationManager::map_callback(
    const octomap_msgs::msg::Octomap::SharedPtr msg) {
  octomap::AbstractOcTree *tree = octomap_msgs::binaryMsgToMap(*msg);
  if (tree) {
    current_octomap_ =
        std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree));
    if (!current_octomap_) {
      RCLCPP_WARN(this->get_logger(), "Received OctoMap is not an OcTree!");
    } else {
      RCLCPP_DEBUG(this->get_logger(),
                   "Received 3D map (%zu nodes, res=%.3f m)",
                   current_octomap_->size(), current_octomap_->getResolution());

      // Mark map as "ready" once it has a minimum number of nodes. This allows
      // the FSM to wait at the entrance until a reasonably populated voxel
      // map is available before requesting exploration goals.
      if (!map_ready_ && current_octomap_->size() > 1000u) {
        map_ready_ = true;
        std_msgs::msg::Bool ready_msg;
        ready_msg.data = true;
        map_ready_pub_->publish(ready_msg);
        RCLCPP_INFO(this->get_logger(),
                    "Exploration map marked as READY (nodes=%zu).",
                    current_octomap_->size());
      }
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to deserialize OctoMap message.");
  }
}

void ExplorationManager::blacklist_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  octomap::point3d pt(static_cast<float>(msg->point.x),
                      static_cast<float>(msg->point.y),
                      static_cast<float>(msg->point.z));
  blacklisted_goals_.push_back(pt);

  bool merged = false;
  for (auto &region : failed_goal_regions_) {
    if (region.center.distance(pt) <= failed_region_merge_radius_) {
      region.hits = std::min(failed_region_max_hits_, region.hits + 1);
      const float alpha = 0.35f;
      region.center = octomap::point3d(
          (1.0f - alpha) * region.center.x() + alpha * pt.x(),
          (1.0f - alpha) * region.center.y() + alpha * pt.y(),
          (1.0f - alpha) * region.center.z() + alpha * pt.z());
      merged = true;
      break;
    }
  }
  if (!merged) {
    failed_goal_regions_.push_back(FailedGoalRegion{pt, 1});
  }

  RCLCPP_INFO(
      this->get_logger(),
      "Blacklisted goal received: (%.2f, %.2f, %.2f). Total blacklisted: %zu, failed regions: %zu",
      msg->point.x, msg->point.y, msg->point.z, blacklisted_goals_.size(),
      failed_goal_regions_.size());
}

// ===========================================================================
// Service handler
// ===========================================================================

void ExplorationManager::handle_get_goal(
    const std::shared_ptr<
        exploring::srv::GetExplorationGoal::Request> /* request */,
    std::shared_ptr<exploring::srv::GetExplorationGoal::Response> response) {
  
  // Performance timing
  auto start_time = std::chrono::high_resolution_clock::now();
  ++request_count_;
  
  RCLCPP_INFO(this->get_logger(), "--- Goal request #%d received ---", request_count_);

  // Guard: need a map
  if (!current_octomap_) {
    response->success = false;
    response->message = "No 3D map available yet.";
    RCLCPP_WARN(this->get_logger(), "No map available.");
    return;
  }

  // Get drone 3D position from TF
  double drone_x, drone_y, drone_z;
  if (!get_drone_position(drone_x, drone_y, drone_z)) {
    response->success = false;
    response->message = "Cannot get drone position from TF.";
    return;
  }
  octomap::point3d drone_pos(static_cast<float>(drone_x),
                             static_cast<float>(drone_y),
                             static_cast<float>(drone_z));

  // Lazy entrance init
  if (!has_entrance_pos_) {
    entrance_pos_ = drone_pos;
    has_entrance_pos_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Entrance reference set at (%.2f, %.2f, %.2f)",
                entrance_pos_.x(), entrance_pos_.y(), entrance_pos_.z());
  }

  
   // -----------------------------------------------------------------------
   // DAI-LITE SAMPLING PIPELINE
   // -----------------------------------------------------------------------

  // 1. Detect frontier voxels (3D)
  auto frontiers = detect_frontier_voxels(*current_octomap_);
  size_t num_voxels_found = frontiers.size();

  if (frontiers.empty()) {
    response->success = false;
    response->message = "No frontiers found.";
    consecutive_failures_++; // Increment to trigger radius expansion on next attempt
    return;
  }

  // 2. Downsample to grid
  auto downsampled = voxel_grid_downsample(frontiers, downsample_grid_);
  size_t num_downsampled = downsampled.size();

  // 3. Random sample candidates (Adaptive Scaling)
  // Increase candidates linearly with consecutive failures to brute-force through clutter
  int effective_candidates = num_candidates_ + (consecutive_failures_ * 10);
  
  auto candidates = sample_candidates(downsampled, effective_candidates);

  // 3.5. Compute stuck mode and apply dynamic filter relaxation
  auto [stuck_mode, severity] = computeStuckMode();
  
  // Compute relaxed thresholds based on stuck mode and severity
  double effective_obstacle_clearance = base_obstacle_clearance_;
  double effective_min_goal_distance = base_min_goal_distance_;
  double effective_blacklist_radius = base_blacklist_radius_;
  double effective_vertical_penalty_weight = vertical_penalty_weight_;
  bool relax_los_short_range = false;
  
  if (stuck_mode != StuckMode::NONE) {
    // Log stuck mode entry (once per mode change)
    static StuckMode last_logged_mode = StuckMode::NONE;
    static int last_logged_failures = 0;
    
    // Log on mode change or every 5 failures
    bool should_log = (stuck_mode != last_logged_mode) || 
                      (consecutive_failures_ - last_logged_failures >= 5);
    
    if (should_log) {
      const char* mode_name = "UNKNOWN";
      switch (stuck_mode) {
        case StuckMode::OBSTACLE: mode_name = "OBSTACLE"; break;
        case StuckMode::LOS: mode_name = "LOS"; break;
        case StuckMode::DISTANCE: mode_name = "DISTANCE"; break;
        case StuckMode::BLACKLIST: mode_name = "BLACKLIST"; break;
        default: break;
      }
      RCLCPP_WARN(this->get_logger(), 
                  "In %s stuck mode (severity=%.2f, failures=%d)",
                  mode_name, severity, consecutive_failures_);
      last_logged_mode = stuck_mode;
      last_logged_failures = consecutive_failures_;
    }
    
    // Apply mode-specific relaxation
    switch (stuck_mode) {
      case StuckMode::OBSTACLE:
        // Relax obstacle clearance: interpolate from 60% to 100% based on severity
        // At severity=0: 100%, at severity=1: 60%
        effective_obstacle_clearance = base_obstacle_clearance_ * (1.0 - 0.4 * severity);
        // Hard lower bound: never below 0.4m
        effective_obstacle_clearance = std::max(0.4, effective_obstacle_clearance);
        if (should_log) {
          RCLCPP_INFO(this->get_logger(), 
                      "  Relaxed obstacle clearance: %.2fm (base: %.2fm)",
                      effective_obstacle_clearance, base_obstacle_clearance_);
        }
        // Also soften vertical penalty to allow climbing/descending around
        // clutter in sloped cave sections.
        effective_vertical_penalty_weight =
            vertical_penalty_weight_ * (1.0 - 0.7 * severity);
        break;
        
      case StuckMode::LOS:
        // Relax LOS for short-range candidates
        relax_los_short_range = true;
        if (should_log) {
          RCLCPP_INFO(this->get_logger(), 
                      "  Relaxed LOS for candidates within %.1fm",
                      los_short_range_threshold_);
        }
        effective_vertical_penalty_weight =
            vertical_penalty_weight_ * (1.0 - 0.5 * severity);
        break;
        
      case StuckMode::DISTANCE:
        // Reduce min goal distance: interpolate from 33% to 100% based on severity
        // At severity=0: 100%, at severity=1: 33% (1.0m for base 3.0m)
        effective_min_goal_distance = base_min_goal_distance_ * (1.0 - 0.67 * severity);
        // Hard lower bound: never below 1.0m
        effective_min_goal_distance = std::max(1.0, effective_min_goal_distance);
        if (should_log) {
          RCLCPP_INFO(this->get_logger(), 
                      "  Relaxed min goal distance: %.2fm (base: %.2fm)",
                      effective_min_goal_distance, base_min_goal_distance_);
        }
        break;
        
      case StuckMode::BLACKLIST:
        // Shrink blacklist radius: interpolate from 50% to 100% based on severity
        // At severity=0: 100%, at severity=1: 50%
        effective_blacklist_radius = base_blacklist_radius_ * (1.0 - 0.5 * severity);
        // Hard lower bound: never below 0.75m
        effective_blacklist_radius = std::max(0.75, effective_blacklist_radius);
        if (should_log) {
          RCLCPP_INFO(this->get_logger(), 
                      "  Relaxed blacklist radius: %.2fm (base: %.2fm)",
                      effective_blacklist_radius, base_blacklist_radius_);
        }
        break;
        
      default:
        break;
    }
  }

  // 4. Evaluate candidates
  const auto forward_ref_xy = compute_forward_reference_xy(drone_pos);
  double best_utility = -1.0;
  FrontierCandidate *best_candidate = nullptr;
  int num_evaluated = 0;
  int filt_blacklist = 0, filt_obstacle = 0, filt_distance = 0, filt_cave = 0,
      filt_los = 0, filt_recent = 0, filt_backtrack = 0;

  for (auto &cand : candidates) {
    // Filter: repeated-failure regions (expanded around goals that failed many times)
    if (is_in_failed_region(cand.position)) {
      ++filt_blacklist;
      continue;
    }

    // Filter: Blacklist (with dynamic radius)
    bool blacklisted = false;
    for (const auto &p : blacklisted_goals_) {
      if (cand.position.distance(p) < effective_blacklist_radius) {
        blacklisted = true;
        break;
      }
    }
    if (blacklisted) {
      ++filt_blacklist;
      continue;
    }

    // Filter: Too close to obstacles (with dynamic clearance)
    if (is_too_close_to_obstacle(cand.position.x(), cand.position.y(), cand.position.z(), 
                                  effective_obstacle_clearance)) {
      ++filt_obstacle;
      continue;
    }

    // Filter: Too close to drone (with dynamic min distance)
    double dist_to_drone = cand.position.distance(drone_pos);
    if (dist_to_drone < effective_min_goal_distance) {
      ++filt_distance;
      continue;
    }

    // Filter: explicit backtracking rejection in normal mode.
    // If we already have a forward reference, drop candidates that go clearly
    // behind the current heading to avoid oscillating in intersections.
    if (forward_ref_xy && stuck_mode == StuckMode::NONE &&
        backtrack_reject_distance_ > 0.0) {
      const octomap::point3d delta = cand.position - drone_pos;
      const double projected = delta.x() * forward_ref_xy->x() +
                               delta.y() * forward_ref_xy->y();
      if (projected < -backtrack_reject_distance_) {
        ++filt_backtrack;
        continue;
      }
    }

    // Filter: Outside cave (behind entrance)
    if (cand.position.x() > cave_entrance_x_) {
      ++filt_cave;
      continue;
    }

    // Filter: hard reject around very recent goals to avoid hall-looping at bifurcations
    if (!recent_issued_goals_.empty() && recent_goal_hard_reject_count_ > 0 &&
        recent_goal_hard_reject_radius_ > 0.0) {
      const int hard_window = std::min<int>(recent_goal_hard_reject_count_, recent_issued_goals_.size());
      bool too_recent = false;
      for (int i = 0; i < hard_window; ++i) {
        const auto &recent = recent_issued_goals_[recent_issued_goals_.size() - 1 - i];
        if (cand.position.distance(recent) < recent_goal_hard_reject_radius_) {
          too_recent = true;
          break;
        }
      }
      if (too_recent) {
        ++filt_recent;
        continue;
      }
    }

    // Filter: Line-of-sight raycast (with dynamic relaxation for short range)
    {
      octomap::point3d direction = cand.position - drone_pos;
      double ray_length = direction.norm();
      direction.normalize();
      octomap::point3d hit_point;
      bool hit_obstacle = current_octomap_->castRay(drone_pos, direction, hit_point,
                                                     true /* ignoreUnknown */, ray_length);
      
      // In LOS stuck mode, relax for short-range candidates
      if (hit_obstacle) {
        bool should_reject = true;
        if (relax_los_short_range && dist_to_drone <= los_short_range_threshold_) {
          // Allow if hit is very close to the candidate (within last 1m of ray)
          double hit_to_cand_dist = hit_point.distance(cand.position);
          if (hit_to_cand_dist < 1.0) {
            should_reject = false;  // Grazing hit, allow it
          }
        }
        
        if (should_reject) {
          ++filt_los;
          continue;
        }
      }
    }

    // Calculate utility
    cand.utility = evaluate_candidate(cand.position, drone_pos,
                                      effective_vertical_penalty_weight,
                                      forward_ref_xy);
    num_evaluated++;

    if (cand.utility > best_utility) {
      best_utility = cand.utility;
      best_candidate = &cand;
    }
  }

  // 5. Select best (Adaptive Utility Threshold)
  if (!best_candidate) {
    RCLCPP_WARN(this->get_logger(),
                "All %zu candidates filtered: blacklist=%d, obstacle=%d, distance=%d, cave=%d, los=%d, recent=%d",
                candidates.size(), filt_blacklist, filt_obstacle, filt_distance,
                filt_cave, filt_los, filt_recent + filt_backtrack);
    response->success = false;
    response->message = "No valid candidates found after filtering.";
    consecutive_failures_++; // Increment failure count to try harder next time
    updateFailureStats(filt_blacklist, filt_obstacle, filt_distance, filt_cave, filt_los, candidates.size());
    return;
  }
  
  // Backtracking Prevention: Check quality threshold
  // Initial threshold 0.1 rejects pure backward moves.
  // Decays by 0.01 per failure --> after 10 failures, threshold is 0.0 (accept anything)
  double quality_threshold = std::max(0.0, 0.1 - (consecutive_failures_ * 0.01));

  // If we are already in a stuck mode, relax backtracking protection completely
  if (consecutive_failures_ >= min_stuck_failures_ && stuck_mode != StuckMode::NONE) {
    quality_threshold = 0.0;
  }
  
  if (best_utility < quality_threshold) {
     RCLCPP_WARN(this->get_logger(),
                 "Rejecting low utility goal (%.4f < %.4f) to prevent premature backtracking. Failure count: %d",
                 best_utility, quality_threshold, consecutive_failures_);
     
     // Treat this as a failure to trigger adaptive scaling (more samples next time)
     consecutive_failures_++;
     // Feed this into stuck statistics as a \"distance/backtracking\"-type failure
     updateFailureStats(0, 0, 1, 0, 0, 1);
     response->success = false;
     response->message = "Candidate utility too low (backtracking protection).";
     return;
  }
  
  // Reset failure count and stuck stats on success
  if (consecutive_failures_ >= min_stuck_failures_) {
    RCLCPP_INFO(this->get_logger(), 
                "Exiting stuck mode - found valid goal after %d failures", 
                consecutive_failures_);
  }
  consecutive_failures_ = 0;
  resetStuckStats();

  // Build response
  geometry_msgs::msg::Point goal_point;
  goal_point.x = best_candidate->position.x();
  goal_point.y = best_candidate->position.y();
  goal_point.z = best_candidate->position.z();

  // Clamp goal altitude around the drone.
  // In stuck mode, allow a wider vertical window to escape floor/ceiling traps.
  double max_z_delta = max_goal_z_delta_;
  if (stuck_mode != StuckMode::NONE) {
    max_z_delta += max_goal_z_delta_stuck_bonus_ * severity;
  }
  if (goal_point.z > drone_z + max_z_delta) {
    goal_point.z = drone_z + max_z_delta;
  } else if (goal_point.z < drone_z - max_z_delta) {
    goal_point.z = drone_z - max_z_delta;
  }

  // Enforce Max Step Distance
  double dist_goal = best_candidate->position.distance(drone_pos);
  if (max_step_distance_ > 0.0 && dist_goal > max_step_distance_) {
    double scale = max_step_distance_ / dist_goal;
    goal_point.x = drone_x + (goal_point.x - drone_x) * scale;
    goal_point.y = drone_y + (goal_point.y - drone_y) * scale;
    goal_point.z = drone_z + (goal_point.z - drone_z) * scale;
    RCLCPP_INFO(this->get_logger(), "Goal clamped to %.2fm step (was %.2fm)", max_step_distance_, dist_goal);
  }

  // Remember issued goal to discourage revisits in junction halls.
  recent_issued_goals_.emplace_back(static_cast<float>(goal_point.x),
                                    static_cast<float>(goal_point.y),
                                    static_cast<float>(goal_point.z));
  while (goal_history_max_size_ > 0 &&
         static_cast<int>(recent_issued_goals_.size()) > goal_history_max_size_) {
    recent_issued_goals_.pop_front();
  }

  response->success = true;
  response->goal.header.frame_id = "world";
  response->goal.header.stamp = this->now();
  response->goal.pose.position = goal_point;
  response->goal.pose.orientation.w = 1.0;
  response->message = "Dai-Lite goal found.";

  // Update preferred heading for future branch commitment.
  // This helps avoid left-right oscillations at cave bifurcations.
  {
    const double hx = goal_point.x - drone_x;
    const double hy = goal_point.y - drone_y;
    const double h_norm = std::hypot(hx, hy);
    if (h_norm > 1e-3) {
      const double nx = hx / h_norm;
      const double ny = hy / h_norm;
      if (!preferred_heading_xy_) {
        preferred_heading_xy_ = octomap::point3d(static_cast<float>(nx),
                                                 static_cast<float>(ny),
                                                 0.0f);
      } else {
        const double a = std::clamp(heading_update_alpha_, 0.0, 1.0);
        const double mx = (1.0 - a) * preferred_heading_xy_->x() + a * nx;
        const double my = (1.0 - a) * preferred_heading_xy_->y() + a * ny;
        const double m_norm = std::hypot(mx, my);
        if (m_norm > 1e-3) {
          preferred_heading_xy_ = octomap::point3d(static_cast<float>(mx / m_norm),
                                                   static_cast<float>(my / m_norm),
                                                   0.0f);
        }
      }
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Goal: (%.2f, %.2f, %.2f) | U=%.3f | Candidates: %d/%zu/%zu | Dist: %.2fm",
              goal_point.x, goal_point.y, goal_point.z,
              best_utility, num_evaluated, candidates.size(), num_voxels_found, dist_goal);

  // Visualize
  publish_visualization(candidates, best_candidate, drone_x, drone_y, drone_z);

  // Log Perf
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  RCLCPP_INFO(this->get_logger(),
              "[PERF] Request #%d completed in %ld ms | %zu voxels -> %zu downsampled -> %d evaluated",
              request_count_, duration_ms, num_voxels_found, num_downsampled, num_evaluated);

  if (perf_log_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);
    
    perf_log_file_ << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << ","
                   << request_count_ << ","
                   << duration_ms << ","
                   << num_voxels_found << ","
                   << num_downsampled << ","
                   << num_evaluated << ","
                   << std::fixed << std::setprecision(3) << best_utility << ","
                   << std::setprecision(2) << drone_x << "," << drone_y << "," << drone_z << ","
                   << goal_point.x << "," << goal_point.y << "," << goal_point.z << "\n";
    perf_log_file_.flush();
  }
}

// ===========================================================================
// Helpers
// ===========================================================================

bool ExplorationManager::is_blacklisted(const octomap::point3d &goal) const {
  for (const auto &p : blacklisted_goals_) {
    if (goal.distance(p) < blacklist_radius_) {
      return true;
    }
  }
  return false;
}

bool ExplorationManager::get_drone_position(double &x, double &y, double &z) {
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("world", "body", tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    z = transform.transform.translation.z;
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return false;
  }
}

// ===========================================================================
// Dai-Lite Implementation
// ===========================================================================

std::vector<octomap::point3d> ExplorationManager::detect_frontier_voxels(const octomap::OcTree &tree) {
  std::vector<octomap::point3d> frontiers;
  
  // Get drone pos for bounds
  double dx, dy, dz;
  if (!get_drone_position(dx, dy, dz)) return frontiers;
  
  // Dynamic search radius: increase by 10m every 5 consecutive failures
  constexpr int kFailuresPerRadiusStep = 5;
  constexpr double kRadiusStepMeters = 10.0;
  constexpr double kMaxRadiusBonusMeters = 50.0;
  
  double radius_bonus = (consecutive_failures_ / kFailuresPerRadiusStep) * kRadiusStepMeters;
  radius_bonus = std::min(radius_bonus, kMaxRadiusBonusMeters);
  double r = frontier_search_radius_ + radius_bonus;
  
  if (radius_bonus > 0.0) {
    RCLCPP_INFO(this->get_logger(), 
                "Dynamic radius active: %.1f m (default %.1f + bonus %.1f) after %d failures",
                r, frontier_search_radius_, radius_bonus, consecutive_failures_);
  }
  
  // Create search BBX centered on drone (use altitude ± search radius, clamped to navigable range)
  double z_lo = std::max(min_z_, dz - r);
  double z_hi = std::min(max_z_, dz + r);
  octomap::point3d bbx_min(dx - r, dy - r, z_lo);
  octomap::point3d bbx_max(dx + r, dy + r, z_hi);

  // Iterate leaves in BBX
  for (auto it = tree.begin_leafs_bbx(bbx_min, bbx_max);
       it != tree.end_leafs_bbx(); ++it) {
    
    // Only check FREE voxels
    if (tree.isNodeOccupied(*it)) continue;

    // Check neighbors (6-connected)
    // If any neighbor is UNKNOWN (null node), then it is a frontier
    bool is_frontier = false;
    double x = it.getX();
    double y = it.getY();
    double z = it.getZ();
    double res = tree.getResolution();
    
    // Neighbors: +/- x, y, z
    double offsets[6][3] = {
      {res, 0, 0}, {-res, 0, 0},
      {0, res, 0}, {0, -res, 0},
      {0, 0, res}, {0, 0, -res}
    };

    for (int i=0; i<6; ++i) {
      octomap::point3d neighbor(x + offsets[i][0], y + offsets[i][1], z + offsets[i][2]);
      octomap::OcTreeNode* n_node = tree.search(neighbor);
      if (!n_node) {
        // Unknown space!
        is_frontier = true;
        break; 
      }
    }

    if (is_frontier) {
      frontiers.emplace_back(x, y, z);
    }
  }
  return frontiers;
}

std::vector<octomap::point3d> ExplorationManager::voxel_grid_downsample(
    const std::vector<octomap::point3d> &points, double grid_size) {
  
  struct GridKey {
    long x, y, z;
    bool operator==(const GridKey &o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };
  struct GridHash {
    size_t operator()(const GridKey &k) const {
      return std::hash<long>()(k.x) ^ (std::hash<long>()(k.y) << 1) ^ (std::hash<long>()(k.z) << 2);
    }
  };

  std::unordered_map<GridKey, octomap::point3d, GridHash> bin;
  for (const auto &p : points) {
    GridKey k;
    k.x = static_cast<long>(std::floor(p.x() / grid_size));
    k.y = static_cast<long>(std::floor(p.y() / grid_size));
    k.z = static_cast<long>(std::floor(p.z() / grid_size));
    bin[k] = p; // Overwrite - simple subsampling
  }

  std::vector<octomap::point3d> out;
  out.reserve(bin.size());
  for (const auto &kv : bin) {
    out.push_back(kv.second);
  }
  return out;
}

std::vector<FrontierCandidate> ExplorationManager::sample_candidates(
    const std::vector<octomap::point3d> &points, int num) {
  
  std::vector<octomap::point3d> pool = points;
  std::vector<FrontierCandidate> candidates;

  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(pool.begin(), pool.end(), g);

  int n = std::min((int)pool.size(), num);
  for (int i=0; i<n; ++i) {
    FrontierCandidate c;
    c.position = pool[i];
    candidates.push_back(c);
  }
  return candidates;
}

double ExplorationManager::evaluate_candidate(
    const octomap::point3d &candidate, const octomap::point3d &drone_pos,
    double vertical_penalty_weight,
    const std::optional<octomap::point3d> &forward_ref_xy) const {

  double dx = candidate.x() - drone_pos.x();
  double dy = candidate.y() - drone_pos.y();
  double dz = candidate.z() - drone_pos.z();
  double dist_3d = std::sqrt(dx*dx + dy*dy + dz*dz);

  // Time to travel
  double time = dist_3d / drone_speed_;
  if (time < 0.1) time = 0.1;

  // Z-penalty (virtual time dilation for altitude change).
  // Apply deadband to avoid over-penalizing useful slope-following moves.
  const double dz_for_penalty =
      std::max(0.0, std::abs(dz) - vertical_penalty_deadband_);
  double z_penalty = 1.0 + vertical_penalty_weight * dz_for_penalty;
  
  double effective_time = time * z_penalty;

  // Base utility = 1 / time
  double utility = 1.0 / effective_time;

  // Directional bias using an adaptive forward reference. This avoids the
  // brittle world-X-only assumption, which breaks at turns/intersections.
  if (forward_ref_xy) {
    const double progress = dx * forward_ref_xy->x() + dy * forward_ref_xy->y();
    if (progress >= 0.0) {
      // Reward progress along current cave heading.
      const double forward_bonus = 1.0 + std::min(3.0, progress / 5.0);
      utility *= forward_bonus;
    } else {
      // Backward moves are discouraged but not zeroed to keep recovery possible.
      utility *= backtrack_penalty_factor_;
    }

  // Penalize very lateral moves relative to the forward projection.
    const double lateral_sq = std::max(0.0, dx * dx + dy * dy - progress * progress);
    const double lateral = std::sqrt(lateral_sq);
    const double forward_mag = std::max(1.0, std::abs(progress));
    const double lateral_ratio = lateral / forward_mag;
    if (lateral_ratio > 1.0) {
      utility *= 1.0 / lateral_ratio;
    }
  }

  // Branch commitment: when a preferred heading exists, reward candidates
  // that continue in the same XY direction to reduce branch-switch oscillation.
  if (branch_commitment_weight_ > 0.0 && preferred_heading_xy_) {
    const double cand_xy_norm = std::hypot(dx, dy);
    if (cand_xy_norm > 1e-3) {
      const double cand_dir_x = dx / cand_xy_norm;
      const double cand_dir_y = dy / cand_xy_norm;
      const double dot = cand_dir_x * preferred_heading_xy_->x() +
                         cand_dir_y * preferred_heading_xy_->y();
      const double alignment = std::max(0.0, dot);  // only reward forward alignment
      const double bonus = 1.0 + max_branch_bonus_ * branch_commitment_weight_ * alignment;
      utility *= bonus;
    }
  }

  // Novelty term: reduce utility near recently issued goals to avoid loops
  // around bifurcation halls where frontiers stay partially visible.
  utility *= compute_revisit_factor(candidate);

  // Unknown-volume bonus: reward candidates that can reveal nearby unknown
  // space, but only when both the candidate and counted samples are strictly
  // inside the cave-side gate.
  if (unknown_volume_weight_ > 0.0) {
    const double unknown_ratio = compute_local_unknown_ratio(candidate);
    utility *= (1.0 + unknown_volume_weight_ * unknown_ratio);
  }
  
  return utility;
}

double ExplorationManager::compute_local_unknown_ratio(
    const octomap::point3d &candidate) const {
  if (!current_octomap_ || unknown_volume_radius_ <= 0.0) {
    return 0.0;
  }

  // Strict gate: never add unknown bonus near/outside entrance.
  const double strict_cave_gate_x =
      cave_entrance_x_ - std::max(0.0, unknown_min_cave_depth_margin_);
  if (candidate.x() > strict_cave_gate_x) {
    return 0.0;
  }

  const double res = current_octomap_->getResolution();
  if (res <= 0.0) {
    return 0.0;
  }
  const double step =
      std::max(res, res * std::max(1.0, unknown_sample_step_multiplier_));
  const double r = unknown_volume_radius_;

  int samples_in_sphere = 0;
  int unknown_samples = 0;

  for (double dx = -r; dx <= r; dx += step) {
    for (double dy = -r; dy <= r; dy += step) {
      for (double dz = -r; dz <= r; dz += step) {
        const double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 > r * r) {
          continue;
        }

        const double sx = candidate.x() + dx;
        if (sx > strict_cave_gate_x) {
          continue;
        }

        const double sy = candidate.y() + dy;
        const double sz = candidate.z() + dz;
        if (sz < min_z_ || sz > max_z_) {
          continue;
        }

        ++samples_in_sphere;
        octomap::point3d sample_point(sx, sy, sz);
        if (!current_octomap_->search(sample_point)) {
          ++unknown_samples;
        }
      }
    }
  }

  if (samples_in_sphere == 0) {
    return 0.0;
  }

  const double ratio = static_cast<double>(unknown_samples) /
                       static_cast<double>(samples_in_sphere);
  return std::clamp(ratio, 0.0, 1.0);
}

std::optional<octomap::point3d>
ExplorationManager::compute_forward_reference_xy(
    const octomap::point3d &drone_pos) const {
  if (preferred_heading_xy_) {
    const double n = std::hypot(preferred_heading_xy_->x(), preferred_heading_xy_->y());
    if (n > 1e-3) {
      return octomap::point3d(static_cast<float>(preferred_heading_xy_->x() / n),
                              static_cast<float>(preferred_heading_xy_->y() / n),
                              0.0f);
    }
  }

  if (has_entrance_pos_) {
    const double ex = drone_pos.x() - entrance_pos_.x();
    const double ey = drone_pos.y() - entrance_pos_.y();
    const double n = std::hypot(ex, ey);
    if (n > 1e-3) {
      return octomap::point3d(static_cast<float>(ex / n),
                              static_cast<float>(ey / n),
                              0.0f);
    }
  }

  return std::nullopt;
}

double ExplorationManager::compute_revisit_factor(const octomap::point3d &candidate) const {
  if (recent_issued_goals_.empty() || revisit_penalty_radius_ <= 0.0 ||
      revisit_penalty_weight_ <= 0.0) {
    return 1.0;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (const auto &p : recent_issued_goals_) {
    min_dist = std::min(min_dist, static_cast<double>(candidate.distance(p)));
  }

  const double sigma = std::max(0.5, revisit_penalty_radius_);
  const double proximity = std::exp(-(min_dist * min_dist) / (2.0 * sigma * sigma));
  const double factor = 1.0 - revisit_penalty_weight_ * proximity;
  return std::clamp(factor, 0.05, 1.0);
}

bool ExplorationManager::is_in_failed_region(const octomap::point3d &candidate) const {
  if (failed_goal_regions_.empty()) {
    return false;
  }

  for (const auto &region : failed_goal_regions_) {
    const double reject_radius = failed_region_base_reject_radius_ +
                                 failed_region_reject_radius_gain_ *
                                     static_cast<double>(std::max(0, region.hits - 1));
    if (candidate.distance(region.center) < reject_radius) {
      return true;
    }
  }
  return false;
}

bool ExplorationManager::is_too_close_to_obstacle(double x, double y, double z) const {
  return is_too_close_to_obstacle(x, y, z, exploration_inflation_radius_);
}

bool ExplorationManager::is_too_close_to_obstacle(double x, double y, double z, double clearance_radius) const {
  if (!current_octomap_) return false;

  const double r = clearance_radius;
  
  // Iterate bounding box around point
  octomap::point3d bbx_min(x-r, y-r, z-r);
  octomap::point3d bbx_max(x+r, y+r, z+r);

  for(auto it = current_octomap_->begin_leafs_bbx(bbx_min, bbx_max); it != current_octomap_->end_leafs_bbx(); ++it) {
    if (current_octomap_->isNodeOccupied(*it)) {
      if (it.getCoordinate().distance(octomap::point3d(x,y,z)) < r) {
        return true;
      }
    }
  }
  return false;
}

void ExplorationManager::publish_visualization(const std::vector<FrontierCandidate> &candidates,
                                               const FrontierCandidate *best,
                                               double drone_x, double drone_y, double drone_z) {
  visualization_msgs::msg::MarkerArray markers;
  
  // 1. Candidates
  visualization_msgs::msg::Marker points;
  points.header.frame_id = "world";
  points.header.stamp = this->now();
  points.ns = "candidates";
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.3;
  points.scale.y = 0.3;
  points.scale.z = 0.3;
  points.color.a = 0.8;
  points.color.r = 0.0;
  points.color.g = 1.0;
  points.color.b = 1.0; // Cyan

  for (const auto &c : candidates) {
    geometry_msgs::msg::Point p;
    p.x = c.position.x();
    p.y = c.position.y();
    p.z = c.position.z();
    points.points.push_back(p);
  }
  markers.markers.push_back(points);

  // 2. Best goal
  if (best) {
    visualization_msgs::msg::Marker best_m;
    best_m.header = points.header;
    best_m.ns = "best_goal";
    best_m.id = 1;
    best_m.type = visualization_msgs::msg::Marker::SPHERE;
    best_m.action = visualization_msgs::msg::Marker::ADD;
    best_m.scale.x = 0.6;
    best_m.scale.y = 0.6;
    best_m.scale.z = 0.6;
    best_m.color.a = 1.0;
    best_m.color.r = 0.0;
    best_m.color.g = 1.0;
    best_m.color.b = 0.0; // Green
    best_m.pose.position.x = best->position.x();
    best_m.pose.position.y = best->position.y();
    best_m.pose.position.z = best->position.z();
    best_m.pose.orientation.w = 1.0;
    markers.markers.push_back(best_m);
  }

  viz_pub_->publish(markers);
}

// ===========================================================================
// Stuck-mode detection and adaptive filter relaxation
// ===========================================================================

void ExplorationManager::updateFailureStats(int filt_blacklist, int filt_obstacle, 
                                            int filt_distance, int filt_cave, 
                                            int filt_los, int total_candidates) {
  if (total_candidates == 0) return;
  
  // Compute current fractions
  double frac_blacklist = static_cast<double>(filt_blacklist) / total_candidates;
  double frac_obstacle = static_cast<double>(filt_obstacle) / total_candidates;
  double frac_distance = static_cast<double>(filt_distance) / total_candidates;
  double frac_cave = static_cast<double>(filt_cave) / total_candidates;
  double frac_los = static_cast<double>(filt_los) / total_candidates;
  
  // Update exponentially weighted moving average
  fail_stat_blacklist_ = stuck_alpha_ * frac_blacklist + (1.0 - stuck_alpha_) * fail_stat_blacklist_;
  fail_stat_obstacle_ = stuck_alpha_ * frac_obstacle + (1.0 - stuck_alpha_) * fail_stat_obstacle_;
  fail_stat_distance_ = stuck_alpha_ * frac_distance + (1.0 - stuck_alpha_) * fail_stat_distance_;
  fail_stat_cave_ = stuck_alpha_ * frac_cave + (1.0 - stuck_alpha_) * fail_stat_cave_;
  fail_stat_los_ = stuck_alpha_ * frac_los + (1.0 - stuck_alpha_) * fail_stat_los_;
}

std::pair<StuckMode, double> ExplorationManager::computeStuckMode() const {
  // Not stuck if below threshold
  if (consecutive_failures_ < min_stuck_failures_) {
    return {StuckMode::NONE, 0.0};
  }
  
  // Compute severity factor: 0 at min_stuck_failures_, 1 at 20+ failures
  double severity = std::min(1.0, static_cast<double>(consecutive_failures_ - min_stuck_failures_) / 10.0);
  
  // Find dominant filter reason
  double max_stat = std::max({fail_stat_blacklist_, fail_stat_obstacle_, 
                              fail_stat_distance_, fail_stat_cave_, fail_stat_los_});
  
  // Must exceed threshold to be considered stuck
  if (max_stat < stuck_fraction_threshold_) {
    return {StuckMode::NONE, 0.0};
  }
  
  // Determine which filter is dominant
  if (max_stat == fail_stat_obstacle_) {
    return {StuckMode::OBSTACLE, severity};
  } else if (max_stat == fail_stat_los_) {
    return {StuckMode::LOS, severity};
  } else if (max_stat == fail_stat_distance_) {
    return {StuckMode::DISTANCE, severity};
  } else if (max_stat == fail_stat_blacklist_) {
    return {StuckMode::BLACKLIST, severity};
  }
  
  return {StuckMode::NONE, 0.0};
}

void ExplorationManager::resetStuckStats() {
  fail_stat_blacklist_ = 0.0;
  fail_stat_obstacle_ = 0.0;
  fail_stat_distance_ = 0.0;
  fail_stat_cave_ = 0.0;
  fail_stat_los_ = 0.0;
}

} // namespace planning
