// Copyright (c) 2021 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include <limits>
#include <fstream>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/file_logger.hpp"

#include "nav2_behavior_tree/plugins/action/remove_passed_goals_action.hpp"

namespace nav2_behavior_tree
{

RemovePassedGoals::RemovePassedGoals(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  sharp_turn_(30.0),
  viapoint_achieved_radius_(0.5),
  viapoint_achieved_radius_sharp_turn_(0.5)
{
  getInput("sharp_turn", sharp_turn_);
  getInput("radius", viapoint_achieved_radius_);
  getInput("radius_sharp_turn", viapoint_achieved_radius_sharp_turn_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("timeout_threshold", timeout_threshold_);
  getInput("start_timeout_radius", start_timeout_radius_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  static bool first = true;
  // Static instance to maintain state across ticks
  static nav2_util::FileLogger file_logger; 

  Goals goal_poses;
  getInput("input_goals", goal_poses);

  if (goal_poses.empty()) {
    setOutput("output_goals", goal_poses);
    return BT::NodeStatus::SUCCESS;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  // --- 1. Log Initial State (Internalized loop in Logger) ---
  if (first) {
    file_logger.logInitialState(goal_poses, "goals_removed_log.txt");
    first = false;
  }

  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  rclcpp::Time now = node->now();
  
  // --- 2. Timeout Logic ---
  if (timeout_threshold_ > 0.0 && goal_poses.size() > 1) {
    double dist_to_first = nav2_util::geometry_utils::euclidean_distance(
      goal_poses[0].pose, current_pose.pose);

    if (dist_to_first < start_timeout_radius_) {
      if (!timer_active_) {
        start_time_ = now;
        timer_active_ = true;
      } else {
        double elapsed = (now - start_time_).seconds();
        if (elapsed > timeout_threshold_) {
          // Log the timeout event to file via the new class
          file_logger.logTimeoutRemoval(goal_poses[0].pose, elapsed, "goals_removed_log.txt");

          RCLCPP_WARN(logger_, "Goal timeout! Removing pose (%.2f, %.2f)", 
                      goal_poses[0].pose.position.x, goal_poses[0].pose.position.y);
          
          goal_poses.erase(goal_poses.begin());
          timer_active_ = false; 
          setOutput("output_goals", goal_poses);
          return BT::NodeStatus::SUCCESS;
          }
      }
    } else {
      // Robot moved out of radius
      timer_active_ = false;
    }
  }

  // --- 3. Proximity/Angle Removal Loop ---
  static size_t i = 0;
  bool first_del_point = true;

  while (goal_poses.size() > 1) {
    double ref_dx = goal_poses[1].pose.position.x - goal_poses[0].pose.position.x;
    double ref_dy = goal_poses[1].pose.position.y - goal_poses[0].pose.position.y;
    double ref_angle = std::atan2(ref_dy, ref_dx);

    double current_dx = goal_poses[0].pose.position.x - current_pose.pose.position.x;
    double current_dy = goal_poses[0].pose.position.y - current_pose.pose.position.y;
    double current_angle = std::atan2(current_dy, current_dx);

    double angle_diff = (std::abs(ref_angle - current_angle) * 180) / M_PI;
    double dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (angle_diff < sharp_turn_) {
      if (dist_to_goal > viapoint_achieved_radius_) {
        break; // do not remove any pose
      }
    } else {
      if (dist_to_goal > viapoint_achieved_radius_sharp_turn_) {
        break; // do not remove any pose
      }
    }

    // Log the removal using the utility class
    file_logger.logRemovalEvent(i, goal_poses[0].pose, current_pose.pose, angle_diff, first_del_point, "goals_removed_log.txt");

    i++;
    first_del_point = false;
    goal_poses.erase(goal_poses.begin());
    timer_active_ = false;
  }

  setOutput("output_goals", goal_poses);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RemovePassedGoals>("RemovePassedGoals");
}
