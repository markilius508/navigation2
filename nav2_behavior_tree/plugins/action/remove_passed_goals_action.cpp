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
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  static bool first = true;

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

  double dist_to_goal;
  std::ofstream logFile("/home/markilius/nav2_ws/src/goals_removed_log.txt", std::ios::app);
  
  if (first) {
    if (logFile.is_open()) {
      // Log initial state of goal_poses
      logFile << "\nThere are " << goal_poses.size() << " poses:\n";
      logFile << "[";
      for (size_t i = 0; i < goal_poses.size(); ++i) {
          const auto& pose = goal_poses[i].pose;
          logFile << i << ":(" << pose.position.x << ", " << pose.position.y << ")";
          if (i < goal_poses.size() - 1) {
              logFile << ", ";
          }
      }
      logFile << "]\n";
    }
    first = false;
  }

  size_t i = 0;
  bool first_del_point = true;
  double ref_dx = 0.0;
  double ref_dy = 0.0;
  double ref_angle = 0.0;

  double current_dx = 0.0;
  double current_dy = 0.0;
  double current_angle = 0.0;

  double angle_diff = 0.0;

  while (goal_poses.size() > 1) {
    ref_dx = goal_poses[1].pose.position.x - goal_poses[0].pose.position.x;
    ref_dy = goal_poses[1].pose.position.y - goal_poses[0].pose.position.y;
    ref_angle = std::atan2(ref_dy, ref_dx);

    current_dx = goal_poses[0].pose.position.x - current_pose.pose.position.x;
    current_dy = goal_poses[0].pose.position.y - current_pose.pose.position.y;
    current_angle = std::atan2(current_dy, current_dx);

    angle_diff = (std::abs(ref_angle - current_angle) * 180) / M_PI;
    
    dist_to_goal = euclidean_distance(goal_poses[0].pose, current_pose.pose);

    if (angle_diff < sharp_turn_) {
      if (dist_to_goal > viapoint_achieved_radius_) {
        break; // do not remove any pose
      }
    } else {
      if (dist_to_goal > viapoint_achieved_radius_sharp_turn_) {
        break; // do not remove any pose
      }
    }

    // Log removal
    if (logFile.is_open()) {
      const auto& pose = goal_poses[0].pose;
      if (first_del_point) {
        logFile << "\nRemoving poses " << i << ":(" << pose.position.x << ", " << pose.position.y;
        logFile << ", " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << angle_diff << ")";
        first_del_point = false;
      } else {
        logFile << ", " << i << ":(" << pose.position.x << ", " << pose.position.y;
        logFile << ", " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << angle_diff << ")";
      }
    }

    i++;
    goal_poses.erase(goal_poses.begin());
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
