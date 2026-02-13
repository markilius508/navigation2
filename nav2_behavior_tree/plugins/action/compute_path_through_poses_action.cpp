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

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "nav2_util/file_logger.hpp"
#include "nav2_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

namespace nav2_behavior_tree
{

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
}

void ComputePathThroughPosesAction::on_tick()
{
  getInput("input_goals", goal_.goals);
  getInput("planner_id", goal_.planner_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }

  static nav2_util::FileLogger file_logger;
  file_logger.logPoses(
    goal_.goals,
    "posesToGoThrough.txt");
}

BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  // 1. Get the current goals from the input port
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  getInput("input_goals", goals);

  // 2. Get the indices to remove from the action result
  auto to_remove = result_.result->invalid_index; // std::vector<uint32_t>

  if (!to_remove.empty()) {
    // 3. Sort indices in DESCENDING order
    std::sort(to_remove.begin(), to_remove.end(), std::greater<uint32_t>());

    // 4. Remove each invalid index
    for (auto const& idx : to_remove) {
      if (idx < goals.size()) {
        goals.erase(goals.begin() + idx);
      }
    }

    // 5. Write the modified list back to the Blackboard
    // This updates the "{goals}" variable so the next BT tick uses the new list
    setOutput("output_goals", goals); 
  }

  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::on_aborted()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::on_cancelled()
{
  nav_msgs::msg::Path empty_path;
  setOutput("path", empty_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathThroughPosesAction>(
        name, "compute_path_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}
