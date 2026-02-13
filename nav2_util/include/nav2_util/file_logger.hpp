#ifndef NAV2_UTIL__FILE_LOGGER_HPP_
#define NAV2_UTIL__FILE_LOGGER_HPP_

#include <string>
#include <fstream>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav_msgs/msg/path.hpp"
#include "nav_2d_utils/conversions.hpp"


namespace nav2_util
{

class FileLogger
{
public:
  explicit FileLogger(const std::string & file_path = "/home/markilius/lawnmower_log/")
  : file_path_(file_path) {}

  void logInitialState(const std::vector<geometry_msgs::msg::PoseStamped> & goals, const std::string & file_name)
  {
    std::ofstream logFile(file_path_ + file_name, std::ios::app);
    if (!logFile.is_open()) {return;}

    logFile << "\nThere are " << goals.size() << " poses:\n[";
    for (size_t i = 0; i < goals.size(); ++i) {
      logFile << i << ":(" << goals[i].pose.position.x << ", " << goals[i].pose.position.y << ")";
      if (i < goals.size() - 1) {
        logFile << ", ";
      }
    }
    logFile << "]\n";
  }

  /**
   * @brief Logs a removal caused by a timeout
   */
  void logTimeoutRemoval(const geometry_msgs::msg::Pose & goal, double elapsed, const std::string & file_name)
  {
    std::ofstream logFile(file_path_ + file_name, std::ios::app);
    if (!logFile.is_open()) {return;}

    logFile << "\n[TIMEOUT] Removing pose: (" << goal.position.x << ", " << goal.position.y 
            << ") after " << elapsed << " seconds.";
  }

  void logRemovalEvent(
    size_t index,
    const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Pose & robot,
    double angle_diff,
    bool is_first,
    const std::string & file_name)
  {
    std::ofstream logFile(file_path_ + file_name, std::ios::app);
    if (!logFile.is_open()) {return;}

    if (is_first) {
      logFile << "\nRemoving poses ";
    } else {
      logFile << ", ";
    }

    logFile << index << ":(" << goal.position.x << ", " << goal.position.y << ", "
            << robot.position.x << ", " << robot.position.y << ", " << angle_diff << ")";
  }

  void logInvalidPoses(const std::vector<uint32_t> & to_remove_idx, const std::string & file_name)
  {
    std::ofstream logFile(file_path_ + file_name, std::ios::app);
    if (!logFile.is_open()) {return;}

    logFile << "\nThere are " << to_remove_idx.size() << " invalid poses:\n";
    logFile << "[";

    for (auto const& idx : to_remove_idx) {
      logFile << idx << ", ";
    }
    logFile << "]\n";
  }

void logPlanTransformation(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & global_plan,
  // Use the underlying vector's iterator type
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator transformation_begin,
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator transformation_end,
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator closest_pose_upper_bound,
  double max_search_dist,
  double costmap_extent,
  const std::string & file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  static bool isFirstCall = true;
  if (isFirstCall) {
    logFile << "INITIAL PARAMETERS:\n";
    logFile << "max_robot_pose_search_dist_: " << max_search_dist << "\n";
    logFile << "max_costmap_extent: " << costmap_extent << "\n\n";
    isFirstCall = false;
  }

  static int callCount = 1;
  size_t i = 0;

  logFile << "--- Call " << callCount++ << " ---\n";
  logFile << "robot_pose: (" << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ")\n";

  for (auto it = global_plan.poses.begin(); it != global_plan.poses.end(); ++it, ++i) {
      if (it == closest_pose_upper_bound) {
          logFile << "closest_pose_upper_bound: " << i << ":("
                  << it->pose.position.x << ", "
                  << it->pose.position.y << ")\n";
      }
      if (it == transformation_begin) {
          logFile << "transformation_begin: " << i << ":("
                  << it->pose.position.x << ", "
                  << it->pose.position.y << ")\n";
      }
      if (it == transformation_end) {
          logFile << "transformation_end: " << i << ":("
                  << it->pose.position.x << ", "
                  << it->pose.position.y << ")\n";
      }
  }

  // --- 3. Global Plan Segment ---
  i = 0;
  logFile << "plan_before_transform: [";
  for (auto it = transformation_begin; it != transformation_end; ++it, i++) {
    logFile << i
            << ":(" << it->pose.position.x << ", "
            << it->pose.position.y << ")"
            << (std::next(it) != transformation_end ? ", " : "");
  }
  logFile << "]\n\n";
}

void logRPPlookAhead(
  geometry_msgs::msg::PoseStamped & carrot_pose,
  const double lookahead_dist,
  const double carrot_dist2,
  const double curvature,
  const double linear_vel,
  const double angular_vel,
  const std::string & file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  logFile << "lookahead_dist: " << lookahead_dist << "\n";
  logFile << "carrot_pose: ("
          << carrot_pose.pose.position.x << ", "
          << carrot_pose.pose.position.y << ")\n";

  logFile << "carrot_dist2: " << carrot_dist2 << "\n";
  logFile << "curvature: " << curvature << "\n";

  logFile << "cmd_vel: ("
          << linear_vel << ", "
          << angular_vel << ")\n\n";
}

void logText(
  const std::string & text,
  const std::string & file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  logFile << text;
}

void logLookAheadIdx(
  const size_t index,
  std::vector<geometry_msgs::msg::PoseStamped>::const_iterator goal_pose_it,
  bool corrected,
  const std::string & file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  if (!corrected) {
    logFile << "lookahead_index: " << index << "\n"
            << "goal_pose: (";
  } else {
    logFile << "corrected lookahead_index: " << index << "\n"
            << "corrected goal_pose: (";
  }

  logFile << goal_pose_it->pose.position.x << ", "
          << goal_pose_it->pose.position.y << ")\n";
}

void logPoses(
  std::vector<geometry_msgs::msg::PoseStamped> & goals,
  const std::string & file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  logFile << "[";
    for (size_t i = 0; i < goals.size(); ++i) {
        const auto& pose = goals[i].pose;
        logFile << i
                << ":("
                << pose.position.x
                << ", "
                << pose.position.y << ")";

        if (i < goals.size() - 1) {
            logFile << ", ";
        }
    }
    logFile << "]\n\n";
}

void logPlanTransformationDWB(
  const nav_2d_msgs::msg::Pose2DStamped & robot_pose,
  const nav_2d_msgs::msg::Path2D & global_plan,
  // Use the underlying vector's iterator type
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator transformation_begin,
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator transformation_end,
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator prune_point,
  bool prune_plan,
  double transform_start_threshold,
  double transform_end_threshold,
  double ref_dx,
  double ref_dy,
  double ref_angle,
  const std::string & file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  static bool isFirstCall = true;
  static int callCount = 1;

  if (isFirstCall) {
    logFile << "prune_plan_: " << prune_plan << "\n"
            << "transform_start_threshold: " << transform_start_threshold << "\n"
            << "transform_end_threshold: " << transform_end_threshold << "\n\n";
    isFirstCall = false;
  }

  logFile << "Call " << callCount << ":\n";

  // Log robot pose
  logFile << "robot_pose: ("
          << robot_pose.pose.x << ", "
          << robot_pose.pose.y << ")\n";

  // Log iterator positions
  size_t i = 0;
  for (auto it = global_plan.poses.begin(); it != global_plan.poses.end(); ++it, ++i) {
      if (it == prune_point) {
          logFile << "prune_point: " << i << ":("
                  << it->x << ", "
                  << it->y << ")\n";
      }
      if (it == transformation_begin) {
          logFile << "transformation_begin: " << i << ":("
                  << it->x << ", "
                  << it->y << ")\n";
      }
      if (it == transformation_end) {
          logFile << "transformation_end: " << i << ":("
                  << it->x << ", "
                  << it->y << ")\n";
      }
  }

  logFile << "\n";

  logFile << "Reference direction: ("
          << ref_dx << ", "
          << ref_dy << ")\n";
  logFile << "Reference angle: "
          << ref_angle
          << " degrees\n";

  callCount++;
}

void logSharpAngleDWB(
  double angle_diff,
  double next_angle_diff,
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator transformation_begin,
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator it,
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator it_next,
  bool sharp_turn,
  const std::string file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  if (sharp_turn) {
    logFile << "Sharp turn detected at point "
            << std::distance(transformation_begin, it)
            << " with angle difference of " << angle_diff << "°\n";
  } else {
    logFile << "Traversing same route " << std::distance(transformation_begin, it)
            << ":(" << it->x << ", "
            << it->y << ") -> "
            << std::distance(transformation_begin, it_next)
            << ":(" << it_next->x << ", "
            << it_next->y << ") "
            << "with next angle difference of " << next_angle_diff << "°\n";
  }
}

void logFinalTranformEndDWB(
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator transformation_begin,
  std::vector<geometry_msgs::msg::Pose2D>::const_iterator transformation_end,
  const std::string file_name)
{
  std::ofstream logFile(file_path_ + file_name, std::ios::app);
  if (!logFile.is_open()) {return;}

  logFile << "Adjusted transformation_end "
          << std::distance(transformation_begin, transformation_end) << ":("
          << transformation_end->x << ", "
          << transformation_end->y << ")\n";
 
  // Log global plan before transform
  size_t i = 0;

  logFile << "Final plan_before_transform ("
          << std::distance(transformation_begin, transformation_end) << " points): \n";
  
  for (auto it = transformation_begin; it != transformation_end; ++it, ++i) {
      logFile << i << ":(" << it->x << ", "
              << it->y << ")";
      if (i % 5 == 0 && i != 0) {
        logFile << "\n";
      }
      else {
          if (std::next(it) != transformation_end) {
          logFile << ", ";
      }
    }
  }
  logFile << "\n\n";
}

private:
  std::string file_path_;
};

}  // namespace nav2_util

#endif