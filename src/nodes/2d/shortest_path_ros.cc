#include "mav_coverage_planning_ros/nodes/2d/shortest_path_ros.h"

namespace mav_coverage_planning {

ShortestPathRos::ShortestPathRos(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : BasePlannerRos(nh, nh_private) {
  // Shortest path planner specific ROS interaction.
  output_prefix_ = kOutputPrefix + "shortest_path_planner_ros]: ";

  // Creating the visibility graph from the received parameters.
  // This operation may take some time.
  resetPlanner();
}

bool ShortestPathRos::solvePlanner(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& goal) {
  return planner_->solveWithOutsideStartAndGoal(start, goal, &solution_);
}

// Reset the planner when a new polygon is set.
bool ShortestPathRos::resetPlanner() {
  ROS_INFO_STREAM(kPrefix << "Start creating the shortest plan graph.");
  planner_.reset(
      new visibility_graph::VisibilityGraph(polygon_, cost_function_));
  if (planner_->isInitialized()) {
    ROS_INFO_STREAM(kPrefix << "Finished creating the shortest plan graph.");
    return true;
  } else {
    ROS_ERROR_STREAM(
        kPrefix << "Failed creating shortest path planner from user input.");
    return false;
  }
}

}  // namespace mav_coverage_planning
