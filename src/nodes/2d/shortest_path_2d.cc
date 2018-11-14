#include "mav_coverage_planning_ros/nodes/2d/shortest_path_2d.h"

namespace mav_coverage_planning {

ShortestPath2D::ShortestPath2D(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : BasePlanner2D(nh, nh_private) {
  // Creating the visibility graph from the received parameters.
  // This operation may take some time.
  resetPlanner();
}

bool ShortestPath2D::solvePlanner(const Point_2& start, const Point_2& goal) {
  return planner_->solveWithOutsideStartAndGoal(start, goal, &solution_);
}

// Reset the planner when a new polygon is set.
bool ShortestPath2D::resetPlanner() {
  ROS_INFO_STREAM("Start creating the shortest plan graph.");
  planner_.reset(new visibility_graph::VisibilityGraph(settings_.polygon,
                                                       settings_.robot_size));
  if (planner_->isInitialized()) {
    ROS_INFO_STREAM("Finished creating the shortest plan graph.");
    return true;
  } else {
    ROS_ERROR_STREAM("Failed creating shortest path planner from user input.");
    return false;
  }
}

}  // namespace mav_coverage_planning
