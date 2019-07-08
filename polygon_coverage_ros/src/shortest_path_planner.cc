#include "polygon_coverage_ros/shortest_path_planner.h"

#include <polygon_coverage_geometry/offset.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace polygon_coverage_planning {

ShortestPathPlanner::ShortestPathPlanner(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private)
    : PolygonPlannerBase(nh, nh_private) {
  // Creating the visibility graph from the received parameters.
  // This operation may take some time.
  resetPlanner();
}

bool ShortestPathPlanner::solvePlanner(const Point_2& start,
                                       const Point_2& goal) {
  return planner_->solveWithOutsideStartAndGoal(start, goal, &solution_);
}

// Reset the planner when a new polygon is set.
bool ShortestPathPlanner::resetPlanner() {
  ROS_INFO_STREAM("Start creating the shortest plan graph.");
  if (!polygon_.has_value()) {
    ROS_WARN("No polygon set. Cannot reset planner.");
    return false;
  }
  PolygonWithHoles temp_poly = polygon_.value();
  std::cout << "offsetting." << std::endl;
  computeOffsetPolygon(temp_poly, wall_distance_, &polygon_.value());
  std::cout << polygon_.value() << std::endl;
  planner_.reset(new visibility_graph::VisibilityGraph(polygon_.value()));
  if (planner_->isInitialized()) {
    ROS_INFO("Finished creating the shortest plan graph.");
    return true;
  } else {
    ROS_ERROR("Failed creating shortest path planner from user input.");
    return false;
  }
}

}  // namespace polygon_coverage_planning
