#ifndef MAV_COVERAGE_PLANNING_SHORTEST_PATH_ROS_H_
#define MAV_COVERAGE_PLANNING_SHORTEST_PATH_ROS_H_

#include <memory>

#include <Eigen/Core>

#include <ros/ros.h>

#include "mav_coverage_planning/graph/visibility_graph.h"
#include "mav_coverage_planning/base_planner_ros.h"

namespace mav_coverage_planning {

// A ros wrapper for the line sweep planner
class ShortestPathRos : BasePlannerRos {
 public:
  // Constructor
  ShortestPathRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  // Call to the shortest path planner library.
  virtual bool solvePlanner(const Eigen::Vector2d& start,
                            const Eigen::Vector2d& goal);

  // Reset the shortest path planner when a new polygon is set.
  virtual bool resetPlanner();

  // The library object that actually does planning.
  std::unique_ptr<visibility_graph::VisibilityGraph> planner_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_SHORTEST_PATH_ROS_H_
