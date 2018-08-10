#ifndef MAV_COVERAGE_PLANNING_ROS_SHORTEST_PATH_2D_H_
#define MAV_COVERAGE_PLANNING_ROS_SHORTEST_PATH_2D_H_

#include <memory>

#include <ros/ros.h>

#include "mav_coverage_planning_ros/nodes/2d/base_planner_2d.h"

#include <mav_2d_coverage_planning/definitions.h>
#include <mav_2d_coverage_planning/graphs/visibility_graph.h>

namespace mav_coverage_planning {

// A ros wrapper for the line sweep planner
class ShortestPath2D : public BasePlanner2D {
 public:
  // Constructor
  ShortestPath2D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  // Call to the shortest path planner library.
  bool solvePlanner(const Point_2& start, const Point_2& goal) override;

  // Reset the shortest path planner when a new polygon is set.
  bool resetPlanner() override;

  // The library object that actually does planning.
  std::unique_ptr<visibility_graph::VisibilityGraph> planner_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_ROS_SHORTEST_PATH_2D_H_
