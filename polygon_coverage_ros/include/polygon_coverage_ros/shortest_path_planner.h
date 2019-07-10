#ifndef POLYGON_COVERAGE_ROS_SHORTEST_PATH_PLANNER_H_
#define POLYGON_COVERAGE_ROS_SHORTEST_PATH_PLANNER_H_

#include <memory>

#include <ros/ros.h>

#include "polygon_coverage_ros/polygon_planner_base.h"

#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/visibility_graph.h>

namespace polygon_coverage_planning {

// A ros wrapper for the line sweep planner
class ShortestPathPlanner : public PolygonPlannerBase {
 public:
  // Constructor
  ShortestPathPlanner(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);

 private:
  // Call to the shortest path planner library.
  bool solvePlanner(const Point_2& start, const Point_2& goal) override;

  // Reset the shortest path planner when a new polygon is set.
  bool resetPlanner() override;

  // The library object that actually does planning.
  std::unique_ptr<visibility_graph::VisibilityGraph> planner_;

  // Set start and goal by clicked point.
  bool setFromClickedPoint(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response);

  ros::ServiceServer clicked_point_srv_;
};

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_ROS_SHORTEST_PATH_PLANNER_H_
