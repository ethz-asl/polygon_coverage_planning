#include <ros/ros.h>

#include <polygon_coverage_planners/planners/polygon_stripmap_planner_exact.h>
#include "polygon_coverage_ros/coverage_planner.h"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "coverage_planner_exact");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Creating the coverage planner with ros interface
  polygon_coverage_planning::CoveragePlanner<
      polygon_coverage_planning::PolygonStripmapPlannerExact>
      planner(nh, nh_private);
  // Spinning (and processing service calls)
  ros::spin();
  // Exit tranquilly
  return 0;
}
