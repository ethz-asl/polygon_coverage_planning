#include <ros/ros.h>

#include <polygon_coverage_planners/planners/polygon_stripmap_planner_exact_preprocessed.h>
#include "polygon_coverage_ros/coverage_planner.h"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "stripmap_planner_2d_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Creating the coverage planner with ros interface
  polygon_coverage_planning::CoveragePlanner<
      polygon_coverage_planning::PolygonStripmapPlannerExactPreprocessed>
      planner(nh, nh_private);
  // Spinning (and processing service calls)
  ros::spin();
  // Exit tranquilly
  return 0;
}
