#include <ros/ros.h>

#include "polygon_coverage_ros/shortest_path_planner.h"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "shortest_path_planner");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Creating the coverage planner with ros interface
  polygon_coverage_planning::ShortestPathPlanner shortest_path(nh, nh_private);
  // Spinning (and processing service calls)
  ros::spin();
  // Exit tranquilly
  return 0;
}
