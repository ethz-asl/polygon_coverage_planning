/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>

#include <polygon_coverage_planners/planners/polygon_stripmap_planner.h>
#include "polygon_coverage_ros/coverage_planner.h"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "coverage_planner");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  // Creating the coverage planner with ros interface
  polygon_coverage_planning::CoveragePlanner<
      polygon_coverage_planning::PolygonStripmapPlanner>
      planner(nh, nh_private);
  // Spinning (and processing service calls)
  ros::spin();
  // Exit tranquilly
  return 0;
}
