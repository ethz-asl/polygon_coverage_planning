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

#include "polygon_coverage_ros/shortest_path_planner.h"

#include <geometry_msgs/PointStamped.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/offset.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/topic.h>

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
  computeOffsetPolygon(temp_poly, wall_distance_, &polygon_.value());
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
