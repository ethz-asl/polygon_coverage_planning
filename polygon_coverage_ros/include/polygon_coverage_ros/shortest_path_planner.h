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
};

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_ROS_SHORTEST_PATH_PLANNER_H_
