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

#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact_preprocessed.h"

#include <ros/assert.h>
#include <ros/console.h>

namespace polygon_coverage_planning {

bool PolygonStripmapPlannerExactPreprocessed::preprocess() {
  ROS_INFO("Precomputing product graph.");
  if (!gtspp_product_graph_.create()) {
    ROS_ERROR("Could not create product graph.");
    return false;
  }
  return true;
}

bool PolygonStripmapPlannerExactPreprocessed::runSolver(
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* solution) const {
  ROS_ASSERT(solution);

  ROS_INFO("Start solving GTSP using exact solver with preprocessing.");
  return gtspp_product_graph_.solve(start, goal, solution);
}

}  // namespace polygon_coverage_planning
