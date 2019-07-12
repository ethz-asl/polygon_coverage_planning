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

#ifndef POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_H_
#define POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_H_

#include <memory>

#include <polygon_coverage_geometry/cgal_definitions.h>
#include "polygon_coverage_planners/cost_functions/path_cost_functions.h"
#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"
#include "polygon_coverage_planners/sensor_models/sensor_model_base.h"

namespace polygon_coverage_planning {

class PolygonStripmapPlanner {
 public:
  // Create a sweep plan for a 2D polygon with holes.
  PolygonStripmapPlanner(
      const sweep_plan_graph::SweepPlanGraph::Settings& settings);

  // Precompute solver essentials. To be run before solving.
  bool setup();

  // Solve the resulting generalized traveling salesman problem.
  // start: the start point.
  // goal: the goal point.
  // solution: the solution waypoints.
  bool solve(const Point_2& start, const Point_2& goal,
             std::vector<Point_2>* solution) const;

  inline bool isInitialized() const { return is_initialized_; }

  inline std::vector<Polygon_2> getDecomposition() {
    return sweep_plan_graph_.getDecomposition();
  }

 protected:
  virtual bool setupSolver() { return true; };
  // Default: Heuristic GTSPP solver.
  virtual bool runSolver(const Point_2& start, const Point_2& goal,
                         std::vector<Point_2>* solution) const;

  // The sweep plan graph with all possible waypoints its node connections.
  sweep_plan_graph::SweepPlanGraph sweep_plan_graph_;

 private:
  // Valid construction.
  bool is_initialized_;
  // The sweep plan settings.
  sweep_plan_graph::SweepPlanGraph::Settings settings_;
};

}  // namespace polygon_coverage_planning
#endif  // POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_H_
