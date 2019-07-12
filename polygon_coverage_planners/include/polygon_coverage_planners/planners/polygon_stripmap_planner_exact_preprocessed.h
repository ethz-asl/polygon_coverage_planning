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

#ifndef POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_PREPROCESSED_H_
#define POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_PREPROCESSED_H_

#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"
#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact.h"

namespace polygon_coverage_planning {

class PolygonStripmapPlannerExactPreprocessed
    : public PolygonStripmapPlannerExact {
 public:
  PolygonStripmapPlannerExactPreprocessed(
      const sweep_plan_graph::SweepPlanGraph::Settings& settings)
      : PolygonStripmapPlannerExact(settings) {}

 private:
  bool runSolver(const Point_2& start, const Point_2& goal,
                 std::vector<Point_2>* solution) const override;
  // Precompute product graph. Allows multiple queries.
  bool preprocess() override;
};

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_PREPROCESSED_H_
