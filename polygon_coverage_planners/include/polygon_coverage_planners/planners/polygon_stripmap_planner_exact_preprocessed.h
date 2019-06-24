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
