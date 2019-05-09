#ifndef MAV_2D_COVERAGE_PLANNING_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_PREPROCESSED_H_
#define MAV_2D_COVERAGE_PLANNING_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_PREPROCESSED_H_

#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner_exact.h"

namespace mav_coverage_planning {

class PolygonStripmapPlannerExactPreprocessed
    : public PolygonStripmapPlannerExact {
 public:
  PolygonStripmapPlannerExactPreprocessed(const Settings& settings)
      : PolygonStripmapPlannerExact(settings) {}

 private:
  bool runSolver(const Point_2& start, const Point_2& goal,
                 std::vector<Point_2>* solution) const override;
  // Precompute product graph. Allows multiple queries.
  bool preprocess() override;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_PREPROCESSED_H_
