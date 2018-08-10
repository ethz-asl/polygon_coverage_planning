#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner_exact_preprocessed.h"

#include <glog/logging.h>

namespace mav_coverage_planning {

bool PolygonStripmapPlannerExactPreprocessed::preprocess() {
  LOG(INFO) << "Precomputing product graph.";
  if (!gtspp_product_graph_.create()) {
    LOG(ERROR) << "Could not create product graph.";
    return false;
  }
  return true;
}

bool PolygonStripmapPlannerExactPreprocessed::runSolver(
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* solution) const {
  CHECK_NOTNULL(solution);

  LOG(INFO) << "Start solving GTSP using exact solver with preprocessing.";
  return gtspp_product_graph_.solve(start, goal, solution);
}

}  // namespace mav_coverage_planning
