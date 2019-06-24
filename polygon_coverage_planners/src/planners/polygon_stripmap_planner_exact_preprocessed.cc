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
