#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner_exact.h"

#include <glog/logging.h>

namespace mav_coverage_planning {

bool PolygonStripmapPlannerExact::setupSolver() {
  LOG(INFO) << "Creating boolean lattice.";
  boolean_lattice_ =
      boolean_lattice::BooleanLattice(decomposition_.size());
  if (!boolean_lattice_.isInitialized()) {
    LOG(ERROR) << "Cannot create boolean lattice.";
    return false;
  }

  LOG(INFO) << "Initializing product graph.";
  gtspp_product_graph_ = gtspp_product_graph::GtsppProductGraph(
      &sweep_plan_graph_, &boolean_lattice_);

  return preprocess();
}

bool PolygonStripmapPlannerExact::preprocess() {
  LOG(INFO) << "Preset product graph.";
  if (!gtspp_product_graph_.createOnline()) {
    LOG(ERROR) << "Could not create product graph.";
    return false;
  }
  return true;
}


bool PolygonStripmapPlannerExact::runSolver(
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* solution) const {
  CHECK_NOTNULL(solution);

  LOG(INFO) << "Start solving GTSP using exact solver without preprocessing.";
  return gtspp_product_graph_.solveOnline(start, goal, solution);
}

}  // namespace mav_coverage_planning
