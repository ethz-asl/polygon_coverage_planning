#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact.h"

#include <ros/assert.h>
#include <ros/console.h>

namespace polygon_coverage_planning {

bool PolygonStripmapPlannerExact::setupSolver() {
  ROS_INFO("Creating boolean lattice.");
  boolean_lattice_ = boolean_lattice::BooleanLattice(
      sweep_plan_graph_.getDecomposition().size());
  if (!boolean_lattice_.isInitialized()) {
    ROS_ERROR("Cannot create boolean lattice.");
    return false;
  }

  ROS_INFO("Initializing product graph.");
  gtspp_product_graph_ = gtspp_product_graph::GtsppProductGraph(
      &sweep_plan_graph_, &boolean_lattice_);

  return preprocess();
}

bool PolygonStripmapPlannerExact::preprocess() {
  ROS_INFO("Preset product graph.");
  if (!gtspp_product_graph_.createOnline()) {
    ROS_ERROR("Could not create product graph.");
    return false;
  }
  return true;
}

bool PolygonStripmapPlannerExact::runSolver(
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* solution) const {
  ROS_ASSERT(solution);

  ROS_INFO("Start solving GTSP using exact solver without preprocessing.");
  return gtspp_product_graph_.solveOnline(start, goal, solution);
}

}  // namespace polygon_coverage_planning
