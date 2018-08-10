#ifndef MAV_2D_COVERAGE_PLANNING_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_H_
#define MAV_2D_COVERAGE_PLANNING_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_H_

#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner.h"

#include <mav_coverage_graph_solvers/boolean_lattice.h>
#include "mav_2d_coverage_planning/graphs/gtspp_product_graph.h"

namespace mav_coverage_planning {

class PolygonStripmapPlannerExact : public PolygonStripmapPlanner {
 public:
  PolygonStripmapPlannerExact(const Settings& settings)
      : PolygonStripmapPlanner(settings) {}

 protected:
  virtual bool preprocess() { return true; }  // No preprocessing.

  // The product of sweep plan graph and boolean lattice.
  gtspp_product_graph::GtsppProductGraph gtspp_product_graph_;

 private:
  bool runSolver(const Point_2& start, const Point_2& goal,
                 std::vector<Point_2>* solution) const override;
  bool setupSolver() override;

  // A boolean lattice to represent all possible convex polygon visiting
  // combinations.
  boolean_lattice::BooleanLattice boolean_lattice_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_H_
