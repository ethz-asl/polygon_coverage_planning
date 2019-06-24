#ifndef POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_H_
#define POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_H_

#include "polygon_coverage_planners/graphs/gtspp_product_graph.h"
#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"
#include "polygon_coverage_planners/planners/polygon_stripmap_planner.h"

#include <polygon_coverage_solvers/boolean_lattice.h>

namespace polygon_coverage_planning {

class PolygonStripmapPlannerExact : public PolygonStripmapPlanner {
 public:
  PolygonStripmapPlannerExact(
      const sweep_plan_graph::SweepPlanGraph::Settings& settings)
      : PolygonStripmapPlanner(settings) {}

 protected:
  virtual bool preprocess();

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
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_PLANNERS_POLYGON_STRIPMAP_PLANNER_EXACT_H_
