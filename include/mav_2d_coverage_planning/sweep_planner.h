#ifndef MAV_COVERAGE_PLANNING_SWEEP_PLANNING_H_
#define MAV_COVERAGE_PLANNING_SWEEP_PLANNING_H_

#include <set>

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/cost_function.h"
#include "mav_coverage_planning/graph/boolean_lattice.h"
#include "mav_coverage_planning/graph/gtspp_product_graph.h"
#include "mav_coverage_planning/graph/sweep_plan_graph.h"
#include "mav_coverage_planning/graph/visibility_graph.h"
#include "mav_coverage_planning/math.h"
#include "mav_coverage_planning/polygon.h"

namespace mav_coverage_planning {
  enum GTSPPSolver { kExactWithPreprocessing = 0 // Precompute the full GTSPP product graph and solve with Dijkstra.
    , kExactOnline // Generate the GTSPP product graph while searching nodes with Dijsktra.
    , kApproximateMA // Use the approximate memantic algorithm solver.
  };

class SweepPlanner {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Create a sweep planner object for a polygon with holes.
  //
  // polygon: the polygon to cover.
  // cost_function: the cost function (time or distance).
  // altitude: constant MAV altitude in [m].
  // fov_camera_rad: the (square) field of view of the camera in [rad].
  // min_view_overlap: the minimum view overlap of two sweep rows [0..1).
  // start: the start point.
  // goal: the goal point.
  // solution: the solution waypoints.
  SweepPlanner(const Polygon& polygon, const CostFunction& cost_function,
               double altitude, double fov_camera_rad, double min_view_overlap,
               enum GTSPPSolver);

  // Solve the resulting generalized traveling salesman problem using a Dijkstra
  // algorithm.
  bool solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
             StdVector2d* solution) const;

  inline bool isInitialized() const { return is_initialized_; }

 private:
  // Check for valid user input.
  bool checkUserInput(const Polygon& polygon, double altitude,
                      double fov_camera_rad, double min_view_overlap) const;

  // Valid construction.
  bool is_initialized_;
  // The user defined polygon.
  Polygon polygon_;
  // The user defined cost function (time or distance).
  CostFunction cost_function_;
  // The sweep plan graph with all possible waypoints its node connections.
  sweep_plan_graph::SweepPlanGraph sweep_plan_graph_;
  // A boolean lattice to represent all possible convex polygon visiting
  // combinations.
  boolean_lattice::BooleanLattice boolean_lattice_;
  // The product of sweep plan graph and boolean lattice.
  gtspp_product_graph::GtsppProductGraph gtspp_product_graph_;
  GTSPPSolver solver_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_SWEEP_PLANNING_H_
