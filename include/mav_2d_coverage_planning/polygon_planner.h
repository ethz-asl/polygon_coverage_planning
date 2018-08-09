#ifndef MAV_2D_COVERAGE_PLANNING_POLYGON_PLANNING_H_
#define MAV_2D_COVERAGE_PLANNING_POLYGON_PLANNING_H_

#include <mav_coverage_planning_comm/common.h>

template <class CostFunctionType, class GTSPPSolver>
class PolygonPlanner {
 public:
  // Create a sweep plan for a polygon with holes.
  //
  // polygon: the polygon to cover.
  // cost_function: the cost function type.
  // altitude: constant MAV altitude in [m].
  // fov_camera_rad: lateral field of view of the sensor in [rad].
  // min_view_overlap: the minimum sensor footprint overlap of two sweep rows
  // [0..1).
  PolygonPlanner(const Polygon& polygon, const CostFunctionType& cost_function,
                 const GTSPPSolver& gtspp_solver, double altitude,
                 double fov_camera_rad, double min_view_overlap);

  // Solve the resulting generalized traveling salesman problem.
  // start: the start point.
  // goal: the goal point.
  // solution: the solution waypoints.
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
  GTSPPSolver solver_;
};

#endif  // MAV_2D_COVERAGE_PLANNING_POLYGON_PLANNING_H_
