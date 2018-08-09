#include "mav_coverage_planning/sweep_planner.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <glog/logging.h>
#include <ros/ros.h>

namespace mav_coverage_planning {

const std::string kPrefix = kOutputPrefix + "sweep_planner]: ";

SweepPlanner::SweepPlanner(const Polygon& polygon,
                           const CostFunction& cost_function, double altitude,
                           double fov_camera_rad, double min_view_overlap,
                           enum GTSPPSolver solver)
    : is_initialized_(true),
      polygon_(polygon),
      cost_function_(cost_function),
      solver_(solver) {
  if (!checkUserInput(polygon, altitude, fov_camera_rad, min_view_overlap)) {
    ROS_ERROR_STREAM(kPrefix << "Wrong user input.");
    is_initialized_ = false;
  } else {
    ROS_DEBUG_STREAM(kPrefix << "Correct user input.");
  }

  // Create convex decomposition.
  std::vector<Polygon> convex_polygons;
  if (is_initialized_) {
    if (!polygon.computeConvexDecomposition(&convex_polygons)) {
      ROS_ERROR_STREAM(kPrefix << "Cannot compute convex decomposition.");
      is_initialized_ = false;
    } else {
      ROS_INFO_STREAM(kPrefix << "Successfully created convex partition with "
                              << convex_polygons.size()
                              << " convex polygon(s).");
    }
  }

  // Create sweep plan graph.
  const double sweep_distance =
      (1 - min_view_overlap) * 2 * altitude * std::tan(fov_camera_rad / 2.0);
  ROS_INFO_STREAM(kPrefix << "Start creating sweep plan graph.");
  sweep_plan_graph_ = sweep_plan_graph::SweepPlanGraph(
      polygon, cost_function_, convex_polygons, sweep_distance);
  if (is_initialized_) {
    if (!sweep_plan_graph_.isInitialized()) {
      ROS_ERROR_STREAM(kPrefix << "Cannot create sweep plan graph.");
      is_initialized_ = false;
    }
  }

  // Create boolean lattice.
  if (solver_ == GTSPPSolver::kExactWithPreprocessing ||
      solver_ == GTSPPSolver::kExactOnline) {
    ROS_INFO_STREAM(kPrefix << "Creating boolean lattice.");
    boolean_lattice_ = boolean_lattice::BooleanLattice(convex_polygons.size());
    if (is_initialized_) {
      if (!boolean_lattice_.isInitialized()) {
        ROS_ERROR_STREAM(kPrefix << "Cannot create boolean lattice.");
        is_initialized_ = false;
      }
    }
  }

  // Create the product graph.
  if (solver_ == GTSPPSolver::kExactWithPreprocessing ||
      solver_ == GTSPPSolver::kExactOnline) {
    gtspp_product_graph_ = gtspp_product_graph::GtsppProductGraph(
        &sweep_plan_graph_, &boolean_lattice_);
    if (solver_ == GTSPPSolver::kExactWithPreprocessing) {
      if (is_initialized_) {
        ROS_INFO_STREAM(kPrefix << "Preprocessing product graph.");
        if (!gtspp_product_graph_.create()) {
          ROS_ERROR_STREAM(kPrefix << "Could not create product graph.");
          is_initialized_ = false;
        }
      }
    }
  }
}

bool SweepPlanner::checkUserInput(const Polygon& polygon, double altitude,
                                  double fov_camera_rad,
                                  double min_view_overlap) const {
  if (!polygon.isValid()) {
    ROS_ERROR_STREAM(kPrefix << "The polygon is not valid.");
    return false;
  }
  if (!polygon.isSimple()) {
    ROS_ERROR_STREAM(kPrefix << "The polygon is not simple.");
    return false;
  } else if (polygon.hasHoles() && !polygon.hasSimpleHoles()) {
    ROS_ERROR_STREAM(kPrefix << "The holes are not simple.");
  } else if (altitude <= 0.0) {
    ROS_ERROR_STREAM(kPrefix << "The user defined altitude is " << altitude
                             << " but needs to be greater than " << 0.0 << ".");
    return false;
  } else if (fov_camera_rad <= 0.0) {
    ROS_ERROR_STREAM(kPrefix << "The user defined camera fov is "
                             << fov_camera_rad
                             << " but needs to be greater than " << 0.0 << ".");
    return false;
  } else if (fov_camera_rad >= M_PI) {
    ROS_ERROR_STREAM(kPrefix << "The user defined camera fov is "
                             << fov_camera_rad << " but needs to be less than "
                             << M_PI << ".");
    return false;
  } else if (min_view_overlap < 0.0) {
    ROS_ERROR_STREAM(kPrefix << "The user defined minimum view overlap is "
                             << min_view_overlap
                             << " but needs to be greater than " << 0.0 << ".");
    return false;
  } else if (min_view_overlap >= 1.0) {
    ROS_ERROR_STREAM(kPrefix << "The user defined minimum view overlap is "
                             << min_view_overlap
                             << " but needs to be less than " << 1.0 << ".");
    return false;
  }
  return true;
}

bool SweepPlanner::solve(const Eigen::Vector2d& start,
                         const Eigen::Vector2d& goal,
                         StdVector2d* solution) const {
  CHECK_NOTNULL(solution);
  solution->clear();

  if (!is_initialized_) {
    ROS_ERROR_STREAM(kPrefix << "Could not create sweep planner for user "
                                "input. Failed to compute solution.");
    return false;
  }

  // Make sure start and end are inside the polygon.
  const Eigen::Vector2d start_new = polygon_.checkPointInPolygon(start)
                                        ? start
                                        : polygon_.projectPointOnHull(start);
  const Eigen::Vector2d goal_new = polygon_.checkPointInPolygon(goal)
                                       ? goal
                                       : polygon_.projectPointOnHull(goal);

  ROS_INFO_STREAM(kPrefix << "Start solving GTSP using method: " << solver_);
  switch (solver_) {
    case GTSPPSolver::kExactWithPreprocessing:
      if (!gtspp_product_graph_.solve(start_new, goal_new, solution)) {
        return false;
      }
      break;
    case GTSPPSolver::kExactOnline:
      if (!gtspp_product_graph_.solveOnline(start_new, goal_new, solution)) {
        return false;
      }
    case GTSPPSolver::kApproximateMA:
      if (!sweep_plan_graph_.solve(start_new, goal_new, solution)) {
        return false;
      }
      break;
    default:
      ROS_ERROR_STREAM(kPrefix << "Solver " << solver_ << " not implemented.");
      return false;
      break;
  }

  // Make sure original start and end are part of the plan.
  if (!polygon_.checkPointInPolygon(start)) {
    solution->insert(solution->begin(), start);
  }
  if (!polygon_.checkPointInPolygon(goal)) {
    solution->insert(solution->end(), goal);
  }
  return true;
}

}  // namespace mav_coverage_planning
