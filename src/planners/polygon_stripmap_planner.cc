#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner.h"

#include <glog/logging.h>

namespace mav_coverage_planning {

PolygonStripmapPlanner::PolygonStripmapPlanner(const Settings& settings)
    : is_initialized_(false), settings_(settings) {}

bool PolygonStripmapPlanner::setup() {
  is_initialized_ = true;

  // Common setup:
  if (!settings_.check()) {
    LOG(ERROR) << "Wrong user input.";
    is_initialized_ = false;
  } else {
    DLOG(INFO) << "Correct user input.";
  }

    // Create convex decomposition.
    if (is_initialized_) {
      Polygon p;
    if (!settings_.polygon.computeOffsetPolygon(settings_.wall_dist, &p)) {
      LOG(WARNING) << "Cannot shrink polygon:" << settings_.polygon
                   << "with distance: " << settings_.wall_dist;
    } else {
      settings_.polygon = p;
    }
      
    if (!settings_.polygon.computeConvexDecompositionFromPolygonWithHoles(
            &convex_decomposition_)) {
      LOG(ERROR) << "Cannot compute convex decomposition.";
      is_initialized_ = false;
    } else {
      LOG(INFO) << "Successfully created convex partition with "
                << convex_decomposition_.size() << " convex polygon(s).";
    }
  }

  // Create sweep plan graph.
  const double max_offset_distance =
      computeSweepDistance();
  LOG(INFO) << "Start creating sweep plan graph.";
  sweep_plan_graph_ = sweep_plan_graph::SweepPlanGraph(
      settings_.polygon, settings_.path_cost_function,
      settings_.segment_cost_function, convex_decomposition_,
      max_offset_distance, max_offset_distance/2);
  if (is_initialized_) {
    if (!sweep_plan_graph_.isInitialized()) {
      LOG(ERROR) << "Cannot create sweep plan graph.";
      is_initialized_ = false;
    }
  }

  // Solver specific setup.
  is_initialized_ = setupSolver();

  return is_initialized_;
}

bool PolygonStripmapPlanner::solve(const Point_2& start, const Point_2& goal,
                                   std::vector<Point_2>* solution) const {
  CHECK_NOTNULL(solution);
  solution->clear();

  if (!is_initialized_) {
    LOG(ERROR) << "Could not create sweep planner for user input. Failed to "
                  "compute solution.";
    return false;
  }

  // Make sure start and end are inside the settings_.polygon.
  const Point_2 start_new = settings_.polygon.pointInPolygon(start)
                                ? start
                                : settings_.polygon.projectPointOnHull(start);
  const Point_2 goal_new = settings_.polygon.pointInPolygon(goal)
                               ? goal
                               : settings_.polygon.projectPointOnHull(goal);

  if (!runSolver(start_new, goal_new, solution)) {
    LOG(ERROR) << "Failed solving graph.";
    return false;
  }

  // Make sure original start and end are part of the plan.
  if (!settings_.polygon.pointInPolygon(start)) {
    solution->insert(solution->begin(), start);
  }
  if (!settings_.polygon.pointInPolygon(goal)) {
    solution->insert(solution->end(), goal);
  }

  return true;
}

bool PolygonStripmapPlanner::runSolver(const Point_2& start,
                                       const Point_2& goal,
                                       std::vector<Point_2>* solution) const {
  CHECK_NOTNULL(solution);

  LOG(INFO) << "Start solving GTSP using GK MA.";
  return sweep_plan_graph_.solve(start, goal, solution);
}

double PolygonStripmapPlanner::computeSweepDistance() const {
  return (1 - settings_.min_view_overlap) * settings_.robot_size;
}

bool PolygonStripmapPlanner::Settings::check() const {
  if (polygon.getPolygon().outer_boundary().size() < 3) {
    LOG(ERROR) << "The polygon is not valid.";
    return false;
  }
  if (!polygon.isStrictlySimple()) {
    LOG(ERROR) << "The polygon or its holes are not simple.";
    return false;
  }
  else if (min_view_overlap < 0.0) {
    LOG(ERROR) << "The user defined minimum view overlap is "
               << min_view_overlap << " but needs to be greater than " << 0.0
               << ".";
    return false;
  } else if (min_view_overlap >= 1.0) {
    LOG(ERROR) << "The user defined minimum view overlap is "
               << min_view_overlap << " but needs to be less than " << 1.0
               << ".";
    return false;
  }
  return true;
}

}  // namespace mav_coverage_planning
