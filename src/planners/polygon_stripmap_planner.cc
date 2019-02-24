#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner.h"
#include <CGAL/Boolean_set_operations_2.h>
#include <glog/logging.h>
#include <cmath>

namespace mav_coverage_planning {

PolygonStripmapPlanner::PolygonStripmapPlanner(const Settings& settings)
    : is_initialized_(false), settings_(settings) {}

bool PolygonStripmapPlanner::setup() {
  is_initialized_ = true;

  // Create decomposition.
  switch (settings_.decomposition_type) {
    case DecompositionType::kBoustrophedeon: {
      if (!settings_.polygon.computeBestBCDFromPolygonWithHoles(
              &decomposition_)) {
        LOG(ERROR) << "Cannot compute boustrophedeon decomposition.";
        is_initialized_ = false;
      } else {
        LOG(INFO) << "Successfully created boustrophedeon decomposition with "
                  << decomposition_.size() << " polygon(s).";
      }
      break;
    }
    case DecompositionType::kConvex: {
      if (!settings_.polygon.computeConvexDecompositionFromPolygonWithHoles(
              &decomposition_)) {
        LOG(ERROR) << "Cannot compute convex decomposition.";
        is_initialized_ = false;
      } else {
        LOG(INFO) << "Successfully created convex decomposition with "
                  << decomposition_.size() << " polygon(s).";
      }
      break;
    }
    case DecompositionType::kTrapezoidal: {
      if (!settings_.polygon
               .computeTrapezoidalDecompositionFromPolygonWithHoles(
                   &decomposition_)) {
        LOG(ERROR) << "Cannot compute trapezoidal decomposition.";
        is_initialized_ = false;
      } else {
        LOG(INFO) << "Successfully created trapezoidal decomposition with "
                  << decomposition_.size() << " polygon(s).";
      }
      break;
    }
    case DecompositionType::kBest: {
      if (!settings_.polygon.computeBestDecompositionFromPolygonWithHoles(
              &decomposition_)) {
        LOG(ERROR) << "Cannot compute best decomposition.";
        is_initialized_ = false;
      } else {
        LOG(INFO) << "Successfully created best decomposition with "
                  << decomposition_.size() << " polygon(s).";
      }
      break;
    }
    default: {
      LOG(ERROR) << "No valid decomposition type set.";
      is_initialized_ = false;
      break;
    }
  }

  if (!updateDecompositionAdjacency()) {
    LOG(ERROR) << "Decomposition not fully connected.";
    is_initialized_ = false;
  }

  if (!offsetRectangularDecomposition()) {
    LOG(ERROR) << "Failed to offset rectangular decomposition.";
    is_initialized_ = false;
  }

  // Create sweep plan graph.
  CHECK_NOTNULL(settings_.sensor_model);
  if (is_initialized_) {
    LOG(INFO) << "Start creating sweep plan graph.";
    sweep_plan_graph_ = sweep_plan_graph::SweepPlanGraph(
        settings_.polygon, settings_.path_cost_function, decomposition_,
        settings_.sensor_model->getSweepDistance());
    if (!sweep_plan_graph_.isInitialized()) {
      LOG(ERROR) << "Cannot create sweep plan graph.";
      is_initialized_ = false;
    }
  }

  // Solver specific setup.
  is_initialized_ = setupSolver();

  return is_initialized_;
}

bool PolygonStripmapPlanner::updateDecompositionAdjacency() {
  for (size_t i = 0; i < decomposition_.size() - 1; ++i) {
    for (size_t j = i + 1; j < decomposition_.size(); ++j) {
      PolygonWithHoles joined;
      if (CGAL::join(decomposition_[i].getPolygon(),
                     decomposition_[j].getPolygon(), joined)) {
        decomposition_adjacency_[i].emplace(j);
        decomposition_adjacency_[j].emplace(i);
      }
    }
  }

  // Check connectivity.
  for (size_t i = 0; i < decomposition_.size(); ++i) {
    if (decomposition_adjacency_.find(i) == decomposition_adjacency_.end()) {
      return false;
    }
  }
  return true;
}

bool PolygonStripmapPlanner::offsetRectangularDecomposition() {
  if (settings_.decomposition_type != DecompositionType::kBoustrophedeon &&
      settings_.decomposition_type != DecompositionType::kTrapezoidal)
    return true;

  // Find overlapping edges.
  std::vector<Polygon> offsetted_decomposition = decomposition_;
  std::vector<Segment_2> offsetted_segments;
  for (size_t i = 0; i < decomposition_.size(); ++i) {
    for (std::set<size_t>::iterator it = decomposition_adjacency_[i].begin();
         it != decomposition_adjacency_[i].end(); it++) {
      const Polygon_2& cell = decomposition_[i].getPolygon().outer_boundary();
      const size_t num_edges_cell = cell.size();
      // If they do not touch anymore, skip.
      PolygonWithHoles joined;
      if (!CGAL::join(decomposition_[i].getPolygon(),
                      decomposition_[*it].getPolygon(), joined))
        continue;

      const Polygon_2& neighbor =
          decomposition_[*it].getPolygon().outer_boundary();
      const size_t num_edges_neighbor = neighbor.size();
      for (size_t cell_e = 0; cell_e < num_edges_cell; ++cell_e) {
        if (std::find(offsetted_segments.begin(), offsetted_segments.end(),
                      cell.edge(cell_e)) != offsetted_segments.end())
          continue;  // Already offsetted this segment.
        for (size_t neighbor_e = 0; neighbor_e < num_edges_neighbor;
             ++neighbor_e) {
          if (std::find(offsetted_segments.begin(), offsetted_segments.end(),
                        neighbor.edge(neighbor_e)) != offsetted_segments.end())
            continue;  // Already offsetted this segment.
          // If segments intersect, offset polygon.
          CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type
              result = CGAL::intersection(cell.edge(cell_e),
                                          neighbor.edge(neighbor_e));
          if (result) {
            if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
              if (*s == cell.edge(cell_e) ||
                  s->opposite() == cell.edge(cell_e)) {
                Polygon offset_cell;
                decomposition_[i].offsetEdge(
                    cell_e, settings_.sensor_model->getSweepDistance(),
                    &offset_cell);
                Polygon intersected_offset;
                if (!offsetted_decomposition[i].intersect(offset_cell,
                                                          &intersected_offset))
                  return false;
                offsetted_decomposition[i] = intersected_offset;
                offsetted_segments.push_back(*s);
                offsetted_segments.push_back(s->opposite());
              } else if (*s == neighbor.edge(neighbor_e) ||
                         s->opposite() == neighbor.edge(neighbor_e)) {
                Polygon offset_neighbor;
                decomposition_[*it].offsetEdge(
                    neighbor_e, settings_.sensor_model->getSweepDistance(),
                    &offset_neighbor);
                Polygon intersected_offset;
                if (!offsetted_decomposition[*it].intersect(
                        offset_neighbor, &intersected_offset))
                  return false;
                offsetted_decomposition[*it] = intersected_offset;
                offsetted_segments.push_back(*s);
                offsetted_segments.push_back(s->opposite());
              } else {
                DLOG(INFO) << "Segment intersection but not identical.";
              }
            } else {
              DLOG(INFO) << "Only point intersection... "
                         << *boost::get<Point_2>(&*result);
            }
          }
        }
      }
    }
  }
  decomposition_ = offsetted_decomposition;

  return true;
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

  // TODO(rikba): Make this part of the optimization.
  if (settings_.sweep_around_obstacles) {
    if (!sweepAroundObstacles(solution)) {
      LOG(ERROR) << "Failed sweeping around obstacles.";
      return false;
    }
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

bool PolygonStripmapPlanner::sweepAroundObstacles(
    std::vector<Point_2>* solution) const {
  Point_2 goal = solution->back();
  solution->pop_back();
  std::vector<Point_2> waypoints;
  visibility_graph::VisibilityGraph visibility_graph(settings_.polygon);
  std::vector<std::vector<Point_2>> holes = settings_.polygon.getHoleVertices();

  for (size_t i = 0u; i < holes.size(); ++i) {
    std::vector<Point_2> hole = holes[i];
    waypoints.clear();
    visibility_graph.solve(solution->back(), hole.front(), &waypoints);
    solution->insert(solution->end(), waypoints.begin() + 1, waypoints.end());
  }

  std::vector<Point_2> hull = settings_.polygon.getHullVertices();
  waypoints.clear();
  visibility_graph.solve(solution->back(), hull.front(), &waypoints);
  solution->insert(solution->end(), waypoints.begin() + 1, waypoints.end());

  waypoints.clear();
  visibility_graph.solve(solution->back(), goal, &waypoints);
  solution->insert(solution->end(), waypoints.begin() + 1, waypoints.end() - 1);
  solution->push_back(goal);
  return true;
}

bool PolygonStripmapPlanner::runSolver(const Point_2& start,
                                       const Point_2& goal,
                                       std::vector<Point_2>* solution) const {
  CHECK_NOTNULL(solution);

  LOG(INFO) << "Start solving GTSP using GK MA.";
  return sweep_plan_graph_.solve(start, goal, solution);
}

}  // namespace mav_coverage_planning
