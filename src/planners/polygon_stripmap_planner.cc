#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner.h"
#include <math.h>
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
      polygon_orig_ = settings_.polygon;
      Polygon p;
    if (!settings_.polygon.computeOffsetPolygon(settings_.wall_dist, &p)) {
      LOG(WARNING) << "Cannot shrink polygon:" << settings_.polygon
                   << "with distance: " << settings_.wall_dist;
    } else {
      settings_.polygon = p;
    }
    if (settings_.decomposition_type.compare("convex") == 0) {
      if (!settings_.polygon.computeConvexDecompositionFromPolygonWithHoles(
              &convex_decomposition_)) {
        LOG(ERROR) << "Cannot compute convex decomposition.";
        is_initialized_ = false;
      } else {
        LOG(INFO) << "Successfully created convex partition with "
                  << convex_decomposition_.size() << " convex polygon(s).";
      }
    }  else if (settings_.decomposition_type.compare("bcd") == 0) {
      if (!settings_.polygon.computeBCDFromPolygonWithHoles(
              &convex_decomposition_)) {
        LOG(ERROR) << "Cannot compute convex decomposition.";
        is_initialized_ = false;
      } else {
        LOG(INFO) << "Successfully created convex partition with "
                  << convex_decomposition_.size() << " convex polygon(s).";
      }
    }
  }

  // Create sweep plan graph.
  const double max_offset_distance =
      computeSweepDistance();
  LOG(INFO) << "Start creating sweep plan graph.";
  sweep_plan_graph_ = sweep_plan_graph::SweepPlanGraph(
      settings_.polygon, settings_.path_cost_function, convex_decomposition_,
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
  
  if (settings_.sweep_around_obstacles) {
    if (!sweepAroundObstacles(solution)){
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
                             std::vector<Point_2>* solution) const{
  Point_2 goal = solution->back();
  solution -> pop_back();
  Polygon p;
  if (!polygon_orig_.computeOffsetPolygon(settings_.robot_size, &p)) {
    LOG(WARNING) << "Cannot shrink polygon:" << polygon_orig_
                 << "with distance: " << settings_.robot_size;
  }
  std::vector<Point_2> waypoints;
  visibility_graph::VisibilityGraph visibility_graph(settings_.polygon, 
         computeSweepDistance());
  std::vector<std::vector<Point_2>> holes = p.getHoleVertices();
  
  for (size_t i = 0u; i < holes.size(); ++i) {
    std::vector<Point_2> hole = holes[i];
    waypoints.clear();
    visibility_graph.solve(solution -> back(), hole.front(), &waypoints);
    solution -> insert(solution -> end(), 
            waypoints.begin() + 1, waypoints.end());
    createCornerSweeps(hole, solution);
  }
  
  std::vector<Point_2> hull = p.getHullVertices();
  waypoints.clear();
  visibility_graph.solve(solution -> back(), hull.front(), &waypoints);
  solution -> insert(solution -> end(), waypoints.begin() + 1, waypoints.end());
  createCornerSweeps(hull, solution);
  
  waypoints.clear();
  visibility_graph.solve(solution -> back(), goal, &waypoints);
  solution -> insert(solution -> end(), waypoints.begin() + 1, 
          waypoints.end() - 1);
  solution -> push_back(goal);
  return true;
}

void PolygonStripmapPlanner::createCornerSweeps(
        std::vector<Point_2>& hull, 
        std::vector<Point_2>* solution) const {
   visibility_graph::VisibilityGraph visibility_graph(polygon_orig_, 
          computeSweepDistance());
  Point_2 start;
  float alpha_min = asin(settings_.robot_size/sqrt(pow(settings_.wall_dist, 2) +
                    pow(settings_.robot_size/2, 2)))*0.75;
  float theta = 0;
  Point_2 prev_point = solution -> back();
  Point_2 next_point;
  
  Vector_2 vector1 = hull[0] - prev_point;
  Vector_2 vector2 = next_point - hull[0];
  double length1 = sqrt(to_double(vector1.squared_length()));
  double length2 = sqrt(to_double(vector2.squared_length()));
  if (length1 != 0 && length2 != 0){
    theta = acos(to_double(vector1*vector2)/(length1*length2));
  } 
  Point_2 temp;
  if (theta > M_PI/2) {
    for (std::size_t i = 0u; i < floor((hull.size()-1)/2); 
            ++i) {
      temp = hull[i+1];
      hull[i+1] = hull[hull.size()-i-1];
      hull[hull.size()-i-1] = temp;
    }
  }
  
  for (std::size_t i = 0u; i < hull.size(); 
            ++i) {
    if ( i > 0 ) {
      prev_point = hull[i-1];
    } 
    if (i < hull.size() - 1) {
      next_point = hull[i+1];
    } else {
      next_point = hull.front();
    }
    vector1 = hull[i] - prev_point;
    vector2 = next_point - hull[i];
    length1 = sqrt(to_double(vector1.squared_length()));
    length2 = sqrt(to_double(vector2.squared_length()));
    if (length1 != 0 && length2 != 0){
      theta = acos(to_double(vector1*vector2)/(length1*length2));
    } else {
      continue;
    }
    if (theta > alpha_min) {
      float dist1 = (settings_.wall_dist + settings_.robot_size - 
              settings_.robot_size/sin(theta))*1.2;
      float dist2 = (cos(M_PI - theta)*dist1 + sin(M_PI-theta)*
              dist1/tan(alpha_min))*1.4;
      Point_2 point1 = prev_point + (hull[i] - prev_point)*
              (length1 - dist1)/length1;
      Point_2 point2 = hull[i] + (next_point - hull[i])*(dist2)/length2;
      
      if (i == 0) {
        vector1 = hull[i] - hull.back();
        length1 = sqrt(to_double(vector1.squared_length()));
        theta = acos(to_double(vector1*vector2)/(length1*length2));
        dist1 = (settings_.wall_dist + settings_.robot_size - 
                settings_.robot_size/sin(theta))*1.2;
        start = hull.back() + (hull[i] - hull.back())*(length1 - dist1)/length1;
      }
      
      solution -> push_back(point1);
        
      std::vector<Point_2> waypoints;
      visibility_graph.solve(solution -> back(), point2, &waypoints);
      solution -> insert(solution -> end(), waypoints.begin() + 1, 
              waypoints.end() - 1);
      
      if (waypoints.size() == 2) {
        solution -> push_back(point2);
      }
    } else {
      if (i == 0) {
        start = hull[i];
      }
      solution->push_back(hull[i]);
    }
  }
    solution->push_back(start);
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
