#ifndef MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_PATH_COST_FUNCTIONS_H_
#define MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_PATH_COST_FUNCTIONS_H_

#include <functional>
#include <vector>

#include <mav_coverage_planning_comm/cgal_definitions.h>

namespace mav_coverage_planning {

typedef std::function<double(const std::vector<Point_2>& path)>
    PathCostFunctionType;
double computeWaypointsPathCost(const std::vector<Point_2>& path);
// Given a vector of waypoints, compute its euclidean distances.
double computeEuclideanPathCost(const std::vector<Point_2>& path);

// Given two waypoints, compute its euclidean distance.
double computeEuclideanSegmentCost(const Point_2& from, const Point_2& to);

// Given a vector of waypoints, compute its time assuming rest-to-rest
// trapazoidal velocity profiles.
double computeVelocityRampPathCost(const std::vector<Point_2>& path,
                                   double v_max, double a_max);

// Given two waypoints, compute its rest-to-rest time assuming trapazoidal
// velocity profiles.
double computeVelocityRampSegmentCost(const Point_2& from, const Point_2& to,
                                      double v_max, double a_max);

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_PATH_COST_FUNCTIONS_H_
