#include "polygon_coverage_planners/cost_functions/path_cost_functions.h"

#include <cmath>

namespace polygon_coverage_planning {

double computeWaypointsPathCost(const std::vector<Point_2>& path) {
  return path.size();
}

double computeEuclideanPathCost(const std::vector<Point_2>& path) {
  double distance = 0.0;
  for (size_t i = 0; i < path.size() - 1; i++)
    distance += computeEuclideanSegmentCost(path[i + 1], path[i]);

  return distance;
}

double computeEuclideanSegmentCost(const Point_2& from, const Point_2& to) {
  return std::sqrt(CGAL::to_double(Segment_2(from, to).squared_length()));
}

double computeVelocityRampPathCost(const std::vector<Point_2>& path,
                                   double v_max, double a_max) {
  double t = 0.0;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    t += computeVelocityRampSegmentCost(path[i + 1], path[i], v_max, a_max);
  }
  return t;
}

double computeVelocityRampSegmentCost(const Point_2& from, const Point_2& to,
                                      double v_max, double a_max) {
  ROS_ASSERT(v_max > 0.0);
  ROS_ASSERT(a_max > 0.0);

  const double distance = computeEuclideanSegmentCost(from, to);
  // Time to accelerate or decelerate to or from maximum velocity:
  const double acc_time = v_max / a_max;
  // Distance covered during complete acceleration or decelerate:
  const double acc_distance = 0.5 * v_max * acc_time;
  // Compute total segment time:
  if (distance < 2.0 * acc_distance) {
    // Case 1: Distance too small to accelerate to maximum velocity.
    return 2.0 * std::sqrt(distance / a_max);
  } else {
    // Case 2: Distance long enough to accelerate to maximum velocity.
    return 2.0 * acc_time + (distance - 2.0 * acc_distance) / v_max;
  }
}

}  // namespace polygon_coverage_planning
