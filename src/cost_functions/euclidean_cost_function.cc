#include "mav_2d_coverage_planning/cost_functions/euclidean_cost_function.h"

#include <cmath>

namespace mav_coverage_planning {

double computeEuclideanPathCost(const std::vector<Point_2>& path) {
  double distance = 0.0;
  for (size_t i = 0; i < path.size() - 1; i++)
    distance += computeEuclideanSegmentCost(path[i + 1], path[i]);

  return distance;
}

double computeEuclideanSegmentCost(const Point_2& from, const Point_2& to) {
  return std::sqrt(CGAL::to_double(Segment_2(from, to).squared_length()));
}

}  // namespace mav_coverage_planning
