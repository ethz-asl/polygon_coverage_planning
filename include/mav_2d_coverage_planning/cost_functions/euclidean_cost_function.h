#ifndef MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_EUCLIDEAN_COST_FUNCTION_H_
#define MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_EUCLIDEAN_COST_FUNCTION_H_

#include "mav_2d_coverage_planning/cost_functions/cost_function.h"

namespace mav_coverage_planning {

class EuclideanCostFunction : public CostFunction {
 public:
  EuclideanCostFunction() : CostFunction() {}

  // Given a vector of waypoints, compute its euclidean distances.
  double computeCost(const std::vector<Point_2>& path) const override;

  // Given two waypoints, compute its euclidean distance.
  double computeCost(const Point_2& from,
                     const Point_2& to) const override;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_EUCLIDEAN_COST_FUNCTION_H_
