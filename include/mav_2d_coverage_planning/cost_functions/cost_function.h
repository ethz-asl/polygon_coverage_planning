#ifndef MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_COST_FUNCTION_H_
#define MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_COST_FUNCTION_H_

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

namespace mav_coverage_planning {
class CostFunction {
 public:
  enum Type {
    kEuclidean = 0,    // Minimize distance.
    kVelocityRampTime  // Minimize flight time.
  };

  CostFunction() {}

  typedef CGAL::Exact_predicates_exact_constructions_kernel K;
  typedef K::Point_2 Point_2;
  typedef K::Segment_2 Segment_2;
  virtual double computeCost(const std::vector<Point_2>& path) const = 0;

  virtual double computeCost(const Point_2& from,
                             const Point_2& to) const = 0;
};
}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_COST_FUNCTIONS_COST_FUNCTION_H_
