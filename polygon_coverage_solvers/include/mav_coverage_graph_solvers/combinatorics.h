#ifndef MAV_COVERAGE_GRAPH_SOLVERS_COMBINATORICS_H_
#define MAV_COVERAGE_GRAPH_SOLVERS_COMBINATORICS_H_

#include <cstddef>
#include <set>
#include <vector>

namespace mav_coverage_planning {

// Calculate the factorial
inline int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

// Calculate the binomial coefficient "n choose k".
inline int nChooseK(int n, int k) {
  return factorial(n) / (factorial(k) * factorial(n - k));
}

// Pick all of the k-element combinations from an ordered set.
void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k,
    std::vector<std::set<size_t>>* result);
// The recursive call.
void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k, int start_pos,
    std::vector<size_t>* combination, std::vector<std::set<size_t>>* result);

}  // namespace mav_coverage_planning

#endif /* MAV_COVERAGE_GRAPH_SOLVERS_COMBINATORICS_H_ */
