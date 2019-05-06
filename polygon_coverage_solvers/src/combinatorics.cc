#include "polygon_coverage_solvers/combinatorics.h"

#include <ros/console.h>
#include <ros/assert.h>

namespace polygon_coverage_planning {

void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k,
    std::vector<std::set<size_t>>* result) {
  ROS_ASSERT(result);
  // Initialization.
  result->clear();
  std::vector<size_t> combination(k);
  getAllCombinationsOfKElementsFromN(sorted_elements, k, 0, &combination,
                                     result);
}

void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k, int start_pos,
    std::vector<size_t>* combination, std::vector<std::set<size_t>>* result) {
  // https://stackoverflow.com/questions/127704/algorithm-to-return-all-combinations-of-k-elements-from-n
  ROS_ASSERT(combination);
  ROS_ASSERT(result);
  // New combination found.
  if (k == 0) {
    std::set<size_t> combination_set;
    for (const size_t element : *combination) {
      combination_set.insert(element);
    }
    result->push_back(combination_set);
    return;
  }

  // Recursively add all possible "sub"-combinations.
  for (size_t i = start_pos; i <= sorted_elements.size() - k; i++) {
    // Pick first combination element.
    (*combination)[combination->size() - k] = sorted_elements[i];
    // Add all combinations of remaining elements.
    getAllCombinationsOfKElementsFromN(sorted_elements, k - 1, i + 1,
                                       combination, result);
  }
}

}  // namespace polygon_coverage_planning
