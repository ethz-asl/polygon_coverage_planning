#include <gtest/gtest.h>

#include "mav_coverage_graph_solvers/combinatorics.h"

using namespace mav_coverage_planner;

TEST(MathTest, KElementsFromNTest) {
  std::vector<size_t> sorted_elements = {0, 1, 2};
  std::vector<std::set<size_t>> combinations;

  for (size_t k = 0; k <= sorted_elements.size(); k++) {
    getAllCombinationsOfKElementsFromN(sorted_elements, k, &combinations);
    EXPECT_EQ(combinations.size(), nChooseK(sorted_elements.size(), k));
    for (const std::set<size_t>& combination : combinations) {
      EXPECT_EQ(combination.size(), k);
    }
    if (k == 1) {
      EXPECT_NE(combinations[0].find(0), combinations[0].end());
      EXPECT_NE(combinations[1].find(1), combinations[1].end());
      EXPECT_NE(combinations[2].find(2), combinations[2].end());
    } else if (k == 2) {
      EXPECT_NE(combinations[0].find(0), combinations[0].end());
      EXPECT_NE(combinations[0].find(1), combinations[0].end());

      EXPECT_NE(combinations[1].find(0), combinations[0].end());
      EXPECT_NE(combinations[1].find(2), combinations[0].end());

      EXPECT_NE(combinations[2].find(1), combinations[0].end());
      EXPECT_NE(combinations[2].find(2), combinations[0].end());
    } else if (k == 3) {
      EXPECT_NE(combinations[0].find(0), combinations[0].end());
      EXPECT_NE(combinations[0].find(1), combinations[0].end());
      EXPECT_NE(combinations[0].find(2), combinations[0].end());
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
