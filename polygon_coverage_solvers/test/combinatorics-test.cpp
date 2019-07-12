/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include "polygon_coverage_solvers/combinatorics.h"

using namespace polygon_coverage_planning;

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
