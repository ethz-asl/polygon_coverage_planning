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

#ifndef POLYGON_COVERAGE_SOLVERS_GLKH_H_
#define POLYGON_COVERAGE_SOLVERS_GLKH_H_

#include <string>
#include <vector>

// Interfaces with the GLKH GTSP solver.
namespace polygon_coverage_planning {
namespace glkh {
struct Task {
  Task(const std::vector<std::vector<int>>& m,
       const std::vector<std::vector<int>>& clusters)
      : m(m), clusters(clusters) {}
  bool mIsSymmetric() const;
  bool mIsSquare() const;
  std::vector<std::vector<int>> m;
  std::vector<std::vector<int>> clusters;
};

// References GLKH. Singleton, because it may only be referenced once during
// runtime.
// https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
class Glkh {
 public:
  inline static Glkh& getInstance() {
    static Glkh instance;
    return instance;
  }
  Glkh(Glkh const&) = delete;
  void operator=(Glkh const&) = delete;

  void setSolver(const Task& task);
  bool solve();
  inline std::vector<int> getSolution() const { return solution_; }

 private:
  Glkh();
  ~Glkh() = default;

  std::vector<int> solution_;
};
}  // namespace glkh
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_SOLVERS_GLKH_H_
