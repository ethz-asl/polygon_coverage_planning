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

#include "polygon_coverage_solvers/glkh.h"
#include "polygon_coverage_solvers/utils.h"

#include <ros/assert.h>
#include <ros/console.h>
#include <ros/package.h>

#include <fstream>
#include <sstream>
#include <filesystem>

namespace polygon_coverage_planning {
namespace glkh {

const std::string GLKH_EXE = "rosrun polygon_coverage_solvers run_glkh.bash";
const std::filesystem::path TMP_DIR{"/tmp/glkh/"};
const std::filesystem::path PROBLEM_FILE = TMP_DIR / "sweep_plan.gtsp";
const std::filesystem::path TOUR_FILE = TMP_DIR / "sweep_plan.tour";

bool Task::mIsSquare() const {
  for (size_t i = 0; i < m.size(); ++i) {
    if (m[i].size() != m.size()) {
      return false;
    }
  }
  return true;
}

bool Task::mIsSymmetric() const {
  if (!mIsSquare()) {
    return false;
  }
  for (size_t i = 0; i < m.size(); ++i) {
    for (size_t j = 0; j < m[i].size(); ++j) {
      if (m[i][j] != m[j][i]) {
        return false;
      }
    }
  }
  return true;
}

Glkh::Glkh() {}

void Glkh::setSolver(const Task& task) {
  // Create problem file in GTSPLIB format
  // see: http://webhotel4.ruc.dk/~keld/research/GLKH/GLKH_Report.pdf

  std::string name = "sweep_plan";
  std::string type = task.mIsSymmetric() ? "GTSP" : "AGTSP";
  std::string comment = "Created by polygon_coverage_solvers.";
  size_t dimension = task.m.size();
  size_t num_sets = task.clusters.size();
  std::string edge_weight_type = "EXPLICIT";
  std::string edge_weight_format = "FULL_MATRIX";

  std::filesystem::create_directory(TMP_DIR);
  std::ofstream f(PROBLEM_FILE);
  f << "Name: " << name << std::endl;
  f << "Type: " << type << std::endl;
  f << "COMMENT: " << comment << std::endl;
  f << "DIMENSION: " << dimension << std::endl;
  f << "GTSP_SETS: " << num_sets << std::endl;
  f << "EDGE_WEIGHT_TYPE: " << edge_weight_type << std::endl;
  f << "EDGE_WEIGHT_FORMAT: " << edge_weight_format << std::endl;

  f << "EDGE_WEIGHT_SECTION:" << std::endl;
  for (size_t i = 0; i < task.m.size(); i++)
  {
    for (auto w_ij : task.m[i])
    {
      if (w_ij == 2147483647)
      {
        // TODO: FIX Overflow
        w_ij = 9999999;
      }
      f << w_ij << " ";
    }
    f << std::endl;
  }

  f << "GTSP_SET_SECTION:" << std::endl;
  for (size_t i = 0; i < task.clusters.size(); i++)
  {
    f << i + 1 << " ";
    for (auto vertex_idx : task.clusters[i])
    {
      f << vertex_idx + 1 << " ";
    }
    f << -1 << std::endl;
  }

  f << "EOF" << std::endl;
  f.close();
}

bool Glkh::solve() {
  std::filesystem::current_path(TMP_DIR);
  std::string cmd = GLKH_EXE + " " + PROBLEM_FILE.string() + " " + TOUR_FILE.string();
  bool error = system(cmd.c_str());
  if (error)
  {
    return false;
  }

  // Read solution file from disk
  std::ifstream infile(TOUR_FILE);
  std::string line;
  int dimension = -1;
  while (std::getline(infile, line))
  {
    if (line != "TOUR_SECTION")
    {
      std::istringstream iss(line);
      std::string key;
      if (std::getline(iss, key, ':'))
      {
        trim(key);
        if (key == "DIMENSION")
        {
          iss >> dimension;
        }
      }
    }
    else
    {
      assert(dimension > -1);
      solution_.resize(dimension);
      for(int i = 0; i < dimension && std::getline(infile, line); i++)
      {
        std::istringstream iss(line);
        int n;
        iss >> n;
        solution_[i] = n - 1;
      }
      assert(std::getline(infile, line) && line == "-1");
    }
  }

  infile.close();
  return true;
}

}  // namespace glkh
}  // namespace polygon_coverage_planning
