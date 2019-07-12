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

#ifndef POLYGON_COVERAGE_PLANNERS_COST_FUNCTIONS_PATH_COST_FUNCTIONS_H_
#define POLYGON_COVERAGE_PLANNERS_COST_FUNCTIONS_PATH_COST_FUNCTIONS_H_

#include <functional>
#include <vector>

#include <polygon_coverage_geometry/cgal_definitions.h>

namespace polygon_coverage_planning {

typedef std::function<double(const std::vector<Point_2>& path)>
    PathCostFunction;

// Returns the number of waypoints as cost.
double computeWaypointsPathCost(const std::vector<Point_2>& path);

// Given a vector of waypoints, compute its euclidean distances.
double computeEuclideanPathCost(const std::vector<Point_2>& path);

// Given two waypoints, compute its euclidean distance.
double computeEuclideanSegmentCost(const Point_2& from, const Point_2& to);

// Given a vector of waypoints, compute its time assuming rest-to-rest
// trapazoidal velocity profiles.
double computeVelocityRampPathCost(const std::vector<Point_2>& path,
                                   double v_max, double a_max);

// Given two waypoints, compute its rest-to-rest time assuming trapazoidal
// velocity profiles.
double computeVelocityRampSegmentCost(const Point_2& from, const Point_2& to,
                                      double v_max, double a_max);

enum CostFunctionType {
  kDistance = 0,  // Minimize distance.
  kTime,          // Minimize flight time.
  kWaypoints
};

inline bool checkCostFunctionTypeValid(const int type) {
  return (type == CostFunctionType::kDistance) ||
         (type == CostFunctionType::kTime) ||
         (type == CostFunctionType::kWaypoints);
}

inline std::string getCostFunctionTypeName(const CostFunctionType& type) {
  switch (type) {
    case CostFunctionType::kDistance:
      return "Euclidean distance";
    case CostFunctionType::kTime:
      return "Time";
    case CostFunctionType::kWaypoints:
      return "Waypoints";
    default:
      return "Unknown!";
  }
}

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_COST_FUNCTIONS_PATH_COST_FUNCTIONS_H_
