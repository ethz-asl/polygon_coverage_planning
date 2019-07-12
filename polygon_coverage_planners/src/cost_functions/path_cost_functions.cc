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

#include "polygon_coverage_planners/cost_functions/path_cost_functions.h"

#include <cmath>
#include <ros/assert.h>

namespace polygon_coverage_planning {

double computeWaypointsPathCost(const std::vector<Point_2>& path) {
  return path.size();
}

double computeEuclideanPathCost(const std::vector<Point_2>& path) {
  double distance = 0.0;
  for (size_t i = 0; i < path.size() - 1; i++)
    distance += computeEuclideanSegmentCost(path[i + 1], path[i]);

  return distance;
}

double computeEuclideanSegmentCost(const Point_2& from, const Point_2& to) {
  return std::sqrt(CGAL::to_double(Segment_2(from, to).squared_length()));
}

double computeVelocityRampPathCost(const std::vector<Point_2>& path,
                                   double v_max, double a_max) {
  double t = 0.0;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    t += computeVelocityRampSegmentCost(path[i + 1], path[i], v_max, a_max);
  }
  return t;
}

double computeVelocityRampSegmentCost(const Point_2& from, const Point_2& to,
                                      double v_max, double a_max) {
  ROS_ASSERT(v_max > 0.0);
  ROS_ASSERT(a_max > 0.0);

  const double distance = computeEuclideanSegmentCost(from, to);
  // Time to accelerate or decelerate to or from maximum velocity:
  const double acc_time = v_max / a_max;
  // Distance covered during complete acceleration or decelerate:
  const double acc_distance = 0.5 * v_max * acc_time;
  // Compute total segment time:
  if (distance < 2.0 * acc_distance) {
    // Case 1: Distance too small to accelerate to maximum velocity.
    return 2.0 * std::sqrt(distance / a_max);
  } else {
    // Case 2: Distance long enough to accelerate to maximum velocity.
    return 2.0 * acc_time + (distance - 2.0 * acc_distance) / v_max;
  }
}

}  // namespace polygon_coverage_planning
