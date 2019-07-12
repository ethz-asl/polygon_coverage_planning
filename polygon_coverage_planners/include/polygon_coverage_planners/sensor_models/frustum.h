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

#ifndef POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_FRUSTUM_H_
#define POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_FRUSTUM_H_

#include <ros/assert.h>
#include <cmath>

#include "polygon_coverage_planners/sensor_models/sensor_model_base.h"

namespace polygon_coverage_planning {

class Frustum : public SensorModelBase {
 public:
  Frustum(double altitude, double lateral_fov, double lateral_overlap)
      : SensorModelBase(lateral_overlap),
        lateral_footprint_(computeFootprint(altitude, lateral_fov)) {}

 protected:
  inline void computeSweepDistance() override {
    sweep_distance_ = (1.0 - lateral_overlap_) * lateral_footprint_;
  }

  inline double computeFootprint(double altitude, double fov) const {
    ROS_ASSERT(fov > 0.0);
    ROS_ASSERT(fov < M_PI);

    double half_fov = 0.5 * fov;
    return 2.0 * altitude * std::tan(half_fov);
  }

  double lateral_footprint_;
};

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_FRUSTUM_H_
