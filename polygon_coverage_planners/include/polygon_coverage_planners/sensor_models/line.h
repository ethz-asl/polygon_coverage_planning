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

#ifndef POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_LINE_H_
#define POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_LINE_H_

#include "polygon_coverage_planners/sensor_models/sensor_model_base.h"

namespace polygon_coverage_planning {

class Line : public SensorModelBase {
 public:
  Line(double lateral_footprint, double lateral_overlap)
      : SensorModelBase(lateral_overlap),
        lateral_footprint_(lateral_footprint) {}

 protected:
  inline void computeSweepDistance() override {
    sweep_distance_ = (1.0 - lateral_overlap_) * lateral_footprint_;
  }

  double lateral_footprint_;
};

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_LINE_H_
