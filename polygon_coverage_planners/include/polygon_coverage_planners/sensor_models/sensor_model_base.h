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

#ifndef POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_SENSOR_MODEL_BASE_H_
#define POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_SENSOR_MODEL_BASE_H_

namespace polygon_coverage_planning {

enum SensorModelType { kLine = 0, kFrustum };

inline bool checkSensorModelTypeValid(const int type) {
  return (type == SensorModelType::kLine) ||
         (type == SensorModelType::kFrustum);
}

inline std::string getSensorModelTypeName(const SensorModelType& type) {
  switch (type) {
    case SensorModelType::kLine:
      return "Line";
    case SensorModelType::kFrustum:
      return "Frustum";
    default:
      return "Unknown!";
  }
}

// Given the overlap parameters, this sensor model allows setting the sweep
// distance.
class SensorModelBase {
 public:
  SensorModelBase(double lateral_overlap)
      : sweep_distance_is_initialized_(false),
        lateral_overlap_(lateral_overlap) {}

  // The sweep distance to meet lateral coverage.
  inline double getSweepDistance() {
    if (!sweep_distance_is_initialized_) {
      computeSweepDistance();
      sweep_distance_is_initialized_ = true;
    }

    return sweep_distance_;
  }

 protected:
  inline virtual void computeSweepDistance() {
    sweep_distance_ = lateral_overlap_;
  }

  bool sweep_distance_is_initialized_;
  double sweep_distance_;
  double lateral_overlap_;
};

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_SENSOR_MODEL_BASE_H_
