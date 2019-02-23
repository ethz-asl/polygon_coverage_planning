#ifndef MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_LINE_H_
#define MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_LINE_H_

#include "mav_2d_coverage_planning/sensor_models/sensor_model_base.h"

namespace mav_coverage_planning {

class Line : public SensorModelBase {
 public:
  Line(double lateral_footprint, double lateral_overlap)
      : SensorModelBase(lateral_overlap),
        lateral_footprint_(lateral_footprint) {}

 protected:
  inline void computeSweepDistance() override {
    sweep_distance_ = (1.0 - lateral_overlap_) * lateral_footprint_;
  }

  inline void computeOffsetDistance() override {
    offset_distance_ = 0.5 * lateral_footprint_;
  }

  double lateral_footprint_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_LINE_H_
