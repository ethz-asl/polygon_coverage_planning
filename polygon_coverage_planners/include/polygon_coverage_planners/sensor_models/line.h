#ifndef POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_LINE_H_
#define POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_LINE_H_

#include "mav_2d_coverage_planning/sensor_models/sensor_model_base.h"

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
