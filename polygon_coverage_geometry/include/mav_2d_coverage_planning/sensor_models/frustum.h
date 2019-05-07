#ifndef MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_FRUSTUM_H_
#define MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_FRUSTUM_H_

#include <glog/logging.h>
#include <cmath>

#include "mav_2d_coverage_planning/sensor_models/sensor_model_base.h"

namespace mav_coverage_planning {

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
    double half_fov = 0.5 * fov;
    CHECK_GE(half_fov, 0.0);
    CHECK_LT(half_fov, 0.5 * M_PI);
    return 2.0 * altitude * std::tan(half_fov);
  }

  double lateral_footprint_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_FRUSTUM_H_
