#ifndef POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_FRUSTUM_H_
#define POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_FRUSTUM_H_

#include <cmath>
#include <ros/assert.h>

#include "mav_2d_coverage_planning/sensor_models/sensor_model_base.h"

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
