#ifndef POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_SENSOR_MODEL_BASE_H_
#define POLYGON_COVERAGE_PLANNERS_SENSOR_MODELS_SENSOR_MODEL_BASE_H_

namespace polygon_coverage_planning {

enum SensorModelType { kLine = 0, kFrustum };

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
