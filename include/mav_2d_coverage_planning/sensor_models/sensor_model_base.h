#ifndef MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_SENSOR_MODEL_BASE_H_
#define MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_SENSOR_MODEL_BASE_H_

namespace mav_coverage_planning {

enum SensorModelType { kLine = 0, kFrustum };

// Given the overlap parameters, this sensor model allows setting the sweep
// distance.
class SensorModelBase {
 public:
  SensorModelBase(double lateral_overlap)
      : offset_distance_is_initialized_(false),
        sweep_distance_is_initialized_(false),
        lateral_overlap_(lateral_overlap) {}

  // The sweep distance to meet lateral coverage.
  inline double getSweepDistance() {
    if (!sweep_distance_is_initialized_) {
      computeSweepDistance();
      sweep_distance_is_initialized_ = true;
    }

    return sweep_distance_;
  }

  // The distance by which the polygon map can be offsetted in order to still
  // meet coverage.
  inline double getOffsetDistance() {
    if (!offset_distance_is_initialized_) {
      computeOffsetDistance();
      offset_distance_is_initialized_ = true;
    }

    return offset_distance_;
  }

 protected:
  inline virtual void computeOffsetDistance() {
    offset_distance_ = getSweepDistance();
  }
  inline virtual void computeSweepDistance() {
    sweep_distance_ = lateral_overlap_;
  }

  bool offset_distance_is_initialized_;
  bool sweep_distance_is_initialized_;
  double offset_distance_;
  double sweep_distance_;
  double lateral_overlap_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_SENSOR_MODELS_SENSOR_MODEL_BASE_H_
