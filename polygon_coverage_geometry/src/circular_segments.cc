#include "polygon_coverage_geometry/circular_segments.h"

namespace polygon_coverage_planning {

Point_2c toCircular(const Point_2& p) {
  K_to_Kc to_circular;
  return to_circular(p);
}

Point_2 fromCircular(const Point_2c& p) {
  Kc_to_K from_circular;
  return from_circular(p);
}

}  // namespace polygon_coverage_planning