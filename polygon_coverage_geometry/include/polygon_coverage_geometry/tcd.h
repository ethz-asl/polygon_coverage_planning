#ifndef POLYGON_COVERAGE_GEOMETRY_TCD_H_
#define POLYGON_COVERAGE_GEOMETRY_TCD_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

// Wrapper functions to interface trapezoidal decomposition (TCD).
namespace polygon_coverage_planning {

std::vector<Polygon_2> computeTCD(const PolygonWithHoles& polygon_in,
                                  const Direction_2& dir);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_TCD_H_
