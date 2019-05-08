#ifndef POLYGON_COVERAGE_GEOMETRY_BCD_H_
#define POLYGON_COVERAGE_GEOMETRY_BCD_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Given a non-degenerate counter-clockwise weakly-simple polygon with
// holes, compute the maximum offset polygon such that no edge collapses.
// Aichholzer, Oswin, et al. "A novel type of skeleton for polygons." J. UCS
// The Journal of Universal Computer Science. Springer, Berlin, Heidelberg,
// 1996. 752-761.
void computeOffsetPolygon(const PolygonWithHoles& pwh, FT max_offset,
                          PolygonWithHoles* offset_polygon);

bool checkValidOffset(
    const PolygonWithHoles& original,
    const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset);

}  // namespace polygon_coverage_geometry

#endif  // POLYGON_COVERAGE_GEOMETRY_BCD_H_
