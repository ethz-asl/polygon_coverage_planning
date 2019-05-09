#ifndef POLYGON_COVERAGE_GEOMETRY_OFFSET_H_
#define POLYGON_COVERAGE_GEOMETRY_OFFSET_H_

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

// Offsets a specific polygon edge by cropping an along the edge infinitly
// long rectangular window with offset width.
bool offsetEdge(const Polygon_2& poly, const size_t& edge_id, double offset,
                Polygon_2* offset_polygon);

// Offset at most radial_offset from corner.
bool offsetEdgeWithRadialOffset(const Polygon_2& poly, const size_t& edge_id,
                                double radial_offset,
                                Polygon_2* offset_polygon);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_OFFSET_H_
