#ifndef POLYGON_COVERAGE_GEOMETRY_BOOLEAN_H_
#define POLYGON_COVERAGE_GEOMETRY_BOOLEAN_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

// Wrapper functions to form the union of a list of polygons.
namespace polygon_coverage_planning {

// Compute difference between hull and holes.
std::list<PolygonWithHoles> computeDifference(
    const std::list<Polygon_2>::const_iterator& hull,
    const std::list<Polygon_2>::const_iterator& holes_begin,
    const std::list<Polygon_2>::const_iterator& holes_end);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_BOOLEAN_H_
