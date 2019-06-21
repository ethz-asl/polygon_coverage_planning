#ifndef POLYGON_COVERAGE_GEOMETRY_DECOMPOSITION_H_
#define POLYGON_COVERAGE_GEOMETRY_DECOMPOSITION_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Get all unique polygon edge directions including opposite directions.
std::vector<Direction_2> findEdgeDirections(const PolygonWithHoles& pwh);

// Get all directions that are perpendicular to the edges found with
// findEdgeDirections.
std::vector<Direction_2> findPerpEdgeDirections(const PolygonWithHoles& pwh);

// Find the best edge direction to sweep. The best direction is the direction
// with the smallest polygon altitude. Returns the smallest altitude.
double findBestSweepDir(const Polygon_2& cell, Direction_2* best_dir = nullptr);

// Compute BCDs for every edge direction. Return any with the smallest possible
// altitude sum.
bool computeBestBCDFromPolygonWithHoles(
    std::vector<PolygonWithHoles>* bcd_polygons);

// Compute TCDs for every edge direction. Return any with the smallest possible
// altitude sum.
bool computeBestTCDFromPolygonWithHoles(std::vector<Polygon_2>* trap_polygons);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_DECOMPOSITION_H_
