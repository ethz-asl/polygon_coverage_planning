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
bool computeBestBCDFromPolygonWithHoles(const PolygonWithHoles& pwh,
                                        std::vector<Polygon_2>* bcd_polygons);

// Compute TCDs for every edge direction. Return any with the smallest possible
// altitude sum.
bool computeBestTCDFromPolygonWithHoles(const PolygonWithHoles& pwh,
                                        std::vector<Polygon_2>* trap_polygons);

enum DecompositionType {
  kBCD = 0,  // Boustrophedon.
  kTCD       // Trapezoidal.
};

inline bool checkDecompositionTypeValid(const int type) {
  return (type == DecompositionType::kBCD) || (type == DecompositionType::kTCD);
}

inline std::string getDecompositionTypeName(const DecompositionType& type) {
  switch (type) {
    case DecompositionType::kBCD:
      return "Boustrophedon Cell Decomposition";
    case DecompositionType::kTCD:
      return "Trapezoidal Cell Decomposition";
    default:
      return "Unknown!";
  }
}

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_DECOMPOSITION_H_
