#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/cgal_definitions.h"
#include "polygon_coverage_geometry/tcd.h"

#include <CGAL/Polygon_vertical_decomposition_2.h>

namespace polygon_coverage_planning {

std::vector<Polygon_2> computeTCD(const PolygonWithHoles& polygon_in,
                                  const Direction_2& dir) {
  // Rotate polygon to have direction aligned with x-axis.
  // TODO(rikba): Make this independent of rotation.
  PolygonWithHoles rotated_polygon = rotatePolygon(polygon_in, dir);

  // TCD
  std::vector<Polygon_2> traps;
  CGAL::Polygon_vertical_decomposition_2<K> decom;
  decom(polygon_in, std::back_inserter(traps));

  // Rotate back all polygons.
  for (auto& p : traps) {
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    p = CGAL::transform(rotation, p);
  }

  return traps;
}

}  // namespace polygon_coverage_planning
