#include "polygon_coverage_geometry/boolean.h"
#include "polygon_coverage_geometry/cgal_definitions.h"

#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <ros/assert.h>
#include <ros/console.h>

namespace polygon_coverage_planning {

std::list<PolygonWithHoles> computeDifference(
    const std::list<Polygon_2>::const_iterator& hull,
    const std::list<Polygon_2>::const_iterator& holes_begin,
    const std::list<Polygon_2>::const_iterator& holes_end) {
  typedef CGAL::Gps_segment_traits_2<K> Traits_2;
  typedef CGAL::General_polygon_set_2<Traits_2> Polygon_set_2;

  Polygon_set_2 gps(*hull);
  for (auto h = holes_begin; h != holes_end; ++h) {
    gps.difference(*h);
  }

  std::list<PolygonWithHoles> res;
  gps.polygons_with_holes(std::back_inserter(res));
  return res;
}

}  // namespace polygon_coverage_planning
