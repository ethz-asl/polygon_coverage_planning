#include "mav_2d_coverage_planning/geometry/sweep.h"

#include "mav_2d_coverage_planning/geometry/is_approx_y_monotone_2.h"
#include "mav_2d_coverage_planning/graphs/visibility_graph.h"

namespace mav_coverage_planning {
bool computeSweep(const Polygon_2& in,
                  const visibility_graph::VisibilityGraph& visibility_graph,
                  const FT offset, const Direction_2& dir,
                  bool counter_clockwise, std::vector<Point_2>* waypoints) {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  // Rotate polygon in direction.
  CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e3);
  rotation = rotation.inverse();
  Polygon_2 poly = CGAL::transform(rotation, poly);

  // Assertions.
  if (!CGAL::is_approx_y_monotone_2(poly.vertices_begin(), poly.vertices_end()))
    return false;
  if (!poly.is_counterclockwise_oriented()) return false;

  // Find start vertex.
  Point_2 start =
      counter_clockwise ? findApproxSouthWest(poly) : findApproxSouthEast(poly);

  const Vector_2 kOffset(0.0, offset);
  Line_2 horz_line(*start, Direction_2(1.0, 0.0));
  while (!intersections.empty()) {
    // Find interception of horizontal sweep.
    std::vector<Point_2> intersections = findIntercections(poly, horz_line);
    // Sort intersections.
    if (!counter_clockwise)
      std::reverse(intersections.begin(), intersections.end());
    Point_2 start = intersections.front();
  }

  return true;
}

Point_2 findApproxSouthEast(const Polygon_2& p) {
  VertexConstIterator v = p.vertices_begin();
  Polygon_2::Traits::Less_y_2 less_y_2;
  Polygon_2::Traits::Less_x_2 less_x_2;
  for (VertexConstIterator it = p.vertices_begin(); it != p.vertices_end();
       ++it) {
    if (isApproxEqual(it->y(), v->y())) {
      *v = less_x_2(*v, *it) ? *it : *v;
    } else if (less_y_2(*it, *v)) {
      *v = *it;
    }
  }

  return *v;
}

Point_2 findApproxSouthWest(const Polygon_2& p) {
  VertexConstIterator v = p.vertices_begin();
  Polygon_2::Traits::Less_y_2 less_y_2;
  Polygon_2::Traits::Less_x_2 less_x_2;
  for (VertexConstIterator it = p.vertices_begin(); it != p.vertices_end();
       ++it) {
    if (isApproxEqual(it->y(), v->y())) {
      *v = less_x_2(*it, *v) ? *it : *v;
    } else if (less_y_2(*it, *v)) {
      *v = *it;
    }
  }

  return *v;
}

bool isApproxEqual(const FT a, const FT b) {
  const double kPrecision = 1.0e-3;
  return std::fabs(CGAL::to_double(a) - CGAL::to_double(b)) < kPrecision;
}

std::vector<Point_2> findIntercections(const Polygon_2& p, const Line_2& l) {
  std::vector<Point_2> intersections;
  typedef CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type
      Intersection;

  for (EdgeConstIterator it = p.edges_begin(); it != p.edges_end(); ++it) {
    Intersection result = CGAL::intersection(*it, l);
    if (result) {
      if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
        intersections.push_back(s->source());
        intersections.push_back(s->target());
      } else {
        intersections.push_back(*boost::get<Point_2>(&*result));
      }
    }
  }

  // Remove redundant points.
  Polygon_2::Traits::Less_x_2 less_x_2;
  std::sort(intersections.begin(), intersections.end(),
            [&less_x_2](const Point_2& a, const Point_2& b) -> bool {
              return less_x_2(a, b);
            });
  intersections.erase(std::unique(intersections.begin(), intersections.end()),
                      intersections.end());

  return intersections;
}

}  // namespace mav_coverage_planning
