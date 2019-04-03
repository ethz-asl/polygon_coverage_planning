#include "mav_2d_coverage_planning/geometry/sweep.h"

#include "mav_2d_coverage_planning/geometry/is_approx_y_monotone_2.h"
#include "mav_2d_coverage_planning/geometry/polygon.h"
#include "mav_2d_coverage_planning/graphs/visibility_graph.h"

namespace mav_coverage_planning {
bool computeSweep(const Polygon_2& in,
                  const visibility_graph::VisibilityGraph& visibility_graph,
                  const FT offset, const Direction_2& dir,
                  bool counter_clockwise, std::vector<Point_2>* waypoints) {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  Polygon poly_in(in);

  // Assertions.
  // TODO(rikba): Check monotone perpendicular to dir.
  if (!in.is_counterclockwise_oriented()) return false;

  // Find start sweep.
  Line_2 sweep(Point_2(0.0, 0.0), dir);
  std::vector<Point_2> sorted_pts = sortVerticesToLine(in, sweep);
  sweep = Line_2(sorted_pts.front(), dir);

  Vector_2 offset_vector = sweep.perpendicular(sorted_pts.front()).to_vector();
  offset_vector = offset * offset_vector /
                  std::sqrt(CGAL::to_double(offset_vector.squared_length()));
  const CGAL::Aff_transformation_2<K> kOffset(CGAL::TRANSLATION, offset_vector);
  std::vector<Point_2> intersections = findIntercections(in, sweep);
  while (!intersections.empty()) {
    // Sort intersections.
    if (counter_clockwise)
      std::reverse(intersections.begin(), intersections.end());
    // Move from previous sweep.
    if (!waypoints->empty()) {
      Polygon start_visibility, goal_visibility;
      if (!poly_in.computeVisibilityPolygon(waypoints->back(),
                                            &start_visibility)) {
        LOG(ERROR) << "Cannot compute visibility polygon from query point "
                   << waypoints->back() << " in polygon: " << poly_in;
        return false;
      }
      if (!poly_in.computeVisibilityPolygon(intersections.front(),
                                            &goal_visibility)) {
        LOG(ERROR) << "Cannot compute visibility polygon from query point "
                   << intersections.front() << " in polygon: " << poly_in;
        return false;
      }
      std::vector<Point_2> shortest_path;
      if(!visibility_graph.solve(waypoints->back(),
                                   start_visibility,
                                   intersections.front(),
                                   goal_visibility,
                                   &shortest_path)) {
        LOG(ERROR) << "Cannot compute shortest path from "
                   << waypoints->back() << " to "
                   << intersections.front() << " in polygon: " << poly_in;
        return false;
      }
    }
    // Traverse sweep.
    waypoints->push_back(intersections.front());
    waypoints->push_back(intersections.back());
    // Offset sweep.
    sweep.transform(kOffset);
    // Find new sweep interception.
    intersections = findIntercections(in, sweep);
  }

  return true;
}

std::vector<Point_2> sortVerticesToLine(const Polygon_2& p, const Line_2& l) {
  // Copy points.
  std::vector<Point_2> pts(p.size());
  std::vector<Point_2>::iterator pts_it = pts.begin();
  for (VertexConstIterator it = p.vertices_begin(); it != p.vertices_end();
       ++it) {
    *(pts_it++) = *it;
  }

  // Sort.
  std::sort(pts.begin(), pts.end(),
            [&l](const Point_2& a, const Point_2& b) -> bool {
              return CGAL::has_smaller_signed_distance_to_line(l, a, b);
            });

  return pts;
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

  CHECK_LE(intersections.size(), 4);

  // Sort.
  Line_2 perp_l = l.perpendicular(l.point(0));
  std::sort(intersections.begin(), intersections.end(),
            [&perp_l](const Point_2& a, const Point_2& b) -> bool {
              return CGAL::has_smaller_signed_distance_to_line(perp_l, a, b);
            });

  return intersections;
}

}  // namespace mav_coverage_planning
