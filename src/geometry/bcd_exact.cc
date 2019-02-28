#include <glog/logging.h>
#include <vector>

#include <mav_2d_coverage_planning/geometry/bcd_exact.h>

namespace mav_coverage_planning {
std::vector<Polygon_2> computeBCD(const PolygonWithHoles& polygon_in,
                                  const Direction_2& dir) {
  // Rotate polygon to have direction aligned with x-axis.
  PolygonWithHoles rotated_polygon = rotatePolygon(polygon_in, -dir);

  // Sort vertices by x value.
  std::vector<VertexConstCirculator> sorted_vertices =
      getXSortedVertices(rotated_polygon);

  // Initialize edge list.
  std::list<Segment_2> L;
  std::list<Polygon_2> open_polygons;
  std::vector<Polygon_2> closed_polygons;
  for (const VertexConstCirculator& v : sorted_vertices) {
    processEvent(v, &L, &open_polygons, &closed_polygons);
  }

  return closed_polygons;
}

std::vector<VertexConstCirculator> getXSortedVertices(
    const PolygonWithHoles& p) {
  std::vector<VertexConstCirculator> sorted_vertices;

  // Get boundary vertices.
  VertexConstCirculator v = p.outer_boundary().vertices_circulator();
  do {
    sorted_vertices.push_back(v);
  } while (++v != p.outer_boundary().vertices_circulator());
  // Get hole vertices.
  for (PolygonWithHoles::Hole_const_iterator hit = p.holes_begin();
       hit != p.holes_end(); ++hit) {
    VertexConstCirculator vh = hit->vertices_circulator();
    do {
      sorted_vertices.push_back(vh);
    } while (++vh != hit->vertices_circulator());
  }
  // Sort.
  Polygon_2::Traits::Compare_x_2 compare_x_2;
  std::sort(sorted_vertices.begin(), sorted_vertices.end(),
            [&compare_x_2](const VertexConstCirculator& a,
                           const VertexConstCirculator& b) -> bool {
              return compare_x_2(*a, *b);
            });

  return sorted_vertices;
}

PolygonWithHoles rotatePolygon(const PolygonWithHoles& polygon_in,
                               const Direction_2& dir) {
  CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
  PolygonWithHoles rotated_polygon = polygon_in;
  rotated_polygon.outer_boundary() =
      CGAL::transform(rotation, polygon_in.outer_boundary());
  PolygonWithHoles::Hole_iterator hit_rot = rotated_polygon.holes_begin();
  for (PolygonWithHoles::Hole_const_iterator hit = polygon_in.holes_begin();
       hit != polygon_in.holes_end(); ++hit) {
    *(hit_rot++) = CGAL::transform(rotation, *hit);
  }

  return rotated_polygon;
}

void processEvent(const VertexConstCirculator& v, std::list<Segment_2>* L,
                  std::list<Polygon_2>* open_polygons,
                  std::vector<Polygon_2>* closed_polygons) {
  // Compute intersection.
  Line_2 l(*v, Direction_2(0, 1));
  std::vector<Intersection> intersections = getIntersections(*L, l);

  // Get adjacent edges.
  VertexConstCirculator v_prev = std::prev(v);
  VertexConstCirculator v_next = std::next(v);

  Polygon_2::Traits::Compare_y_2 compare_y_2;
  VertexConstCirculator v_lower =
      compare_y_2(*v_prev, *v_next) ? v_prev : v_next;
  VertexConstCirculator v_upper =
      !compare_y_2(*v_prev, *v_next) ? v_prev : v_next;

  Polygon_2::Traits::Compare_x_2 compare_x_2;

  if (compare_x_2(*v_lower, *v) && compare_x_2(*v_upper, *v)) {  // OUT
    Segment_2 e_lower(*v, *v_lower);
    Segment_2 e_upper(*v_upper, *v);
    // Close two cells, open one.
    // Delete e_lower and e_upper.
  } else if (!compare_x_2(*v_lower, *v) && !compare_x_2(*v_upper, *v)) {  // IN
    Segment_2 e_lower(*v_lower, *v);
    Segment_2 e_upper(*v, *v_upper);
    // Close current cell.
    if (!L->empty()) {
    }
    // Add e_lower and e_upper
    // 1. Find
  } else if (compare_x_2(*v_lower, *v) &&
             !compare_x_2(*v_upper, *v)) {  // MIDDLE 1
    Segment_2 e_lower(*v, *v_lower);
    Segment_2 e_upper(*v_upper, *v);
    // Update cell.
    // Delete e_lower, insert e_upper.
  } else {  // Middle 2
    Segment_2 e_lower(*v_lower, *v);
    Segment_2 e_upper(*v, *v_upper);
    // Update cell.
    // Delete e_lower, insert e_upper.
  }
}

std::vector<Intersection> getIntersections(const std::list<Segment_2>& L,
                                           const Line_2& l) {
  std::vector<Intersection> intersections(L.size());
  std::vector<Intersection>::iterator intersection = intersections.begin();
  for (std::list<Segment_2>::const_iterator it = L.begin(); it != L.end();
       ++it) {
    *(intersection++) = CGAL::intersection(*it, l);
    CHECK(*std::prev(intersection));
  }

  // // Intersect boundary edges.
  // EdgeConstCirculator e = p.outer_boundary().edges_circulator();
  // do {
  //   CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type result =
  //       CGAL::intersection(*e, l);
  //   if (result) intersections.emplace_back(e, result);
  // } while (++e != p.outer_boundary().edges_circulator());
  //
  // // Intersect hole edges.
  // for (PolygonWithHoles::Hole_const_iterator hit = p.holes_begin();
  //      hit != p.holes_end(); ++hit) {
  //   EdgeConstCirculator eh = hit->edges_circulator();
  //   do {
  //     CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type result =
  //         CGAL::intersection(*eh, l);
  //     if (result) intersections.emplace_back(eh, result);
  //   } while (++eh != hit->edges_circulator());
  // }

  return intersections;
}

}  // namespace mav_coverage_planning
