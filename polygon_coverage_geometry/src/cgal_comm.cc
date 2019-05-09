#include "polygon_coverage_geometry/cgal_comm.h"

#include <ros/assert.h>

namespace polygon_coverage_planning {

bool pointInPolygon(const PolygonWithHoles& pwh, const Point_2& p) {
  // Point inside outer boundary.
  CGAL::Bounded_side result =
      CGAL::bounded_side_2(pwh.outer_boundary().vertices_begin(),
                           pwh.outer_boundary().vertices_end(), p, K());
  if (result == CGAL::ON_UNBOUNDED_SIDE) return false;

  // Point outside hole.
  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    result = CGAL::bounded_side_2(hit->vertices_begin(), hit->vertices_end(), p,
                                  K());
    if (result == CGAL::ON_BOUNDED_SIDE) return false;
  }

  return true;
}

bool pointsInPolygon(const PolygonWithHoles& pwh,
                     const std::vector<Point_2>::iterator& begin,
                     const std::vector<Point_2>::iterator& end) {
  for (std::vector<Point_2>::iterator it = begin; it != end; ++it) {
    if (!pointInPolygon(pwh, *it)) return false;
  }
  return true;
}

bool isStrictlySimple(const PolygonWithHoles& pwh) {
  for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
       hi != pwh.holes_end(); ++hi)
    if (!hi->is_simple()) return false;
  return pwh.outer_boundary().is_simple();
}

Point_2 projectOnPolygon2(const Polygon_2& poly, const Point_2& p,
                          FT* squared_distance) {
  ROS_ASSERT(squared_distance);

  // Find the closest edge.
  std::vector<std::pair<FT, EdgeConstIterator>> edge_distances(poly.size());
  std::vector<std::pair<FT, EdgeConstIterator>>::iterator dit =
      edge_distances.begin();
  for (EdgeConstIterator eit = poly.edges_begin(); eit != poly.edges_end();
       eit++, dit++) {
    dit->first = CGAL::squared_distance(*eit, p);
    dit->second = eit;
  }

  std::vector<std::pair<FT, EdgeConstIterator>>::iterator closest_pair =
      std::min_element(edge_distances.begin(), edge_distances.end(),
                       [](const std::pair<FT, EdgeConstIterator>& lhs,
                          const std::pair<FT, EdgeConstIterator>& rhs) {
                         return lhs.first < rhs.first;
                       });

  EdgeConstIterator closest_edge = closest_pair->second;
  *squared_distance = closest_pair->first;

  // Project p on supporting line of closest edge.
  Point_2 projection = closest_edge->supporting_line().projection(p);
  // Check if p is on edge. If not snap it to source or target.
  if (!closest_edge->has_on(projection)) {
    FT d_source = CGAL::squared_distance(p, closest_edge->source());
    FT d_target = CGAL::squared_distance(p, closest_edge->target());
    projection =
        d_source < d_target ? closest_edge->source() : closest_edge->target();
  }

  return projection;
}

Point_2 projectPointOnHull(const PolygonWithHoles& pwh, const Point_2& p) {
  // Project point on outer boundary.
  FT min_distance;
  Point_2 projection =
      projectOnPolygon2(pwh.outer_boundary(), p, &min_distance);

  // Project on holes.
  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    FT temp_distance;
    Point_2 temp_projection = projectOnPolygon2(*hit, p, &temp_distance);
    if (temp_distance < min_distance) {
      min_distance = temp_distance;
      projection = temp_projection;
    }
  }

  return projection;
}

FT computeArea(const PolygonWithHoles& pwh) {
  FT area =
      CGAL::abs(CGAL::polygon_area_2(pwh.outer_boundary().vertices_begin(),
                                     pwh.outer_boundary().vertices_end(), K()));
  for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
       hi != pwh.holes_end(); ++hi)
    area -= CGAL::abs(
        CGAL::polygon_area_2(hi->vertices_begin(), hi->vertices_end(), K()));
  return area;
}

void simplifyPolygon(Polygon_2* polygon) {
  ROS_ASSERT(polygon);

  std::vector<Polygon_2::Vertex_circulator> v_to_erase;

  Polygon_2::Vertex_circulator vc = polygon->vertices_circulator();
  // Find collinear vertices.
  do {
    if (CGAL::collinear(*std::prev(vc), *vc, *std::next(vc))) {
      v_to_erase.push_back(vc);
    }
  } while (++vc != polygon->vertices_circulator());

  // Remove intermediate vertices.
  for (std::vector<Polygon_2::Vertex_circulator>::reverse_iterator rit =
           v_to_erase.rbegin();
       rit != v_to_erase.rend(); ++rit) {
    polygon->erase(*rit);
  }
}

void simplifyPolygon(PolygonWithHoles* pwh) {
  ROS_ASSERT(pwh);

  simplifyPolygon(&pwh->outer_boundary());

  for (PolygonWithHoles::Hole_iterator hi = pwh->holes_begin();
       hi != pwh->holes_end(); ++hi)
    simplifyPolygon(&*hi);
}

}  // namespace polygon_coverage_planning
