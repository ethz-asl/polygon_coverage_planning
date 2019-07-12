/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

PolygonWithHoles rotatePolygon(const PolygonWithHoles& polygon_in,
                               const Direction_2& dir) {
  CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
  rotation = rotation.inverse();
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

void sortVertices(PolygonWithHoles* pwh) {
  if (pwh->outer_boundary().is_clockwise_oriented())
    pwh->outer_boundary().reverse_orientation();

  for (PolygonWithHoles::Hole_iterator hi = pwh->holes_begin();
       hi != pwh->holes_end(); ++hi)
    if (hi->is_counterclockwise_oriented()) hi->reverse_orientation();
}

std::vector<Point_2> getHullVertices(const PolygonWithHoles& pwh) {
  std::vector<Point_2> vec(pwh.outer_boundary().size());
  std::vector<Point_2>::iterator vecit = vec.begin();
  for (VertexConstIterator vit = pwh.outer_boundary().vertices_begin();
       vit != pwh.outer_boundary().vertices_end(); ++vit, ++vecit)
    *vecit = *vit;
  return vec;
}

std::vector<std::vector<Point_2>> getHoleVertices(const PolygonWithHoles& pwh) {
  std::vector<std::vector<Point_2>> hole_vertices(pwh.number_of_holes());
  std::vector<std::vector<Point_2>>::iterator hvit = hole_vertices.begin();
  for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
       hi != pwh.holes_end(); ++hi, ++hvit) {
    hvit->resize(hi->size());
    std::vector<Point_2>::iterator it = hvit->begin();
    for (VertexConstIterator vit = hi->vertices_begin();
         vit != hi->vertices_end(); ++vit, ++it)
      *it = *vit;
  }
  return hole_vertices;
}

}  // namespace polygon_coverage_planning
