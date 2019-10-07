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
#include "polygon_coverage_geometry/offset.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Cartesian_converter.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <boost/make_shared.hpp>

#include <ros/assert.h>
#include <ros/console.h>

namespace polygon_coverage_planning {

void computeOffsetPolygon(const PolygonWithHoles& pwh, FT max_offset,
                          PolygonWithHoles* offset_polygon) {
  ROS_ASSERT(offset_polygon);

  PolygonWithHoles sorted_pwh = pwh;
  sortVertices(&sorted_pwh);

  // TODO(rikba): Check weak simplicity.

  // Try maximum offsetting.
  std::vector<boost::shared_ptr<PolygonWithHoles>> result =
      CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
          max_offset, sorted_pwh);
  if (checkValidOffset(sorted_pwh, result)) {
    *offset_polygon = *result.front();
    return;
  } else {
    ROS_WARN(
        "Polygon offsetting changes topology. Reducing offsetting distance.");
    result = {boost::make_shared<PolygonWithHoles>(sorted_pwh)};
  }

  // Binary search for smaller valid offset.
  FT min = 0.0;
  FT max = max_offset;
  const FT kBinarySearchResolution = 0.1;
  while (max - min > kBinarySearchResolution) {
    const FT mid = (min + max) / 2.0;
    std::vector<boost::shared_ptr<PolygonWithHoles>> temp_result =
        CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
            mid, sorted_pwh);
    if (checkValidOffset(sorted_pwh, temp_result)) {
      min = mid;
      result = temp_result;
    } else {
      max = mid;
    }
  }

  *offset_polygon = *result.front();
}

bool checkValidOffset(
    const PolygonWithHoles& original,
    const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset) {
  // Valid if number of vertices remains constant.
  if (offset.size() != 1) {
    return false;
  }

  PolygonWithHoles::Hole_const_iterator offset_hit =
      offset.front()->holes_begin();
  for (PolygonWithHoles::Hole_const_iterator original_hit =
           original.holes_begin();
       original_hit != original.holes_end(); ++original_hit, ++offset_hit) {
    if (original_hit->size() != offset_hit->size()) return false;
  }
  return true;
}

bool offsetEdge(const Polygon_2& poly, const size_t& edge_id, double offset,
                Polygon_2* offset_polygon) {
  ROS_ASSERT(offset_polygon);
  *offset_polygon = poly;

  if (!poly.is_simple()) {
    ROS_WARN_STREAM("Polygon is not simple.");
    return false;
  }

  // Create mask.
  // Copy of polygon with edge start being first vertex.
  Polygon_2 polygon;
  Polygon_2::Vertex_circulator vc_start =
      std::next(poly.vertices_circulator(), edge_id);
  Polygon_2::Vertex_circulator vc = vc_start;
  do {
    polygon.push_back(*vc);
  } while (++vc != vc_start);

  // Transform polygon to have first vertex in origin and first edge aligned
  // with x-axis.
  // TODO(rikba): Do this without transformation.
  CGAL::Aff_transformation_2<K> translation(
      CGAL::TRANSLATION,
      Vector_2(-polygon.vertices_begin()->x(), -polygon.vertices_begin()->y()));
  polygon = CGAL::transform(translation, polygon);

  CGAL::Aff_transformation_2<K> rotation(
      CGAL::ROTATION, polygon.edges_begin()->direction(), 1, 1e9);
  rotation = rotation.inverse();
  polygon = CGAL::transform(rotation, polygon);

  // Calculate all remaining sweeps.
  const double kMaskOffset = 1e-9;  // To cope with numerical imprecision.
  double min_y = -kMaskOffset;
  double max_y = offset + kMaskOffset;
  if (0.5 * polygon.bbox().ymax() <= max_y) {
    max_y = 0.5 * polygon.bbox().ymax() - kMaskOffset;
    ROS_WARN_STREAM("Offset too large. Re-adjusting.");
  }
  double min_x = polygon.bbox().xmin() - kMaskOffset;
  double max_x = polygon.bbox().xmax() + kMaskOffset;
  // Create sweep mask rectangle.
  Polygon_2 mask;
  mask.push_back(Point_2(min_x, min_y));
  mask.push_back(Point_2(max_x, min_y));
  mask.push_back(Point_2(max_x, max_y));
  mask.push_back(Point_2(min_x, max_y));

  // Intersect mask and polygon.
  std::vector<PolygonWithHoles> intersection_list;
  CGAL::intersection(polygon, mask, std::back_inserter(intersection_list));
  if (intersection_list.size() != 1) {
    ROS_WARN_STREAM("Not exactly one resulting intersections.");
    ROS_WARN_STREAM("Polygon: " << polygon);
    ROS_WARN_STREAM("Mask: " << mask);
    ROS_WARN_STREAM("Intersections:");
    for (auto p : intersection_list) ROS_WARN_STREAM(p << "\n");
    return false;
  }
  if (intersection_list[0].number_of_holes() > 0) {
    ROS_WARN_STREAM("Mask intersection has holes.");
    return false;
  }
  Polygon_2 intersection = intersection_list[0].outer_boundary();

  // Difference intersection and polygon.
  std::vector<PolygonWithHoles> diff_list;
  CGAL::difference(polygon, intersection, std::back_inserter(diff_list));
  if (diff_list.size() != 1) {
    ROS_WARN_STREAM("Not exactly one resulting difference polygon."
                    << diff_list.size());
    return false;
  }
  if (diff_list[0].number_of_holes() > 0) {
    ROS_WARN_STREAM("Polygon difference has holes.");
    return false;
  }
  *offset_polygon = diff_list[0].outer_boundary();

  // Transform back.
  translation = translation.inverse();
  rotation = rotation.inverse();
  *offset_polygon = CGAL::transform(rotation, *offset_polygon);
  *offset_polygon = CGAL::transform(translation, *offset_polygon);
  return true;
}

bool offsetEdgeWithRadialOffset(const Polygon_2& poly, const size_t& edge_id,
                                double radial_offset,
                                Polygon_2* offset_polygon) {
  // Find perpendicular distance.
  Polygon_2::Edge_const_circulator e =
      std::next(poly.edges_circulator(), edge_id);
  Polygon_2::Edge_const_circulator e_prev = std::prev(e);
  Polygon_2::Edge_const_circulator e_next = std::next(e);

  typedef CGAL::Cartesian_converter<InexactKernel, K> IK_to_EK;
  typedef CGAL::Cartesian_converter<K, InexactKernel> EK_to_IK;

  EK_to_IK toInexact;
  InexactKernel::Line_2 l = toInexact(e->supporting_line());
  InexactKernel::Line_2 l_prev = toInexact(e_prev->supporting_line());
  InexactKernel::Line_2 l_next = toInexact(e_next->supporting_line());

  IK_to_EK toExact;
  Line_2 bi_prev = toExact(CGAL::bisector(l, l_prev));
  Line_2 bi_next = toExact(CGAL::bisector(l, l_next));

  Polygon_2::Traits::Equal_2 eq_2;
  ROS_ASSERT(eq_2(e->source(), e_prev->target()));
  ROS_ASSERT(eq_2(e->target(), e_next->source()));

  double len_l_prev =
      std::sqrt(CGAL::to_double(bi_prev.to_vector().squared_length()));
  Vector_2 offset_prev = radial_offset / len_l_prev * bi_prev.to_vector();
  Point_2 p_prev = e->source() + offset_prev;

  double len_l_next =
      std::sqrt(CGAL::to_double(bi_next.to_vector().squared_length()));
  Vector_2 offset_next = radial_offset / len_l_next * bi_next.to_vector();
  Point_2 p_next = e->target() + offset_next;

  Point_2 p_prev_proj = e->supporting_line().projection(p_prev);
  Point_2 p_next_proj = e->supporting_line().projection(p_next);

  Segment_2 s_prev(p_prev, p_prev_proj);
  Segment_2 s_next(p_next, p_next_proj);

  FT offset_distance_prev = s_prev.squared_length();
  FT offset_distance_next = s_next.squared_length();

  double offset_distance_sq =
      std::min(CGAL::to_double(radial_offset * radial_offset),
               CGAL::to_double(offset_distance_prev));
  offset_distance_sq =
      std::min(CGAL::to_double(offset_distance_next), offset_distance_sq);

  return offsetEdge(poly, edge_id, std::sqrt(offset_distance_sq),
                    offset_polygon);
}

}  // namespace polygon_coverage_planning
