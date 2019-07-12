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

#include <ros/assert.h>
#include <ros/console.h>

#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/visibility_polygon.h"

namespace polygon_coverage_planning {

bool computeVisibilityPolygon(const PolygonWithHoles& pwh,
                              const Point_2& query_point,
                              Polygon_2* visibility_polygon) {
  ROS_ASSERT(visibility_polygon);

  // Preconditions.
  ROS_ASSERT_MSG(pointInPolygon(pwh, query_point),
                 "Query point outside of polygon.");
  ROS_ASSERT_MSG(isStrictlySimple(pwh), "Polygon is not strictly simple.");

  // Create 2D arrangement.
  typedef CGAL::Arr_segment_traits_2<K> VisibilityTraits;
  typedef CGAL::Arrangement_2<VisibilityTraits> VisibilityArrangement;
  VisibilityArrangement poly;
  CGAL::insert(poly, pwh.outer_boundary().edges_begin(),
               pwh.outer_boundary().edges_end());
  // Store main face.
  ROS_ASSERT_MSG(poly.number_of_unbounded_faces() == 1,
                 "Polygon has unbounded curves.");
  ROS_ASSERT_MSG(poly.number_of_faces() == 2,
                 "More than one bounded face in polygon.");

  VisibilityArrangement::Face_const_handle main_face = poly.faces_begin();
  while (main_face->is_unbounded()) {
    main_face++;
  }

  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit)
    CGAL::insert(poly, hit->edges_begin(), hit->edges_end());

  // Create Triangular Expansion Visibility object.
  typedef CGAL::Triangular_expansion_visibility_2<VisibilityArrangement,
                                                  CGAL::Tag_true>
      TEV;
  TEV tev(poly);

  // We need to determine the halfedge or face to which the query point
  // corresponds.
  typedef CGAL::Arr_naive_point_location<VisibilityArrangement> NaivePL;
  typedef CGAL::Arr_point_location_result<VisibilityArrangement>::Type PLResult;
  NaivePL pl(poly);
  PLResult pl_result = pl.locate(query_point);

  VisibilityArrangement::Vertex_const_handle* v = nullptr;
  VisibilityArrangement::Halfedge_const_handle* e = nullptr;
  VisibilityArrangement::Face_const_handle* f = nullptr;

  typedef VisibilityArrangement::Face_handle VisibilityFaceHandle;
  VisibilityFaceHandle fh;
  VisibilityArrangement visibility_arr;
  if ((f = boost::get<VisibilityArrangement::Face_const_handle>(&pl_result))) {
    // Located in face.
    fh = tev.compute_visibility(query_point, *f, visibility_arr);
  } else if ((v = boost::get<VisibilityArrangement::Vertex_const_handle>(
                  &pl_result))) {
    // Located on vertex.
    // Search the incident halfedge that contains the polygon face.
    VisibilityArrangement::Halfedge_const_handle he = poly.halfedges_begin();
    while ((he->target()->point() != (*v)->point()) ||
           (he->face() != main_face)) {
      he++;
      if (he == poly.halfedges_end()) {
        ROS_ERROR_STREAM("Cannot find halfedge corresponding to vertex.");
        return false;
      }
    }

    fh = tev.compute_visibility(query_point, he, visibility_arr);
  } else if ((e = boost::get<VisibilityArrangement::Halfedge_const_handle>(
                  &pl_result))) {
    // Located on halfedge.
    // Find halfedge that has polygon interior as face.
    VisibilityArrangement::Halfedge_const_handle he =
        (*e)->face() == main_face ? (*e) : (*e)->twin();
    fh = tev.compute_visibility(query_point, he, visibility_arr);
  } else {
    ROS_ERROR_STREAM("Cannot locate query point on arrangement.");
    return false;
  }

  // Result assertion.
  if (fh->is_fictitious()) {
    ROS_ERROR_STREAM("Visibility polygon is fictitious.");
    return false;
  }
  if (fh->is_unbounded()) {
    ROS_ERROR_STREAM("Visibility polygon is unbounded.");
    return false;
  }

  // Convert to polygon.
  VisibilityArrangement::Ccb_halfedge_circulator curr = fh->outer_ccb();
  *visibility_polygon = Polygon_2();
  do {
    visibility_polygon->push_back(curr->source()->point());
  } while (++curr != fh->outer_ccb());

  simplifyPolygon(visibility_polygon);
  if (visibility_polygon->is_clockwise_oriented())
    visibility_polygon->reverse_orientation();

  return true;
}

}  // namespace polygon_coverage_planning
