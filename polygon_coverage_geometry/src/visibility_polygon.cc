#include <ros/assert.h>
#include <ros/console.h>

#include "polygon_coverage_geometry/visibility_polygon.h"

namespace polygon_coverage_planning {

bool computeVisibilityPolygon(const PolygonWithHoles& poly,
                              const Point_2& query_point,
                              Polygon_2* visibility_polygon) {
  ROS_ASSERT(visibility_polygon);

  // Preconditions.
  if (!pointInPolygon(query_point)) {
    LOG(ERROR) << "Query point " << query_point
               << " outside of polygon. Cannot create visibility polygon.";
    return false;
  }
  if (!is_strictly_simple_) {
    LOG(ERROR) << "Polygon not strictly simple.";
    return false;
  }

  // Create 2D arrangement.
  typedef CGAL::Arr_segment_traits_2<K> VisibilityTraits;
  typedef CGAL::Arrangement_2<VisibilityTraits> VisibilityArrangement;
  VisibilityArrangement poly;
  CGAL::insert(poly, polygon_.outer_boundary().edges_begin(),
               polygon_.outer_boundary().edges_end());
  // Store main face.
  if (poly.number_of_unbounded_faces() != 1) {
    LOG(ERROR) << "Polygon has unbounded curves.";
    return false;
  }
  if (poly.number_of_faces() != 2) {
    LOG(ERROR) << "More than one bounded face in polygon.";
    return false;
  }
  VisibilityArrangement::Face_const_handle main_face = poly.faces_begin();
  while (main_face->is_unbounded()) {
    main_face++;
  }

  for (PolygonWithHoles::Hole_const_iterator hit = polygon_.holes_begin();
       hit != polygon_.holes_end(); ++hit)
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
        LOG(ERROR) << "Cannot find halfedge corresponding to vertex.";
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
    LOG(ERROR) << "Cannot locate query point on arrangement.";
    return false;
  }

  // Result assertion.
  if (fh->is_fictitious()) {
    LOG(ERROR) << "Visibility polygon is fictitious.";
    return false;
  }
  if (fh->is_unbounded()) {
    LOG(ERROR) << "Visibility polygon is unbounded.";
    return false;
  }

  // Convert to polygon.
  VisibilityArrangement::Ccb_halfedge_circulator curr = fh->outer_ccb();
  Polygon_2 vis_poly_2;
  do {
    vis_poly_2.push_back(curr->source()->point());
  } while (++curr != fh->outer_ccb());

  *visibility_polygon = Polygon(vis_poly_2);

  return true;
}

}  // namespace polygon_coverage_planning
