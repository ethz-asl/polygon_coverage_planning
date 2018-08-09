#include "mav_2d_coverage_planning/polygon.h"

#include <algorithm>
#include <limits>

#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Partition_is_valid_traits_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/connect_holes.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/partition_2.h>
#include <glog/logging.h>
#include <boost/make_shared.hpp>

namespace mav_coverage_planner {

Polygon::Polygon() {}

Polygon::Polygon(VertexConstIterator v_begin, VertexConstIterator v_end)
    : Polygon(Polygon_2(v_begin, v_end)) {}

Polygon::Polygon(const Polygon_2& polygon)
    : Polygon(PolygonWithHoles(polygon)) {}

Polygon::Polygon(const PolygonWithHoles& polygon)
    : polygon_(polygon),
      is_strictly_simple_(checkStrictlySimple()),
      is_convex_(checkConvexity()) {
  sortCC();
  simplify();
}

bool Polygon::computeOffsetPolygon(FT max_offset,
                                   Polygon* offset_polygon) const {
  CHECK_NOTNULL(offset_polygon);

  // TODO(rikba): Check weak simplicity.

  // Try maximum offsetting.
  std::vector<boost::shared_ptr<PolygonWithHoles>> result =
      CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
          max_offset, polygon_);
  if (checkValidOffset(polygon_, result)) {
    *offset_polygon = Polygon(*(result.front()));
    return true;
  } else {
    result = {boost::make_shared<PolygonWithHoles>(polygon_)};
  }

  // Binary search for smaller valid offset.
  FT min = 0.0;
  FT max = max_offset;
  const FT kBinarySearchResolution = 0.1;
  while (max - min > kBinarySearchResolution) {
    const FT mid = (min + max) / 2.0;
    std::vector<boost::shared_ptr<PolygonWithHoles>> temp_result =
        CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
            mid, polygon_);
    if (checkValidOffset(polygon_, temp_result)) {
      min = mid;
      result = temp_result;
    } else {
      max = mid;
    }
  }

  *offset_polygon = Polygon(*(result.front()));
  return true;
}

bool Polygon::computeConvexDecomposition(
    std::vector<Polygon>* convex_polygons) const {
  CHECK_NOTNULL(convex_polygons);

  // Precondition.
  if (polygon_.number_of_holes() > 0) return false;
  if (!is_strictly_simple_) return false;

  // Convex decomposition.
  typedef CGAL::Partition_traits_2<K> PartitionTraits;
  typedef PartitionTraits::Polygon_2 PartitionPolygon;
  std::vector<PartitionPolygon> partition_polygons;
  CGAL::optimal_convex_partition_2(polygon_.outer_boundary().vertices_begin(),
                                   polygon_.outer_boundary().vertices_end(),
                                   std::back_inserter(partition_polygons),
                                   PartitionTraits());

  // Validity check.
  typedef CGAL::Is_convex_2<PartitionTraits> IsConvex2;
  typedef CGAL::Partition_is_valid_traits_2<PartitionTraits, IsConvex2>
      PartitionValidityTraits;
  if (!CGAL::partition_is_valid_2(
          polygon_.outer_boundary().vertices_begin(),
          polygon_.outer_boundary().vertices_end(), partition_polygons.begin(),
          partition_polygons.end(), PartitionValidityTraits())) {
    // Partition is overlapping, union area != original area or non convex.
    return false;
  }

  convex_polygons->clear();
  convex_polygons->reserve(partition_polygons.size());
  for (const PartitionPolygon& p : partition_polygons) {
    // TODO(rikba): Some unnecessary move from list to vector because I don't
    // know how to handle these partition traits.
    std::vector<Point_2> vertices = {
        std::make_move_iterator(p.vertices_begin()),
        std::make_move_iterator(p.vertices_end())};
    convex_polygons->emplace_back(vertices.begin(), vertices.end());
  }

  return true;
}

bool Polygon::convertPolygonWithHolesToPolygonWithoutHoles(
    Polygon* polygon_without_holes) const {
  CHECK_NOTNULL(polygon_without_holes);

  if (!is_strictly_simple_) return false;

  if (polygon_.number_of_holes() == 0) {
    *polygon_without_holes = Polygon(polygon_);
    return true;
  }

  std::vector<Point_2> pts;
  CGAL::connect_holes(polygon_, std::back_inserter(pts));

  // Tiny hack to make resulting polygon strictly simple. Add small offset to
  // seperate coincident edges.
  // TODO(rikba): Only offset coincident edges.
  const FT kOffset = std::numeric_limits<double>::epsilon();
  Polygon weakly_simple_polygon_without_holes(pts.begin(), pts.end());
  if (!weakly_simple_polygon_without_holes.computeOffsetPolygon(
          kOffset, polygon_without_holes))
    return false;

  return true;
}

bool Polygon::computeConvexDecompositionFromPolygonWithHoles(
    std::vector<Polygon>* convex_polygons) {
  CHECK_NOTNULL(convex_polygons);

  Polygon polygon_without_holes;
  if (!convertPolygonWithHolesToPolygonWithoutHoles(&polygon_without_holes))
    return false;

  if (!polygon_without_holes.computeConvexDecomposition(convex_polygons))
    return false;

  return true;
}

bool Polygon::checkValidOffset(
    const PolygonWithHoles& original,
    const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset) const {
  // Valid if number of vertices remains constant.
  if (offset.size() != 1) return false;
  if (original.number_of_holes() != offset.front()->number_of_holes())
    return false;
  if (original.outer_boundary().size() !=
      offset.front()->outer_boundary().size())
    return false;

  PolygonWithHoles::Hole_const_iterator offset_hit =
      offset.front()->holes_begin();
  for (PolygonWithHoles::Hole_const_iterator
           original_hit = original.holes_begin();
       original_hit != original.holes_end(); ++original_hit, ++offset_hit) {
    if (original_hit->size() != offset_hit->size()) return false;
  }
  return true;
}

void Polygon::print() const {
  std::cout << "Polygon_with_holes having " << polygon_.number_of_holes()
            << " holes" << std::endl;

  printPolygon(polygon_.outer_boundary());

  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi)
    printPolygon(*hi);
}

void Polygon::printPolygon(const Polygon_2& poly) const {
  std::cout << "Polygon with " << poly.size() << " vertices" << std::endl;
  for (Polygon_2::Vertex_const_iterator vi = poly.vertices_begin();
       vi != poly.vertices_end(); ++vi) {
    std::cout << "(" << vi->x() << "," << vi->y() << ")" << std::endl;
  }
}

FT Polygon::computeArea() const {
  FT area = CGAL::abs(
      CGAL::polygon_area_2(polygon_.outer_boundary().vertices_begin(),
                           polygon_.outer_boundary().vertices_end(), K()));
  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi)
    area -= CGAL::abs(
        CGAL::polygon_area_2(hi->vertices_begin(), hi->vertices_end(), K()));
  return area;
}

bool Polygon::checkStrictlySimple() const {
  if (!polygon_.outer_boundary().is_simple()) return false;
  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi)
    if (!hi->is_simple()) return false;
  return true;
}

bool Polygon::checkConvexity() const {
  if (!polygon_.outer_boundary().is_convex()) return false;
  if (polygon_.number_of_holes() > 0) return false;
  return true;
}

void Polygon::sortCC() {
  if (polygon_.outer_boundary().is_clockwise_oriented())
    polygon_.outer_boundary().reverse_orientation();

  for (PolygonWithHoles::Hole_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi)
    if (hi->is_counterclockwise_oriented()) hi->reverse_orientation();
}

void Polygon::simplifyPolygon(Polygon_2* polygon) {
  CHECK_NOTNULL(polygon);

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

void Polygon::simplify() {
  simplifyPolygon(&polygon_.outer_boundary());

  for (PolygonWithHoles::Hole_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi)
    simplifyPolygon(&*hi);
}

bool Polygon::computeLineSweepPlan(double max_sweep_distance,
                                   size_t start_vertex_idx,
                                   bool counter_clockwise,
                                   std::vector<Point_2>* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  // Preconditions.
  if (!is_strictly_simple_) return false;
  if (!is_convex_) return false;
  if (polygon_.outer_boundary().size() < 3) return false;

  // Copy of polygon with start_vertex_idx being first vertex.
  Polygon_2 polygon;
  Polygon_2::Vertex_circulator vc_start = std::next(
      polygon_.outer_boundary().vertices_circulator(), start_vertex_idx);
  Polygon_2::Vertex_circulator vc = vc_start;
  do {
    polygon.push_back(*vc);
  } while (++vc != vc_start);

  // Change orientation according to arguments.
  if (!counter_clockwise) polygon.reverse_orientation();

  // Transform polygon to have first vertex in origin and first edge aligned
  // with x-axis.
  CGAL::Aff_transformation_2<K> translation(
      CGAL::TRANSLATION,
      Vector_2(-polygon.vertices_begin()->x(), -polygon.vertices_begin()->y()));
  polygon = CGAL::transform(translation, polygon);

  CGAL::Aff_transformation_2<K> rotation(
      CGAL::ROTATION, polygon.edges_begin()->direction(), 1, 1e16);
  rotation = rotation.inverse();
  polygon = CGAL::transform(rotation, polygon);

  // Initial waypoints.
  waypoints->push_back(*polygon.vertices_begin());
  waypoints->push_back(*std::next(polygon.vertices_begin()));

  // Compute sweep distance for equally spaced sweeps.
  double polygon_length = polygon.bbox().ymax() - polygon.bbox().ymin();
  int num_sweeps =
      static_cast<int>(std::ceil(polygon_length / max_sweep_distance)) + 1;
  double sweep_distance = polygon_length / (num_sweeps - 1);

  // Calculate all remaining sweeps.
  const double kMaskOffset = 1e-6;  // To cope with numerical imprecision.
  double min_y = -kMaskOffset;
  double max_y = sweep_distance + kMaskOffset;
  double min_x = polygon.bbox().xmin() - kMaskOffset;
  double max_x = polygon.bbox().xmax() + kMaskOffset;
  // Create sweep mask rectangle.
  Polygon_2 sweep_mask;
  sweep_mask.push_back(Point_2(min_x, min_y));
  sweep_mask.push_back(Point_2(max_x, min_y));
  sweep_mask.push_back(Point_2(max_x, max_y));
  sweep_mask.push_back(Point_2(min_x, max_y));
  // Compute sweep mask transformation. The sweep line gets shifted in
  // y-direction with every new sweep.
  CGAL::Aff_transformation_2<K> sweep_mask_trafo(CGAL::TRANSLATION,
                                                 Vector_2(0.0, sweep_distance));

  bool sweep_is_cc = true;  // Toggle.
  // Initially clockwise sweeps get handled slightly different.
  if (!counter_clockwise) {
    sweep_mask_trafo = sweep_mask_trafo.inverse();
    sweep_mask = CGAL::transform(sweep_mask_trafo, sweep_mask);
    sweep_mask.reverse_orientation();
    sweep_is_cc = !sweep_is_cc;
  }
  for (int i = 1; i < num_sweeps; ++i) {
    // Find intersection between sweep mask and polygon that includes the sweep
    // waypoints to be added.
    std::vector<PolygonWithHoles> intersections;
    CGAL::intersection(polygon, sweep_mask, std::back_inserter(intersections));
    // Some assertions.
    if (intersections.size() != 1) return false;
    if (intersections.front().number_of_holes() != 0) return false;

    // Orientate intersection according to sweep direction.
    Polygon_2& intersection = intersections.front().outer_boundary();
    CGAL::Orientation intersection_orientation = intersection.orientation();
    if ((intersection_orientation == CGAL::CLOCKWISE && sweep_is_cc) ||
        (intersection_orientation == CGAL::COUNTERCLOCKWISE && !sweep_is_cc))
      intersection.reverse_orientation();

    // Add intersection vertices to waypoints. We skip the previous sweep and
    // stop at the current sweep. For a counter clockwise polygon the two bottom
    // most waypoints represent the vertices after the top most vertices
    // represent the current sweep.

    // 1. Find the two top and bottom most vertices.
    VertexConstCirculator bot_v = intersection.vertices_circulator();
    VertexConstCirculator sec_bot_v =
        std::next(intersection.vertices_circulator());
    if (sec_bot_v->y() < bot_v->y()) std::swap(bot_v, sec_bot_v);

    VertexConstCirculator top_v = intersection.vertices_circulator();
    VertexConstCirculator sec_top_v =
        std::next(intersection.vertices_circulator());
    if (sec_top_v->y() > top_v->y()) std::swap(top_v, sec_top_v);

    VertexConstCirculator vit = intersection.vertices_circulator();
    do {
      // Bottom.
      if (vit->y() < bot_v->y()) {
        sec_bot_v = bot_v;
        bot_v = vit;
      } else if ((vit->y() < sec_bot_v->y()) && (vit != bot_v))
        sec_bot_v = vit;

      // Top.
      if (vit->y() > top_v->y()) {
        sec_top_v = top_v;
        top_v = vit;
      } else if ((vit->y() > sec_top_v->y()) && (vit != top_v))
        sec_top_v = vit;
    } while (++vit != intersection.vertices_circulator());

    // 2. Find the edge that represents the bottom sweep and top sweep.
    EdgeConstCirculator bottom_sweep = intersection.edges_circulator();
    EdgeConstCirculator top_sweep = intersection.edges_circulator();
    EdgeConstCirculator eit = intersection.edges_circulator();

    bool has_bottom_sweep = false, has_top_sweep = false;
    do {
      // Bottom.
      if ((eit->source() == *bot_v && eit->target() == *sec_bot_v) ||
          (eit->source() == *sec_bot_v && eit->target() == *bot_v)) {
        has_bottom_sweep = true;
        bottom_sweep = eit;
      }

      // Top.
      if ((eit->source() == *top_v && eit->target() == *sec_top_v) ||
          (eit->source() == *sec_top_v && eit->target() == *top_v)) {
        has_top_sweep = true;
        top_sweep = eit;
      }
    } while (++eit != intersection.edges_circulator());
    // Assertion.
    if (!has_bottom_sweep || !has_top_sweep) return false;

    // 3. Add waypoints from previous sweep to current sweep.
    // Note: The last sweep is handled differently. Here the full polygon
    // contour is followed until reaching the previous sweep source again.
    EdgeConstCirculator prev_sweep = bottom_sweep;
    EdgeConstCirculator current_sweep = top_sweep;
    if (!counter_clockwise) std::swap(prev_sweep, current_sweep);

    eit = prev_sweep;
    if (i < num_sweeps - 1)
      while (++eit != std::next(current_sweep))
        waypoints->push_back(eit->target());
    else
      while (++eit != prev_sweep) waypoints->push_back(eit->target());

    // Prepare for next sweep.
    sweep_mask = CGAL::transform(sweep_mask_trafo, sweep_mask);
    sweep_is_cc = !sweep_is_cc;
  }

  // Transform back waypoints.
  translation = translation.inverse();
  rotation = rotation.inverse();
  for (Point_2& p : *waypoints) {
    p = rotation(p);
    p = translation(p);
  }

  return true;
}

bool Polygon::pointInPolygon(const Point_2& p) const {
  // Point inside outer boundary.
  CGAL::Bounded_side result =
      CGAL::bounded_side_2(polygon_.outer_boundary().vertices_begin(),
                           polygon_.outer_boundary().vertices_end(), p, K());
  if (result == CGAL::ON_UNBOUNDED_SIDE) return false;

  // Point outside hole.
  for (PolygonWithHoles::Hole_const_iterator hit = polygon_.holes_begin();
       hit != polygon_.holes_end(); ++hit) {
    result = CGAL::bounded_side_2(hit->vertices_begin(), hit->vertices_end(), p,
                                  K());
    if (result == CGAL::ON_BOUNDED_SIDE) return false;
  }

  return true;
}

bool Polygon::computeVisibilityPolygon(const Point_2& query_point,
                                       Polygon* visibility_polygon) const {
  CHECK_NOTNULL(visibility_polygon);

  if (!pointInPolygon(query_point)) {
    LOG(ERROR) << "Query point " << query_point
               << " outside of polygon. Cannot create visibility polygon.";
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
    std::cout << "On face." << std::endl;
    fh = tev.compute_visibility(query_point, *f, visibility_arr);
  } else if ((v = boost::get<VisibilityArrangement::Vertex_const_handle>(
                  &pl_result))) {
    // Located on vertex.
    std::cout << "On vertex." << std::endl;

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
    std::cout << "On he." << std::endl;
    std::cout << (*e)->source()->point() << std::endl;
    std::cout << (*e)->target()->point() << std::endl;
    fh = tev.compute_visibility(query_point, *e, visibility_arr);
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
  std::cout << "Convert." << std::endl;
  VisibilityArrangement::Ccb_halfedge_circulator curr = fh->outer_ccb();
  Polygon_2 vis_poly_2;
  do {
    vis_poly_2.push_back(curr->source()->point());
  } while (++curr != fh->outer_ccb());

  *visibility_polygon = Polygon(vis_poly_2);

  return true;
}

}  // namespace mav_coverage_planner
