#ifndef MAV_2D_COVERAGE_PLANNING_GEOMETRY_POLYGON_H_
#define MAV_2D_COVERAGE_PLANNING_GEOMETRY_POLYGON_H_

#include <iostream>
#include <set>
#include <sstream>
#include <vector>

#include <CGAL/Boolean_set_operations_2.h>
#include <mav_coverage_planning_comm/cgal_definitions.h>

#include "mav_2d_coverage_planning/geometry/BCD.h"
#include "mav_2d_coverage_planning/geometry/plane_transformation.h"

namespace mav_coverage_planning {

enum DecompositionType { kBoustrophedeon, kTrapezoidal };

class Polygon {
 public:
  Polygon();
  Polygon(VertexConstIterator v_begin, VertexConstIterator v_end,
          const PlaneTransformation<K>& plane_tf = PlaneTransformation<K>());
  Polygon(const Polygon_2& polygon,
          const PlaneTransformation<K>& plane_tf = PlaneTransformation<K>());
  Polygon(const PolygonWithHoles& polygon,
          const PlaneTransformation<K>& plane_tf = PlaneTransformation<K>());

  // Given a simple, convex polygon, compute a sweep plan.
  // max_sweep_distance: maximum distance between two sweeping lines to still
  // meet the overlap criterion.
  // start_vertex_idx: the polygon vertex from which to start the sweep plan.
  // clockwise: The line sweep plan is clockwise or counter-clockwise.
  //
  // The algorithm uses CGALs boolean operations on polygons to retrieve the
  // sweeps along the polygon. A sweep window polygon is moved across the
  // original polygon and the intersection defines the waypoints for the sweep.
  bool computeLineSweepPlan(double max_sweep_distance, size_t start_vertex_idx,
                            bool counter_clockwise,
                            std::vector<Point_2>* waypoints) const;

  // Given a non-degenerate counter-clockwise weakly-simple polygon with
  // holes, compute the maximum offset polygon such that no edge collapses.
  // Aichholzer, Oswin, et al. "A novel type of skeleton for polygons." J. UCS
  // The Journal of Universal Computer Science. Springer, Berlin, Heidelberg,
  // 1996. 752-761.
  bool computeOffsetPolygon(FT max_offset, Polygon* offset_polygon) const;

  // Offsets a specific polygon edge by cropping an along the edge infinitly
  // long rectangular window with offset width.
  bool offsetEdge(const size_t& edge_id, double offset,
                  Polygon* offset_polygon) const;

  // Offset at most radial_offset from corner.
  bool offsetEdgeWithRadialOffset(const size_t& edge_id, double radial_offset,
                                  Polygon* offset_polygon) const;

  bool computeBCDFromPolygonWithHoles(std::vector<Polygon>* bcd_polygons) const;

  // Compute BCDs for every edge direction. Return any with the smallest number
  // of cells.
  bool computeBestBCDFromPolygonWithHoles(
      std::vector<Polygon>* bcd_polygons) const;

  // TODO(rikba): implement.
  bool computeBestDecompositionFromPolygonWithHoles(
      std::vector<Polygon>* bcd_polygons) const {
    return computeBCDFromPolygonWithHoles(bcd_polygons);
  }

  bool computeTrapezoidalDecompositionFromPolygonWithHoles(
      std::vector<Polygon>* trap_polygons) const;

  // The best TCD is considered the one with the smallest bound on sweeps.
  bool computeBestTrapezoidalDecompositionFromPolygonWithHoles(
      std::vector<Polygon>* trap_polygons) const;

  // Compute the visibility polygon given a point inside a strictly simple
  // polygon. Francisc Bungiu, Michael Hemmer, John Hershberger, Kan Huang, and
  // Alexander Kröller. Efficient computation of visibility polygons. CoRR,
  // abs/1403.3905, 2014.
  bool computeVisibilityPolygon(const Point_2& query_point,
                                Polygon* visibility_polygon) const;

  inline const PolygonWithHoles& getPolygon() const { return polygon_; }
  std::vector<Point_2> getHullVertices() const;
  std::vector<std::vector<Point_2>> getHoleVertices() const;

  // Helper to check whether a point is inside or on the boundary of the
  // polygon.
  bool pointInPolygon(const Point_2& p) const;
  bool pointsInPolygon(const std::vector<Point_2>::iterator& begin,
                       const std::vector<Point_2>::iterator& end) const;

  // Appends all concave outer boundary vertices.
  bool appendConcaveOuterBoundaryVertices(
      std::vector<VertexConstCirculator>* concave_vertices) const;
  // Appends all convex hole vertices.
  bool appendConvexHoleVertices(
      std::vector<VertexConstCirculator>* convex_vertices) const;
  // Project a point on the polygon boundary.
  Point_2 projectPointOnHull(const Point_2& p) const;

  FT computeArea() const;

  inline bool doIntersect(const Polygon& p) const {
    return CGAL::do_intersect(p.getPolygon(), polygon_);
  }

  bool intersect(const Polygon& p, Polygon* intersection) const;

  bool isStrictlySimple() const { return is_strictly_simple_; }
  bool isConvex() const { return is_convex_; }

  friend std::ostream& operator<<(std::ostream& stream, const Polygon& p);

  Polyhedron_3 toMesh() const;
  inline PlaneTransformation<K> getPlaneTransformation() const {
    return plane_tf_;
  }

  std::vector<Direction_2> findEdgeDirections() const;
  std::vector<Direction_2> findPerpEdgeDirections() const;
  std::vector<Direction_2> getUniformDirections(const int num) const;
  std::vector<Polygon> rotatePolygon(
      const std::vector<Direction_2>& dirs) const;
  double findMinAltitude(const Polygon& subregion) const;

 private:
  bool checkValidOffset(
      const PolygonWithHoles& original,
      const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset) const;

  // Definition according to
  // https://doc.cgal.org/latest/Straight_skeleton_2/index.html
  bool checkStrictlySimple() const;
  bool checkConvexity() const;

  // Project a point on a polygon.
  Point_2 projectOnPolygon2(const Polygon_2& poly, const Point_2& p,
                            FT* squared_distance) const;

  // Sort boundary to be counter-clockwise and holes to be clockwise.
  void sortCC();

  // Merge consecutive collinear edges.
  void simplifyPolygon(Polygon_2* polygon);
  void simplify();

  // Helper function to check for full coverage in convex polygon.
  bool isCovered(const Point_2& p,
                 const std::vector<EdgeConstCirculator>& edges,
                 double sweep_distance, double margin) const;

  std::stringstream printPolygon(const Polygon_2& poly) const;

  // Data.
  // Counter-clockwise boundary, clockwise holes.
  PolygonWithHoles polygon_;
  // Property cache.
  bool is_strictly_simple_;
  bool is_convex_;
  PlaneTransformation<K> plane_tf_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_GEOMETRY_POLYGON_H_
