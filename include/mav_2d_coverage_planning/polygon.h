#ifndef MAV_2D_COVERAGE_PLANNING_POLYGON_H_
#define MAV_2D_COVERAGE_PLANNING_POLYGON_H_

#include <iostream>
#include <sstream>
#include <vector>

#include "mav_2d_coverage_planning/definitions.h"

namespace mav_coverage_planning {
class Polygon {
 public:
  Polygon();
  Polygon(VertexConstIterator v_begin, VertexConstIterator v_end);
  Polygon(const Polygon_2& polygon);
  Polygon(const PolygonWithHoles& polygon);

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
  bool computeOffsetPolygon(FT max_offset, Polygon* offset_polygon) const;

  // Given a simple polygon without holes, compute its convex decomposition.
  // Daniel H. Greene. The decomposition of polygons into convex parts. In
  // Franco P. Preparata, editor, Computational Geometry, volume 1 of Adv.
  // Comput. Res., pages 235–259. JAI Press, Greenwich, Conn., 1983.
  bool computeConvexDecomposition(std::vector<Polygon>* convex_polygons) const;

  // Given a strictly simple polygon with holes, compute its simple equivalent
  // without holes. Note: New edges are added to the polygon.
  // Uses
  // https://doc.cgal.org/latest/Boolean_set_operations_2/group__boolean__connect__holes.html
  bool convertPolygonWithHolesToPolygonWithoutHoles(
      Polygon* polygon_without_holes) const;

  // Convenience function that first calls
  // convertPolygonWithHolesToPolygonWithoutHoles, then
  // computeConvexDecomposition.
  bool computeConvexDecompositionFromPolygonWithHoles(
      std::vector<Polygon>* convex_polygons);

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
  bool pointsInPolygon(const std::vector<Point_2>::iterator& begin, const std::vector<Point_2>::iterator& end) const;

  // Appends all concave outer boundary vertices.
  bool appendConcaveOuterBoundaryVertices(
      std::vector<VertexConstCirculator>* concave_vertices) const;
  // Appends all convex hole vertices.
  bool appendConvexHoleVertices(
      std::vector<VertexConstCirculator>* convex_vertices) const;
  // Project a point on the polygon boundary.
  Point_2 projectPointOnHull(const Point_2& p) const;

  FT computeArea() const;

  bool isStrictlySimple() const { return is_strictly_simple_; }
  bool isConvex() const { return is_convex_; }

  friend std::ostream& operator<<(std::ostream& stream, const Polygon& p);

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

  std::stringstream printPolygon(const Polygon_2& poly) const;

  // Data.
  // Counter-clockwise boundary, clockwise holes.
  PolygonWithHoles polygon_;
  // Property cache.
  bool is_strictly_simple_;
  bool is_convex_;
};

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_POLYGON_H_
