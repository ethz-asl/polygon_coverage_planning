#ifndef MAV_2D_COVERAGE_PLANNING_POLYGON_H_
#define MAV_2D_COVERAGE_PLANNING_POLYGON_H_

#include <vector>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_2 Point_2;
typedef K::Vector_2 Vector_2;
typedef K::Line_2 Line_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef Polygon_2::Vertex_const_iterator VertexConstIterator;
typedef Polygon_2::Vertex_const_circulator VertexConstCirculator;
typedef Polygon_2::Edge_const_iterator EdgeConstIterator;
typedef Polygon_2::Edge_const_circulator EdgeConstCirculator;
typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;

namespace mav_coverage_planner {
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

  // Compute the visibility polygon given a point inside the polygon.
  // Francisc Bungiu, Michael Hemmer, John Hershberger, Kan Huang, and Alexander
  // Kröller. Efficient computation of visibility polygons. CoRR, abs/1403.3905,
  // 2014.
  bool computeVisibilityPolygon(const Point_2& query_point,
                                Polygon* visibility_polygon) const;

  inline const PolygonWithHoles& getPolygon() const { return polygon_; }

  // Helper to check whether a point is inside or on the boundary of the
  // polygon.
  bool pointInPolygon(const Point_2& p) const;

  void print() const;
  FT computeArea() const;

 private:
  bool checkValidOffset(
      const PolygonWithHoles& original,
      const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset) const;

  // Definition according to
  // https://doc.cgal.org/latest/Straight_skeleton_2/index.html
  bool checkStrictlySimple() const;
  bool checkConvexity() const;

  // Sort boundary to be counter-clockwise and holes to be clockwise.
  void sortCC();

  // Merge consecutive collinear edges.
  void simplifyPolygon(Polygon_2* polygon);
  void simplify();

  void printPolygon(const Polygon_2& poly) const;

  // Data.
  // Counter-clockwise boundary, clockwise holes.
  PolygonWithHoles polygon_;
  // Property cache.
  bool is_strictly_simple_;
  bool is_convex_;
};
}  // namespace mav_coverage_planner

#endif  // MAV_2D_COVERAGE_PLANNING_POLYGON_H_
