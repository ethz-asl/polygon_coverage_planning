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

  inline const PolygonWithHoles& getPolygon() const { return polygon_; }
  std::vector<Point_2> getHullVertices() const;
  std::vector<std::vector<Point_2>> getHoleVertices() const;

  // Appends all convex hole vertices.
  bool appendConvexHoleVertices(
      std::vector<VertexConstCirculator>* convex_vertices) const;

  inline bool doIntersect(const Polygon& p) const {
    return CGAL::do_intersect(p.getPolygon(), polygon_);
  }

  bool intersect(const Polygon& p, Polygon* intersection) const;

  bool isConvex() const { return is_convex_; }

  friend std::ostream& operator<<(std::ostream& stream, const Polygon& p);

  Polyhedron_3 toMesh() const;
  inline PlaneTransformation<K> getPlaneTransformation() const {
    return plane_tf_;
  }

  std::vector<Direction_2> getUniformDirections(const int num) const;

 private:

  bool checkConvexity() const;

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
