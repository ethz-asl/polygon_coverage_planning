#ifndef MAV_COVERAGE_PLANNING_MATH_H_
#define MAV_COVERAGE_PLANNING_MATH_H_

#include <cmath>
#include <set>
#include <vector>

#include <Eigen/Core>

#include "mav_coverage_planning/common.h"

namespace mav_coverage_planning {
constexpr double kRayCastPrecision = 1.0e-10;
constexpr double kSimplePolyOffset = 1.0e-14;
constexpr double kBinarySearchResolution = 0.3;

// Checks if a polygon is 1. clock-wise defined, 2. does not repeat vertices, 3.
// is convex.
bool isPolygonValid(const StdVector2d& vertices);

// Check if vertex is convex.
bool checkVertexConvexity(const StdVector2d& vertices, int vertex,
                          bool is_clockwise = true);

// Check if a polygon is convex.
bool checkConvexity(const StdVector2d& vertices, bool is_clockwise = true);

// Check if a polygon has clockwise oriented vertices.
bool checkClockwise(const StdVector2d& vertices);

// Compute right pointing normal.
Eigen::Vector2d computeRightFacingNormal(const Eigen::Vector2d& edge);

// Compute left pointing normal.
Eigen::Vector2d computeLeftFacingNormal(const Eigen::Vector2d& edge);

// Compute the inward-facing normals of a polygon.
void computeInwardFacingNormals(const StdVector2d& vertices,
                                StdVector2d* normals, bool is_clockwise = true);

// Convenience function to compute the edges of a polygon.
void computePolygonEdges(const StdVector2d& vertices, StdVector2d* edges);

// The centroid of a polygon is the average of all points in the shape.
void computePolygonCentroid(const StdVector2d& vertices,
                            Eigen::Vector2d* centroid);

// Shrink a polygon towards its centroid. Scales down the polygon such that the
// corners are at most distance transformed.
bool shrinkPolygon(const StdVector2d& original_vertices, double distance,
                   StdVector2d* new_vertices);

// Shrink a polygon as much as possible.
bool shrinkPolygonBinarySearch(const StdVector2d& original_vertices,
                               double distance, StdVector2d* new_vertices);

// Project a segment onto the hull of a polygon. Assumes the start and end
// point lie on the polygon.
// Returns the shadowed vertices sorted from start to end including these.
// half_space_direction defines which side they should be connected to.
bool castPerpendicularShadowOnVertices(const StdVector2d& vertices,
                                       const Eigen::Vector2d& line_start,
                                       const Eigen::Vector2d& line_end,
                                       const Eigen::Vector2d& direction,
                                       StdVector2d* shadowed_vertices);

// Find the closest point on a polygon hull to a point.
Eigen::Vector2d projectPointOnPolygon(const StdVector2d& vertices,
                                      const Eigen::Vector2d& point);

// Compute the perpendicular distance from a point to an (infinite) line.
void computeDistancePointToLine(const Eigen::Vector2d& point,
                                const Eigen::Vector2d& line_start,
                                const Eigen::Vector2d& line_end,
                                double* distance);

// Compute the distances (with direction) to all vertices from a vertex in a
// specific direction.
void getProjectedVertexDistances(const StdVector2d& vertices,
                                 const Eigen::Vector2d& vertex,
                                 const Eigen::Vector2d& direction,
                                 std::vector<double>* distances);

// Compute the cross product (v x w) of two 2D vectors.
inline double cross(const Eigen::Vector2d& v, const Eigen::Vector2d& w) {
  return v.x() * w.y() - v.y() * w.x();
}

// Compute the distance at which a ray intersects a line. Returns the scalar t_1
// and t_2 of the following line equations:
// x_1(t_1) = ray_origin + ray_direction * t_1, t_1 \in [0, inf[
// x_2(t_2) = line_start + (line_end - line_start) * t_2, t_2 \in [0, 1]
// If there is no intersection it returns false.
bool computeRayLineIntersection(const Eigen::Vector2d& ray_origin,
                                const Eigen::Vector2d& ray_direction,
                                const Eigen::Vector2d& line_start,
                                const Eigen::Vector2d& line_end, double* t_1,
                                double* t_2);
inline bool computeRayLineIntersection(const Eigen::Vector2d& ray_origin,
                                       const Eigen::Vector2d& ray_direction,
                                       const Eigen::Vector2d& line_start,
                                       const Eigen::Vector2d& line_end,
                                       double* t_1) {
  double t_2_dummy;
  return computeRayLineIntersection(ray_origin, ray_direction, line_start,
                                    line_end, t_1, &t_2_dummy);
}

// x_1(t_1) = line_1_start + (line_1_end - line_1_start) * t_1, t_1 \in [0, 1]
// x_2(t_2) = line_2_start + (line_2_end - line_2_start) * t_2, t_2 \in [0, 1]
bool checkLineLineIntersection(const Eigen::Vector2d& line_1_start,
                               const Eigen::Vector2d& line_1_end,
                               const Eigen::Vector2d& line_2_start,
                               const Eigen::Vector2d& line_2_end, double* t_1,
                               double* t_2);

// Check whether a line intersects a polygon.
// Does not signal intersections if the line and edge are parallel or the
// segments ends on a line.
bool checkLinePolygonEdgesIntersection(const Eigen::Vector2d& line_start,
                                       const Eigen::Vector2d& line_end,
                                       const StdVector2d& vertices);

// Check whether a line intersects any of a set of polygons.
// Does not signal intersections if the line starts or ends in a vertex.
inline bool checkLinePolygonEdgesVectorIntersection(
    const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
    const std::vector<StdVector2d>& polygons) {
  for (const StdVector2d& polygon : polygons) {
    if (checkLinePolygonEdgesIntersection(line_start, line_end, polygon)) {
      return true;
    }
  }
  return false;
}

// Compute whether a point lies inside a half-space defined by a line and its
// normal.
bool pointInHalfSpace(const Eigen::Vector2d& point,
                      const Eigen::Vector2d& point_on_line,
                      const Eigen::Vector2d& normal);

// Compute whether a point is inside a convex polygon using a half-space method.
bool pointInConvexPolygon(const Eigen::Vector2d& point,
                          const StdVector2d& vertices,
                          const StdVector2d& normals);

// Convenience method that calculates the inward facing normals each time.
inline bool pointInConvexPolygon(const Eigen::Vector2d& point,
                                 const StdVector2d& vertices) {
  StdVector2d normals;
  computeInwardFacingNormals(vertices, &normals);
  return pointInConvexPolygon(point, vertices, normals);
}

bool createRandomConvexPolygon(double x0, double y0, double r,
                               StdVector2d* convex_polygon);

// Correct user input. Returns false if only 2 vertices are left after
// correcting.
bool correctVertices(StdVector2d* vertices);
inline bool absCompare(double a, double b) {
  return (std::abs(a) < std::abs(b));
}

inline bool checkVerticesIdentical(const Eigen::Vector2d& a,
                                   const Eigen::Vector2d& b) {
  return (b - a).norm() < kRayCastPrecision;
}

// Check point intersects a vertex and sets idx to the intersected vertex if so.
bool checkPointOnVertex(const StdVector2d& vertices,
                        const Eigen::Vector2d& point, int* idx);

// Removes duplicate vertices and vertices creating collinear adjacent edges.
void removeRedundantVertices(StdVector2d* vertices);

// Create a simple polygon from a polygon with holes.
// Assumption: polygon and holes are simple.
void polygonWithHolesToSimplePolygon(
    const StdVector2d& cw_polygon_boundary,
    const std::vector<StdVector2d>& cc_polygon_holes,
    StdVector2d* simple_polygon);

bool checkLinesParallel(const Eigen::Vector2d& line_1_direction,
                        const Eigen::Vector2d& line_2_direction);

// Check if a line is parallel to any edge of a set.
bool checkLineParallelToAnyEdge(const Eigen::Vector2d& line_start,
                                const Eigen::Vector2d& line_end,
                                const StdVector2d& edges);

// Check if a point is in the polygon using the winding number test.
int windingNumberTest(const Eigen::Vector2d& point,
                      const StdVector2d& vertices);

// Check if a point is left >0 / on =0 / right <0 of an infinite line.
double isLeft(const Eigen::Vector2d& point, const Eigen::Vector2d& line_start,
              const Eigen::Vector2d& line_end);

// Check if a point is inside an arbitrary polygon using the winding number
// test.
bool pointInPolygonWindingNumber(const Eigen::Vector2d& point,
                                 const StdVector2d& vertices);

double distancePointToSegment(const Eigen::Vector2d& point,
                              const Eigen::Vector2d& segment_start,
                              const Eigen::Vector2d& segment_end);

// Check if a point has zero distance to a segment.
bool pointOnSegment(const Eigen::Vector2d& point,
                    const Eigen::Vector2d& segment_start,
                    const Eigen::Vector2d& segment_end);

// Check if a point lies on a polygon.
// Returns start and end vertex idx.
bool pointOnPolygonHull(const Eigen::Vector2d& point,
                        const StdVector2d& vertices, int* source_idx,
                        int* target_idx);

bool pointOnPolygonHull(const Eigen::Vector2d& point,
                        const StdVector2d& vertices);

// Compute the length of a open vertex list.
double computeVertexChainLength(const StdVector2d& vertices);

// Compute the length of a closed vertex list.
double computeHullLength(const StdVector2d& vertices);

// Given a simple, convex, clockwise polygon, compute a sweep plan.
// cw_vertices: clockwise polygon vertices.
// inward_facing_normals: inward facing normals.
// max_sweep_distance: maximum distance between two sweeping lines to still meet
// the overlap criterion. start_vertex_idx: the polygon vertex from which to
// start the sweep plan.
bool createLineSweepPlan(const StdVector2d& cw_vertices,
                         const StdVector2d& cw_edges,
                         const StdVector2d& inward_facing_normals,
                         double max_sweep_distance, size_t start_vertex_idx,
                         StdVector2d* waypoints);

// Calculate the factorial
inline int factorial(int n) {
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

// Calculate the binomial coefficient "n choose k".
inline int nChooseK(int n, int k) {
  return factorial(n) / (factorial(k) * factorial(n - k));
}

// Pick all of the k-element combinations from an ordered set.
void getAllCombinationsOfKElementsFromN(const std::vector<size_t>& sorted_elements,
                                        int k,
                                        std::vector<std::set<size_t>>* result);
// The recursive call.
void getAllCombinationsOfKElementsFromN(const std::vector<size_t>& sorted_elements,
                                        int k, int start_pos,
                                        std::vector<size_t>* combination,
                                        std::vector<std::set<size_t>>* result);



}  // namespace mav_coverage_planning

#endif /* MAV_COVERAGE_PLANNING_MATH_H_ */
