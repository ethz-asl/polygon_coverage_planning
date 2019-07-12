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

#ifndef POLYGON_COVERAGE_GEOMETRY_VISIBILITY_GRAPH_H_
#define POLYGON_COVERAGE_GEOMETRY_VISIBILITY_GRAPH_H_

#include <map>

#include <polygon_coverage_solvers/graph_base.h>

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {
namespace visibility_graph {

struct NodeProperty {
  NodeProperty() : coordinates(Point_2(CGAL::ORIGIN)) {}
  NodeProperty(const Point_2& coordinates, const Polygon_2& visibility)
      : coordinates(coordinates), visibility(visibility) {}
  Point_2 coordinates;   // The 2D coordinates.
  Polygon_2 visibility;  // The visibile polygon from the vertex.
};

struct EdgeProperty {};

// Shortest path calculation in the reduced visibility graph.
// https://www.david-gouveia.com/pathfinding-on-a-2d-polygonal-map
class VisibilityGraph : public GraphBase<NodeProperty, EdgeProperty> {
 public:
  // Creates an undirected, weighted visibility graph.
  VisibilityGraph(const PolygonWithHoles& polygon);
  VisibilityGraph(const Polygon_2& polygon)
      : VisibilityGraph(PolygonWithHoles(polygon)) {}

  VisibilityGraph() : GraphBase() {}

  virtual bool create() override;

  // Compute the shortest path in a polygon with holes using A* and the
  // precomputed visibility graph.
  // If start or goal are outside the polygon, they are snapped (projected) back
  // into it.
  bool solve(const Point_2& start, const Point_2& goal,
             std::vector<Point_2>* waypoints) const;
  // Same as solve but provide a precomputed visibility graph for the polygon.
  // Note: Start and goal need to be contained in the polygon_.
  bool solve(const Point_2& start, const Polygon_2& start_visibility_polygon,
             const Point_2& goal, const Polygon_2& goal_visibility_polygon,
             std::vector<Point_2>* waypoints) const;

  // Convenience function: addtionally adds original start and goal to shortest
  // path, if they were outside of polygon.
  bool solveWithOutsideStartAndGoal(const Point_2& start, const Point_2& goal,
                                    std::vector<Point_2>* waypoints) const;
  // Given a solution, get the concatenated 2D waypoints.
  bool getWaypoints(const Solution& solution,
                    std::vector<Point_2>* waypoints) const;

  inline PolygonWithHoles getPolygon() const { return polygon_; }

 private:
  // Adds all line of sight neighbors.
  // The graph is acyclic and undirected and thus forms a symmetric adjacency
  // matrix.
  virtual bool addEdges() override;

  // Calculate the Euclidean distance to goal for all given nodes.
  virtual bool calculateHeuristic(size_t goal,
                                  Heuristic* heuristic) const override;

  // Find and append concave outer boundary vertices.
  void findConcaveOuterBoundaryVertices(
      std::vector<VertexConstCirculator>* concave_vertices) const;
  // Find and append convex hole vertices.
  void findConvexHoleVertices(
      std::vector<VertexConstCirculator>* convex_vertices) const;

  // Given two waypoints, compute its euclidean distance.
  double computeEuclideanSegmentCost(const Point_2& from,
                                     const Point_2& to) const;

  PolygonWithHoles polygon_;
};

}  // namespace visibility_graph
}  // namespace polygon_coverage_planning

#endif /* POLYGON_COVERAGE_GEOMETRY_VISIBILITY_GRAPH_H_ */
