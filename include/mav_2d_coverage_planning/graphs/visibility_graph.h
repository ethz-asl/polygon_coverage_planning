#ifndef MAV_2D_COVERAGE_PLANNING_GRAPHS_VISIBILITY_GRAPH_H_
#define MAV_2D_COVERAGE_PLANNING_GRAPHS_VISIBILITY_GRAPH_H_

#include <map>

#include <mav_coverage_graph_solvers/graph_base.h>

#include <mav_coverage_planning_comm/cgal_definitions.h>
#include "mav_2d_coverage_planning/geometry/polygon.h"

namespace mav_coverage_planning {
namespace visibility_graph {

struct NodeProperty {
  NodeProperty() : coordinates(Point_2(CGAL::ORIGIN)) {}
  NodeProperty(const Point_2& coordinates, const Polygon& visibility)
      : coordinates(coordinates), visibility(visibility) {}
  Point_2 coordinates;  // The 2D coordinates.
  Polygon visibility;   // The visibile polygon from the vertex.
};

struct EdgeProperty {};

// Points-of-visibility pathfinding.
// http://www.david-gouveia.com/portfolio/pathfinding-on-a-2d-polygonal-map/
class VisibilityGraph : public GraphBase<NodeProperty, EdgeProperty> {
 public:
  // Creates an undirected, weighted visibility graph.
  VisibilityGraph(const Polygon& polygon, double offset_distance = 0.0);

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
  bool solve(const Point_2& start, const Polygon& start_visibility_polygon,
             const Point_2& goal, const Polygon& goal_visibility_polygon,
             std::vector<Point_2>* waypoints) const;

  // Convenience function: addtionally adds original start and goal to shortest
  // path, if they were outside of polygon.
  bool solveWithOutsideStartAndGoal(const Point_2& start, const Point_2& goal,
                                    std::vector<Point_2>* waypoints) const;
  // Given a solution, get the concatenated 2D waypoints.
  bool getWaypoints(const Solution& solution,
                    std::vector<Point_2>* waypoints) const;

  inline Polygon getPolygon() const { return polygon_; }

 private:
  // Adds all line of sight neighbors.
  // The graph is acyclic and undirected and thus forms a symmetric adjacency
  // matrix.
  virtual bool addEdges() override;

  // Calculate the Euclidean distance to goal for all given nodes.
  virtual bool calculateHeuristic(size_t goal,
                                  Heuristic* heuristic) const override;

  Polygon polygon_;
  double offset_distance_;
};

}  // namespace visibility_graph
}  // namespace mav_coverage_planning

#endif /* MAV_2D_COVERAGE_PLANNING_GRAPHS_VISIBILITY_GRAPH_H_ */
