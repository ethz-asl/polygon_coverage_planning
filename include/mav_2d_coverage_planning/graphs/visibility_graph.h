#ifndef MAV_COVERAGE_PLANNING_GRAPH_VISIBILITY_GRAPH_H_
#define MAV_COVERAGE_PLANNING_GRAPH_VISIBILITY_GRAPH_H_

#include <map>

#include <Eigen/Core>

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/cost_function.h"
#include "mav_coverage_planning/graph/graph_base.h"
#include "mav_coverage_planning/polygon.h"

namespace mav_coverage_planning {
namespace visibility_graph {

struct NodeProperty {
  NodeProperty() : coordinates(Eigen::Vector2d::Zero()) {}
  NodeProperty(const Eigen::Vector2d& coordinates, const Polygon& visibility)
      : coordinates(coordinates), visibility(visibility) {}
  Eigen::Vector2d coordinates;  // The 2D coordinates.
  Polygon visibility;           // The visibile polygon from the vertex.
};

struct EdgeProperty {};

// Points-of-visibility pathfinding.
// http://www.david-gouveia.com/portfolio/pathfinding-on-a-2d-polygonal-map/
class VisibilityGraph : public GraphBase<NodeProperty, EdgeProperty> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Creates an undirected, weighted visibility graph.
  VisibilityGraph(const Polygon& polygon, const CostFunction& cost_function);
  VisibilityGraph();

  virtual bool create() override;

  // Compute the shortest path in a polygon with holes using A* and the
  // precomputed visibility graph.
  // If start or goal are outside the polygon, they are snapped (projected) back
  // into it.
  bool solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
             StdVector2d* waypoints) const;
  // Same as solve but provide a precomputed visibility graph for the polygon.
  // Note: Start and goal need to be contained in the polygon_.
  bool solve(const Eigen::Vector2d& start,
             const Polygon& start_visibility_polygon,
             const Eigen::Vector2d& goal,
             const Polygon& goal_visibility_polygon, StdVector2d* waypoints) const;

  // Convenience function: addtionally adds original start and goal to shortest
  // path, if they were outside of polygon.
  bool solveWithOutsideStartAndGoal(const Eigen::Vector2d& start,
                                    const Eigen::Vector2d& goal,
                                    StdVector2d* waypoints) const;
  // Given a solution, get the concatenated 2D waypoints.
  bool getWaypoints(const Solution& solution, StdVector2d* waypoints) const;

  inline Polygon getPolygon() const { return polygon_; }

 private:
  // Adds all line of sight neighbors.
  // The graph is acyclic and undirected and thus forms a symmetric adjacency
  // matrix.
  virtual bool addEdges() override;

  // Select all concave hull vertices and convex hole vertices.
  void selectVertices(StdVector2d* graph_vertices) const;

  // Calculate the Euclidean distance to goal for all given nodes.
  virtual bool calculateHeuristic(size_t goal,
                                  Heuristic* heuristic) const override;

  // The polygon.
  Polygon polygon_;
  // The cost function.
  CostFunction cost_function_;
};
}  // namespace visibility_graph
}  // namespace mav_coverage_planning

#endif /* MAV_COVERAGE_PLANNING_GRAPH_VISIBILITY_GRAPH_H_ */
