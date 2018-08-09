#ifndef MAV_2D_COVERAGE_PLANNING_GRAPHS_SWEEP_PLAN_GRAPH_H_
#define MAV_2D_COVERAGE_PLANNING_GRAPHS_SWEEP_PLAN_GRAPH_H_

#include "mav_coverage_planning/graph/graph_base.h"

#include "mav_2d_coverage_planning/graphs/visibility_graph.h"
#include "mav_2d_coverage_planning/polygon.h"

#include "mav_2d_coverage_planning/cost_functions/euclidean_cost_function.h"

// TODO(rikba): Make cost function template.

namespace mav_coverage_planning {
namespace sweep_plan_graph {
// Internal node property. Stores the sweep plan / waypoint information.
struct NodeProperty {
  NodeProperty() : cost(-1.0), cluster(0) {}
  NodeProperty(const std::vector<Point_2>& waypoints,
               const EuclideanCostFunction& cost_function, size_t cluster,
               const std::vector<Polygon>& visibility_polygons)
      : waypoints(waypoints),
        cost(cost_function.computeCost(waypoints)),
        cluster(cluster),
        visibility_polygons(visibility_polygons) {}
  NodeProperty(const Point_2& waypoint,
               const EuclideanCostFunction& cost_function, size_t cluster,
               const Polygon& visibility_polygon)
      : NodeProperty(std::vector<Point_2>({waypoint}), cost_function, cluster,
                     std::vector<Polygon>({visibility_polygon})) {}
  std::vector<Point_2> waypoints;  // The sweep path or start / goal waypoint.
  double cost;                     // The length of the path.
  size_t cluster;                  // The cluster these waypoints are covering.
  std::vector<Polygon> visibility_polygons;  // The visibility polygons at start
                                             // and goal of sweep.

  // Checks whether this node property is non-optimal compared to any node
  // in node_properties.
  bool isNonOptimal(const visibility_graph::VisibilityGraph& visibility_graph,
                    const std::vector<NodeProperty>& node_properties,
                    const EuclideanCostFunction& cost_function) const;
};

// Internal edge property storage, i.e., shortest path.
struct EdgeProperty {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeProperty() : cost(-1.0) {}
  EdgeProperty(const std::vector<Point_2>& waypoints,
               const EuclideanCostFunction& cost_function)
      : waypoints(waypoints), cost(cost_function.computeCost(waypoints)) {}
  std::vector<Point_2> waypoints;  // The waypoints defining the edge.
  double cost;                     // The shortest path length.
};

// The adjacency graph contains all sweep plans (and waypoints) and its
// interconnections (edges). It is a dense, asymmetric, bidirectional graph.
class SweepPlanGraph : public GraphBase<NodeProperty, EdgeProperty> {
 public:
    SweepPlanGraph(const Polygon& polygon,
                 const std::vector<Polygon>& polygon_clusters,
                 double sweep_distance)
      : GraphBase(),
        visibility_graph_(polygon),
        cost_function_(cost_function),
        polygon_clusters_(polygon_clusters),
        sweep_distance_(sweep_distance) {
    is_created_ = create();  // Auto-create.
  }
  SweepPlanGraph() : GraphBase() {}

  // Compute the sweep paths for each given cluster and create the adjacency
  // graph out of these.
  virtual bool create() override;

  inline EuclideanCostFunction getEuclideanCostFunction() const {
    return cost_function_;
  }

  // Solve the GTSP using GK MA.
  bool solve(const Point_2& start, const Point_2& goal,
             std::vector<Point_2>* waypoints) const;

  // Given a solution, get the concatenated 2D waypoints.
  bool getWaypoints(const Solution& solution,
                    std::vector<Point_2>* waypoints) const;

  bool getClusters(std::vector<std::vector<int>>* clusters) const;

  // Note: projects the start and goal inside the polygon.
  bool createNodeProperty(size_t cluster, std::vector<Point_2>* waypoints,
                          NodeProperty* node) const;
  inline bool createNodeProperty(size_t cluster, const Point_2& waypoint,
                                 NodeProperty* node) const {
    std::vector<Point_2> waypoints({waypoint});
    return createNodeProperty(cluster, &waypoints, node);
  }

 private:
  virtual bool addEdges() override;
  bool computeEdge(const EdgeId& edge_id, EdgeProperty* edge_property) const;
  // Calculate cost to go to node.
  // cost = from_sweep_cost + cost(from_end, to_start)
  bool computeCost(const EdgeId& edge_id, const EdgeProperty& edge_property,
                   double* cost) const;

  // Two sweeps are potentially connected if
  // - they are from different clusters AND
  // - 'to' node is not the start node AND
  // - 'from' node is not the goal AND
  // - edge is not between start and goal.
  bool isConnected(const EdgeId& edge_id) const;

  // Compute all possible sweep plans for a given simple polygon.
  bool computeLineSweepPlans(
      const Polygon& polygon,
      std::vector<std::vector<Point_2>>* cluster_sweeps) const;

  // Compute the start and goal visibility polygon of a sweep. Also resets the
  // start and goal vertex in case they are not inside the polygon.
  bool computeStartAndGoalVisibility(
      const Polygon& polygon, std::vector<Point_2>* sweep,
      std::vector<Polygon>* visibility_polygons) const;

  // Projects vertex into polygon and computes its visibility polygon.
  bool computeVisibility(const Polygon& polygon, Point_2* vertex,
                         Polygon* visibility_polygon) const;

  visibility_graph::VisibilityGraph
      visibility_graph_;                   // The visibility to compute edges.
  EuclideanCostFunction cost_function_;    // The user defined cost function.
  std::vector<Polygon> polygon_clusters_;  // The polygon clusters.
  double sweep_distance_;                  // The distance between the sweeps.
};

}  // namespace sweep_plan_graph
}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_GRAPHS_SWEEP_PLAN_GRAPH_H_
