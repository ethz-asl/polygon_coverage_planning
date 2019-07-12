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

#ifndef POLYGON_COVERAGE_PLANNING_GRAPHS_SWEEP_PLAN_GRAPH_H_
#define POLYGON_COVERAGE_PLANNING_GRAPHS_SWEEP_PLAN_GRAPH_H_

#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/decomposition.h>
#include <polygon_coverage_geometry/visibility_graph.h>
#include <polygon_coverage_solvers/graph_base.h>

#include "polygon_coverage_planners/cost_functions/path_cost_functions.h"
#include "polygon_coverage_planners/sensor_models/sensor_model_base.h"

namespace polygon_coverage_planning {
namespace sweep_plan_graph {
// Internal node property. Stores the sweep plan / waypoint information.
struct NodeProperty {
  NodeProperty() : cost(-1.0), cluster(0) {}
  NodeProperty(const std::vector<Point_2>& waypoints,
               const PathCostFunction& cost_function, size_t cluster,
               const std::vector<Polygon_2>& visibility_polygons)
      : waypoints(waypoints),
        cost(cost_function(waypoints)),
        cluster(cluster),
        visibility_polygons(visibility_polygons) {}
  NodeProperty(const Point_2& waypoint, const PathCostFunction& cost_function,
               size_t cluster, const Polygon_2& visibility_polygon)
      : NodeProperty(std::vector<Point_2>({waypoint}), cost_function, cluster,
                     std::vector<Polygon_2>({visibility_polygon})) {}
  std::vector<Point_2> waypoints;  // The sweep path or start / goal waypoint.
  double cost;                     // The length of the path.
  size_t cluster;                  // The cluster these waypoints are covering.
  std::vector<Polygon_2> visibility_polygons;  // The visibility polygons at
                                               // start and goal of sweep.

  // Checks whether this node property is non-optimal compared to any node
  // in node_properties.
  bool isNonOptimal(const visibility_graph::VisibilityGraph& visibility_graph,
                    const std::vector<NodeProperty>& node_properties,
                    const PathCostFunction& cost_function) const;
};

// Internal edge property storage, i.e., shortest path.
struct EdgeProperty {
  EdgeProperty() : cost(-1.0) {}
  EdgeProperty(const std::vector<Point_2>& waypoints,
               const PathCostFunction& cost_function)
      : waypoints(waypoints), cost(cost_function(waypoints)) {}
  std::vector<Point_2> waypoints;  // The waypoints defining the edge.
  double cost;                     // The shortest path length.
};

// The adjacency graph contains all sweep plans (and waypoints) and its
// interconnections (edges). It is a dense, asymmetric, bidirectional graph.
class SweepPlanGraph : public GraphBase<NodeProperty, EdgeProperty> {
 public:
  struct Settings {
    PolygonWithHoles polygon;        // The input polygon to cover.
    PathCostFunction cost_function;  // The user defined cost function.
    std::shared_ptr<SensorModelBase> sensor_model;  // The sensor model.
    DecompositionType decomposition_type =
        DecompositionType::kBCD;  // The decomposition type.
    FT wall_distance = 0.0;       // The minimum distance to the polygon walls.
    bool offset_polygons = true;  // Flag to offset neighboring cells.
    bool sweep_single_direction =
        false;  // Flag to sweep only in best direction.
  };

  SweepPlanGraph(const Settings& settings)
      : GraphBase(), settings_(settings), visibility_graph_(settings_.polygon) {
    is_created_ = create();  // Auto-create.
  }
  SweepPlanGraph() : GraphBase() {}

  // Compute the sweep paths for each given cluster and create the adjacency
  // graph out of these.
  virtual bool create() override;

  // Solve the GTSP using GK MA.
  bool solve(const Point_2& start, const Point_2& goal,
             std::vector<Point_2>* waypoints) const;

  // Given a solution, get the concatenated 2D waypoints.
  bool getWaypoints(const Solution& solution,
                    std::vector<Point_2>* waypoints) const;

  bool getClusters(std::vector<std::vector<int>>* clusters) const;

  inline std::vector<Polygon_2> getDecomposition() const {
    return polygon_clusters_;
  }

  inline size_t getDecompositionSize() const {
    return polygon_clusters_.size();
  }

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

  // Compute the start and goal visibility polygon of a sweep. Also resets the
  // start and goal vertex in case they are not inside the polygon.
  bool computeStartAndGoalVisibility(
      const PolygonWithHoles& polygon, std::vector<Point_2>* sweep,
      std::vector<Polygon_2>* visibility_polygons) const;

  // Projects vertex into polygon and computes its visibility polygon.
  bool computeVisibility(const PolygonWithHoles& polygon, Point_2* vertex,
                         Polygon_2* visibility_polygon) const;

  // Offset the polygon from wall.
  void offsetPolygonFromWalls();

  // Compute the desired decomposition.
  bool computeDecomposition();

  // Offset neighboring decomposition cells from eachother.
  bool offsetDecomposition();

  // Check which decomposition cells are adjacent. Return whether each cell has
  // at least one neighbor.
  bool calculateDecompositionAdjacency(
      std::map<size_t, std::set<size_t>>* decomposition_adjacency);

  bool offsetAdjacentCells(const std::map<size_t, std::set<size_t>>& adj);

  Settings settings_;  // User input settings.
  visibility_graph::VisibilityGraph
      visibility_graph_;                     // The visibility to compute edges.
  std::vector<Polygon_2> polygon_clusters_;  // The polygon clusters.
};

}  // namespace sweep_plan_graph
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNING_GRAPHS_SWEEP_PLAN_GRAPH_H_
