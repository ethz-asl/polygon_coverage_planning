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

#ifndef POLYGON_COVERAGE_PLANNERS_GRAPHS_GTSPP_PRODUCT_GRAPH_H_
#define POLYGON_COVERAGE_PLANNERS_GRAPHS_GTSPP_PRODUCT_GRAPH_H_

#include <limits>
#include <vector>

#include <polygon_coverage_solvers/boolean_lattice.h>
#include <polygon_coverage_solvers/graph_base.h>

#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"

namespace polygon_coverage_planning {
namespace gtspp_product_graph {
// Internal node property. Stores the product graph information, i.e., the
// corresponding sweep plan graph ID and boolean lattice ID.
struct NodeProperty {
  NodeProperty()
      : sweep_plan_graph_id(std::numeric_limits<size_t>::max()),
        boolean_lattice_id(std::numeric_limits<size_t>::max()) {}
  NodeProperty(size_t sweep_plan_graph_id, size_t boolean_lattice_id)
      : sweep_plan_graph_id(sweep_plan_graph_id),
        boolean_lattice_id(boolean_lattice_id) {}
  size_t sweep_plan_graph_id;
  size_t boolean_lattice_id;
};

// Internal edge property. Stores information if an edge is E1 or E2.
struct EdgeProperty {
  enum Type { kE1 = 0, kE2, kUnknown };
  EdgeProperty() : type(kUnknown) {}
  EdgeProperty(Type type) : type(type) {}
  Type type;
};

// The GTSPP product graph is a product of boolean lattice and sweep plan graph.
// The product graph only has edges between clusters that have not been visited,
// yet. Thus a normal graph search algorithm, e.g., Dijkstra, finds an optimal
// solution. For details see: M. Rice, V. Tsotras, "Exact Graph Search
// Algorithms for Generalized Traveling Salesman Path Problems"
// It's a directed graph.
class GtsppProductGraph : public GraphBase<NodeProperty, EdgeProperty> {
 public:
  GtsppProductGraph() : GtsppProductGraph(nullptr, nullptr) {}
  GtsppProductGraph(const sweep_plan_graph::SweepPlanGraph* sweep_plan_graph,
                    const boolean_lattice::BooleanLattice* boolean_lattice)
      : GraphBase(),
        sweep_plan_graph_(sweep_plan_graph),
        boolean_lattice_(boolean_lattice) {}

  // Compute the product graph given sweep plan graph and boolean lattice.
  virtual bool create() override;
  // Allocate nodes but do not create edges.
  bool createOnline();
  virtual void clear() override;
  // Add a start node.
  virtual bool addStartNode(const NodeProperty& node_property) override;
  bool addStartNode();
  // Add a goal node.
  virtual bool addGoalNode(const NodeProperty& node_property) override;
  bool addGoalNode();

  // Update adjacency graph, e.g., when setting start and goal.
  inline void setSweepPlanGraph(
      const sweep_plan_graph::SweepPlanGraph* sweep_plan_graph) {
    sweep_plan_graph_ = sweep_plan_graph;
  }
  // Update boolean lattice, e.g., when setting start and goal.
  inline void setBooleanLattice(
      const boolean_lattice::BooleanLattice* boolean_lattice) {
    boolean_lattice_ = boolean_lattice;
  }

  // Solve the graph with Dijsktra search.
  bool solve(const Point_2& start, const Point_2& goal,
             std::vector<Point_2>* waypoints) const;
  // Build the graph while performing Dijkstra search.
  bool solveOnline(const Point_2& start, const Point_2& goal,
                   std::vector<Point_2>* waypoints) const;
  // Given a solution, get the concatenated sweep plan graph waypoints.
  bool getWaypoints(const Solution& solution,
                    std::vector<Point_2>* waypoints) const;

 private:
  // Nodes are connected based on E1 or E2 criterion.
  bool addEdges();

  // E1: Two vertices u, v are connected inside the same "combination set" c if
  // - they share the same boolean lattice node (c=c') AND
  // - they are connected in the original sweep plan graph ((u,v) \in E) AND
  // - v does belong to the covered polygons of c AND
  // - u does not belong to the covered polygons of c.
  bool isE1(const EdgeId& edge_id) const;
  bool isE1(size_t from_boolean_lattice_id, size_t to_boolean_lattice_id,
            size_t from_sweep_plan_graph_id, size_t to_sweep_plan_graph_id,
            const std::set<size_t>& cluster_set, size_t from_sweep_cluster,
            size_t to_sweep_cluster) const;

  // E2: Two vertices are connected between two "combination sets" c, c' if
  // - the covering node c is different from c' AND
  // - they represent the same sweep plan graph vertex (u=v) AND
  // - the two covering sets are connected in the original covering graph
  // ((c,c') \in E(B_k)) AND
  // - the "to"-vertex v polygon_id is NOT in the covered polygons of c AND
  // - the "to"-vertex v polygon_id is in the covered polygons of c'
  bool isE2(const EdgeId& edge_id) const;
  bool isE2(size_t from_sweep_plan_graph_id, size_t to_sweep_plan_graph_id,
            size_t from_boolean_lattice_id, size_t to_boolean_lattice_id,
            const std::set<size_t>& from_cluster_set,
            const std::set<size_t>& to_cluster_set, size_t sweep_cluster) const;

  const boolean_lattice::NodeProperty* getBooleanLatticeNodeProperty(
      size_t node_id) const;
  const sweep_plan_graph::NodeProperty* getSweepPlanGraphNodeProperty(
      size_t node_id) const;
  bool getSweepPlanGraphEdgeCost(const EdgeId& edge_id, double* cost) const;
  bool getBooleanLatticeEdgeCost(const EdgeId& edge_id, double* cost) const;

  // Create the graph while solving Dijkstra.
  bool createDijkstra(Solution* solution);

  // Corresponding sweep plan graph.
  const sweep_plan_graph::SweepPlanGraph* sweep_plan_graph_;
  // Corresponding boolean lattice.
  const boolean_lattice::BooleanLattice* boolean_lattice_;
};
}  // namespace gtspp_product_graph
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNERS_GRAPHS_GTSPP_PRODUCT_GRAPH_H_
