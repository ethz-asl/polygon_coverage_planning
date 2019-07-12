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

#ifndef POLYGON_COVERAGE_SOLVERS_BOOLEAN_LATTICE_H_
#define POLYGON_COVERAGE_SOLVERS_BOOLEAN_LATTICE_H_

#include <set>
#include <numeric>

#include "polygon_coverage_solvers/graph_base.h"

namespace polygon_coverage_planning {
namespace boolean_lattice {
// Internal node property. Stores the set of visited clusters.
struct NodeProperty {
  NodeProperty() {}
  NodeProperty(const std::set<size_t>& visited_clusters)
      : visited_clusters(visited_clusters) {}
  std::set<size_t> visited_clusters;
  inline bool includesCluster(size_t cluster) const {
    return visited_clusters.count(cluster) > 0;
  }
};
struct EdgeProperty {};

// A boolean lattice stores the sets of already visited clusters / polygons. The
// sets are connected in a directed graph with zero edge cost. It starts off
// with the empty set and ends with the set of all clusters vistited.
// See also:
// https://en.wiktionary.org/wiki/Boolean_lattice and
// M. Rice, V. Tsotras, "Exact Graph Search Algorithms for Generalized
// Traveling Salesman Path Problems"
class BooleanLattice : public GraphBase<NodeProperty, EdgeProperty> {
 public:
  // num_polygons: Number of polygons to visit, excluding start and goal
  // cluster.
  BooleanLattice(size_t num_clusters)
      : GraphBase(),
        num_clusters_(num_clusters),
        start_cluster_(0),
        goal_cluster_(0) {
    sorted_original_clusters_.resize(num_clusters_);
    std::iota(sorted_original_clusters_.begin(),
              sorted_original_clusters_.end(), 0);
    is_created_ = create(); // Auto-create.
  }
  BooleanLattice() : GraphBase() {}

  virtual void clear() override;
  // Create a boolean lattice with num_clusters_ possible clusters.
  virtual bool create() override;
  virtual bool addStartNode(const NodeProperty& node_property)
      override;  // To be called after create().
  // Convenience method as NodeProperty argument is ignored anyways.
  bool addStartNode();
  virtual bool addGoalNode(const NodeProperty& node_property)
      override;  // To be called after create().
  // Convenience method as NodeProperty argument is ignored anyways.
  bool addGoalNode();

  inline size_t getStartCluster() const { return start_cluster_; }
  inline size_t getGoalCluster() const { return goal_cluster_; }

 private:
  virtual bool addEdges() override;

  // A node is predecessor of another node, if
  // - the node contains one combination element less than the other AND
  // - the node combination is a subset of the other node
  bool isConnected(const EdgeId& edge_id);

  std::vector<size_t> sorted_original_clusters_; // E.g., [0, 1, 2, ..., n-1].
  size_t num_clusters_;   // Total number of clusters including start and goal.
  size_t start_cluster_;  // Unique start cluster.
  size_t goal_cluster_;   // Unique goal cluster.
};
}  // namespace boolean_lattice
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_SOLVERS_BOOLEAN_LATTICE_H_
