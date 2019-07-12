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

#include <cmath>
#include <numeric>

#include <ros/console.h>

#include "polygon_coverage_solvers/boolean_lattice.h"
#include "polygon_coverage_solvers/combinatorics.h"

namespace polygon_coverage_planning {
namespace boolean_lattice {

void BooleanLattice::clear() {
  GraphBase::clear();
  num_clusters_ = sorted_original_clusters_.size();
  start_cluster_ = 0;
  goal_cluster_ = 0;
}

bool BooleanLattice::create() {
  clear();

  // Create all k elements of n combinations for all k in [0 .. n].
  graph_.reserve(
      std::exp2(num_clusters_));  // A boolean lattice has 2^n elements.
  for (size_t k = 0; k < num_clusters_ + 1; ++k) {
    std::vector<std::set<size_t>> combinations;
    getAllCombinationsOfKElementsFromN(sorted_original_clusters_, k,
                                       &combinations);
    // Add combinations as nodes to the graph.
    for (const std::set<size_t>& combination : combinations) {
      if (!addNode(NodeProperty(combination))) {
        return false;
      }
    }
  }

  ROS_INFO_STREAM("Successfully created boolean lattice with "
                  << graph_.size() << " nodes and " << edge_properties_.size()
                  << " edges.");

  is_created_ = true;
  return graph_.size() == std::exp2(num_clusters_)  // 2^n elements.
         && edge_properties_.size() ==
                num_clusters_ *
                    std::exp2(num_clusters_ - 1)  //  n * 2^(n-1) edges.
         && graph_.size() == node_properties_.size();
}

bool BooleanLattice::addEdges() {
  if (graph_.empty()) {
    ROS_INFO_STREAM("Cannot add edges to an empty graph.");
    return false;
  }

  const size_t new_id = graph_.size() - 1;
  for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
    EdgeId backwards_edge_id(adj_id, new_id);
    EdgeId forwards_edge_id(new_id, adj_id);
    if (isConnected(backwards_edge_id)) {
      if (!addEdge(backwards_edge_id, EdgeProperty(), 0.0)) {
        return false;
      }
    } else if (isConnected(forwards_edge_id)) {
      if (!addEdge(forwards_edge_id, EdgeProperty(), 0.0)) {
        return false;
      }
    }
  }
  return true;
}

bool BooleanLattice::isConnected(const EdgeId& edge_id) {
  // Access node properties:
  const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
  const NodeProperty* to_node_property = getNodeProperty(edge_id.second);
  if (from_node_property == nullptr || to_node_property == nullptr) {
    return false;
  }

  // Check if edge from_id->to_id exists:
  return from_node_property->visited_clusters.size() + 1 ==
             to_node_property->visited_clusters.size() &&
         std::includes(to_node_property->visited_clusters.begin(),
                       to_node_property->visited_clusters.end(),
                       from_node_property->visited_clusters.begin(),
                       from_node_property->visited_clusters.end());
}

bool BooleanLattice::addStartNode(const NodeProperty& node_property) {
  if (!is_created_) {
    ROS_ERROR_STREAM("create() needs to be called first.");
    return false;
  }

  start_idx_ = graph_.size();
  // Add start cluster to all existing node properties.
  start_cluster_ = num_clusters_;  // Unique start cluster.
  num_clusters_++;
  for (size_t i = 0; i < node_properties_.size(); ++i) {
    if (node_properties_.count(i) == 0) {
      is_created_ = false;  // Need to reset graph state.
      return false;
    }
    node_properties_[i].visited_clusters.insert(start_cluster_);
  }

  // Add new node with empty visited set.
  if (!addNode(NodeProperty())) {
    is_created_ = false;  // Need to reset graph state.
    return false;
  }

  return true;
}

bool BooleanLattice::addStartNode() { return addStartNode(NodeProperty()); }

bool BooleanLattice::addGoalNode(const NodeProperty& node_property) {
  if (!is_created_) {
    ROS_ERROR_STREAM("create() needs to be called first.");
    return false;
  }

  goal_idx_ = graph_.size();
  goal_cluster_ = num_clusters_;  // Unique goal cluster.
  num_clusters_++;

  // Goal node is connected to maximum visited cluster set.
  // Find maximum set.
  NodeProperty max_node_property =
      std::max_element(node_properties_.begin(), node_properties_.end(),
                       [](const std::pair<size_t, NodeProperty>& a,
                          const std::pair<size_t, NodeProperty>& b) {
                         return a.second.visited_clusters.size() <
                                b.second.visited_clusters.size();
                       })
          ->second;
  max_node_property.visited_clusters.insert(goal_cluster_);

  // Add new node with completely visited set.
  if (!addNode(max_node_property)) {
    is_created_ = false;
    return false;
  }

  return true;
}

bool BooleanLattice::addGoalNode() { return addGoalNode(NodeProperty()); }

}  // namespace boolean_lattice
}  // namespace polygon_coverage_planning
