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

#include <ros/console.h>
#include <chrono>
#include <numeric>

#include "polygon_coverage_planners/graphs/gtspp_product_graph.h"

namespace polygon_coverage_planning {
namespace gtspp_product_graph {

const double kTimeOut = 200.0;

bool GtsppProductGraph::create() {
  if (sweep_plan_graph_ == nullptr || boolean_lattice_ == nullptr) {
    ROS_ERROR("Sweep plan graph or boolean lattice not set.");
    return false;
  }

  graph_.reserve(boolean_lattice_->size() * sweep_plan_graph_->size());

  for (size_t lattice_id = 0; lattice_id < boolean_lattice_->size();
       ++lattice_id) {
    for (size_t sweep_id = 0; sweep_id < sweep_plan_graph_->size();
         ++sweep_id) {
      if (!addNode(NodeProperty(sweep_id, lattice_id))) {
        return false;
      }
    }
  }

  ROS_INFO_STREAM("Created GTSPP product graph with "
                  << graph_.size() << " nodes and " << edge_properties_.size()
                  << " edges.");
  is_created_ = true;
  return true;
}

bool GtsppProductGraph::createOnline() {
  if (sweep_plan_graph_ == nullptr || boolean_lattice_ == nullptr) {
    ROS_ERROR("Sweep plan graph or boolean lattice not set.");
    return false;
  }

  graph_.resize(boolean_lattice_->size() * sweep_plan_graph_->size());

  for (size_t lattice_id = 0; lattice_id < boolean_lattice_->size();
       ++lattice_id) {
    for (size_t sweep_id = 0; sweep_id < sweep_plan_graph_->size();
         ++sweep_id) {
      // Add node without edges.
      const size_t idx = lattice_id * sweep_plan_graph_->size() + sweep_id;
      node_properties_.insert(
          std::make_pair(idx, NodeProperty(sweep_id, lattice_id)));
    }
  }

  ROS_INFO_STREAM("Created GTSPP product graph with "
                  << graph_.size() << " nodes without edges.");
  is_created_ = true;
  return true;
}

void GtsppProductGraph::clear() {
  GraphBase::clear();
  sweep_plan_graph_ = nullptr;
  boolean_lattice_ = nullptr;
}

bool GtsppProductGraph::addStartNode(const NodeProperty& node_property) {
  if (boolean_lattice_ == nullptr) {
    return false;
  }
  auto start_time = std::chrono::high_resolution_clock::now();
  for (size_t lattice_id = 0; lattice_id < boolean_lattice_->size();
       ++lattice_id) {
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    if (elapsed.count() > kTimeOut) {
      ROS_ERROR("Timout addStartNode.");
      return false;
    }
    if (lattice_id == boolean_lattice_->getStartIdx()) {
      start_idx_ = graph_.size();
    }
    if (!addNode(NodeProperty(sweep_plan_graph_->getStartIdx(), lattice_id))) {
      return false;
    }
  }
  return true;
}

bool GtsppProductGraph::addStartNode() { return addStartNode(NodeProperty()); }

bool GtsppProductGraph::addGoalNode(const NodeProperty& node_property) {
  if (boolean_lattice_ == nullptr) {
    return false;
  }
  auto start_time = std::chrono::high_resolution_clock::now();
  for (size_t lattice_id = 0; lattice_id < boolean_lattice_->size();
       ++lattice_id) {
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    if (elapsed.count() > kTimeOut) {
      ROS_ERROR("Timout addGoalNode.");
      return false;
    }
    if (lattice_id == boolean_lattice_->getGoalIdx()) {
      goal_idx_ = graph_.size();
    }
    if (!addNode(NodeProperty(sweep_plan_graph_->getGoalIdx(), lattice_id))) {
      return false;
    }
  }
  return true;
}

bool GtsppProductGraph::addGoalNode() { return addGoalNode(NodeProperty()); }

bool GtsppProductGraph::solve(const Point_2& start, const Point_2& goal,
                              std::vector<Point_2>* waypoints) const {
  if (!is_created_) {
    return false;
  }
  // Create copies of graphs to add start and goal.
  GtsppProductGraph temp_gtspp_product_graph = *this;
  sweep_plan_graph::SweepPlanGraph temp_sweep_plan_graph = *sweep_plan_graph_;
  boolean_lattice::BooleanLattice temp_boolean_lattice = *boolean_lattice_;

  // Add start and goal to temporary graphs.
  if (!temp_boolean_lattice.addStartNode() ||
      !temp_boolean_lattice.addGoalNode()) {
    return false;
  }

  sweep_plan_graph::NodeProperty start_sweep_node, goal_sweep_node;
  if (!temp_sweep_plan_graph.createNodeProperty(
          temp_boolean_lattice.getStartCluster(), start, &start_sweep_node) ||
      !temp_sweep_plan_graph.createNodeProperty(
          temp_boolean_lattice.getGoalCluster(), goal, &goal_sweep_node)) {
    return false;
  }

  if (!temp_sweep_plan_graph.addStartNode(start_sweep_node) ||
      !temp_sweep_plan_graph.addGoalNode(goal_sweep_node)) {
    return false;
  }

  temp_gtspp_product_graph.setSweepPlanGraph(&temp_sweep_plan_graph);
  temp_gtspp_product_graph.setBooleanLattice(&temp_boolean_lattice);
  if (!temp_gtspp_product_graph.addStartNode() ||
      !temp_gtspp_product_graph.addGoalNode()) {
    return false;
  }

  // Solve graph using Dijkstra.
  Solution solution;
  if (!temp_gtspp_product_graph.solveDijkstra(&solution)) {
    ROS_ERROR("Dijkstra failed.");
    return false;
  }

  return temp_gtspp_product_graph.getWaypoints(solution, waypoints);
}

bool GtsppProductGraph::solveOnline(const Point_2& start, const Point_2& goal,
                                    std::vector<Point_2>* waypoints) const {
  ROS_ASSERT(waypoints);
  waypoints->clear();

  // Create temporary graph structure.
  GtsppProductGraph temp_gtspp_product_graph = *this;
  sweep_plan_graph::SweepPlanGraph temp_sweep_plan_graph = *sweep_plan_graph_;
  boolean_lattice::BooleanLattice temp_boolean_lattice = *boolean_lattice_;

  // Add start and goal to temporary graphs.
  if (!temp_boolean_lattice.addStartNode() ||
      !temp_boolean_lattice.addGoalNode()) {
    return false;
  }

  sweep_plan_graph::NodeProperty start_sweep_node, goal_sweep_node;
  if (!temp_sweep_plan_graph.createNodeProperty(
          temp_boolean_lattice.getStartCluster(), start, &start_sweep_node) ||
      !temp_sweep_plan_graph.createNodeProperty(
          temp_boolean_lattice.getGoalCluster(), goal, &goal_sweep_node)) {
    return false;
  }

  if (!temp_sweep_plan_graph.addStartNode(start_sweep_node) ||
      !temp_sweep_plan_graph.addGoalNode(goal_sweep_node)) {
    return false;
  }

  temp_gtspp_product_graph.setSweepPlanGraph(&temp_sweep_plan_graph);
  temp_gtspp_product_graph.setBooleanLattice(&temp_boolean_lattice);
  if (!temp_gtspp_product_graph.addStartNode() ||
      !temp_gtspp_product_graph.addGoalNode()) {
    return false;
  }

  // Create graph while solving Dijkstra.
  Solution solution;
  if (!temp_gtspp_product_graph.createDijkstra(&solution)) {
    ROS_ERROR("Dijkstra failed.");
    return false;
  }

  return temp_gtspp_product_graph.getWaypoints(solution, waypoints);
}

bool GtsppProductGraph::getWaypoints(const Solution& solution,
                                     std::vector<Point_2>* waypoints) const {
  ROS_ASSERT(waypoints);
  waypoints->clear();

  if (sweep_plan_graph_ == nullptr) {
    ROS_ERROR("Sweep plan graph not set.");
    return false;
  }
  // Translate product graph solution in sweep plan graph indices.
  Solution sweep_plan_solution;
  for (size_t i = 0; i < solution.size() - 1; ++i) {
    // Access edge and node properties.
    const EdgeId edge_id(solution[i], solution[i + 1]);
    const EdgeProperty* edge_property = getEdgeProperty(edge_id);
    const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
    const NodeProperty* to_node_property = getNodeProperty(edge_id.second);
    if (edge_property == nullptr || from_node_property == nullptr ||
        to_node_property == nullptr) {
      return false;
    }

    // Check edge type (E1) and add to solution.
    if (edge_property->type == EdgeProperty::Type::kE1) {
      if (sweep_plan_solution.empty()) {
        sweep_plan_solution.push_back(from_node_property->sweep_plan_graph_id);
      }
      sweep_plan_solution.push_back(to_node_property->sweep_plan_graph_id);
    }
  }

  // Get waypoints.
  return sweep_plan_graph_->getWaypoints(sweep_plan_solution, waypoints);
}

bool GtsppProductGraph::addEdges() {
  // Find 'new' node properties.
  if (graph_.empty()) {
    ROS_ERROR("Cannot add edges to an empty graph.");
    return false;
  }

  const size_t new_id = graph_.size() - 1;
  // Possible 'adjacent' nodes:
  for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
    // Connected based on E1 or E2 criterion.
    const EdgeId forwards_edge_id(new_id, adj_id);
    const EdgeId backwards_edge_id(adj_id, new_id);
    if (isE1(forwards_edge_id)) {
      double cost = -1.0;
      EdgeProperty edge_property(EdgeProperty::Type::kE1);
      if (!getSweepPlanGraphEdgeCost(forwards_edge_id, &cost) ||
          !addEdge(forwards_edge_id, edge_property, cost)) {
        return false;
      }
    } else if (isE1(backwards_edge_id)) {
      double cost = -1.0;
      EdgeProperty edge_property(EdgeProperty::Type::kE1);
      if (!getSweepPlanGraphEdgeCost(backwards_edge_id, &cost) ||
          !addEdge(backwards_edge_id, edge_property, cost)) {
        return false;
      }
    } else if (isE2(forwards_edge_id)) {
      double cost = -1.0;
      EdgeProperty edge_property(EdgeProperty::Type::kE2);
      if (!getBooleanLatticeEdgeCost(forwards_edge_id, &cost) ||
          !addEdge(forwards_edge_id, edge_property, cost)) {
        return false;
      }
    } else if (isE2(backwards_edge_id)) {
      double cost = -1.0;
      EdgeProperty edge_property(EdgeProperty::Type::kE2);
      if (!getBooleanLatticeEdgeCost(backwards_edge_id, &cost) ||
          !addEdge(backwards_edge_id, edge_property, cost)) {
        return false;
      }
    }
  }

  return true;
}

bool GtsppProductGraph::isE1(const EdgeId& edge_id) const {
  // Access node properties.
  const NodeProperty* from = getNodeProperty(edge_id.first);
  const NodeProperty* to = getNodeProperty(edge_id.second);
  const boolean_lattice::NodeProperty* c =
      getBooleanLatticeNodeProperty(edge_id.first);
  const sweep_plan_graph::NodeProperty* from_v =
      getSweepPlanGraphNodeProperty(edge_id.first);
  const sweep_plan_graph::NodeProperty* to_v =
      getSweepPlanGraphNodeProperty(edge_id.second);
  if (from == nullptr || to == nullptr || c == nullptr || from_v == nullptr ||
      to_v == nullptr || sweep_plan_graph_ == nullptr) {
    return false;  // Bad access.
  }

  return isE1(from->boolean_lattice_id, to->boolean_lattice_id,
              from->sweep_plan_graph_id, to->sweep_plan_graph_id,
              c->visited_clusters, from_v->cluster, to_v->cluster);
}

bool GtsppProductGraph::isE1(size_t from_boolean_lattice_id,
                             size_t to_boolean_lattice_id,
                             size_t from_sweep_plan_graph_id,
                             size_t to_sweep_plan_graph_id,
                             const std::set<size_t>& cluster_set,
                             size_t from_sweep_cluster,
                             size_t to_sweep_cluster) const {
  return from_boolean_lattice_id == to_boolean_lattice_id  // Same lattice node.
         && sweep_plan_graph_->edgeExists(EdgeId(
                from_sweep_plan_graph_id,
                to_sweep_plan_graph_id))  // Sweep plan graph edge exists.
         && cluster_set.count(from_sweep_cluster) >
                0  // From vertex is covered already.
         && cluster_set.count(to_sweep_cluster) ==
                0;  // To vertex is not covered, yet.
}

bool GtsppProductGraph::isE2(const EdgeId& edge_id) const {
  // Access node properties.
  const NodeProperty* from = getNodeProperty(edge_id.first);
  const NodeProperty* to = getNodeProperty(edge_id.second);
  const boolean_lattice::NodeProperty* from_c =
      getBooleanLatticeNodeProperty(edge_id.first);
  const boolean_lattice::NodeProperty* to_c =
      getBooleanLatticeNodeProperty(edge_id.second);
  const sweep_plan_graph::NodeProperty* v =
      getSweepPlanGraphNodeProperty(edge_id.first);
  if (from == nullptr || to == nullptr || from_c == nullptr ||
      to_c == nullptr || v == nullptr || boolean_lattice_ == nullptr) {
    return false;  // Bad access.
  }

  return isE2(from->sweep_plan_graph_id, to->sweep_plan_graph_id,
              from->boolean_lattice_id, to->boolean_lattice_id,
              from_c->visited_clusters, to_c->visited_clusters, v->cluster);
}

bool GtsppProductGraph::isE2(size_t from_sweep_plan_graph_id,
                             size_t to_sweep_plan_graph_id,
                             size_t from_boolean_lattice_id,
                             size_t to_boolean_lattice_id,
                             const std::set<size_t>& from_cluster_set,
                             const std::set<size_t>& to_cluster_set,
                             size_t sweep_cluster) const {
  return from_sweep_plan_graph_id ==
             to_sweep_plan_graph_id  // Same sweep plan graph node.
         && boolean_lattice_->edgeExists(
                EdgeId(from_boolean_lattice_id,
                       to_boolean_lattice_id))  // Boolean lattice edge exists.
         && from_cluster_set.count(sweep_cluster) ==
                0  // Vertex is not in visited clusters of from_c.
         && to_cluster_set.count(sweep_cluster) >
                0;  // Vertex is in visited clusters of to_c.
}

const boolean_lattice::NodeProperty*
GtsppProductGraph::getBooleanLatticeNodeProperty(size_t node_id) const {
  const NodeProperty* node_property = getNodeProperty(node_id);
  if (boolean_lattice_ == nullptr || node_property == nullptr) {
    return nullptr;
  }
  return boolean_lattice_->getNodeProperty(node_property->boolean_lattice_id);
}

const sweep_plan_graph::NodeProperty*
GtsppProductGraph::getSweepPlanGraphNodeProperty(size_t node_id) const {
  const NodeProperty* node_property = getNodeProperty(node_id);
  if (sweep_plan_graph_ == nullptr || node_property == nullptr) {
    return nullptr;
  }
  return sweep_plan_graph_->getNodeProperty(node_property->sweep_plan_graph_id);
}

bool GtsppProductGraph::getSweepPlanGraphEdgeCost(const EdgeId& edge_id,
                                                  double* cost) const {
  ROS_ASSERT(cost);
  *cost = -1.0;
  const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
  const NodeProperty* to_node_property = getNodeProperty(edge_id.second);

  if (sweep_plan_graph_ == nullptr || from_node_property == nullptr ||
      to_node_property == nullptr) {
    return false;
  }

  EdgeId sweep_plan_graph_edge(from_node_property->sweep_plan_graph_id,
                               to_node_property->sweep_plan_graph_id);
  return sweep_plan_graph_->getEdgeCost(sweep_plan_graph_edge, cost);
}

bool GtsppProductGraph::getBooleanLatticeEdgeCost(const EdgeId& edge_id,
                                                  double* cost) const {
  ROS_ASSERT(cost);
  *cost = -1.0;
  const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
  const NodeProperty* to_node_property = getNodeProperty(edge_id.second);

  if (boolean_lattice_ == nullptr || from_node_property == nullptr ||
      to_node_property == nullptr) {
    return false;
  }

  EdgeId boolean_lattice_edge(from_node_property->boolean_lattice_id,
                              to_node_property->boolean_lattice_id);
  return boolean_lattice_->getEdgeCost(boolean_lattice_edge, cost);
}

bool GtsppProductGraph::createDijkstra(Solution* solution) {
  ROS_ASSERT(solution);
  solution->clear();
  if (!nodeExists(start_idx_) || !nodeExists(goal_idx_)) {
    return false;
  }
  clearEdges();

  // https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
  // Initialization.
  std::set<size_t> open_set = {start_idx_};  // Nodes to evaluate.
  std::set<size_t> closed_set;               // Nodes already evaluated.
  std::map<size_t, size_t> came_from;  // Get previous node on optimal path.
  std::map<size_t, double> cost;       // Optimal cost from start.
  for (size_t i = 0; i < graph_.size(); i++) {
    cost[i] = std::numeric_limits<double>::max();
  }
  cost[start_idx_] = 0.0;

  auto start_time = std::chrono::high_resolution_clock::now();
  while (!open_set.empty()) {
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    if (elapsed.count() > kTimeOut) {
      ROS_ERROR("Timout createDijkstra.");
      return false;
    }
    // Pop vertex with lowest score from open set.
    size_t current = *std::min_element(
        open_set.begin(), open_set.end(),
        [&cost](size_t i, size_t j) { return cost[i] < cost[j]; });
    if (current == goal_idx_) {  // Reached goal.
      *solution = reconstructSolution(came_from, current);
      return true;
    }
    open_set.erase(current);
    closed_set.insert(current);

    // Create all neighbors.
    for (size_t adj_id = 0; adj_id < graph_.size(); ++adj_id) {
      const EdgeId forwards_edge_id(current, adj_id);
      // E1 edges.
      if (isE1(forwards_edge_id)) {
        double temp_cost = -1.0;
        EdgeProperty edge_property(EdgeProperty::Type::kE1);
        if (!getSweepPlanGraphEdgeCost(forwards_edge_id, &temp_cost) ||
            !addEdge(forwards_edge_id, edge_property, temp_cost)) {
          return false;
        }
      } else if (isE2(forwards_edge_id)) {
        double temp_cost = -1.0;
        EdgeProperty edge_property(EdgeProperty::Type::kE2);
        if (!getBooleanLatticeEdgeCost(forwards_edge_id, &temp_cost) ||
            !addEdge(forwards_edge_id, edge_property, temp_cost)) {
          return false;
        }
      }
    }

    // Check all neighbors.
    for (const std::pair<size_t, double>& n : graph_[current]) {
      if (closed_set.count(n.first) > 0) {
        continue;  // Ignore already evaluated neighbors.
      }
      open_set.insert(n.first);  // Add to open set if not already in.

      // The distance from start to a neighbor.
      const double tentative_cost = cost[current] + n.second;
      if (tentative_cost >= cost[n.first]) {
        continue;  // This is not a better path to n.
      } else {
        // This path is the best path to n until now.
        came_from[n.first] = current;
        cost[n.first] = tentative_cost;
      }
    }
  }

  return false;
}

}  // namespace gtspp_product_graph
}  // namespace polygon_coverage_planning
