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

#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"
#include "polygon_coverage_planners/timing.h"

#include <ros/assert.h>

#include <polygon_coverage_geometry/bcd.h>
#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_geometry/decomposition.h>
#include <polygon_coverage_geometry/offset.h>
#include <polygon_coverage_geometry/sweep.h>
#include <polygon_coverage_geometry/tcd.h>
#include <polygon_coverage_geometry/visibility_polygon.h>

#include <polygon_coverage_solvers/glkh.h>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

namespace polygon_coverage_planning {
namespace sweep_plan_graph {

bool NodeProperty::isNonOptimal(
    const visibility_graph::VisibilityGraph& visibility_graph,
    const std::vector<NodeProperty>& node_properties,
    const PathCostFunction& cost_function) const {
  if (waypoints.empty()) {
    ROS_WARN_STREAM("Node does not have waypoints.");
    return false;
  }

  for (const NodeProperty& node_property : node_properties) {
    if (node_property.waypoints.empty()) {
      ROS_WARN_STREAM("Comparison node does not have waypoints.");
      continue;
    }
    if (node_property.cluster == cluster) {
      std::vector<Point_2> path_front_front, path_back_back;
      if (!visibility_graph.solve(
              waypoints.front(), visibility_polygons.front(),
              node_property.waypoints.front(),
              node_property.visibility_polygons.front(), &path_front_front) ||
          !visibility_graph.solve(node_property.waypoints.back(),
                                  node_property.visibility_polygons.back(),
                                  waypoints.back(), visibility_polygons.back(),
                                  &path_back_back)) {
        continue;
      }
      if (cost_function(path_front_front) + node_property.cost +
              cost_function(path_back_back) <
          cost) {
        return true;
      }
    }
  }
  return false;
}

void SweepPlanGraph::offsetPolygonFromWalls() {
  if (settings_.wall_distance <= 0.0) return;

  PolygonWithHoles temp_poly = settings_.polygon;
  computeOffsetPolygon(temp_poly, settings_.wall_distance, &settings_.polygon);
  // Update visibility graph.
  visibility_graph_ = visibility_graph::VisibilityGraph(settings_.polygon);
}

bool SweepPlanGraph::create() {
  clear();
  offsetPolygonFromWalls();
  if (!computeDecomposition()) {
    ROS_ERROR("Failed to compute decomposition.");
    return false;
  }
  if (!offsetDecomposition()) {
    ROS_ERROR("Failed to offset neighboring decomposition cells.");
    return false;
  }

  // Create adjacency graph.
  size_t num_sweep_plans = 0;
  // Create sweep plans for each cluster.
  for (size_t cluster = 0; cluster < polygon_clusters_.size(); ++cluster) {
    // Compute all cluster sweeps.
    std::vector<std::vector<Point_2>> cluster_sweeps;
    timing::Timer timer_line_sweeps("line_sweeps");
    if (settings_.sweep_single_direction) {
      Direction_2 best_dir;
      findBestSweepDir(polygon_clusters_[cluster], &best_dir);
      visibility_graph::VisibilityGraph vis_graph(polygon_clusters_[cluster]);
      cluster_sweeps.resize(1);
      ROS_ASSERT(settings_.sensor_model);
      if (!computeSweep(polygon_clusters_[cluster], vis_graph,
                        settings_.sensor_model->getSweepDistance(), best_dir,
                        true, &cluster_sweeps.front())) {
        ROS_ERROR_STREAM(
            "Cannot compute single sweep for cluster: " << cluster);
        return false;
      }
    } else {
      ROS_ASSERT(settings_.sensor_model);
      if (!computeAllSweeps(polygon_clusters_[cluster],
                            settings_.sensor_model->getSweepDistance(),
                            &cluster_sweeps)) {
        ROS_ERROR_STREAM("Cannot create all sweep plans for cluster "
                         << cluster);
        return false;
      }
    }
    num_sweep_plans += cluster_sweeps.size();
    timer_line_sweeps.Stop();

    // Create node properties.
    timing::Timer timer_node_creation("node_creation");
    std::vector<NodeProperty> node_properties;
    node_properties.resize(cluster_sweeps.size());
    for (size_t i = 0; i < node_properties.size(); ++i) {
      NodeProperty node;
      if (!createNodeProperty(cluster, &cluster_sweeps[i], &node)) {
        return false;
      }
      node_properties[i] = node;
    }
    timer_node_creation.Stop();

    timing::Timer timer_pruning("pruning");
    // Prune nodes that are definitely not optimal.
    std::vector<NodeProperty>::iterator new_end = std::remove_if(
        node_properties.begin(), node_properties.end(),
        [node_properties, this](const NodeProperty& node_property) {
          return node_property.isNonOptimal(visibility_graph_, node_properties,
                                            settings_.cost_function);
        });
    node_properties.erase(new_end, node_properties.end());
    timer_pruning.Stop();

    // For each remaining sweep create a node.
    timing::Timer timer_edge_creation("edge_creation");
    for (const NodeProperty& node_property : node_properties) {
      if (!addNode(node_property)) {
        return false;
      }
    }
    timer_edge_creation.Stop();
  }

  ROS_INFO_STREAM("Created sweep plan graph with "
                  << graph_.size() << " nodes and " << edge_properties_.size()
                  << " edges.");
  ROS_INFO_STREAM("Initially created " << num_sweep_plans << " nodes.");
  ROS_INFO_STREAM("Pruned " << num_sweep_plans - graph_.size() << " nodes.");
  is_created_ = true;
  return true;
}

bool SweepPlanGraph::computeDecomposition() {
  // Create decomposition.
  timing::Timer timer_decom("decomposition");
  switch (settings_.decomposition_type) {
    case DecompositionType::kBCD: {
      if (!computeBestBCDFromPolygonWithHoles(settings_.polygon,
                                              &polygon_clusters_)) {
        ROS_ERROR_STREAM("Cannot compute boustrophedon decomposition.");
        return false;
      } else {
        ROS_INFO_STREAM("Successfully created boustrophedon decomposition with "
                        << polygon_clusters_.size() << " polygon(s).");
      }
      break;
    }
    case DecompositionType::kTCD: {
      if (!computeBestTCDFromPolygonWithHoles(settings_.polygon,
                                              &polygon_clusters_)) {
        ROS_ERROR_STREAM("Cannot compute trapezoidal decomposition.");
        return false;
      } else {
        ROS_INFO_STREAM("Successfully created trapezoidal decomposition with "
                        << polygon_clusters_.size() << " polygon(s).");
      }
      break;
    }
    default: {
      ROS_ERROR_STREAM("No valid decomposition type set.");
      return false;
      break;
    }
  }
  timer_decom.Stop();

  return true;
}

bool SweepPlanGraph::offsetDecomposition() {
  // Compute adjacency.
  timing::Timer timer_poly_adj("polygon_adjacency");
  std::map<size_t, std::set<size_t>> adj;
  if (settings_.offset_polygons && !calculateDecompositionAdjacency(&adj)) {
    ROS_ERROR("Decomposition not fully connected.");
    return false;
  }
  timer_poly_adj.Stop();

  // Offset adjacent cells.
  timing::Timer timer_poly_offset("poly_offset");
  if (settings_.offset_polygons && !offsetAdjacentCells(adj)) {
    ROS_ERROR("Failed to offset rectangular decomposition.");
    return false;
  }
  timer_poly_offset.Stop();
  return true;
}

bool SweepPlanGraph::calculateDecompositionAdjacency(
    std::map<size_t, std::set<size_t>>* decomposition_adjacency) {
  ROS_ASSERT(decomposition_adjacency);
  (*decomposition_adjacency)[0] = std::set<size_t>();

  for (size_t i = 0; i < polygon_clusters_.size() - 1; ++i) {
    for (size_t j = i + 1; j < polygon_clusters_.size(); ++j) {
      PolygonWithHoles joined;
      if (CGAL::join(polygon_clusters_[i], polygon_clusters_[j], joined)) {
        (*decomposition_adjacency)[i].emplace(j);
        (*decomposition_adjacency)[j].emplace(i);
      }
    }
  }

  // Check connectivity.
  if (polygon_clusters_.size() == 1)
    return decomposition_adjacency->count(0) == 1;
  for (size_t i = 0; i < polygon_clusters_.size(); ++i) {
    if (decomposition_adjacency->find(i) == decomposition_adjacency->end()) {
      return false;
    }
  }
  return true;
}

bool SweepPlanGraph::offsetAdjacentCells(
    const std::map<size_t, std::set<size_t>>& adj) {
  // Find overlapping edges.
  std::vector<Polygon_2> offsetted_decomposition = polygon_clusters_;
  std::vector<Segment_2> offsetted_segments;
  // For all cells...
  for (size_t i = 0; i < polygon_clusters_.size(); ++i) {
    const Polygon_2& cell = polygon_clusters_[i];
    const size_t num_edges_cell = cell.size();
    // Go through all neighbors of cell i.
    for (auto it = adj.at(i).begin(); it != adj.at(i).end(); it++) {
      const Polygon_2& neighbor = polygon_clusters_[*it];
      // If they do not touch anymore, skip.
      PolygonWithHoles joined;
      if (!CGAL::join(cell, neighbor, joined)) continue;

      const size_t num_edges_neighbor = neighbor.size();
      for (size_t cell_e = 0; cell_e < num_edges_cell; ++cell_e) {
        if (std::find(offsetted_segments.begin(), offsetted_segments.end(),
                      cell.edge(cell_e)) != offsetted_segments.end())
          continue;  // Already offsetted this segment.
        for (size_t neighbor_e = 0; neighbor_e < num_edges_neighbor;
             ++neighbor_e) {
          if (std::find(offsetted_segments.begin(), offsetted_segments.end(),
                        neighbor.edge(neighbor_e)) != offsetted_segments.end())
            continue;  // Already offsetted this segment.
          // If segments intersect, offset polygon.
          CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type
              result = CGAL::intersection(cell.edge(cell_e),
                                          neighbor.edge(neighbor_e));
          if (result) {
            if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
              if (*s == cell.edge(cell_e) ||
                  s->opposite() == cell.edge(cell_e)) {
                Polygon_2 offset_cell;
                ROS_ASSERT(settings_.sensor_model);
                if (!offsetEdgeWithRadialOffset(
                        cell, cell_e,
                        settings_.sensor_model->getSweepDistance(),
                        &offset_cell))
                  return false;
                std::vector<PolygonWithHoles> intersection_list;
                CGAL::intersection(offsetted_decomposition[i], offset_cell,
                                   std::back_inserter(intersection_list));
                if (intersection_list.size() != 1) return false;
                if (intersection_list.front().number_of_holes() != 0)
                  return false;
                offsetted_decomposition[i] =
                    intersection_list.front().outer_boundary();
                offsetted_segments.push_back(*s);
                offsetted_segments.push_back(s->opposite());
              } else if (*s == neighbor.edge(neighbor_e) ||
                         s->opposite() == neighbor.edge(neighbor_e)) {
                Polygon_2 offset_neighbor;
                ROS_ASSERT(settings_.sensor_model);
                if (!offsetEdgeWithRadialOffset(
                        neighbor, neighbor_e,
                        settings_.sensor_model->getSweepDistance(),
                        &offset_neighbor))
                  return false;
                std::vector<PolygonWithHoles> intersection_list;
                CGAL::intersection(offsetted_decomposition[*it],
                                   offset_neighbor,
                                   std::back_inserter(intersection_list));
                if (intersection_list.size() != 1) return false;
                if (intersection_list.front().number_of_holes() != 0)
                  return false;
                offsetted_decomposition[*it] =
                    intersection_list.front().outer_boundary();
                offsetted_segments.push_back(*s);
                offsetted_segments.push_back(s->opposite());
              } else {
                ROS_DEBUG_STREAM("Segment intersection but not identical.");
              }
            } else {
              ROS_DEBUG_STREAM("Only point intersection... "
                               << *boost::get<Point_2>(&*result));
            }
          }
        }
      }
    }
  }
  polygon_clusters_ = offsetted_decomposition;

  return true;
}

bool SweepPlanGraph::getClusters(
    std::vector<std::vector<int>>* clusters) const {
  ROS_ASSERT(clusters);
  clusters->clear();

  std::set<size_t> cluster_set;
  for (size_t i = 0; i < graph_.size(); ++i) {
    const NodeProperty* node = getNodeProperty(i);
    if (node == nullptr) {
      return false;  // Node property does not exist.
    }
    cluster_set.insert(node->cluster);
  }

  std::set<size_t> expected_clusters;
  for (size_t i = 0; i < cluster_set.size(); ++i) {
    expected_clusters.insert(i);
  }
  if (cluster_set != expected_clusters) {
    return false;  // Clusters not enumerated [0 .. n-1].
  }

  clusters->resize(cluster_set.size());
  for (size_t i = 0; i < clusters->size(); ++i) {
    for (size_t j = 0; j < graph_.size(); ++j) {
      const NodeProperty* node = getNodeProperty(j);
      if (node == nullptr) {
        return false;  // Node property does not exist.
      }
      if (node->cluster == i) {
        (*clusters)[i].push_back(static_cast<int>(j));
      }
    }
  }

  return true;
}

bool SweepPlanGraph::createNodeProperty(size_t cluster,
                                        std::vector<Point_2>* waypoints,
                                        NodeProperty* node) const {
  ROS_ASSERT(waypoints);
  ROS_ASSERT(node);

  std::vector<Polygon_2> visibility_polygons;
  if (!computeStartAndGoalVisibility(settings_.polygon, waypoints,
                                     &visibility_polygons)) {
    ROS_ERROR("Cannot compute start and goal visibility graph.");
    return false;
  }

  *node = NodeProperty(*waypoints, settings_.cost_function, cluster,
                       visibility_polygons);

  return true;
}

bool SweepPlanGraph::addEdges() {
  if (graph_.empty()) {
    ROS_ERROR("Cannot add edges to an empty graph.");
    return false;
  }

  const size_t new_id = graph_.size() - 1;
  for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
    EdgeId forwards_edge_id(new_id, adj_id);
    EdgeProperty edge_property;
    if (isConnected(forwards_edge_id) &&
        computeEdge(forwards_edge_id, &edge_property)) {
      double cost = -1.0;
      if (!computeCost(forwards_edge_id, edge_property, &cost) ||
          !addEdge(forwards_edge_id, edge_property, cost)) {
        return false;
      }
    }
    EdgeId backwards_edge_id(adj_id, new_id);
    if (isConnected(backwards_edge_id) &&
        computeEdge(backwards_edge_id, &edge_property)) {
      double cost = -1.0;
      if (!computeCost(backwards_edge_id, edge_property, &cost) ||
          !addEdge(backwards_edge_id, edge_property, cost)) {
        return false;
      }
    }
  }

  return true;
}

bool SweepPlanGraph::computeEdge(const EdgeId& edge_id,
                                 EdgeProperty* edge_property) const {
  ROS_ASSERT(edge_property);

  // Access node properties:
  const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
  const NodeProperty* to_node_property = getNodeProperty(edge_id.second);
  if (from_node_property == nullptr || to_node_property == nullptr) {
    return false;
  }

  // Calculate shortest path.
  if (from_node_property->waypoints.empty() ||
      to_node_property->waypoints.empty()) {
    ROS_ERROR("Waypoints in node property are empty.");
    return false;
  }

  std::vector<Point_2> shortest_path;
  if (!visibility_graph_.solve(from_node_property->waypoints.back(),
                               from_node_property->visibility_polygons.back(),
                               to_node_property->waypoints.front(),
                               to_node_property->visibility_polygons.front(),
                               &shortest_path)) {
    ROS_ERROR_STREAM("Cannot compute shortest path from "
                     << from_node_property->waypoints.back() << " to "
                     << to_node_property->waypoints.front());
    return false;
  }

  *edge_property = EdgeProperty(shortest_path, settings_.cost_function);

  return true;
}

bool SweepPlanGraph::computeCost(const EdgeId& edge_id,
                                 const EdgeProperty& edge_property,
                                 double* cost) const {
  // cost = from_sweep_cost + cost(from_end, to_start)
  const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
  if (from_node_property == nullptr) {
    return false;
  }
  *cost = from_node_property->cost + edge_property.cost;
  return true;
}

bool SweepPlanGraph::isConnected(const EdgeId& edge_id) const {
  // Access node properties:
  const NodeProperty* from_node_property = getNodeProperty(edge_id.first);
  const NodeProperty* to_node_property = getNodeProperty(edge_id.second);
  if (from_node_property == nullptr || to_node_property == nullptr) {
    return false;
  }

  return from_node_property->cluster !=
             to_node_property->cluster    // Different clusters.
         && edge_id.first != goal_idx_    // No connection from goal.
         && edge_id.second != start_idx_  // No connection to start.
         && !(edge_id.first == start_idx_ &&
              edge_id.second ==
                  goal_idx_);  // No direct connection between start and goal.
}

bool SweepPlanGraph::solve(const Point_2& start, const Point_2& goal,
                           std::vector<Point_2>* waypoints) const {
  ROS_ASSERT(waypoints);
  waypoints->clear();

  if (!is_created_) {
    ROS_ERROR("Graph not created.");
    return false;
  }

  // Create temporary copies to add start and goal.
  SweepPlanGraph temp_gtsp_graph = *this;

  NodeProperty start_node, goal_node;
  if (!createNodeProperty(polygon_clusters_.size(), start, &start_node) ||
      !createNodeProperty(polygon_clusters_.size() + 1, goal, &goal_node)) {
    return false;
  }

  if (!temp_gtsp_graph.addStartNode(start_node) ||
      !temp_gtsp_graph.addGoalNode(goal_node)) {
    ROS_ERROR("Cannot add start and goal.");
    return false;
  }
  const size_t goal_idx = temp_gtsp_graph.size() - 1;
  const size_t start_idx = temp_gtsp_graph.size() - 2;

  // Solve using GLKH.
  std::vector<std::vector<int>> m = temp_gtsp_graph.getAdjacencyMatrix();
  std::vector<std::vector<int>> clusters;
  if (!temp_gtsp_graph.getClusters(&clusters)) {
    ROS_ERROR("Cannot get clusters.");
    return false;
  }
  glkh::Task task(m, clusters);
  glkh::Glkh& solver = glkh::Glkh::getInstance();
  solver.setSolver(task);

  ROS_INFO("Start solving GTSP");
  if (!solver.solve()) {
    ROS_ERROR("GLKH solution failed.");
    return false;
  }
  ROS_INFO("Finished solving GTSP");
  std::vector<int> solution_int = solver.getSolution();
  Solution solution(solution_int.size());
  std::copy(solution_int.begin(), solution_int.end(), solution.begin());

  // Sort solution such that start node is at begin.
  Solution::iterator start_it =
      std::find(solution.begin(), solution.end(), start_idx);
  if (start_it == solution.end()) {
    ROS_ERROR("Cannot find start node in solution.");
    return false;
  }
  std::rotate(solution.begin(), start_it, solution.end());
  if (solution.back() != goal_idx) {
    ROS_ERROR("Goal node is not at back of solution.");
    return false;
  }

  if (!temp_gtsp_graph.getWaypoints(solution, waypoints)) {
    ROS_ERROR("Cannot recover waypoints.");
    return false;
  }

  return true;
}

bool SweepPlanGraph::getWaypoints(const Solution& solution,
                                  std::vector<Point_2>* waypoints) const {
  ROS_ASSERT(waypoints);
  waypoints->clear();

  for (size_t i = 0; i < solution.size() - 1; ++i) {
    const EdgeId edge_id(solution[i], solution[i + 1]);

    // Add sweep plan / start / goal waypoints.
    const NodeProperty* node_property = getNodeProperty(edge_id.first);
    if (node_property == nullptr) {
      return false;
    }
    waypoints->insert(waypoints->end(), node_property->waypoints.begin(),
                      node_property->waypoints.end());

    // Add shortest path.
    const EdgeProperty* edge_property = getEdgeProperty(edge_id);
    if (edge_property == nullptr) {
      return false;
    }
    // Crop first and last waypoint as these are included in sweep plan.
    waypoints->insert(waypoints->end(), edge_property->waypoints.begin() + 1,
                      edge_property->waypoints.end() - 1);
    // Add last waypoint.
    if (i == solution.size() - 2) {
      waypoints->insert(waypoints->end(), edge_property->waypoints.end() - 1,
                        edge_property->waypoints.end());
    }
  }
  return true;
}

bool SweepPlanGraph::computeStartAndGoalVisibility(
    const PolygonWithHoles& polygon, std::vector<Point_2>* sweep,
    std::vector<Polygon_2>* visibility_polygons) const {
  ROS_ASSERT(sweep);
  ROS_ASSERT(visibility_polygons);
  visibility_polygons->resize(2);

  if (sweep->front() == sweep->back()) {
    visibility_polygons->resize(1);
    if (!computeVisibility(polygon, &sweep->front(),
                           &visibility_polygons->front())) {
      return false;
    } else {
      sweep->back() = sweep->front();
      return true;
    }
  } else {
    visibility_polygons->resize(2);
    return computeVisibility(polygon, &sweep->front(),
                             &visibility_polygons->front()) &&
           computeVisibility(polygon, &sweep->back(),
                             &visibility_polygons->back());
  }
}

bool SweepPlanGraph::computeVisibility(const PolygonWithHoles& polygon,
                                       Point_2* vertex,
                                       Polygon_2* visibility_polygon) const {
  ROS_ASSERT(vertex);
  ROS_ASSERT(visibility_polygon);

  *vertex = pointInPolygon(polygon, *vertex)
                ? *vertex
                : projectPointOnHull(polygon, *vertex);
  return computeVisibilityPolygon(polygon, *vertex, visibility_polygon);
}

}  // namespace sweep_plan_graph
}  // namespace polygon_coverage_planning
