#include "mav_coverage_planning/graph/visibility_graph.h"

#include <glog/logging.h>
#include <ros/ros.h>

#include "mav_coverage_planning/math.h"

namespace mav_coverage_planning {
namespace visibility_graph {

const std::string kPrefix = kOutputPrefix + "visibility_graph]: ";

VisibilityGraph::VisibilityGraph(const Polygon& polygon,
                                 const CostFunction& cost_function)
    : GraphBase(), polygon_(polygon), cost_function_(cost_function) {
  // Build visibility graph.
  is_created_ = polygon_.isValid() && polygon_.isSimple() &&
                (!polygon_.hasHoles() || polygon_.hasSimpleHoles()) && create();
}

VisibilityGraph::VisibilityGraph() : GraphBase() {}

bool VisibilityGraph::create() {
  clear();

  // Select vertices.
  StdVector2d graph_vertices;
  selectVertices(&graph_vertices);

  for (const Eigen::Vector2d& vertex : graph_vertices) {
    // Compute visibility polygon.
    Polygon visibility;
    if (!polygon_.computeVisibilityPolygon(vertex, &visibility)) {
      ROS_ERROR_STREAM(kPrefix << "Cannot compute visibility polygon.");
      return false;
    }
    if (!addNode(NodeProperty(vertex, visibility))) {
      return false;
    }
  }

  ROS_DEBUG_STREAM(kPrefix << "Created visibility graph with " << graph_.size()
                           << " nodes and " << edge_properties_.size()
                           << " edges.");
  is_created_ = true;
  return true;
}

bool VisibilityGraph::addEdges() {
  if (graph_.empty()) {
    ROS_ERROR_STREAM(kPrefix << "Cannot add edges to an empty graph.");
    return false;
  }

  const size_t new_id = graph_.size() - 1;
  for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
    const NodeProperty* new_node_property = getNodeProperty(new_id);
    const NodeProperty* adj_node_property = getNodeProperty(adj_id);
    if (adj_node_property == nullptr) {
      ROS_ERROR_STREAM(kPrefix << "Cannot access potential neighbor.");
      return false;
    }
    if (new_node_property->visibility.checkPointInPolygon(
            adj_node_property->coordinates)) {
      EdgeId forwards_edge_id(new_id, adj_id);
      EdgeId backwards_edge_id(adj_id, new_id);
      const double cost = cost_function_.computeCost(
          new_node_property->coordinates,
          adj_node_property->coordinates);  // Symmetric cost.
      if (!addEdge(forwards_edge_id, EdgeProperty(), cost) ||
          !addEdge(backwards_edge_id, EdgeProperty(), cost)) {
        return false;
      }
    }
  }
  return true;
}

bool VisibilityGraph::solve(const Eigen::Vector2d& start,
                            const Eigen::Vector2d& goal,
                            StdVector2d* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  // Make sure start and end are inside the polygon.
  const Eigen::Vector2d start_new = polygon_.checkPointInPolygon(start)
                                        ? start
                                        : polygon_.projectPointOnHull(start);
  const Eigen::Vector2d goal_new = polygon_.checkPointInPolygon(goal)
                                       ? goal
                                       : polygon_.projectPointOnHull(goal);

  // Compute start and goal visibility polygon.
  Polygon start_visibility, goal_visibility;
  if (!polygon_.computeVisibilityPolygon(start_new, &start_visibility) ||
      !polygon_.computeVisibilityPolygon(goal_new, &goal_visibility)) {
    return false;
  }

  // Find shortest path.
  return solve(start_new, start_visibility, goal_new, goal_visibility,
               waypoints);
}

bool VisibilityGraph::solve(const Eigen::Vector2d& start,
                            const Polygon& start_visibility_polygon,
                            const Eigen::Vector2d& goal,
                            const Polygon& goal_visibility_polygon,
                            StdVector2d* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  if (!is_created_) {
    ROS_ERROR_STREAM(kPrefix << "Visibility graph not initialized.");
    return false;
  } else if (!polygon_.checkPointInPolygon(start) ||
             !polygon_.checkPointInPolygon(goal)) {
    ROS_ERROR_STREAM(kPrefix << "Start or goal is not in polygon.");
    return false;
  }

  VisibilityGraph temp_visibility_graph = *this;
  // Add start and goal node.
  if (!temp_visibility_graph.addStartNode(
          NodeProperty(start, start_visibility_polygon)) ||
      !temp_visibility_graph.addGoalNode(
          NodeProperty(goal, goal_visibility_polygon))) {
    return false;
  }

  // Check if start and goal are in line of sight.
  size_t start_idx = temp_visibility_graph.getStartIdx();
  size_t goal_idx = temp_visibility_graph.getGoalIdx();
  const NodeProperty* start_node_property =
      temp_visibility_graph.getNodeProperty(start_idx);
  if (start_node_property == nullptr) {
    return false;
  }
  if (start_node_property->visibility.checkPointInPolygon(goal)) {
    waypoints->push_back(start);
    waypoints->push_back(goal);
    return true;
  }

  // Find shortest way using A*.
  Solution solution;
  if (!temp_visibility_graph.solveAStar(start_idx, goal_idx, &solution)) {
    ROS_ERROR_STREAM(
        kPrefix
        << "Could not find shortest path. Graph not strongly connected.");
    return false;
  }

  // Reconstruct waypoints.
  return temp_visibility_graph.getWaypoints(solution, waypoints);
}

bool VisibilityGraph::getWaypoints(const Solution& solution,
                                   StdVector2d* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->resize(solution.size());
  for (size_t i = 0; i < solution.size(); i++) {
    const NodeProperty* node_property = getNodeProperty(solution[i]);
    if (node_property == nullptr) {
      ROS_ERROR_STREAM(kPrefix << "Cannot reconstruct solution.");
      return false;
    }
    (*waypoints)[i] = node_property->coordinates;
  }
  return true;
}

bool VisibilityGraph::calculateHeuristic(size_t goal,
                                         Heuristic* heuristic) const {
  CHECK_NOTNULL(heuristic);
  heuristic->clear();

  const NodeProperty* goal_node_property = getNodeProperty(goal);
  if (goal_node_property == nullptr) {
    ROS_ERROR_STREAM(
        kPrefix << "Cannot find goal node property to calculate heuristic.");
    return false;
  }

  for (size_t adj_id = 0; adj_id < graph_.size(); ++adj_id) {
    const NodeProperty* adj_node_property = getNodeProperty(adj_id);
    if (adj_node_property == nullptr) {
      ROS_ERROR_STREAM(
          kPrefix
          << "Cannot access adjacent node property to calculate heuristic.");
      return false;
    }
    (*heuristic)[adj_id] = cost_function_.computeCost(
        adj_node_property->coordinates, goal_node_property->coordinates);
  }

  return true;
}

bool VisibilityGraph::solveWithOutsideStartAndGoal(
    const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
    StdVector2d* waypoints) const {
  CHECK_NOTNULL(waypoints);

  if (solve(start, goal, waypoints)) {
    if (!polygon_.checkPointInPolygon(start)) {
      waypoints->insert(waypoints->begin(), start);
    }
    if (!polygon_.checkPointInPolygon(goal)) {
      waypoints->push_back(goal);
    }
    return true;
  } else {
    return false;
  }
}

void VisibilityGraph::selectVertices(StdVector2d* graph_vertices) const {
  CHECK_NOTNULL(graph_vertices);
  graph_vertices->clear();

  // Get all concave polygon vertices.
  if (!polygon_.isConvex()) {
    for (size_t i = 0; i < polygon_.getNumVertices(); i++) {
      if (!checkVertexConvexity(polygon_.getVertices(), i,
                                polygon_.isClockwise())) {
        graph_vertices->push_back(polygon_.getVertices()[i]);
      }
    }
  }

  // Get all convex hole vertices.
  for (const Polygon& hole : polygon_.getHoles()) {
    for (size_t i = 0; i < hole.getNumVertices(); i++) {
      if (checkVertexConvexity(hole.getVertices(), i, hole.isClockwise())) {
        graph_vertices->push_back(hole.getVertices()[i]);
      }
    }
  }
}

}  // namespace visibility_graph
}  // namespace mav_coverage_planning
