#include "mav_2d_coverage_planning/graphs/visibility_graph.h"

#include <glog/logging.h>

namespace mav_coverage_planning {
namespace visibility_graph {

VisibilityGraph::VisibilityGraph(const Polygon& polygon)
    : GraphBase(), polygon_(polygon) {
  // Build visibility graph.
  is_created_ = create();
}

bool VisibilityGraph::create() {
  clear();

  // Select shortest path vertices.
  std::vector<VertexConstCirculator> graph_vertices;
  if (!polygon_.appendConcaveOuterBoundaryVertices(&graph_vertices))
    return false;
  if (!polygon_.appendConvexHoleVertices(&graph_vertices)) return false;

  for (const VertexConstCirculator& v : graph_vertices) {
    // Compute visibility polygon.
    Polygon visibility;
    if (!polygon_.computeVisibilityPolygon(*v, &visibility)) {
      LOG(ERROR) << "Cannot compute visibility polygon.";
      return false;
    }
    if (!addNode(NodeProperty(*v, visibility))) {
      return false;
    }
  }

  DLOG(INFO) << "Created visibility graph with " << graph_.size()
             << " nodes and " << edge_properties_.size() << " edges.";

  return true;
}

bool VisibilityGraph::addEdges() {
  if (graph_.empty()) {
    LOG(ERROR) << "Cannot add edges to an empty graph.";
    return false;
  }

  const size_t new_id = graph_.size() - 1;
  for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
    const NodeProperty* new_node_property = getNodeProperty(new_id);
    const NodeProperty* adj_node_property = getNodeProperty(adj_id);
    if (adj_node_property == nullptr) {
      LOG(ERROR) << "Cannot access potential neighbor.";
      return false;
    }
    if (new_node_property->visibility.pointInPolygon(
            adj_node_property->coordinates)) {
      EdgeId forwards_edge_id(new_id, adj_id);
      EdgeId backwards_edge_id(adj_id, new_id);
      const double cost =
          computeCost(new_node_property->coordinates,
                      adj_node_property->coordinates);  // Symmetric cost.
      if (!addEdge(forwards_edge_id, EdgeProperty(), cost) ||
          !addEdge(backwards_edge_id, EdgeProperty(), cost)) {
        return false;
      }
    }
  }
  return true;
}

bool VisibilityGraph::solve(const Point_2& start, const Point_2& goal,
                            std::vector<Point_2>* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  // Make sure start and end are inside the polygon.
  const Point_2 start_new = polygon_.pointInPolygon(start)
                                ? start
                                : polygon_.projectPointOnHull(start);
  const Point_2 goal_new =
      polygon_.pointInPolygon(goal) ? goal : polygon_.projectPointOnHull(goal);

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

bool VisibilityGraph::solve(const Point_2& start,
                            const Polygon& start_visibility_polygon,
                            const Point_2& goal,
                            const Polygon& goal_visibility_polygon,
                            std::vector<Point_2>* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  if (!is_created_) {
    LOG(ERROR) << "Visibility graph not initialized.";
    return false;
  } else if (!polygon_.pointInPolygon(start) ||
             !polygon_.pointInPolygon(goal)) {
    LOG(ERROR) << "Start or goal is not in polygon.";
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
  if (start_node_property->visibility.pointInPolygon(goal)) {
    waypoints->push_back(start);
    waypoints->push_back(goal);
    return true;
  }

  // Find shortest way using A*.
  Solution solution;
  if (!temp_visibility_graph.solveAStar(start_idx, goal_idx, &solution)) {
    LOG(ERROR) << "Could not find shortest path. Graph not strongly connected.";
    return false;
  }

  // Reconstruct waypoints.
  return temp_visibility_graph.getWaypoints(solution, waypoints);
}

bool VisibilityGraph::getWaypoints(const Solution& solution,
                                   std::vector<Point_2>* waypoints) const {
  CHECK_NOTNULL(waypoints);
  waypoints->resize(solution.size());
  for (size_t i = 0; i < solution.size(); i++) {
    const NodeProperty* node_property = getNodeProperty(solution[i]);
    if (node_property == nullptr) {
      LOG(ERROR) << "Cannot reconstruct solution.";
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
    LOG(ERROR) << "Cannot find goal node property to calculate heuristic.";
    return false;
  }

  for (size_t adj_id = 0; adj_id < graph_.size(); ++adj_id) {
    const NodeProperty* adj_node_property = getNodeProperty(adj_id);
    if (adj_node_property == nullptr) {
      LOG(ERROR)
          << "Cannot access adjacent node property to calculate heuristic.";
      return false;
    }
    (*heuristic)[adj_id] = computeCost(adj_node_property->coordinates,
                                       goal_node_property->coordinates);
  }

  return true;
}

bool VisibilityGraph::solveWithOutsideStartAndGoal(
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* waypoints) const {
  CHECK_NOTNULL(waypoints);

  if (solve(start, goal, waypoints)) {
    if (!polygon_.pointInPolygon(start)) {
      waypoints->insert(waypoints->begin(), start);
    }
    if (!polygon_.pointInPolygon(goal)) {
      waypoints->push_back(goal);
    }
    return true;
  } else {
    return false;
  }
}

double VisibilityGraph::computeCost(const Point_2& from,
                                    const Point_2& to) const {
  return std::sqrt(CGAL::to_double(Segment_2(from, to).squared_length()));
}

}  // namespace visibility_graph
}  // namespace mav_coverage_planning
