#include "polygon_coverage_geometry/visibility_graph.h"

#include <ros/assert.h>
#include <ros/console.h>

namespace polygon_coverage_planning {
namespace visibility_graph {

VisibilityGraph::VisibilityGraph(const PolygonWithHoles& polygon)
    : GraphBase(), polygon_(polygon) {
  // Build visibility graph.
  is_created_ = create();
}

bool VisibilityGraph::create() {
  clear();
  // Select shortest path vertices.
  std::vector<VertexConstCirculator> graph_vertices;
  polygon_.findConcaveOuterBoundaryVertices(&graph_vertices);
  polygon_.findConvexHoleVertices(&graph_vertices);

  for (const VertexConstCirculator& v : graph_vertices) {
    // Compute visibility polygon.
    Polygon_2 visibility;
    if (!polygon_.computeVisibilityPolygon(*v, &visibility)) {
      ROS_ERROR_STREAM("Cannot compute visibility polygon.");
      return false;
    }
    if (!addNode(NodeProperty(*v, visibility))) {
      return false;
    }
  }

  ROS_DEBUG_STREAM("Created visibility graph with "
                   << graph_.size() << " nodes and " << edge_properties_.size()
                   << " edges.");

  return true;
}

void VisibilityGraph::findConcaveOuterBoundaryVertices(
    std::vector<VertexConstCirculator>* concave_vertices) const {
  ROS_ASSERT(concave_vertices);
  ROS_ASSERT(polygon_.outer_boundary().is_simple());
  ROS_ASSERT(polygon_.outer_boundary().is_counterclockwise_oriented());

  VertexConstCirculator vit = polygon_.outer_boundary().vertices_circulator();
  do {
    Triangle_2 triangle(*std::prev(vit), *vit, *std::next(vit));
    CGAL::Orientation orientation = triangle.orientation();
    ROS_ASSERT(orientation != CGAL::COLLINEAR);
    if (orientation == CGAL::CLOCKWISE) concave_vertices->push_back(vit);
  } while (++vit != polygon_.outer_boundary().vertices_circulator());
}

void VisibilityGraph::findConvexHoleVertices(
    std::vector<VertexConstCirculator>* convex_vertices) const {
  ROS_ASSERT(convex_vertices);

  for (PolygonWithHoles::Hole_const_iterator hit = polygon_.holes_begin();
       hit != polygon_.holes_end(); ++hit) {
    ROS_ASSERT(hit->is_simple());
    ROS_ASSERT(hit->is_clockwise_oriented());
    VertexConstCirculator vit = hit->vertices_circulator();
    do {
      Triangle_2 triangle(*std::prev(vit), *vit, *std::next(vit));
      CGAL::Orientation orientation = triangle.orientation();
      ROS_ASSERT(orientation != CGAL::COLLINEAR);
      if (orientation == CGAL::CLOCKWISE) convex_vertices->push_back(vit);
    } while (++vit != hit->vertices_circulator());
  }
}

bool VisibilityGraph::addEdges() {
  if (graph_.empty()) {
    ROS_ERROR_STREAM("Cannot add edges to an empty graph.");
    return false;
  }

  const size_t new_id = graph_.size() - 1;
  for (size_t adj_id = 0; adj_id < new_id; ++adj_id) {
    const NodeProperty* new_node_property = getNodeProperty(new_id);
    const NodeProperty* adj_node_property = getNodeProperty(adj_id);
    if (adj_node_property == nullptr) {
      ROS_ERROR_STREAM("Cannot access potential neighbor.");
      return false;
    }
    if (new_node_property->visibility.pointInPolygon(
            adj_node_property->coordinates)) {
      EdgeId forwards_edge_id(new_id, adj_id);
      EdgeId backwards_edge_id(adj_id, new_id);
      const double cost = computeEuclideanSegmentCost(
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
    ROS_ERROR_STREAM("Visibility graph not initialized.");
    return false;
  } else if (!polygon_.pointInPolygon(start) ||
             !polygon_.pointInPolygon(goal)) {
    ROS_ERROR_STREAM("Start or goal is not in polygon.");
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
    ROS_ERROR_STREAM(
        "Could not find shortest path. Graph not fully connected.");
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
      ROS_ERROR_STREAM("Cannot reconstruct solution.");
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
    ROS_ERROR_STREAM("Cannot find goal node property to calculate heuristic.");
    return false;
  }

  for (size_t adj_id = 0; adj_id < graph_.size(); ++adj_id) {
    const NodeProperty* adj_node_property = getNodeProperty(adj_id);
    if (adj_node_property == nullptr) {
      ROS_ERROR_STREAM(
          "Cannot access adjacent node property to calculate heuristic.");
      return false;
    }
    (*heuristic)[adj_id] = computeEuclideanSegmentCost(
        adj_node_property->coordinates, goal_node_property->coordinates);
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

}  // namespace visibility_graph
}  // namespace polygon_coverage_planning
