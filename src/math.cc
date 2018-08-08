// Copyright 2000 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

#include "mav_coverage_planning/math.h"

#include <cmath>

#include <glog/logging.h>
#include <ros/ros.h>

namespace mav_coverage_planning {

const std::string kPrefix = kOutputPrefix + "math]: ";

const double kRayOffset = 1.0;

bool isPolygonValid(const StdVector2d& vertices) {
  if (vertices.size() < 3) {
    return false;
  }
  return checkConvexity(vertices);
}

bool checkVertexConvexity(const StdVector2d& vertices, int vertex,
                          bool is_clockwise /*= true*/) {
  // http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex
  int i = vertex % vertices.size();
  Eigen::Vector2d current = vertices[i];
  Eigen::Vector2d next = vertices[(i + 1) % vertices.size()];
  Eigen::Vector2d previous = vertices[i == 0 ? vertices.size() - 1 : i - 1];

  Eigen::Vector2d v = current - previous;
  Eigen::Vector2d w = next - current;
  double turning_direction = cross(v, w);
  if ((is_clockwise && turning_direction >= 0.0) ||
      (!is_clockwise && turning_direction <= 0.0)) {
    return false;
  } else {
    return true;
  }
}

bool checkConvexity(const StdVector2d& vertices, bool is_clockwise /*= true*/) {
  // http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex
  for (size_t i = 1; i < vertices.size(); i++) {
    if (!checkVertexConvexity(vertices, i, is_clockwise)) {
      return false;
    }
  }
  return true;
}

bool checkClockwise(const StdVector2d& vertices) {
  // Shoe lace formula: http://blog.element84.com/polygon-winding.html
  double sum = 0.0;
  for (size_t i = 0; i < vertices.size(); i++) {
    Eigen::Vector2d v1 = vertices[i];
    Eigen::Vector2d v2 = vertices[(i + 1) % vertices.size()];
    sum += (v2.x() - v1.x()) * (v2.y() + v1.y());
  }
  return sum > 0.0;
}

Eigen::Vector2d computeRightFacingNormal(const Eigen::Vector2d& edge) {
  Eigen::Vector2d normal;
  normal.x() = edge.y();
  normal.y() = -edge.x();
  normal.normalize();
  return normal;
}

Eigen::Vector2d computeLeftFacingNormal(const Eigen::Vector2d& edge) {
  Eigen::Vector2d normal = computeRightFacingNormal(edge);
  return -1.0 * normal;
}

void computeInwardFacingNormals(const StdVector2d& vertices,
                                StdVector2d* normals,
                                bool is_clockwise /*= true*/) {
  CHECK_NOTNULL(normals);
  // Compute the inward facing normals of a polygon.
  normals->resize(vertices.size());
  StdVector2d edges;
  computePolygonEdges(vertices, &edges);
  for (size_t i = 0; i < normals->size(); i++) {
    if (is_clockwise) {
      (*normals)[i] = computeRightFacingNormal(edges[i]);
    } else {
      (*normals)[i] = computeLeftFacingNormal(edges[i]);
    }
  }
}

void computePolygonEdges(const StdVector2d& vertices, StdVector2d* edges) {
  CHECK_NOTNULL(edges);
  edges->resize(vertices.size());
  for (size_t i = 0; i < edges->size(); i++) {
    size_t from_vertex = i;
    size_t to_vertex = (i + 1) % vertices.size();
    (*edges)[i] = vertices[to_vertex] - vertices[from_vertex];
  }
}

void computePolygonCentroid(const StdVector2d& vertices,
                            Eigen::Vector2d* centroid) {
  CHECK_NOTNULL(centroid);
  centroid->setZero();
  for (size_t i = 0; i < vertices.size(); i++) {
    *centroid += vertices[i];
  }
  *centroid *= 1.0 / double(vertices.size());
}

bool shrinkPolygonBinarySearch(const StdVector2d& original_vertices,
                               double distance, StdVector2d* new_vertices) {
  CHECK_NOTNULL(new_vertices);
  new_vertices->clear();

  double min = 0.0;
  double max = distance;

  bool search = true;
  while (search) {
    const double mid = (min + max) / 2.0;  // Mid distance.
    if (shrinkPolygon(original_vertices, max, new_vertices)) {
      return true;  // Found max shrinking distance.
    } else if (shrinkPolygon(original_vertices, mid, new_vertices)) {
      search = max - min > kBinarySearchResolution;
      if (!search) {
        return true;
      }
      min = mid;  // Repeat search in top half.
    } else {
      max = mid;  // Repeat search in bottom half.
    }
  }

  *new_vertices = original_vertices;
  return false;
}

bool shrinkPolygon(const StdVector2d& original_vertices, double distance,
                   StdVector2d* new_vertices) {
  CHECK_NOTNULL(new_vertices);
  Eigen::Vector2d centroid;
  computePolygonCentroid(original_vertices, &centroid);
  *new_vertices = original_vertices;
  double scale = 0.0;
  for (size_t i = 0; i < new_vertices->size(); i++) {
    // Shift vertices to have centroid in origin.
    (*new_vertices)[i] -= centroid;
    // Calculate the maximum scaling factor in [0.0, 1.0].
    const double new_vertex_distance =
        (*new_vertices)[i].norm() - std::abs(distance);
    if (new_vertex_distance < 0.0) {
      // Can not make the polygon this small.
      *new_vertices = original_vertices;
      return false;
    }
    scale = std::max(new_vertex_distance / (*new_vertices)[i].norm(), scale);
  }
  // Scale and shift back all vertices.
  for (size_t i = 0; i < new_vertices->size(); i++) {
    (*new_vertices)[i] *= scale;
    (*new_vertices)[i] += centroid;
  }
  return true;
}

void getProjectedVertexDistances(const StdVector2d& vertices,
                                 const Eigen::Vector2d& vertex,
                                 const Eigen::Vector2d& direction,
                                 std::vector<double>* distances) {
  CHECK_NOTNULL(distances);
  distances->clear();
  distances->resize(vertices.size());

  for (size_t i = 0; i < vertices.size(); i++) {
    const double projected_distance = (vertices[i] - vertex).dot(direction);
    (*distances)[i] = projected_distance;
  }
}

bool computeRayLineIntersection(const Eigen::Vector2d& ray_origin,
                                const Eigen::Vector2d& ray_direction,
                                const Eigen::Vector2d& line_start,
                                const Eigen::Vector2d& line_end, double* t_1,
                                double* t_2) {
  CHECK_NOTNULL(t_1);
  CHECK_NOTNULL(t_2);
  CHECK_GT(ray_direction.norm(), 0.0);
  // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  // x_1(t_1) = ray_origin + ray_direction * t_1, t_1 \in [0, inf[
  // x_2(t_2) = line_start + (line_end - line_start) * t_2, t_2 \in [0, 1]
  *t_1 = -1.0;
  *t_2 = -1.0;
  Eigen::Vector2d line_direction = (line_end - line_start);
  double denom = cross(ray_direction, line_direction);
  // Check parallel.
  if (std::abs(denom) < std::numeric_limits<double>::epsilon()) {
    // Check collinear.
    if (std::abs(cross(line_start - ray_origin, ray_direction)) <
        std::numeric_limits<double>::epsilon()) {
      // Check the ray intersecting the start or end point of the line.
      double t_1_start = (line_start - ray_origin).dot(ray_direction) /
                         ray_direction.dot(ray_direction);
      bool ray_hits_start_point = (t_1_start >= 0.0);
      double t_1_end = t_1_start + line_direction.dot(ray_direction) /
                                       ray_direction.dot(ray_direction);
      bool ray_hits_end_point = (t_1_end >= 0.0);
      // Ray must hit at least one of the points.
      if (ray_hits_start_point || ray_hits_end_point) {
        // If the ray hits both points, then we determine which point it hits
        // first.
        if (ray_hits_start_point && ray_hits_end_point) {
          if (t_1_start < t_1_end) {
            *t_1 = t_1_start;
            *t_2 = 0.0;
          } else {
            *t_1 = t_1_end;
            *t_2 = 1.0;
          }
          return true;
        } else {
          // The ray hits only one point. It must start inside the line.
          *t_1 = 0.0;
          double line_length = (line_end - line_start).norm();
          if (line_length == 0.0) {
            *t_2 = 0.0;
          } else {
            *t_2 = (ray_origin - line_start).norm() / line_length;
          }
          return true;
        }
      } else {
        // The ray starts outside the line and is opened away from it.
        return false;
      }
    } else {
      // Parallel non-intersecting
      return false;
    }
  }
  // Compute the intersection of two infinite lines:
  *t_1 = cross((line_start - ray_origin), line_direction) / denom;
  *t_2 = cross((line_start - ray_origin), ray_direction) / denom;
  if (*t_1 < 0.0) {
    // The ray faces away from the line.
    return false;
  } else if (*t_2 < 0.0 || *t_2 > 1.0) {
    // The ray opens towards the line, but the line is too short.
    return false;
  } else {
    // The ray cuts the line.
    return true;
  }
}

void computeDistancePointToLine(const Eigen::Vector2d& point,
                                const Eigen::Vector2d& line_start,
                                const Eigen::Vector2d& line_end,
                                double* distance) {
  CHECK_NOTNULL(distance);
  Eigen::Vector2d edge = line_end - line_start;
  Eigen::Vector2d normal = Eigen::Vector2d(edge.y(), -edge.x());
  normal.normalize();
  CHECK_GT(normal.norm(), 0.0);
  computeRayLineIntersection(point, normal, line_start, line_end, distance);
  *distance = std::abs(*distance);
}

Eigen::Vector2d projectPointOnPolygon(const StdVector2d& vertices,
                                      const Eigen::Vector2d& point) {
  Eigen::Vector2d projection = point;

  // For all edges find the smallest projected distance.
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < vertices.size(); i++) {
    const Eigen::Vector2d edge_start = vertices[i];
    const Eigen::Vector2d edge_end = vertices[(i + 1) % vertices.size()];
    Eigen::Vector2d normal = computeRightFacingNormal(edge_end - edge_start);
    // Make normal face segment.
    if ((edge_start - point).dot(normal) < 0.0) {
      normal = -1.0 * normal;
    }
    double t_1, t_2;
    Eigen::Vector2d temp_proj;
    if (computeRayLineIntersection(point, normal, edge_start, edge_end, &t_1,
                                   &t_2)) {
      temp_proj = point + t_1 * normal;
    } else {
      // Shortest projection on edge is start or end.
      temp_proj = t_2 < 0.0 ? edge_start : edge_end;
    }
    const double distance = (point - temp_proj).norm();
    if (distance < min_distance) {
      projection = temp_proj;
      min_distance = distance;
    }
  }
  return projection;
}

bool castPerpendicularShadowOnVertices(const StdVector2d& vertices,
                                       const Eigen::Vector2d& line_start,
                                       const Eigen::Vector2d& line_end,
                                       const Eigen::Vector2d& direction,
                                       StdVector2d* shadowed_vertices) {
  CHECK_NOTNULL(shadowed_vertices);
  shadowed_vertices->clear();
  shadowed_vertices->reserve(vertices.size());

  if (!pointOnPolygonHull(line_start, vertices) ||
      !pointOnPolygonHull(line_end, vertices)) {
    ROS_ERROR_STREAM(kPrefix
                     << "Shadow casting only implemented for line on polygon.");
    return false;
  }

  if (line_start == line_end) {
    ROS_ERROR_STREAM(
        kPrefix << "Shadow casting only implemented for segments, not points.");
    return false;
  }

  // Find out perpendicular ray direction.
  const double is_left = isLeft(line_start + direction, line_start, line_end);
  Eigen::Vector2d ray_direction;
  if (is_left > kRayCastPrecision) {
    ray_direction = computeLeftFacingNormal(line_end - line_start);
  } else if (is_left < kRayCastPrecision) {
    ray_direction = computeRightFacingNormal(line_end - line_start);
  } else {
    ROS_ERROR_STREAM(kPrefix << "Shadow direction parallel to segment.");
    return false;
  }

  // Find all vertices that are shadowed by the segment.
  std::vector<size_t> shadowed_v_idx;
  for (size_t i = 0; i < vertices.size(); ++i) {
    // Add vertices that are
    // - within line length
    // - inside half space
    // - not in the vector already.
    if ((line_end - line_start).dot(vertices[i] - line_start) >= 0.0 &&
        (line_start - line_end).dot(vertices[i] - line_end) >= 0.0 &&
        pointInHalfSpace(vertices[i], line_start, ray_direction)) {
      shadowed_v_idx.push_back(i);
    }
  }

  // Create copys and determine whether they should be sorted increasing or
  // decreasing:
  std::vector<size_t> inc_idx, dec_idx;
  inc_idx = dec_idx = shadowed_v_idx;
  std::sort(inc_idx.begin(), inc_idx.end());
  std::sort(dec_idx.begin(), dec_idx.end(),
            [](size_t i, size_t j) { return i > j; });

  StdVector2d inc_v, dec_v;
  inc_v.reserve(inc_idx.size());
  dec_v.reserve(dec_idx.size());
  inc_v = dec_v = {line_start};
  for (size_t i = 0; i < inc_idx.size(); ++i) {
    if (vertices[inc_idx[i]] != line_start) {
      inc_v.push_back(vertices[inc_idx[i]]);
    }
    if (vertices[dec_idx[i]] != line_start) {
      dec_v.push_back(vertices[dec_idx[i]]);
    }
  }

  // Append goal if not in already.
  if (std::find(inc_v.begin(), inc_v.end(), line_end) == inc_v.end()) {
    inc_v.push_back(line_end);
  }
  if (std::find(dec_v.begin(), dec_v.end(), line_end) == dec_v.end()) {
    dec_v.push_back(line_end);
  }

  // The shorter path is the correct turning direction.
  const double inc_v_l = computeVertexChainLength(inc_v);
  const double dec_v_l = computeVertexChainLength(dec_v);
  if (inc_v_l < dec_v_l) {
    *shadowed_vertices = inc_v;
    return true;
  } else if (dec_v_l < inc_v_l) {
    *shadowed_vertices = dec_v;
    return true;
  } else if (dec_v.size() == inc_v.size() &&
             (inc_v.size() == 2 || inc_v.size() == 3)) {
    *shadowed_vertices = dec_v;
    return true;
  } else {
    ROS_ERROR_STREAM(kPrefix << "Bug in casting shadows.");
    return false;
  }
}

bool pointInHalfSpace(const Eigen::Vector2d& point,
                      const Eigen::Vector2d& point_on_line,
                      const Eigen::Vector2d& normal) {
  CHECK_GT(normal.norm(), 0.0);
  // The sign of the dot product between any line connecting the edge and the
  // point and the inward facing normal of this edge must be positive.
  double dot_product = (point - point_on_line).dot(normal.transpose());
  if (dot_product < -kRayCastPrecision) {
    return false;
  } else {
    return true;
  }
}

bool pointInConvexPolygon(const Eigen::Vector2d& point,
                          const StdVector2d& vertices,
                          const StdVector2d& normals) {
  CHECK_EQ(vertices.size(), normals.size());
  // Test for all edges if the point is in its half plane.
  bool point_in_half_space = true;
  for (size_t i = 0; i < normals.size(); i++) {
    point_in_half_space = pointInHalfSpace(point, vertices[i], normals[i]);
    if (!point_in_half_space) {
      return false;
    }
  }
  return true;
}

bool createRandomConvexPolygon(double x0, double y0, double r,
                               StdVector2d* convex_polygon) {
  CHECK_NOTNULL(convex_polygon);
  // http://stackoverflow.com/questions/21690008/how-to-generate-random-vertices-to-form-a-convex-polygon-in-c
  convex_polygon->clear();
  double a, x, y;
  for (a = 0.0; a > -2.0 * M_PI;)  // full circle
  {
    x = x0 + (r * cos(a));
    y = y0 + (r * sin(a));
    // random angle step [20 .. 169] degrees
    a -= (20.0 + double((std::rand() % 150))) * M_PI / 180.0;

    convex_polygon->push_back(Eigen::Vector2d(x, y));
  }
  return correctVertices(convex_polygon);
}

bool correctVertices(StdVector2d* vertices) {
  CHECK_NOTNULL(vertices);
  // Delete identical adjacent vertices.
  StdVector2d::iterator it = std::adjacent_find(
      vertices->begin(), vertices->end(), checkVerticesIdentical);
  while (it != vertices->end()) {
    vertices->erase(it);
    it = std::adjacent_find(vertices->begin(), vertices->end(),
                            checkVerticesIdentical);
  }
  // Check first and last:
  if (checkVerticesIdentical(vertices->front(), vertices->back())) {
    vertices->pop_back();
  }
  return vertices->size() > 2;
}

bool checkPointOnVertex(const StdVector2d& vertices,
                        const Eigen::Vector2d& point, int* idx) {
  CHECK_NOTNULL(idx);

  for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
    if (checkVerticesIdentical(vertices[i], point)) {
      *idx = i;
      return true;
    }
  }
  return false;
}

void removeRedundantVertices(StdVector2d* vertices) {
  CHECK_NOTNULL(vertices);

  // Find redundant vertices.
  std::vector<size_t> elements_to_erase;
  for (size_t i = 0; i < vertices->size(); ++i) {
    const size_t start = i;
    const size_t middle = (i + 1) % vertices->size();
    const size_t end = (i + 2) % vertices->size();
    if (pointOnSegment((*vertices)[middle], (*vertices)[start],
                       (*vertices)[end])) {
      elements_to_erase.push_back(middle);
    }
  }

  // Sort in decreasing order.
  std::sort(elements_to_erase.begin(), elements_to_erase.end(),
            [](size_t i, size_t j) { return i > j; });
  for (const size_t& i : elements_to_erase) {
    vertices->erase(std::next(vertices->begin(), i));
  }
}

void polygonWithHolesToSimplePolygon(
    const StdVector2d& cw_polygon_boundary,
    const std::vector<StdVector2d>& cc_polygon_holes,
    StdVector2d* simple_polygon) {
  // Loop over all holes and connect one corner to the closest visible vertex.
  // TODO(rikba): Connect to the closest visible edge of the boundary.
  // https://cs.stackexchange.com/questions/43758/turning-a-simple-polygon-with-holes-into-exterior-bounded-only
  // Assumptions: - holes lie in polygon
  CHECK_NOTNULL(simple_polygon);
  *simple_polygon = cw_polygon_boundary;

  for (const StdVector2d& hole : cc_polygon_holes) {
    std::pair<size_t, size_t> split_pair;
    double split_distance = std::numeric_limits<double>::max();
    for (size_t hole_vertex_idx = 0; hole_vertex_idx < hole.size();
         ++hole_vertex_idx) {
      for (size_t polygon_vertex_idx = 0;
           polygon_vertex_idx < simple_polygon->size(); ++polygon_vertex_idx) {
        // Check boundary vertex is visible.
        if (checkLinePolygonEdgesIntersection(
                hole[hole_vertex_idx], (*simple_polygon)[polygon_vertex_idx],
                *simple_polygon)) {
          continue;  // Hole-vertex connection intercepts polygon edge.
        }
        // TODO(rikba): Already connected holes do not need to be checked.
        if (checkLinePolygonEdgesVectorIntersection(
                hole[hole_vertex_idx], (*simple_polygon)[polygon_vertex_idx],
                cc_polygon_holes)) {
          continue;  // Hole-vertex connection intercepts hole.
        }
        double distance =
            (hole[hole_vertex_idx] - (*simple_polygon)[polygon_vertex_idx])
                .norm();
        if (distance < split_distance) {
          split_pair.first = polygon_vertex_idx;
          split_pair.second = hole_vertex_idx;
          split_distance = distance;
        }
      }
    }
    // Connect the polygon to the hole.
    Eigen::Vector2d split_vertex = (*simple_polygon)[split_pair.first];
    size_t insert_idx = (split_pair.first + 1) % simple_polygon->size();
    StdVector2d::iterator insert_it =
        std::next(simple_polygon->begin(), insert_idx);
    size_t hole_vertex_idx = split_pair.second;
    for (size_t i = 0; i < hole.size(); i++) {
      insert_it = simple_polygon->insert(insert_it, hole[hole_vertex_idx]);
      insert_it = std::next(insert_it);
      hole_vertex_idx = (hole_vertex_idx + 1) % hole.size();
    }
    // Connect the hole to the polygon leaving a tiny gap.
    // TODO(rikba): Implement decomposition such that this strict simple polygon
    // assumtion is not necessary anymore.
    Eigen::Vector2d gap =
        computeRightFacingNormal(split_vertex - hole[split_pair.second]);
    gap *= kSimplePolyOffset;
    insert_it =
        simple_polygon->insert(insert_it, hole[split_pair.second] + gap);
    insert_it = std::next(insert_it);
    simple_polygon->insert(insert_it, split_vertex + gap);
  }
}

bool checkLineLineIntersection(const Eigen::Vector2d& line_1_start,
                               const Eigen::Vector2d& line_1_end,
                               const Eigen::Vector2d& line_2_start,
                               const Eigen::Vector2d& line_2_end, double* t_1,
                               double* t_2) {
  CHECK_NOTNULL(t_1);
  CHECK_NOTNULL(t_2);
  *t_1 = *t_2 = 0.0;
  Eigen::Vector2d line_1_direction = (line_1_end - line_1_start);
  if (!computeRayLineIntersection(line_1_start, line_1_direction, line_2_start,
                                  line_2_end, t_1, t_2)) {
    return false;
  }
  if (*t_1 > 1.0) {
    return false;  // Line 1 too short to intersect line 2.
  } else {
    return true;
  }
}

bool checkLinePolygonEdgesIntersection(const Eigen::Vector2d& line_start,
                                       const Eigen::Vector2d& line_end,
                                       const StdVector2d& vertices) {
  Eigen::Vector2d line = (line_end - line_start);
  for (size_t i = 0; i < vertices.size(); i++) {
    double t_1, t_2;
    Eigen::Vector2d edge_start = vertices[i];
    Eigen::Vector2d edge_end = vertices[(i + 1) % vertices.size()];
    Eigen::Vector2d edge = edge_end - edge_start;
    if (!checkLineLineIntersection(line_start, line_end, edge_start, edge_end,
                                   &t_1, &t_2)) {
      continue;  // No intersection.
    } else if (checkLinesParallel(line, edge)) {
      continue;  // Line parallel to polygon edge.
    } else if (t_1 != 0.0 && t_1 != 1.0 && t_2 != 0.0 && t_2 != 1.0) {
      return true;  // Line crosses polygon edge.
    }
  }
  return false;
}

bool checkLinesParallel(const Eigen::Vector2d& line_1_direction,
                        const Eigen::Vector2d& line_2_direction) {
  return std::abs(std::abs(line_1_direction.normalized().dot(
                      line_2_direction.normalized())) -
                  1.0) < kRayCastPrecision;
}

bool checkLineParallelToAnyEdge(const Eigen::Vector2d& line_start,
                                const Eigen::Vector2d& line_end,
                                const StdVector2d& edges) {
  Eigen::Vector2d line = line_end - line_start;
  for (const Eigen::Vector2d& edge : edges) {
    if (checkLinesParallel(line, edge)) {
      return true;
    }
  }
  return false;
}

double isLeft(const Eigen::Vector2d& point, const Eigen::Vector2d& line_start,
              const Eigen::Vector2d& line_end) {
  // http://geomalgorithms.com/a01-_area.html
  return (line_end.x() - line_start.x()) * (point.y() - line_start.y()) -
         (point.x() - line_start.x()) * (line_end.y() - line_start.y());
}

int windingNumberTest(const Eigen::Vector2d& point,
                      const StdVector2d& vertices) {
  // Following http://geomalgorithms.com/a03-_inclusion.html#Winding-Number
  int wn = 0;  // The winding number counter.

  // Loop through all edges of the polygon.
  for (size_t i = 0; i < vertices.size(); i++) {
    size_t start = i;
    size_t end = (i + 1) % vertices.size();
    if (vertices[start].y() <= point.y()) {
      if (vertices[end].y() > point.y()) {
        if (isLeft(point, vertices[start], vertices[end]) > 0.0) {
          ++wn;  // Valid up intersect.
        }
      }
    } else {
      if (vertices[end].y() <= point.y()) {
        if (isLeft(point, vertices[start], vertices[end]) < 0.0) {
          --wn;  // Valid down intersect.
        }
      }
    }
  }
  return wn;
}

bool pointInPolygonWindingNumber(const Eigen::Vector2d& point,
                                 const StdVector2d& vertices) {
  // Hack: The winding number test above has a bug, that it may count points on
  // an edge outside of the polygon. Thus we additionally check for the point
  // begin on an edge.
  if (windingNumberTest(point, vertices) == 0 &&
      !pointOnPolygonHull(point, vertices)) {
    return false;
  }
  return true;
}

double distancePointToSegment(const Eigen::Vector2d& point,
                              const Eigen::Vector2d& segment_start,
                              const Eigen::Vector2d& segment_end) {
  // http://geomalgorithms.com/a02-_lines.html#Distance-to-Ray-or-Segment
  Eigen::Vector2d v = segment_end - segment_start;
  Eigen::Vector2d w = point - segment_start;

  double c_1 = w.dot(v);
  if (c_1 <= 0) {
    return (point - segment_start).norm();  // Point "before" segment.
  }

  double c_2 = v.dot(v);
  if (c_2 <= c_1) {
    return (point - segment_end).norm();  // Point "behind" segment.
  }

  double b = c_1 / c_2;
  // Perpendicular intersection point on segment.
  Eigen::Vector2d point_b = segment_start + b * v;
  return (point - point_b).norm();  // Point "perpendicular" to segment.
}

bool pointOnSegment(const Eigen::Vector2d& point,
                    const Eigen::Vector2d& segment_start,
                    const Eigen::Vector2d& segment_end) {
  if (distancePointToSegment(point, segment_start, segment_end) <=
      kRayCastPrecision) {
    return true;
  }
  return false;
}

bool pointOnPolygonHull(const Eigen::Vector2d& point,
                        const StdVector2d& vertices, int* source_idx,
                        int* target_idx) {
  for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
    *source_idx = i;
    *target_idx = (i + 1) % vertices.size();
    if (pointOnSegment(point, vertices[*source_idx], vertices[*target_idx])) {
      return true;
    }
  }
  return false;
}

bool pointOnPolygonHull(const Eigen::Vector2d& point,
                        const StdVector2d& vertices) {
  int s, t;
  return pointOnPolygonHull(point, vertices, &s, &t);
}

double computeVertexChainLength(const StdVector2d& vertices) {
  double distance = 0.0;
  for (size_t i = 0; i < vertices.size() - 1; i++) {
    distance += (vertices[i + 1] - vertices[i]).norm();
  }
  return distance;
}

double computeHullLength(const StdVector2d& vertices) {
  double distance = computeVertexChainLength(vertices);
  distance += (vertices.back() - vertices.front()).norm();
  return distance;
}

bool createLineSweepPlan(const StdVector2d& vertices, const StdVector2d& edges,
                         const StdVector2d& inward_facing_normals,
                         double max_sweep_distance, size_t start_vertex_idx,
                         StdVector2d* waypoints) {
  CHECK_NOTNULL(waypoints);
  waypoints->clear();

  // Create zig zag waypoints.
  const size_t start_vertex_idx_mod = start_vertex_idx % vertices.size();
  // Initial waypoint.
  waypoints->push_back(vertices[start_vertex_idx_mod]);
  waypoints->push_back(vertices[(start_vertex_idx_mod + 1) % vertices.size()]);

  // The vertical direction from one to the next horizontal sweep line.
  const Eigen::Vector2d sweep_normal =
      inward_facing_normals[start_vertex_idx_mod];

  // Calculate a smaller sweep distance to have equally spaced sweeps.
  std::vector<double> vertex_distances_sweep_normal;
  getProjectedVertexDistances(vertices, waypoints->front(), sweep_normal,
                              &vertex_distances_sweep_normal);
  const double poly_length_sweep_normal_dir = std::abs(
      *std::max_element(vertex_distances_sweep_normal.begin(),
                        vertex_distances_sweep_normal.end(), absCompare));
  const int num_sweeps =
      static_cast<int>(
          std::ceil(poly_length_sweep_normal_dir / max_sweep_distance)) +
      1;
  const double sweep_distance = poly_length_sweep_normal_dir / (num_sweeps - 1);

  // Calculate the outer-most point in opposite sweep direction.
  Eigen::Vector2d sweep_direction = edges[start_vertex_idx_mod];
  sweep_direction.normalize();
  std::vector<double> vertex_distances_sweep_direction;
  getProjectedVertexDistances(vertices, waypoints->front(), sweep_direction,
                              &vertex_distances_sweep_direction);
  const double outer_most_point =
      *std::min_element(vertex_distances_sweep_direction.begin(),
                        vertex_distances_sweep_direction.end());
  // The offset makes sure the ray starts outside the polygon.
  Eigen::Vector2d ray_origin =
      waypoints->front() + (outer_most_point - kRayOffset) * sweep_direction;

  // Calculate the num_sweeps-2 sweeps between the first and the last sweep.
  // Sweep direction factor (1: original dir., -1: opposite original dir.)
  int sweep_direction_factor = 1;
  for (int i = 1; i < num_sweeps - 1; ++i) {
    ray_origin += sweep_distance * sweep_normal;
    std::vector<double> ray_scales;
    // Find edge intersections.
    for (size_t i = 0; i < vertices.size(); ++i) {
      Eigen::Vector2d line_start = vertices[i];
      Eigen::Vector2d line_end = line_start + edges[i];
      double t_1;
      if (computeRayLineIntersection(ray_origin, sweep_direction, line_start,
                                     line_end, &t_1)) {
        // Ignore double intersections from multiple edges (ray hits vertex).
        bool scale_was_found = false;
        for (size_t i = 0; i < ray_scales.size(); ++i) {
          if (std::abs(ray_scales[i] - t_1) < kRayCastPrecision) {
            scale_was_found = true;
          }
        }
        if (!scale_was_found) {
          ray_scales.push_back(t_1);
        }
      }
    }
    if (ray_scales.size() != 2) {
      // This should not happen.
      ROS_ERROR_STREAM(kPrefix << "There need to be exactly 2 ray "
                                  "intersections in a convex polygon.");
      return false;
    }
    // Add waypoints. Switching sweep directions.
    const double min_scale =
        *std::min_element(ray_scales.begin(), ray_scales.end());
    const double max_scale =
        *std::max_element(ray_scales.begin(), ray_scales.end());
    const Eigen::Vector2d close_waypoint =
        ray_origin + min_scale * sweep_direction;
    const Eigen::Vector2d far_waypoint =
        ray_origin + max_scale * sweep_direction;
    sweep_direction_factor *= -1;
    if (sweep_direction_factor == 1) {
      // Close -> far
      // Compute turning segment by projecting the new straight-line segment on
      // to the polygon.
      StdVector2d turning_projection;
      if (!castPerpendicularShadowOnVertices(vertices, waypoints->back(),
                                             close_waypoint, -sweep_direction,
                                             &turning_projection)) {
        ROS_ERROR_STREAM(kPrefix << "Could not compute turning segment.");
        return false;
      }
      // Append turning_projection. Don't append the last waypoint again.
      waypoints->insert(waypoints->end(), turning_projection.begin() + 1,
                        turning_projection.end());
      waypoints->push_back(far_waypoint);
    } else {
      // Far -> close
      // Compute turning segment by projecting the new straight-line segment on
      // to the polygon.
      StdVector2d turning_projection;
      if (!castPerpendicularShadowOnVertices(vertices, waypoints->back(),
                                             far_waypoint, sweep_direction,
                                             &turning_projection)) {
        ROS_ERROR_STREAM(kPrefix << "Could not compute turning segment.");
        return false;
      }
      // Append turning_projection swapped, don't the last waypoint again.
      waypoints->insert(waypoints->end(), turning_projection.begin() + 1,
                        turning_projection.end());
      waypoints->push_back(close_waypoint);
    }
  }

  // Add last sweep: visit the remaining adjacent vertices that have not been
  // covered by the last regular sweep.
  // 1. Find uncovered vertices.
  const StdVector2d::iterator point_on_last_sweep = std::prev(waypoints->end());
  StdVector2d remaining_vertices;
  for (size_t i = 0; i < vertices.size(); ++i) {
    // Start from start_vertex such that remaining vertices are ordered from far
    // to close.
    size_t idx = (i + start_vertex_idx_mod) % vertices.size();
    if (pointInHalfSpace(vertices[idx], *point_on_last_sweep, sweep_normal)) {
      remaining_vertices.push_back(vertices[idx]);
    }
  }
  // 2. Depending on sweep direction go clockwise or counter clockwise through
  // the uncovered vertices.
  if (sweep_direction_factor == -1) {
    // Previous sweep was far to close. The remaining vertices need to be
    // travelled starting from the closer remaining vertex.
    std::reverse(remaining_vertices.begin(), remaining_vertices.end());
  }
  // 3. Finally, add the second last waypoint to cover area that eventually may
  // have been left out.
  remaining_vertices.push_back(*std::prev(waypoints->end(), 2));
  waypoints->insert(waypoints->end(), remaining_vertices.begin(),
                    remaining_vertices.end());

  return true;
}

void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k,
    std::vector<std::set<size_t>>* result) {
  CHECK_NOTNULL(result);
  // Initialization.
  result->clear();
  std::vector<size_t> combination(k);
  getAllCombinationsOfKElementsFromN(sorted_elements, k, 0, &combination,
                                     result);
}

void getAllCombinationsOfKElementsFromN(
    const std::vector<size_t>& sorted_elements, int k, int start_pos,
    std::vector<size_t>* combination, std::vector<std::set<size_t>>* result) {
  // https://stackoverflow.com/questions/127704/algorithm-to-return-all-combinations-of-k-elements-from-n
  CHECK_NOTNULL(combination);
  CHECK_NOTNULL(result);
  // New combination found.
  if (k == 0) {
    std::set<size_t> combination_set;
    for (const size_t element : *combination) {
      combination_set.insert(element);
    }
    result->push_back(combination_set);
    return;
  }

  // Recursively add all possible "sub"-combinations.
  for (size_t i = start_pos; i <= sorted_elements.size() - k; i++) {
    // Pick first combination element.
    (*combination)[combination->size() - k] = sorted_elements[i];
    // Add all combinations of remaining elements.
    getAllCombinationsOfKElementsFromN(sorted_elements, k - 1, i + 1,
                                       combination, result);
  }
}

}  // namespace mav_coverage_planning
