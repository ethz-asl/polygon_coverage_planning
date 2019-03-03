#include <glog/logging.h>
#include <vector>

#include "mav_2d_coverage_planning/geometry/bcd_exact.h"

namespace mav_coverage_planning {
std::vector<Polygon_2> computeBCDExact(const PolygonWithHoles& polygon_in,
                                       const Direction_2& dir) {
  // Rotate polygon to have direction aligned with x-axis.
  PolygonWithHoles rotated_polygon = rotatePolygon(polygon_in, dir);
  LOG(INFO) << "// Rotate polygon to have direction aligned with x-axis.";
  LOG(INFO) << polygon_in;
  LOG(INFO) << rotated_polygon;

  // Sort vertices by x value.
  std::vector<VertexConstCirculator> sorted_vertices =
      getXSortedVertices(rotated_polygon);

  // Initialize edge list.
  std::list<Segment_2> L;
  std::list<Polygon_2> open_polygons;
  std::vector<Polygon_2> closed_polygons;
  std::vector<Point_2> processed_vertices;
  for (const VertexConstCirculator& v : sorted_vertices) {
    LOG(INFO) << "Process event: " << *v;
    // v already processed.
    if (std::find(processed_vertices.begin(), processed_vertices.end(), *v) !=
        processed_vertices.end())
      continue;
    processEvent(v, &processed_vertices, &L, &open_polygons, &closed_polygons);
    LOG(INFO) << "L: ";
    for (std::list<Segment_2>::iterator it = L.begin(); it != L.end(); ++it) {
      LOG(INFO) << *it;
    }
  }

  // Rotate back all polygons.
  for (Polygon_2& p : closed_polygons) {
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    p = CGAL::transform(rotation, p);
  }

  return closed_polygons;
}

std::vector<VertexConstCirculator> getXSortedVertices(
    const PolygonWithHoles& p) {
  std::vector<VertexConstCirculator> sorted_vertices;

  // Get boundary vertices.
  VertexConstCirculator v = p.outer_boundary().vertices_circulator();
  do {
    sorted_vertices.push_back(v);
  } while (++v != p.outer_boundary().vertices_circulator());
  // Get hole vertices.
  for (PolygonWithHoles::Hole_const_iterator hit = p.holes_begin();
       hit != p.holes_end(); ++hit) {
    VertexConstCirculator vh = hit->vertices_circulator();
    do {
      sorted_vertices.push_back(vh);
    } while (++vh != hit->vertices_circulator());
  }
  // Sort.
  Polygon_2::Traits::Less_x_2 less_x_2;
  std::sort(sorted_vertices.begin(), sorted_vertices.end(),
            [&less_x_2](const VertexConstCirculator& a,
                        const VertexConstCirculator& b) -> bool {
              return less_x_2(*a, *b);
            });

  return sorted_vertices;
}

PolygonWithHoles rotatePolygon(const PolygonWithHoles& polygon_in,
                               const Direction_2& dir) {
  CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
  rotation = rotation.inverse();
  PolygonWithHoles rotated_polygon = polygon_in;
  rotated_polygon.outer_boundary() =
      CGAL::transform(rotation, polygon_in.outer_boundary());
  PolygonWithHoles::Hole_iterator hit_rot = rotated_polygon.holes_begin();
  for (PolygonWithHoles::Hole_const_iterator hit = polygon_in.holes_begin();
       hit != polygon_in.holes_end(); ++hit) {
    *(hit_rot++) = CGAL::transform(rotation, *hit);
  }

  return rotated_polygon;
}

void processEvent(const VertexConstCirculator& v,
                  std::vector<Point_2>* processed_vertices,
                  std::list<Segment_2>* L, std::list<Polygon_2>* open_polygons,
                  std::vector<Polygon_2>* closed_polygons) {
  // Compute intersection.
  Line_2 l(*v, Direction_2(0, 1));
  std::vector<Point_2> intersections = getIntersections(*L, l);

  // Get e_lower and e_upper.
  Segment_2 e_prev(*v, *std::prev(v));
  Segment_2 e_next(*v, *std::next(v));
  // Catch vertical edges.
  Polygon_2::Traits::Equal_x_2 eq_x_2;
  if (eq_x_2(e_prev.source(), e_prev.target())) {
    e_prev = Segment_2(*std::prev(v), *std::prev(v, 2));
  } else if (eq_x_2(e_next.source(), e_next.target())) {
    e_next = Segment_2(*std::next(v), *std::next(v, 2));
  }

  Polygon_2::Traits::Less_y_2 less_y_2;
  Segment_2 e_lower =
      less_y_2(e_prev.target(), e_next.target()) ? e_prev : e_next;
  Segment_2 e_upper = e_lower == e_prev ? e_next : e_prev;

  LOG(INFO) << "e_lower: " << e_lower;
  LOG(INFO) << "e_upper: " << e_upper;

  Polygon_2::Traits::Less_x_2 less_x_2;
  if (less_x_2(e_lower.target(), e_lower.source()) &&
      less_x_2(e_upper.target(), e_upper.source())) {  // OUT
    // Termination.
    if (L->size() == 2) {
      LOG(INFO) << "Termination.";
      CHECK(open_polygons->size() == 1);
      open_polygons->front().push_back(e_lower.source());
      Polygon_2::Traits::Equal_2 eq_2;
      if (!eq_2(e_lower.source(), e_upper.source())) {
        open_polygons->front().push_back(e_upper.source());
      }
      closed_polygons->push_back(open_polygons->front());
      open_polygons->clear();
      L->clear();
    } else {
      LOG(INFO) << "OUT";
      // Find edges to remove.
      std::list<Segment_2>::iterator e_lower_it = L->begin();
      CHECK(!L->empty());
      size_t e_lower_id = 0;
      for (; e_lower_it != L->end(); ++e_lower_it) {
        if (*e_lower_it == e_lower || *e_lower_it == e_lower.opposite()) {
          break;
        }
        e_lower_id++;
      }
      std::list<Segment_2>::iterator e_upper_it = std::next(e_lower_it);
      size_t e_upper_id = e_lower_id + 1;
      std::cout << "e_lower_id: " << e_lower_id;
      std::cout << "e_upper_id: " << e_upper_id;

      // Close two cells, open one.
      size_t lower_cell_id = e_lower_id / 2;
      size_t upper_cell_id = e_upper_id / 2;
      std::cout << "lower_cell_id: " << lower_cell_id;
      std::cout << "upper_cell_id: " << upper_cell_id;

      // Close lower cell.
      CHECK(intersections.size() > e_upper_id + 1);
      std::list<Polygon_2>::iterator lower_cell =
          std::next(open_polygons->begin(), lower_cell_id);
      lower_cell->push_back(intersections[e_lower_id - 1]);
      lower_cell->push_back(intersections[e_lower_id]);
      closed_polygons->push_back(*lower_cell);
      // Close upper cell.
      std::list<Polygon_2>::iterator upper_cell =
          std::next(open_polygons->begin(), upper_cell_id);
      upper_cell->push_back(intersections[e_upper_id]);
      upper_cell->push_back(intersections[e_upper_id + 1]);
      closed_polygons->push_back(*upper_cell);

      // Delete e_lower and e_upper from list.
      L->erase(e_lower_it);
      L->erase(e_upper_it);

      // Open one new cell.
      std::list<Polygon_2>::iterator new_polygon =
          open_polygons->insert(lower_cell, Polygon_2());
      new_polygon->push_back(intersections[e_upper_id + 1]);
      new_polygon->push_back(intersections[e_lower_id - 1]);

      open_polygons->erase(lower_cell);
      open_polygons->erase(upper_cell);
    }
  } else if (!less_x_2(e_lower.target(), e_lower.source()) &&
             !less_x_2(e_upper.target(), e_upper.source())) {
    // IN
    LOG(INFO) << "IN";
    // Initialization.
    if (L->empty()) {
      L->push_back(e_lower);
      L->push_back(e_upper);
      open_polygons->push_back(Polygon_2());
      open_polygons->front().push_back(e_upper.source());
      Polygon_2::Traits::Equal_2 eq_2;
      if (!eq_2(e_lower.source(), e_upper.source())) {
        open_polygons->front().push_back(e_lower.source());
      }
    } else {
      // Close one cell, open two.
      // Find edge to update.
      size_t e_LOWER_id = 0;
      for (size_t i = 0; i < intersections.size() - 1; i = i + 2) {
        if (less_y_2(intersections[i], e_lower.source()) &&
            less_y_2(e_upper.source(), intersections[i + 1])) {
          e_LOWER_id = i;
          break;
        }
      }
      std::list<Segment_2>::iterator e_LOWER =
          std::next(L->begin(), e_LOWER_id);
      std::list<Polygon_2>::iterator cell =
          std::next(open_polygons->begin(), e_LOWER_id / 2);

      // Close this cell.
      cell->push_back(intersections[e_LOWER_id]);
      cell->push_back(intersections[e_LOWER_id + 1]);
      closed_polygons->push_back(*cell);

      // Add e_lower and e_upper
      std::list<Segment_2>::iterator e_lower_it =
          L->insert(std::next(e_LOWER), e_lower);
      L->insert(std::next(e_lower_it), e_upper);

      // Open two new cells
      // Lower polygon.
      std::list<Polygon_2>::iterator new_polygon =
          open_polygons->insert(cell, Polygon_2());

      new_polygon->push_back(e_lower.source());
      new_polygon->push_back(intersections[e_LOWER_id]);

      // Upper polygon.
      new_polygon = open_polygons->insert(cell, Polygon_2());
      new_polygon->push_back(intersections[e_LOWER_id + 1]);
      new_polygon->push_back(e_upper.source());
      // Close old cell.
      open_polygons->erase(cell);
    }
  } else {
    LOG(INFO) << "MIDDLE";
    // Find edge to update.
    std::list<Segment_2>::iterator old_e_it = L->begin();
    Segment_2 new_edge;
    size_t edge_id = 0;
    for (; old_e_it != L->end(); ++old_e_it) {
      if (*old_e_it == e_upper || *old_e_it == e_upper.opposite()) {
        new_edge = e_lower;
        break;
      } else if (*old_e_it == e_lower || *old_e_it == e_lower.opposite()) {
        new_edge = e_upper;
        break;
      }
      edge_id++;
    }

    // Update cell with new vertex.
    size_t cell_id = edge_id / 2;
    std::list<Polygon_2>::iterator cell =
        std::next(open_polygons->begin(), cell_id);

    if ((edge_id % 2) == 0) {
      // Case 1: Insert new vertex at end.
      cell->push_back(new_edge.source());
    } else {
      // Case 2: Insert new vertex at begin.
      cell->insert(cell->vertices_begin(), new_edge.source());
    }
    // Update edge.
    L->insert(old_e_it, new_edge);
    L->erase(old_e_it);
  }
  processed_vertices->push_back(e_lower.source());
  Polygon_2::Traits::Equal_2 eq_2;
  if (!eq_2(e_lower.source(), e_upper.source())) {
    processed_vertices->push_back(e_upper.source());
  }
}
std::vector<Point_2> getIntersections(const std::list<Segment_2>& L,
                                      const Line_2& l) {
  typedef CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type
      Intersection;

  std::vector<Point_2> intersections(L.size());
  std::vector<Point_2>::iterator intersection = intersections.begin();
  for (std::list<Segment_2>::const_iterator it = L.begin(); it != L.end();
       ++it) {
    Intersection result = CGAL::intersection(*it, l);
    if (result) {
      if (boost::get<Segment_2>(&*result)) {
        CHECK(false) << "Vertical intersection.";
      } else {
        const Point_2* p = boost::get<Point_2>(&*result);
        *(intersection++) = *p;
      }
    } else {
      LOG(ERROR) << "No intersection found!";
    }
  }

  return intersections;
}  // namespace mav_coverage_planning

}  // namespace mav_coverage_planning
