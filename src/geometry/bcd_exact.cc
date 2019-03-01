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
  for (const VertexConstCirculator& v : sorted_vertices) {
    LOG(INFO) << "Process event: " << *v;
    processEvent(v, &L, &open_polygons, &closed_polygons);
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

void processEvent(const VertexConstCirculator& v, std::list<Segment_2>* L,
                  std::list<Polygon_2>* open_polygons,
                  std::vector<Polygon_2>* closed_polygons) {
  // Compute intersection.
  Line_2 l(*v, Direction_2(0, 1));
  std::vector<Point_2> intersections = getIntersections(*L, l);

  // Get adjacent edges.
  VertexConstCirculator v_prev = std::prev(v);
  VertexConstCirculator v_next = std::next(v);
  LOG(INFO) << "v_prev: " << *v_prev;
  LOG(INFO) << "v_next: " << *v_next;

  Polygon_2::Traits::Less_y_2 less_y_2;
  VertexConstCirculator v_lower = less_y_2(*v_prev, *v_next) ? v_prev : v_next;
  VertexConstCirculator v_upper = v_lower == v_prev ? v_next : v_prev;

  LOG(INFO) << "v_lower: " << *v_lower;
  LOG(INFO) << "v_upper: " << *v_upper;

  Polygon_2::Traits::Less_x_2 less_x_2;

  if (less_x_2(*v_lower, *v) && less_x_2(*v_upper, *v)) {  // OUT
    Segment_2 e_lower(*v, *v_lower);
    Segment_2 e_upper(*v_upper, *v);
    LOG(INFO) << "OUT";
    LOG(INFO) << "e_lower: " << e_lower;
    LOG(INFO) << "e_upper: " << e_upper;
    // Close two cells, open one.
    // Find cells to close.
    // The new vertex lies between the two intersections.
    size_t cell_id = 0;
    for (size_t i = 0; i < intersections.size() - 3; i = i + 2) {
      Polygon_2::Traits::Less_y_2 less_y_2;
      if (less_y_2(intersections[i], *v) &&
          !less_y_2(intersections[i + 3], *v)) {
        cell_id = i;
        break;
      }
    }

    // Close lower cell.
    std::list<Polygon_2>::iterator lower_cell =
        std::next(open_polygons->begin(), cell_id);
    lower_cell->push_back(intersections[cell_id]);
    lower_cell->push_back(intersections[cell_id + 1]);
    closed_polygons->push_back(*lower_cell);
    // Close upper cell.
    std::list<Polygon_2>::iterator upper_cell =
        std::next(open_polygons->begin(), cell_id + 1);
    upper_cell->push_back(intersections[cell_id + 3]);
    upper_cell->push_back(intersections[cell_id + 1]);
    closed_polygons->push_back(*upper_cell);

    // Delete e_lower and e_upper from list.
    L->remove(e_lower);
    L->remove(e_upper);

    // Open one new cell.
    std::list<Polygon_2>::iterator new_polygon =
        open_polygons->insert(lower_cell, Polygon_2());
    new_polygon->push_back(intersections[cell_id + 3]);
    new_polygon->push_back(intersections[cell_id]);

    open_polygons->erase(lower_cell);
    open_polygons->erase(upper_cell);
  } else if (!less_x_2(*v_lower, *v) && !less_x_2(*v_upper, *v)) {  // IN
    Segment_2 e_lower(*v, *v_lower);
    Segment_2 e_upper(*v_upper, *v);
    LOG(INFO) << "IN";
    LOG(INFO) << "e_lower: " << e_lower;
    LOG(INFO) << "e_upper: " << e_upper;
    // Initialization.
    if (L->empty()) {
      L->push_back(e_lower);
      L->push_back(e_upper);
      open_polygons->push_back(Polygon_2());
      open_polygons->front().push_back(*v);
    } else {
      // Close one cell, open two.
      // Find position where to insert new edges.
      // The new vertex lies between the two intersections.
      size_t cell_id = 0;
      for (size_t i = 0; i < intersections.size() - 1; i = i + 2) {
        Polygon_2::Traits::Less_y_2 less_y_2;
        if (less_y_2(intersections[i], *v) &&
            !less_y_2(intersections[i + 1], *v)) {
          cell_id = i;
          break;
        }
      }
      // Close this cell.
      std::list<Polygon_2>::iterator current_cell =
          std::next(open_polygons->begin(), cell_id);
      current_cell->push_back(intersections[cell_id]);
      current_cell->push_back(intersections[cell_id + 1]);
      closed_polygons->push_back(*current_cell);

      // Add e_lower and e_upper
      std::list<Segment_2>::iterator inserter;
      inserter = std::next(L->begin(), cell_id + 1);
      inserter = L->insert(inserter, e_upper);
      inserter = L->insert(inserter, e_lower);

      // Open two new cells
      // Lower polygon.
      std::list<Polygon_2>::iterator new_polygon =
          open_polygons->insert(current_cell, Polygon_2());
      new_polygon->push_back(*v);
      new_polygon->push_back(intersections[cell_id]);
      // Upper polygon.
      new_polygon = open_polygons->insert(current_cell, Polygon_2());
      new_polygon->push_back(intersections[cell_id + 1]);
      new_polygon->push_back(*v);
      // Close old cell.
      open_polygons->erase(current_cell);
    }
  } else {
    LOG(INFO) << "MIDDLE";
    // Find edge to update.
    size_t edge_id = 0;
    for (size_t i = 0; i < intersections.size(); ++i) {
      if (intersections[i] == *v) {
        edge_id = i;
        break;
      }
    }
    std::list<Segment_2>::iterator edge = std::next(L->begin(), edge_id);
    std::list<Polygon_2>::iterator cell =
        std::next(open_polygons->begin(), edge_id / 2);

    // Identify e_lower and e_upper.
    Segment_2 edge_1(*v_prev, *v);
    Segment_2 edge_2(*v_next, *v);
    Segment_2 e_lower;
    Segment_2 e_upper;
    if ((edge_1 == *edge) || (edge_2.opposite() == *edge)) {
      e_lower = edge_1;
      e_upper = edge_2.opposite();
    } else if ((edge_2 == *edge) || (edge_1.opposite() == *edge)) {
      e_lower = edge_2;
      e_upper = edge_1.opposite();
    } else {
      CHECK(false) << "Finding e_lower and e_upper failed.";
    }

    // Case 1: delete e_lower from list and insert e_upper.
    // Insert new vertex at end.
    if ((edge_id % 2) == 0) {
      // Update cell.
      cell->push_back(*v);
      // Update edge.
      L->insert(edge, e_upper);
      L->erase(edge);
    }
    // Case 2: delete e_upper from list and insert e_lower
    // Insert new vertex at begin.
    else {
      // Update cell.
      cell->insert(cell->vertices_begin(), *v);
      // Update edge.
      L->insert(edge, e_lower);
      L->erase(edge);
    }
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
      if (const Segment_2* s = boost::get<Segment_2>(&*result)) {
        // TODO(rikba): Handle this case!!
        LOG(ERROR) << "Segment intersection!";
        *(intersection++) = s->source();
      } else {
        const Point_2* p = boost::get<Point_2>(&*result);
        *(intersection++) = *p;
      }
    } else {
      LOG(ERROR) << "No intersection found!";
    }
  }

  return intersections;
}

}  // namespace mav_coverage_planning
