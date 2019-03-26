#include <glog/logging.h>
#include <vector>

#include "mav_2d_coverage_planning/geometry/bcd_exact.h"

namespace mav_coverage_planning {
std::vector<Polygon_2> computeBCDExact(const PolygonWithHoles& polygon_in,
                                       const Direction_2& dir) {
  // Rotate polygon to have direction aligned with x-axis.
  PolygonWithHoles rotated_polygon = rotatePolygon(polygon_in, dir);
  LOG(INFO) << "Input polygon: " << rotated_polygon;
  sortPolygon(&rotated_polygon);

  // Sort vertices by x value.
  std::vector<VertexConstCirculator> sorted_vertices =
      getXSortedVertices(rotated_polygon);

  // Initialize edge list.
  std::list<Segment_2> L;
  std::list<Polygon_2> open_polygons;
  std::vector<Polygon_2> closed_polygons;
  std::vector<Point_2> processed_vertices;
  for (const VertexConstCirculator& v : sorted_vertices) {
    // v already processed.
    if (std::find(processed_vertices.begin(), processed_vertices.end(), *v) !=
        processed_vertices.end())
      continue;
    processEvent(rotated_polygon, v, &processed_vertices, &L, &open_polygons,
                 &closed_polygons);
    LOG(INFO) << "L.";
    for (std::list<Segment_2>::iterator it = L.begin(); it != L.end(); ++it) {
      LOG(INFO) << *it;
    }
  }

  LOG(INFO) << "Polygons.";
  // Rotate back all polygons.
  for (Polygon_2& p : closed_polygons) {
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    p = CGAL::transform(rotation, p);
    LOG(INFO) << p;
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
  Polygon_2::Traits::Less_xy_2 less_xy_2;
  std::sort(sorted_vertices.begin(), sorted_vertices.end(),
            [&less_xy_2](const VertexConstCirculator& a,
                         const VertexConstCirculator& b) -> bool {
              return less_xy_2(*a, *b);
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

void processEvent(const PolygonWithHoles& pwh, const VertexConstCirculator& v,
                  std::vector<Point_2>* processed_vertices,
                  std::list<Segment_2>* L, std::list<Polygon_2>* open_polygons,
                  std::vector<Polygon_2>* closed_polygons) {
  Polygon_2::Traits::Equal_2 eq_2;

  LOG(INFO) << "Event: " << *v;
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
  Polygon_2::Traits::Less_x_2 less_x_2;
  Segment_2 e_lower = e_prev;
  Segment_2 e_upper = e_next;
  if (less_x_2(e_prev.target(), e_prev.source()) &&
      less_x_2(e_next.target(), e_next.source())) {  // OUT

    Point_2 p_on_upper = eq_2(e_lower.source(), e_upper.source())
                             ? e_upper.target()
                             : e_upper.source();
    if (e_lower.supporting_line().has_on_positive_side(p_on_upper))
      std::swap(e_lower, e_upper);

    LOG(INFO) << "OUT";
    // Determine whether we close one or close two and open one.
    // TODO(rikba): instead of looking at unbounded side, look at adjacent edge
    // angle
    bool close_one = outOfPWH(pwh, *v + Vector_2(1e-6, 0));

    // Find edges to remove.
    std::list<Segment_2>::iterator e_lower_it = L->begin();
    size_t e_lower_id = 0;
    for (; e_lower_it != L->end(); ++e_lower_it) {
      if (*e_lower_it == e_lower || *e_lower_it == e_lower.opposite()) {
        break;
      }
      e_lower_id++;
    }

    std::list<Segment_2>::iterator e_upper_it = std::next(e_lower_it);
    size_t e_upper_id = e_lower_id + 1;
    size_t lower_cell_id = e_lower_id / 2;
    size_t upper_cell_id = e_upper_id / 2;
    LOG(INFO) << "e_lower " << e_lower;
    LOG(INFO) << "e_upper " << e_upper;
    LOG(INFO) << "e_upper_id " << e_upper_id;
    LOG(INFO) << "lower_cell_id " << lower_cell_id;
    LOG(INFO) << "upper_cell_id " << upper_cell_id;

    if (close_one) {
      LOG(INFO) << "Close one.";
      std::list<Polygon_2>::iterator cell =
          std::next(open_polygons->begin(), lower_cell_id);
      cell->push_back(e_lower.source());
      Polygon_2::Traits::Equal_2 eq_2;
      if (!eq_2(e_lower.source(), e_upper.source())) {
        cell->push_back(e_upper.source());
      }
      if (cleanupPolygon(&*cell)) closed_polygons->push_back(*cell);
      L->erase(e_lower_it);
      L->erase(e_upper_it);
      open_polygons->erase(cell);
    } else {
      LOG(INFO) << "Close two cells, open one.";
      // Close two cells, open one.
      // Close lower cell.
      LOG(INFO) << "// Close lower cell.";
      CHECK_GT(e_lower_id, 0);
      CHECK_GT(intersections.size(), e_upper_id + 1);
      std::list<Polygon_2>::iterator lower_cell =
          std::next(open_polygons->begin(), lower_cell_id);
      LOG(INFO) << "Lower cell: " << *lower_cell;
      lower_cell->push_back(intersections[e_lower_id - 1]);
      lower_cell->push_back(intersections[e_lower_id]);
      if (cleanupPolygon(&*lower_cell)) closed_polygons->push_back(*lower_cell);
      LOG(INFO) << "// Close upper cell.";
      // Close upper cell.
      std::list<Polygon_2>::iterator upper_cell =
          std::next(open_polygons->begin(), upper_cell_id);
      upper_cell->push_back(intersections[e_upper_id]);
      upper_cell->push_back(intersections[e_upper_id + 1]);
      if (cleanupPolygon(&*upper_cell)) closed_polygons->push_back(*upper_cell);

      // Delete e_lower and e_upper from list.
      LOG(INFO) << "// Delete e_lower and e_upper from list.";
      L->erase(e_lower_it);
      L->erase(e_upper_it);

      // Open one new cell.
      LOG(INFO) << "// Open one new cell.";
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
    Polygon_2::Traits::Equal_2 eq_2;
    LOG(INFO) << "IN";
    Point_2 p_on_lower = eq_2(e_lower.source(), e_upper.source())
                             ? e_lower.target()
                             : e_lower.source();
    if (e_upper.supporting_line().has_on_positive_side(p_on_lower))
      std::swap(e_lower, e_upper);

    // Determine whether we open one or close one and open two.
    bool open_one = outOfPWH(pwh, *v - Vector_2(1e-6, 0));

    // Find edge to update.
    size_t e_LOWER_id = 0;
    bool found_e_lower_id = false;
    for (size_t i = 0; i < intersections.size() - 1; i = i + 2) {
      if (intersections.empty()) break;
      if (open_one) {
        if (less_y_2(intersections[i], e_lower.source()) &&
            less_y_2(intersections[i + 1], e_upper.source())) {
          e_LOWER_id = i;
          found_e_lower_id = true;
        }
      } else {
        if (less_y_2(intersections[i], e_lower.source()) &&
            less_y_2(e_upper.source(), intersections[i + 1])) {
          e_LOWER_id = i;
        }
      }
    }
    if (open_one) {
      LOG(INFO) << "Open one.";
      // Add one new cell above e_UPPER.
      std::list<Segment_2>::iterator e_UPPER = L->begin();
      std::list<Polygon_2>::iterator open_cell = open_polygons->begin();
      if (!L->empty() && found_e_lower_id) {
        LOG(INFO) << "e_LOWER_id: " << e_LOWER_id;
        e_UPPER = std::next(e_UPPER, e_LOWER_id + 1);
        open_cell = std::next(open_cell, e_LOWER_id / 2 + 1);
      }
      // Update edge list.
      if (L->empty()) {
        L->insert(L->end(), e_lower);
        L->insert(L->end(), e_upper);
      } else if (!L->empty() && !found_e_lower_id) {
        L->insert(L->begin(), e_upper);
        L->insert(L->begin(), e_lower);
      } else {
        std::list<Segment_2>::iterator inserter = std::next(e_UPPER);
        L->insert(inserter, e_lower);
        L->insert(inserter, e_upper);
      }

      // Create new polygon.
      std::list<Polygon_2>::iterator open_polygon =
          open_polygons->insert(open_cell, Polygon_2());
      open_polygon->push_back(e_upper.source());
      if (!eq_2(e_lower.source(), e_upper.source())) {
        open_polygon->push_back(e_lower.source());
      }
    } else {
      LOG(INFO) << "Close one, open two.";
      // Add new polygon between e_LOWER and e_UPPER.
      std::list<Segment_2>::iterator e_LOWER =
          std::next(L->begin(), e_LOWER_id);
      std::list<Polygon_2>::iterator cell =
          std::next(open_polygons->begin(), e_LOWER_id / 2);

      // Add e_lower and e_upper
      std::list<Segment_2>::iterator e_lower_it =
          L->insert(std::next(e_LOWER), e_lower);
      L->insert(std::next(e_lower_it), e_upper);

      // Add new cell.
      std::list<Polygon_2>::iterator new_polygon =
          open_polygons->insert(cell, Polygon_2());

      // Close one cell.
      cell->push_back(intersections[e_LOWER_id]);
      cell->push_back(intersections[e_LOWER_id + 1]);
      if (cleanupPolygon(&*cell)) closed_polygons->push_back(*cell);
      // Open two new cells
      // Lower polygon.
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
    LOG(INFO) << "MIDDLE.";
    // Correct vertical edges.

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
        *(intersection++) = it->target();
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

void sortPolygon(PolygonWithHoles* pwh) {
  if (pwh->outer_boundary().is_clockwise_oriented())
    pwh->outer_boundary().reverse_orientation();

  for (PolygonWithHoles::Hole_iterator hi = pwh->holes_begin();
       hi != pwh->holes_end(); ++hi)
    if (hi->is_counterclockwise_oriented()) hi->reverse_orientation();
}

bool cleanupPolygon(Polygon_2* poly) {
  Polygon_2::Traits::Equal_2 eq_2;
  bool erase_one = true;
  while (erase_one) {
    Polygon_2::Vertex_circulator vit = poly->vertices_circulator();
    erase_one = false;
    do {
      if (eq_2(*vit, *std::next(vit))) {
        poly->erase(vit++);
        erase_one = true;
      } else
        vit++;
    } while (vit != poly->vertices_circulator());
  }

  if (poly->is_simple() && poly->area() != 0.0) {
    LOG(INFO) << "Adding polygon: " << *poly;
  }
  return poly->is_simple() && poly->area() != 0.0;
}

bool outOfPWH(const PolygonWithHoles& pwh, const Point_2& p) {
  if (pwh.outer_boundary().has_on_negative_side(p)) return true;

  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    if (pwh.outer_boundary().has_on_negative_side(p)) return true;
  }

  return false;
}

}  // namespace mav_coverage_planning
