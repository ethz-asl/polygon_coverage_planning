#include "mav_2d_coverage_planning/geometry/BCD.h"
#include <glog/logging.h>
#include <math.h>

const double kEps = 0.001;

namespace mav_coverage_planning {
BCD::BCD(const PolygonWithHoles& polygon) : polygon_(polygon) {
  polygon_nr_ = 0;
  // TODO(luliu): how do I write this without this loop?
  int size = 0;
  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi) {
    size++;
  }
  // Reserve 2 per polygon plus outer boundary.
  edge_list_.reserve((size + 1) * 2);
  created_polygons_.reserve(size * 4);
}

bool BCD::computeBCDFromPolygonWithHoles(std::vector<Polygon_2>* polygons) {
  CHECK_NOTNULL(polygons);
  polygons->clear();

  double min_polygons = 0;
  double alpha = M_PI / 4;
  for (size_t times = 0; times < 4; ++times) {
    outer_events_.clear();
    all_inner_events_.clear();
    edge_list_.clear();
    created_polygons_.clear();
    polygon_nr_ = 0;

    createEvents(&polygon_.outer_boundary(), &outer_events_);
    for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
         hi != polygon_.holes_end(); ++hi) {
      std::vector<Event> inner_events;
      Polygon_2 hole = *hi;
      createEvents(&hole, &inner_events);
      all_inner_events_.push_back(inner_events);
    }
    // Start algorithm
    bool first_vertex = false;
    bool outer = false;
    std::vector<Edge> upper_vertices;

    Event* next_event = nullptr;
    findNextEvent(&first_vertex, &next_event, &outer);
    Edge edge = {.loc = next_event->location, .dir = next_event->floor};
    edge_list_.push_back(edge);
    Edge edge2 = {.loc = next_event->location2, .dir = next_event->ceiling};
    edge_list_.push_back(edge2);
    next_event->closed = true;
    while (!edge_list_.empty()) {
      int edge_upper;
      int edge_lower;
      getEdges(&edge_upper, &edge_lower);
      std::vector<Edge> new_polygon;
      new_polygon.push_back(edge_list_[edge_upper]);
      new_polygon.push_back(edge_list_[edge_lower]);
      created_polygons_.push_back(new_polygon);
      findNextEvent(created_polygons_[polygon_nr_].front(),
                    created_polygons_[polygon_nr_].back(), &first_vertex,
                    &next_event, &outer);
      int type = next_event->type;
      upper_vertices.clear();
      while (type == 2) {
        if (upper_vertices.size() == 0) {
          findNextEvent(created_polygons_[polygon_nr_].front(),
                        created_polygons_[polygon_nr_].back(), &first_vertex,
                        &next_event, &outer);
        } else {
          findNextEvent(upper_vertices.back(),
                        created_polygons_[polygon_nr_].back(), &first_vertex,
                        &next_event, &outer);
        }
        if (next_event->type == 2) {
          createVertices(first_vertex, outer, next_event, &upper_vertices);
        }
        type = next_event->type;
      }
      innerPolygonEnd(first_vertex, upper_vertices, edge_upper, edge_lower,
                      outer, next_event);
      polygon_nr_++;
    }
    if (polygon_nr_ < min_polygons || min_polygons == 0) {
      LOG(WARNING) << "Times" << times;
      removeDublicatedVeritices();
      polygons->clear();
      polygons->reserve(created_polygons_.size());
      Polygon_2 polygon;
      int i = 0;

      for (std::vector<std::vector<Edge>>::iterator it =
               created_polygons_.begin();
           it != created_polygons_.end(); ++it) {
        polygon.clear();
        for (std::vector<Edge>::iterator poly = it->begin(); poly != it->end();
             ++poly) {
          Point_2 new_point(poly->loc.x() * cos(-alpha * times) -
                                poly->loc.y() * sin(-alpha * times),
                            poly->loc.x() * sin(-alpha * times) +
                                poly->loc.y() * cos(-alpha * times));
          polygon.push_back(new_point);
        }
        polygons->emplace_back(polygon);
        i++;
      }
      min_polygons = polygon_nr_;
    }
    polygon_ = rotPolygon(alpha);
  }

  return true;
}

PolygonWithHoles BCD::rotPolygon(double alpha) {
  Polygon_2 hull;
  for (VertexConstIterator it = polygon_.outer_boundary().vertices_begin();
       it != polygon_.outer_boundary().vertices_end(); ++it) {
    Point_2 new_point(it->x() * cos(alpha) - it->y() * sin(alpha),
                      it->x() * sin(alpha) + it->y() * cos(alpha));
    hull.push_back(new_point);
  }
  PolygonWithHoles pwh(hull);
  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi) {
    Polygon_2 hole;
    for (VertexConstIterator it = hi->vertices_begin();
         it != hi->vertices_end(); ++it) {
      Point_2 new_point(it->x() * cos(alpha) - it->y() * sin(alpha),
                        it->x() * sin(alpha) + it->y() * cos(alpha));
      hole.push_back(new_point);
    }
    pwh.add_hole(hole);
  }
  return pwh;
}

void BCD::removeDublicatedVeritices() {
  for (size_t i = 0; i < created_polygons_.size(); ++i) {
    std::vector<Edge>::iterator first = created_polygons_[i].begin();
    std::vector<Edge>::iterator last = created_polygons_[i].end();
    std::vector<Edge>::iterator result = first;
    while (++first != last) {
      if (!(abs(result->loc.x() - first->loc.x()) < kEps &&
            abs(result->loc.y() - first->loc.y()) < kEps) &&
          ++result != first) {
        *result = std::move(*first);
      }
    }
    created_polygons_[i].erase(++result, created_polygons_[i].end());
  }
  for (size_t i = 0; i < created_polygons_.size(); ++i) {
    std::vector<Edge>::iterator first = created_polygons_[i].begin() + 1;
    std::vector<Edge>::iterator last = created_polygons_[i].end();
    std::vector<Edge>::iterator result = created_polygons_[i].begin();
    std::vector<Edge>::iterator result2 = created_polygons_[i].begin() + 1;
    while (++first < last) {
      if ((abs(result->loc.x() - first->loc.x()) < kEps &&
           abs(result2->loc.x() - first->loc.x()) < kEps) ||
          (abs(result->loc.y() - first->loc.y()) < kEps &&
           abs(result2->loc.y() - first->loc.y()) < kEps)) {
        *result2 = std::move(*first);
      } else {
        result++;
        if (++result2 != first) {
          *result2 = std::move(*first);
        }
      }
    }
    created_polygons_[i].erase(++result2, created_polygons_[i].end());
  }
}

void BCD::innerPolygonEnd(bool first_vertex,
                          const std::vector<Edge>& upper_vertices,
                          int edge_upper, int edge_lower, bool outer,
                          Event* event) {
  CHECK_NOTNULL(event);

  double x_current = CGAL::to_double(event->location.x());
  Point_2 next_point;
  Edge upper_vertex;
  if (first_vertex) {
    next_point = event->location;
  } else {
    next_point = event->location2;
  }
  size_t edge_list_size = edge_list_.size();
  if (upper_vertices.size() == 0) {
    upper_vertex = created_polygons_[polygon_nr_].front();
  } else {
    upper_vertex = upper_vertices.back();
  }
  Edge lower_vertex = created_polygons_[polygon_nr_].back();
  closePolygon(x_current, upper_vertices, edge_lower, edge_upper);
  if (abs(next_point.y() - calculateVertex(x_current, upper_vertex)) > kEps) {
    if (edge_list_.size() < edge_list_size - 1) {
      closeSecondEvent(x_current, calculateVertex(x_current, upper_vertex),
                       false);
    }
    event->upper = true;
  } else {
    if (edge_list_.size() < edge_list_size - 1) {
      closeSecondEvent(x_current, calculateVertex(x_current, lower_vertex),
                       true);
    }
    event->lower = true;
  }
  if (event->upper && event->lower) {
    event->closed = true;
  }
}

void BCD::closeSecondEvent(double x_current, double y_event, bool upper) {
  for (size_t i = 0; i < all_inner_events_.size(); ++i) {
    for (size_t a = 0; a < all_inner_events_[i].size(); ++a) {
      updateEventStatus(upper, x_current, y_event, &all_inner_events_[i][a]);
    }
  }
  for (size_t i = 0; i < outer_events_.size(); ++i) {
    updateEventStatus(upper, x_current, y_event, &outer_events_[i]);
  }
}

void BCD::updateEventStatus(bool upper, double x_current, double y_event,
                            Event* event) {
  CHECK_NOTNULL(event);

  if (abs(x_current - event->location.x()) < kEps &&
      (abs(event->location.y() - y_event) < kEps ||
       abs(event->location2.y() - y_event) < kEps)) {
    if (upper) {
      event->upper = true;
    } else {
      event->lower = true;
    }
    if (event->upper && event->lower) {
      event->closed = true;
    }
  }
}

void BCD::closePolygon(double x_current,
                       const std::vector<Edge>& upper_vertices, int edge_lower,
                       int edge_upper) {
  // Close previous polygon
  Point_2 location(
      x_current,
      calculateVertex(x_current, created_polygons_[polygon_nr_].back()));
  Edge new_edge = {.loc = location,
                   .dir = created_polygons_[polygon_nr_].back().dir};
  created_polygons_[polygon_nr_].push_back(new_edge);
  if (abs(edge_list_[edge_lower].dir.x() - location.x()) < kEps &&
      abs(edge_list_[edge_lower].dir.y() - location.y()) < kEps) {
    if (edge_upper > edge_lower) {
      edge_upper--;
    }
    edge_list_.erase(edge_list_.begin() + edge_lower);
  } else {
    edge_list_[edge_lower].loc = location;
  }
  if (upper_vertices.size() == 0) {
    Point_2 new_location(
        x_current,
        calculateVertex(x_current, created_polygons_[polygon_nr_].front()));
    new_edge.loc = new_location;
    new_edge.dir = created_polygons_[polygon_nr_].front().dir;
  } else {
    Point_2 new_location(x_current,
                         calculateVertex(x_current, upper_vertices.back()));
    new_edge.loc = new_location;
    new_edge.dir = upper_vertices.back().dir;
  }
  created_polygons_[polygon_nr_].push_back(new_edge);
  if (abs(edge_list_[edge_upper].dir.x() - new_edge.loc.x()) < kEps &&
      abs(edge_list_[edge_upper].dir.y() - new_edge.loc.y()) < kEps) {
    edge_list_.erase(edge_list_.begin() + edge_upper);
  } else {
    edge_list_[edge_upper].loc = new_edge.loc;
  }
  for (std::vector<Edge>::const_reverse_iterator i = upper_vertices.rbegin();
       i != upper_vertices.rend(); ++i) {
    created_polygons_[polygon_nr_].push_back(*i);
  }
}

bool BCD::createEvents(Polygon_2* polygon, std::vector<Event>* events) {
  CHECK_NOTNULL(polygon);
  CHECK_NOTNULL(events);
  events->clear();

  if (polygon->is_counterclockwise_oriented()) {
    polygon->reverse_orientation();
  } else if (polygon->is_collinear_oriented()) {
    return false;
  }
  std::vector<Vertex> vertices;
  VertexConstCirculator vit = polygon->vertices_circulator();
  do {
    Vertex vertex;
    initVertex(vit, &vertex);
    vertices.push_back(vertex);
  } while (++vit != polygon->vertices_circulator());

  for (size_t i = 0; i < vertices.size(); ++i) {
    if (!vertices[(i - 1 + vertices.size()) % vertices.size()].merge_next) {
      Event event;
      if (vertices[i].merge_next) {
        initEvent(vertices[i], vertices[(i + 1) % vertices.size()], &event);
      } else {
        initEvent(vertices[i], vertices[i], &event);
      }
      events->push_back(event);
    }
  }
  return true;
}

void BCD::initVertex(const VertexConstCirculator& orig_vertex,
                     Vertex* vertex) const {
  CHECK_NOTNULL(vertex);

  vertex->location = *orig_vertex;
  vertex->ceiling = *std::next(orig_vertex);
  vertex->floor = *std::prev(orig_vertex);
  vertex->merge_next = false;
  vertex->ceiling.x() >= vertex->location.x() ? vertex->colour2 = 0
                                              : vertex->colour2 = 1;
  vertex->floor.x() > vertex->location.x() ? vertex->colour1 = 1
                                           : vertex->colour1 = 0;
  if (abs(vertex->ceiling.x() - vertex->location.x()) < kEps) {
    vertex->merge_next = true;
  }
}

void BCD::initEvent(const Vertex& vertex_now, const Vertex& vertex_next,
                    Event* event) const {
  CHECK_NOTNULL(event);

  event->location = vertex_now.location;
  event->location2 = vertex_next.location;
  event->closed = false;
  event->upper = false;
  event->lower = false;
  event->ceiling = vertex_next.ceiling;
  event->floor = vertex_now.floor;
  if (vertex_now.colour1 == 0 && vertex_next.colour2 == 1) {
    event->type = 1;
  } else if (vertex_now.colour1 == 1 && vertex_next.colour2 == 0) {
    event->type = 0;
  } else {
    event->type = 2;
  }
}

bool BCD::findNextEvent(const Edge& edge1, const Edge& edge2,
                        bool* first_vertex, Event** next_event, bool* outer) {
  CHECK_NOTNULL(outer);
  CHECK_NOTNULL(first_vertex);
  CHECK_NOTNULL(next_event);

  *next_event = nullptr;
  double x_search =
      CGAL::to_double(min(edge1.dir.x(), edge2.dir.x())) + 2 * kEps;
  double x_min = CGAL::to_double((max(edge1.loc.x(), edge2.loc.x()))) - kEps;
  int type = -1;
  for (size_t i = 0; i < outer_events_.size(); ++i) {
    if (outer_events_[i].location.x() >= x_min &&
        outer_events_[i].location.x() <= x_search + kEps &&
        !outer_events_[i].closed) {
      if (outer_events_[i].type > type ||
          outer_events_[i].location.x() < x_search - kEps) {
        double y1 = calculateVertex(
            CGAL::to_double(outer_events_[i].location.x()), edge1);
        double y2 = calculateVertex(
            CGAL::to_double(outer_events_[i].location.x()), edge2);
        if ((outer_events_[i].location.y() <= y1 + kEps &&
             outer_events_[i].location.y() >= y2 - kEps) ||
            (outer_events_[i].location2.y() <= y1 + kEps &&
             outer_events_[i].location2.y() >= y2 - kEps)) {
          x_search = CGAL::to_double(outer_events_[i].location.x());
          type = outer_events_[i].type;
          *next_event = &outer_events_[i];
          if (outer_events_[i].location.y() <= y1 + kEps &&
              outer_events_[i].location.y() >= y2 - kEps) {
            *first_vertex = true;
          } else {
            *first_vertex = false;
          }
          *outer = true;
        }
      }
    }
  }
  for (size_t a = 0; a < all_inner_events_.size(); ++a) {
    std::vector<Event> inner_events = all_inner_events_[a];
    for (size_t i = 0; i < inner_events.size(); ++i) {
      if (inner_events[i].location.x() >= x_min &&
          inner_events[i].location.x() <= x_search + kEps &&
          !inner_events[i].closed) {
        if ((inner_events[i].type > type) ||
            inner_events[i].location.x() < x_search - kEps) {
          double y1 = calculateVertex(
              CGAL::to_double(inner_events[i].location.x()), edge1);
          double y2 = calculateVertex(
              CGAL::to_double(inner_events[i].location.x()), edge2);
          if ((inner_events[i].location.y() <= y1 + kEps &&
               inner_events[i].location.y() >= y2 - kEps) ||
              (inner_events[i].location2.y() <= y1 + kEps &&
               inner_events[i].location2.y() >= y2 - kEps)) {
            *next_event = &all_inner_events_[a][i];
            type = inner_events[i].type;
            x_search = CGAL::to_double(inner_events[i].location.x());

            if ((inner_events[i].location.y() <= y1 + kEps &&
                 inner_events[i].location.y() >= y2 - kEps) &&
                (inner_events[i].location2.y() <= y1 + kEps &&
                 inner_events[i].location2.y() >= y2 - kEps)) {
              if (std::min(abs(inner_events[i].location.y() - y1),
                           abs(inner_events[i].location.y() - y2)) <
                  min(abs(inner_events[i].location2.y() - y1),
                      abs(inner_events[i].location2.y() - y2))) {
                *first_vertex = true;
              } else {
                *first_vertex = false;
              }
            } else if (inner_events[i].location.y() <= y1 + kEps &&
                       inner_events[i].location.y() >= y2 - kEps) {
              *first_vertex = true;
            } else {
              *first_vertex = false;
            }
            *outer = false;
          }
        }
      }
    }
  }
  if (*next_event) {
    return true;
  } else {
    return false;
  }
}

bool BCD::findNextEvent(bool* first_vertex, Event** next_event, bool* outer) {
  CHECK_NOTNULL(first_vertex);
  CHECK_NOTNULL(next_event);
  CHECK_NOTNULL(outer);

  *next_event = nullptr;
  double inf = std::numeric_limits<double>::max();
  double x_search = inf;
  int type = -1;
  for (size_t i = 0; i < outer_events_.size(); ++i) {
    if (outer_events_[i].location.x() <= x_search + kEps &&
        !outer_events_[i].closed) {
      if (outer_events_[i].type > type ||
          outer_events_[i].location.x() < x_search - kEps) {
        x_search = CGAL::to_double(outer_events_[i].location.x());
        type = outer_events_[i].type;
        *next_event = &outer_events_[i];
        *first_vertex = true;
        *outer = true;
      }
    }
  }
  for (size_t a = 0; a < all_inner_events_.size(); ++a) {
    std::vector<Event> inner_events = all_inner_events_[a];
    for (size_t i = 0; i < inner_events.size(); ++i) {
      if (inner_events[i].location.x() <= x_search + kEps &&
          !inner_events[i].closed) {
        if ((inner_events[i].type > type) ||
            inner_events[i].location.x() < x_search - kEps) {
          *next_event = &all_inner_events_[a][i];
          type = inner_events[i].type;
          x_search = CGAL::to_double(inner_events[i].location.x());
          *first_vertex = true;
          *outer = false;
        }
      }
    }
  }
  if (*next_event) {
    return true;
  } else {
    return false;
  }
}

double BCD::calculateVertex(double x, const Edge& edge) const {
  double x_delta = x - CGAL::to_double(edge.loc.x());
  Vector_2 vector = edge.dir - edge.loc;
  double frac = x_delta / CGAL::to_double(vector.x());
  double y_res = CGAL::to_double(edge.loc.y() + vector.y() * frac);

  return y_res;
}

void BCD::getEdges(int* edge_upper_number, int* edge_lower_number) {
  CHECK_NOTNULL(edge_upper_number);
  CHECK_NOTNULL(edge_lower_number);

  double inf = std::numeric_limits<double>::max();
  double value_loc_x = inf;
  double value_dir_y = inf;
  *edge_lower_number = -1;
  *edge_upper_number = -1;
  for (size_t i = 0; i < edge_list_.size(); ++i) {
    if (edge_list_[i].loc.x() < value_loc_x - kEps) {
      value_loc_x = CGAL::to_double(edge_list_[i].loc.x());
    }
  }
  bool add_more = true;
  while (add_more) {
    Event* next_event = nullptr;
    bool first_vertex;
    bool outer;
    findNextEvent(&first_vertex, &next_event, &outer);
    double x_now = CGAL::to_double(next_event->location.x());
    if (next_event->type == 0 && value_loc_x >= x_now - kEps) {
      Edge edge1 = {.loc = next_event->location, .dir = next_event->floor};
      edge_list_.push_back(edge1);
      Edge edge2 = {.loc = next_event->location2, .dir = next_event->ceiling};
      edge_list_.push_back(edge2);
      next_event->closed = true;
      value_loc_x = x_now;
    } else {
      add_more = false;
    }
  }
  Point_2 location(value_loc_x, inf);
  Point_2 direction(inf, inf);
  Edge edge = {.loc = location, .dir = direction};
  for (size_t i = 0; i < edge_list_.size(); ++i) {
    if (*edge_lower_number >= 0) {
      value_dir_y =
          calculateVertex(CGAL::to_double(edge_list_[i].dir.x()), edge);
    }
    if ((abs(edge_list_[i].loc.x() - edge.loc.x()) < kEps &&
         edge.loc.y() > edge_list_[i].loc.y() + kEps) ||
        (abs(edge_list_[i].loc.x() - edge.loc.x()) < kEps &&
         abs(edge.loc.y() - edge_list_[i].loc.y()) < kEps &&
         value_dir_y > edge_list_[i].dir.y() + kEps)) {
      *edge_lower_number = i;
      edge = edge_list_[i];
    }
  }
  edge = {.loc = location, .dir = direction};
  value_dir_y = inf;
  for (size_t i = 0; i < edge_list_.size(); ++i) {
    if (*edge_upper_number >= 0) {
      value_dir_y =
          calculateVertex(CGAL::to_double(edge_list_[i].dir.x()), edge);
    }
    if ((abs(edge_list_[i].loc.x() - edge.loc.x()) < kEps &&
         static_cast<int>(i) != *edge_lower_number) &&
        (edge.loc.y() > edge_list_[i].loc.y() + kEps ||
         (abs(edge.loc.y() - edge_list_[i].loc.y()) < kEps &&
          value_dir_y > edge_list_[i].dir.y() + kEps))) {
      *edge_upper_number = i;
      edge = edge_list_[i];
    }
  }
}

void BCD::createVertices(bool first_vertex, bool outer, Event* event,
                         std::vector<Edge>* upper_vertices) {
  CHECK_NOTNULL(upper_vertices);
  CHECK_NOTNULL(event);

  double upper_point;
  Point_2 new_point;
  Point_2 new_point2;
  Point_2 direction1;
  Point_2 direction2;
  if (!outer) {
    direction1 = event->floor;
    direction2 = event->ceiling;
  } else {
    direction1 = event->ceiling;
    direction2 = event->floor;
  }
  double x_current = CGAL::to_double(event->location.x());
  bool two_vertices = false;

  if (!first_vertex && !outer) {
    new_point = event->location2;
    new_point2 = event->location;
  } else {
    new_point = event->location;
    new_point2 = event->location2;
  }
  if (upper_vertices->size() == 0) {
    upper_point =
        calculateVertex(x_current, created_polygons_[polygon_nr_].front());
  } else {
    upper_point = calculateVertex(x_current, upper_vertices->back());
  }
  if (abs(event->location2.y() - event->location.y()) > kEps) {
    two_vertices = true;
  }
  if (abs(upper_point - new_point.y()) < kEps) {
    addEvent(two_vertices, new_point, direction1, new_point2, upper_vertices);
    updateEdgeList(upper_vertices->back().loc, direction1, event);
  } else {
    if (outer) {
      addEvent(two_vertices, new_point2, direction2, new_point,
               &created_polygons_[polygon_nr_]);
    } else {
      addEvent(two_vertices, new_point, direction2, new_point2,
               &created_polygons_[polygon_nr_]);
    }
    updateEdgeList(created_polygons_[polygon_nr_].back().loc, direction2,
                   event);
  }
}

void BCD::updateEdgeList(Point_2 location, Point_2 direction, Event* event) {
  size_t i = 0;
  while (i <= edge_list_.size()) {
    if ((abs(edge_list_[i].dir.x() - event->location.x()) < kEps &&
         abs(edge_list_[i].dir.y() - event->location.y()) < kEps) ||
        (abs(edge_list_[i].dir.x() - event->location2.x()) < kEps &&
         abs(edge_list_[i].dir.y() - event->location2.y()) < kEps)) {
      edge_list_[i].loc = location;
      edge_list_[i].dir = direction;
      event->closed = true;
      break;
    } else {
      i = i + 1;
    }
  }
}

void BCD::addEvent(bool two_vertices, Point_2 point1, Point_2 direction,
                   Point_2 point2, std::vector<Edge>* poly) {
  CHECK_NOTNULL(poly);

  Edge edge = {.loc = point1, .dir = direction};
  poly->push_back(edge);
  if (two_vertices) {
    Edge edge2 = {.loc = point2, .dir = direction};
    poly->push_back(edge2);
  }
}

}  // namespace mav_coverage_planning
