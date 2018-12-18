#include "mav_2d_coverage_planning/geometry/BCD.h"
#include <glog/logging.h>

namespace mav_coverage_planning {
BCD::BCD(const PolygonWithHoles& polygon): polygon_(polygon){
  polygon_nr = 0;
  //TODO(luliu): how do I write this without this loop?
  int size = 0;
  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi) {
    size++;
  }
  //Reserve 2 per polygon plus outer boundary.
  edge_list.reserve((size+1)*2);
  created_polygons.reserve(size*4);
}

bool BCD::computeBCDFromPolygonWithHoles(std::vector<Polygon_2>& polygons) {
  createEvents(polygon_.outer_boundary(), outer_events);
  for (PolygonWithHoles::Hole_const_iterator hi = polygon_.holes_begin();
       hi != polygon_.holes_end(); ++hi) {
    std::vector<Event> inner_events;
    Polygon_2 hole = *hi;
    createEvents (hole, inner_events);
    all_inner_events.push_back(inner_events);
  }
  //Start algorithm
  bool first_vertex = false;
  bool outer = false;
  std::vector<Edge> upper_vertices;
  
  Event* next_event;
  find_next_event(first_vertex, next_event, outer);
  Edge edge = {.loc = next_event->location, .dir = next_event->floor};
  edge_list.push_back(edge);
  Edge edge2 = {.loc = next_event->location2, .dir = next_event->ceiling};
  edge_list.push_back(edge2);
  next_event->closed = true; 
  while (!edge_list.empty()) {
    int edge_upper;
    int edge_lower;
    getEdges(edge_upper, edge_lower);
    std::vector<Edge> new_polygon;
    new_polygon.push_back(edge_list[edge_upper]);
    new_polygon.push_back(edge_list[edge_lower]);
    created_polygons.push_back(new_polygon);
    find_next_event(first_vertex, 
            next_event, outer, created_polygons[polygon_nr].front(), 
            created_polygons[polygon_nr].back());
    int type = next_event->type;
    upper_vertices.clear();
    while (type == 2) {
      if (upper_vertices.size() == 0) {
        find_next_event(first_vertex,
                next_event, outer, created_polygons[polygon_nr].front(), 
                created_polygons[polygon_nr].back());
      } else {
        find_next_event(first_vertex, next_event, outer, upper_vertices.back(), 
                created_polygons[polygon_nr].back());
      }
      if (next_event->type == 2) {
        createVertices(first_vertex, next_event, upper_vertices, outer);
      }
      type = next_event->type;
    }
    innerPolygonEnd(first_vertex, next_event, 
            upper_vertices, edge_upper, edge_lower, outer);
    polygon_nr ++;
  }
  removeDublicatedVeritices();
  polygons.clear();
  polygons.reserve(created_polygons.size());
  Polygon_2 polygon;
  int i = 0;
  for(std::vector<std::vector<Edge>>::iterator it = created_polygons.begin(); it != created_polygons.end(); ++it) {
    polygon.clear();
    for(std::vector<Edge>::iterator poly = it->begin(); poly != it->end(); ++poly) {
      LOG(WARNING) << "Poly"<<i<<" x: "<<poly->loc.x()<<" y: "<<poly->loc.y();
      polygon.push_back(poly->loc);
    }
    polygons.emplace_back(polygon);
    i++;
  }
  return true;
}

void BCD::removeDublicatedVeritices() {
  for (size_t i = 0; i < created_polygons.size(); ++i) {
    std::vector<Edge>::iterator first = created_polygons[i].begin();
    std::vector<Edge>::iterator last = created_polygons[i].end();
    std::vector<Edge>::iterator result = first;
    while (++first != last) {
        if (!(result->loc == first->loc) && ++result != first) {
            *result = std::move(*first);
        }
    }
    created_polygons[i].erase(++result, created_polygons[i].end()); 
  }
}

void BCD::innerPolygonEnd(bool first_vertex, Event* event, 
        std::vector<Edge>& upper_vertices, int edge_upper, int edge_lower, 
        bool outer) {
    double x_current = CGAL::to_double(event->location.x());
    Point_2 next_point;
    Edge upper_vertex;
    if (first_vertex) {
      next_point = event->location;
    } else {
      next_point = event->location2;
    }
    size_t edge_list_size = edge_list.size();
    if (upper_vertices.size() == 0) {
      upper_vertex = created_polygons[polygon_nr].front();
    } else {
      upper_vertex = upper_vertices.back();
    }
    Edge lower_vertex = created_polygons[polygon_nr].back();
    closePolygon(x_current, upper_vertices, edge_lower, edge_upper);
    if (abs(next_point.y() - calculateVertex(x_current, upper_vertex))>eps) {
      if (edge_list.size() < edge_list_size - 1) {
        closeSecondEvent(x_current,calculateVertex(x_current, upper_vertex), false);
      }
      event -> upper = true;
    } else {
      if (edge_list.size() < edge_list_size - 1) {
        closeSecondEvent(x_current, calculateVertex(x_current, lower_vertex), true);
      }
      event -> lower = true;
    }
    if (event -> upper && event->lower) {
        event -> closed = true;
    }
}

void BCD::closeSecondEvent(double x_current, double y_event, bool upper) {
  for (size_t i = 0; i<all_inner_events.size();++i) {
    for (size_t a = 0; a < all_inner_events[i].size(); ++a) {
      updateEventStatus(all_inner_events[i][a], upper, x_current, y_event);
    }
  }
  for (size_t i = 0; i<outer_events.size(); ++i) {
    updateEventStatus(outer_events[i], upper, x_current, y_event);
  }
}

void BCD::updateEventStatus(Event& event, bool upper, double x_current, double y_event) {
   if (abs(x_current - event.location.x())<eps&&
           ( abs(event.location.y() - y_event)<eps || 
           abs(event.location2.y() - y_event)<eps)) {
        if (upper) {
            event.upper = true;
        } else {
            event.lower = true;
        }
       if (event.upper&&event.lower) {
            event.closed = true;
       }
   }
}

void BCD::closePolygon(double x_current, std::vector<Edge>& upper_vertices, int edge_lower, int edge_upper) {
  //Close previous polygon
  Point_2 location(x_current,calculateVertex(x_current, created_polygons[polygon_nr].back()));
  Edge new_edge = {.loc = location, .dir = created_polygons[polygon_nr].back().dir};
  created_polygons[polygon_nr].push_back(new_edge);
  if (abs(edge_list[edge_lower].dir.x() - location.x())<eps && abs(edge_list[edge_lower].dir.y() - location.y())<eps) {
    if (edge_upper > edge_lower) {
      edge_upper--;
    }
    edge_list.erase(edge_list.begin()+edge_lower); //Check this
  } else {  
    edge_list[edge_lower].loc = location;
  }
  if (upper_vertices.size() == 0) {
    Point_2 new_location(x_current, calculateVertex(x_current, created_polygons[polygon_nr].front()));
    new_edge.loc = new_location; 
    new_edge.dir = created_polygons[polygon_nr].front().dir;
  } else {
    Point_2 new_location(x_current, calculateVertex(x_current, upper_vertices.back()));
    new_edge.loc = new_location; 
    new_edge.dir = upper_vertices.back().dir;
  }
  created_polygons[polygon_nr].push_back(new_edge);
  if (abs(edge_list[edge_upper].dir.x() - new_edge.loc.x())<eps && abs(edge_list[edge_upper].dir.y() - new_edge.loc.y())<eps) {
    edge_list.erase(edge_list.begin()+edge_upper);
  } else {
    edge_list[edge_upper].loc = new_edge.loc;
  }
  for (std::vector<Edge>::reverse_iterator i = upper_vertices.rbegin(); 
        i != upper_vertices.rend(); ++i ) { 
    created_polygons[polygon_nr].push_back(*i);
  }
}
   
bool BCD::createEvents(Polygon_2& polygon, std::vector<Event>& events){
  events.clear();
  if (polygon.is_counterclockwise_oriented()) { 
    polygon.reverse_orientation();
  } else if (polygon.is_collinear_oriented ()) {
    return false;
  }
  std::vector<Vertex> vertices; 
  VertexConstCirculator vit = polygon.vertices_circulator();
  do {
    Vertex vertex;
    initVertex(vertex, vit);
    vertices.push_back(vertex);
  } while (++vit != polygon.vertices_circulator());
  
  for(size_t i = 0; i < vertices.size(); ++i ) {
    if (!vertices[(i-1+vertices.size())%vertices.size()].merge_next) {
      Event event;
      if (vertices[i].merge_next) {
        initEvent(event, vertices[i], vertices[(i+1)%vertices.size()]);
      } else {
        initEvent(event, vertices[i], vertices[i]);
      }
      events.push_back(event);
    }
  }
  return true;
}

void BCD::initVertex(Vertex& vertex, VertexConstCirculator orig_vertex) {
  vertex.location = *orig_vertex; 
  vertex.ceiling = *std::next(orig_vertex);
  vertex.floor = *std::prev(orig_vertex);
  vertex.merge_next = false;
  vertex.ceiling.x() >= vertex.location.x() ? 
    vertex.colour2 = 0 : vertex.colour2 = 1;
  vertex.floor.x() > vertex.location.x() ? 
    vertex.colour1 = 1 : vertex.colour1 = 0;
  if (abs(vertex.ceiling.x() - vertex.location.x()) < eps) {
     vertex.merge_next = true;
  }
}

void BCD::initEvent(Event& event, Vertex vertex_now, Vertex vertex_next) {
  event.location = vertex_now.location;
  event.location2 = vertex_next.location;
  event.closed = false;
  event.upper = false;
  event.lower = false;
  event.ceiling = vertex_next.ceiling;
  event.floor = vertex_now.floor;
  if (vertex_now.colour1 == 0 && vertex_next.colour2 == 1) {
      event.type = 1;
  } else if (vertex_now.colour1 == 1 && vertex_next.colour2 == 0) {
      event.type = 0;
  } else {
      event.type = 2;
  }
}

bool BCD::find_next_event(bool& first_vertex, Event*& next_event, bool &outer, Edge edge1, Edge edge2) {
  next_event = nullptr;
  double x_search =  CGAL::to_double(min(edge1.dir.x(), edge2.dir.x()))+2*eps;
  double x_min =  CGAL::to_double((max(edge1.loc.x(), edge2.loc.x())))-eps;
  int type = -1;
  for (size_t i=0; i<outer_events.size();++i) {
    if (outer_events[i].location.x() >= x_min && 
            outer_events[i].location.x() <= x_search+eps && 
            !outer_events[i].closed) {
      if (outer_events[i].type > type || 
              outer_events[i].location.x() < x_search-eps) {
          double y1 = calculateVertex(CGAL::to_double(outer_events[i].location.x()), 
                  edge1);
          double y2 = calculateVertex(CGAL::to_double(outer_events[i].location.x()), 
                  edge2);
        if ((outer_events[i].location.y()<=y1+eps && 
                outer_events[i].location.y()>=y2-eps) || 
                (outer_events[i].location2.y()<=y1+eps && 
                outer_events[i].location2.y()>=y2-eps)) {
          x_search= CGAL::to_double(outer_events[i].location.x());
          type = outer_events[i].type;
          next_event = &outer_events[i];
          if (outer_events[i].location.y()<=y1+eps && 
                  outer_events[i].location.y()>=y2-eps) {
              first_vertex = true;
          } else {
              first_vertex = false;
          }
          outer = true;
        }
      }
    }
  }
  for (size_t a=0; a < all_inner_events.size();++a) {
    std::vector<Event> inner_events = all_inner_events[a];
    for (size_t i=0; i < inner_events.size();++i) {
      if (inner_events[i].location.x() >= x_min && 
              inner_events[i].location.x() <= x_search+eps && 
              !inner_events[i].closed) {
        if ((inner_events[i].type > type) || 
                inner_events[i].location.x() < x_search-eps ) {
            double y1 = calculateVertex(CGAL::to_double(inner_events[i].location.x()), 
                    edge1);
            double y2 = calculateVertex(CGAL::to_double(inner_events[i].location.x()), 
                    edge2);
          if ((inner_events[i].location.y()<=y1+eps && 
                  inner_events[i].location.y()>=y2-eps)||
                  (inner_events[i].location2.y()<=y1+eps && 
                  inner_events[i].location2.y()>=y2-eps)) {
            next_event = &all_inner_events[a][i];
            type = inner_events[i].type;
            x_search= CGAL::to_double(inner_events[i].location.x());
            
            if ((inner_events[i].location.y()<=y1+eps && 
                    inner_events[i].location.y()>=y2-eps)&&
                    (inner_events[i].location2.y()<=y1+eps && 
                    inner_events[i].location2.y()>=y2-eps)) {
              if (std::min(abs(inner_events[i].location.y()-y1),
                      abs(inner_events[i].location.y()-y2)) < 
                      min(abs(inner_events[i].location2.y()-y1),
                      abs(inner_events[i].location2.y()-y2))){
                  first_vertex = true;
              } else {
                  first_vertex = false;
              }
            } else if (inner_events[i].location.y()<=y1+eps && 
                    inner_events[i].location.y()>=y2-eps) {
                first_vertex = true;
            } else {
                first_vertex = false;
            }
            outer = false;
          }
        }
      }
    }
  }
  if (next_event) {
    return true;
  } else {
    return false;
  }    
}

bool BCD::find_next_event(bool& first_vertex, Event*& next_event, bool &outer) {
  next_event = nullptr;
  double inf = std::numeric_limits<double>::max();
  double x_search = inf;
  int type = -1;
  for (size_t i=0; i<outer_events.size();++i) {
    if (outer_events[i].location.x() <= x_search+eps && 
            !outer_events[i].closed) {
      if (outer_events[i].type > type || 
              outer_events[i].location.x() < x_search-eps) {
          x_search= CGAL::to_double(outer_events[i].location.x());
          type = outer_events[i].type;
          next_event = &outer_events[i];
          first_vertex = true;
          outer = true;
      }
    }
  }
  for (size_t a=0; a < all_inner_events.size();++a) {
    std::vector<Event> inner_events = all_inner_events[a];
    for (size_t i=0; i < inner_events.size();++i) {
      if (inner_events[i].location.x() <= x_search+eps && 
              !inner_events[i].closed) {
        if ((inner_events[i].type > type) || 
                inner_events[i].location.x() < x_search-eps ) {
            next_event = &all_inner_events[a][i];
            type = inner_events[i].type;
            x_search= CGAL::to_double(inner_events[i].location.x());
            first_vertex = true;
            outer = false;
        }
      }
    }
  }
  if (next_event) {
    return true;
  } else {
    return false;
  }    
}

double BCD::calculateVertex(double x, Edge edge) {
  
  double x_delta = x-CGAL::to_double(edge.loc.x());
  Vector_2 vector = edge.dir-edge.loc;
  double frac = x_delta/CGAL::to_double(vector.x());
  double y_res = CGAL::to_double(edge.loc.y()+vector.y()*frac);
  
  return y_res;
}

void BCD::getEdges(int& edge_upper_number, int& edge_lower_number) {
  double inf = std::numeric_limits<double>::max();
  double value_loc_x = inf;
  double value_dir_y = inf;
  edge_lower_number = -1;
  edge_upper_number = -1;
  for (size_t i = 0; i < edge_list.size(); ++i) {
    if (edge_list[i].loc.x() < value_loc_x-eps) {
      value_loc_x = CGAL::to_double(edge_list[i].loc.x());
    }
  }
  bool add_more = true;
  while (add_more) {
    Event* next_event;
    bool first_vertex;
    bool outer;
    find_next_event(first_vertex, next_event, outer);
    double x_now = CGAL::to_double(next_event -> location.x());
    if (next_event->type == 0 && value_loc_x >= x_now - eps) {
      Edge edge1 = {.loc = next_event->location, .dir = next_event->floor};
      edge_list.push_back(edge1);
      Edge edge2 = {.loc = next_event->location2, .dir = next_event->ceiling};
      edge_list.push_back(edge2);
      next_event->closed = true;
      value_loc_x = x_now;
    } else { 
        add_more = false;
    }
  }
  Point_2 location(value_loc_x, inf);
  Point_2 direction(inf, inf);
  Edge edge = {.loc = location, .dir = direction};
  for (size_t i= 0; i< edge_list.size(); ++i) {
    if (edge_lower_number >= 0) {
      value_dir_y = calculateVertex( CGAL::to_double(edge_list[i].dir.x()), edge);
    }
    if ((abs(edge_list[i].loc.x()-edge.loc.x())<eps && 
            edge.loc.y() > edge_list[i].loc.y()+eps)|| 
            (abs(edge_list[i].loc.x()-edge.loc.x())<eps && 
            abs(edge.loc.y()-edge_list[i].loc.y())<eps && 
            value_dir_y > edge_list[i].dir.y()+eps)) {
        edge_lower_number = i;
        edge = edge_list[i];
    }
  }
  edge = {.loc = location, .dir = direction};
  value_dir_y = inf;
  for (size_t i= 0; i< edge_list.size(); ++i) {
    if (edge_upper_number >= 0) {
      value_dir_y = calculateVertex( CGAL::to_double(edge_list[i].dir.x()), edge);
    }
    if ((abs(edge_list[i].loc.x()-edge.loc.x())<eps && static_cast<int>(i) != edge_lower_number) 
            && (edge.loc.y() > edge_list[i].loc.y()+eps|| 
            (abs(edge.loc.y()-edge_list[i].loc.y())<eps && 
            value_dir_y > edge_list[i].dir.y()+eps))) {
      edge_upper_number = i;
      edge = edge_list[i];
    }
  }
}

void BCD::createVertices(bool first_vertex, Event* event, std::vector<Edge>& upper_vertices, bool outer){
  double upper_point;
  Point_2 new_point;
  Point_2 new_point2;
  Point_2 direction1;
  Point_2 direction2;
  if (!outer) {
    direction1 = event -> floor;
    direction2 = event -> ceiling;
  } else {
    direction1 = event -> ceiling;
    direction2 = event -> floor;
  }
  double x_current = CGAL::to_double(event->location.x());
  bool two_vertices = false;
  
  if (!first_vertex && !outer) {
    new_point = event -> location2;
    new_point2 = event -> location;
  } else {
    new_point = event -> location;
    new_point2 = event-> location2;
  }
  if (upper_vertices.size() == 0) {
    upper_point = calculateVertex(x_current, created_polygons[polygon_nr].front());
  } else {
    upper_point = calculateVertex(x_current, upper_vertices.back());
  }
  if (abs(event -> location2.y() - event -> location.y()) > eps) {
    two_vertices = true;
  }
  if (abs(upper_point - new_point.y()) < eps) {
    addEvent(upper_vertices, two_vertices, new_point, direction1, new_point2);
    updateEdgeList(event, upper_vertices.back().loc, direction1);
  } else {
    if (outer) {
      addEvent(created_polygons[polygon_nr], two_vertices, 
              new_point2, direction2, new_point);
    } else {
      addEvent(created_polygons[polygon_nr], two_vertices, 
              new_point, direction2, new_point2);
    }
    updateEdgeList(event, created_polygons[polygon_nr].back().loc, direction2);
  }
}

void BCD::updateEdgeList(Event* event, Point_2 location, Point_2 direction) {
  size_t i = 0;
  while (i <= edge_list.size()) {
    if((abs(edge_list[i].dir.x() - event -> location.x()) < eps && 
            abs(edge_list[i].dir.y() - event -> location.y())<eps) || 
            (abs(edge_list[i].dir.x() - event -> location2.x()) <eps && 
            abs(edge_list[i].dir.y() - event-> location2.y())<eps)) {
      edge_list[i].loc = location;
      edge_list[i].dir = direction;
      event -> closed = true;
      break;
    } else {
      i = i+1;
    }
  }
}

void BCD::addEvent(std::vector<Edge>& poly, bool two_vertices, 
        Point_2 point1, Point_2 direction, Point_2 point2) {
  Edge edge = {.loc = point1, .dir = direction};
  poly.push_back(edge);
  if (two_vertices) {
    Edge edge2 = {.loc = point2, .dir = direction};
    poly.push_back(edge2);
  }
}



}

