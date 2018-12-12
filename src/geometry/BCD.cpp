#include "mav_2d_coverage_planning/geometry/BCD.h"

BCD::BCD(const PolygonWithHoles& polygon): polygon_(polygon){
  polygon_nr = 0;
}

bool BCD::computeBCDFromPolygonWithHoles(std::vector<Polygon>* polygons) {
 
  createEvents(polygon_.outer_boundary(), &outer_events);
  for (PolygonWithHoles::Hole_const_iterator hole = polygon_.holes_begin();
       hole != polygon_.holes_end(); ++hole) {
    std::vector<event> inner_events;
    createEvents (hole, &inner_events);
    all_inner_events.push_back(inner_events);
  }
  
  //Start algorithm
  bool first_vertex = false;
  bool outer = false;
  std::vector<Edge> upper_vertices;
  
  Event* next_event;
  find_next_event([], [], first_vertex, next_event, outer);
  Edge edge = {.loc = next_event->location, .dir = next_event->floor};
  edge_list.push_back(edge);
  Edge edge = {.loc = next_event->location2, .dir = next_event->ceiling};
  edge_list.push_back(edge);
  next_event->closed = true; 
  
  while (!edge_list.empty()) {
    polygon_nr ++;
    Edge* edge_upper;
    Edge* edge_lower;
    getEdges(edge_upper, edge_lower);
    created_polygons.push_back(*edge_upper);
    created_polygons.push_back(*edge_lower);
    find_next_vertex(created_polygons[polygon_nr].front(), 
            created_polygons[polygon_nr].back(), first_event, 
            next_event, outer);
    double x_current = next_event->location.x;
    int type = next_event->type;
    upper_vertices.clear();
    while (type == 2) {
      if (upper_vertices.size() == 0) {
        find_next_vertex(created_polygons[polygon_nr].front(), 
                created_polygons[polygon_nr].back(), first_event,
                next_event, outer);
      } else {
        find_next_vertex(upper_vertices.back(), 
                created_polygons[polygon_nr].back(), 
                first_event, next_event, outer);
      }
      x_current = next_event->location.x;
      if (next_event->type == 2) {
        createVertices(first_vertex, next_event, upper_vertices, outer);
      }
      type = next_event->type;
    }
    innerPolygonEnd(first_vertex, next_event, 
            upper_vertices, edge_upper, edge_lower, outer);
  }
  removeDublicatedVeritices;
  polygons = &created_polygons;
}

void BCD::removeDublicatedVeritices() {
  for (size_t i = 0; i < created_polygons.size(); ++i) {
    std::vector<Edge>::iterator it;
    it = std::unique (created_polygons[i].begin(), created_polygons[i].end());  
    created_polygons[i].resize( std::distance(created_polygons[i].begin(),it) );
  }
}

void BCD::innerPolygonEnd(bool first_vertex, Event* event, 
        std::vector<Edge>& upper_vertices, Edge* edge_upper, Edge* edge_lower, 
        bool outer) {
    double x_current = event->location.x;  
    if (first_vertex) {
      Point_2 next_point = event.location;
    } else {
      Point_2 next_point = event.location2;
    }
    int edge_list_size = edge_list.size();
    if (upper_vertices.size() == 0) {
      Edge upper_vertex = created_polygons[polygon_nr].front();
    } else {
      Edge upper_vertex = upper_vertices.back();
    }
    Edge lower_vertex = created_polygons[polygon_nr].back();
    closePolygon(x_current, upper_vertices, edge_lower, edge_upper);
    if (abs(next_point.y - calculateVertex(x_current, upper_vertex))>eps) {
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
    Event* inner_events = all_inner_events[i].begin();
    for (size_t a = 0; a < inner_events.size(); ++a) {
      updateEventStatus(inner_events+a, upper, x_current, y_event); //Check this
    }
  }
  Event* outer_event = outer_events.begin();
  for (size_t i = 0; i<outer_events.size(); ++i) {
    updateEventStatus(outer_event+i, upper, x_current, y_event);
  }
}

void BCD::updateEventStatus(Event* event, bool upper, double x_current, double y_event) {
   if (abs(x_current - event -> location.x<eps&&
           ( abs(event -> location.y - y_event)<eps || 
           abs(event-> location2.y - y_event)<eps)) {
        if (upper) {
            event-> upper = true;
        } else {
            event-> lower = true;
        }
       if (event->upper&&event->lower) {
            event->closed = true;
       }
   }
}

void BCD::closePolygon(double x_current, std::vector<Edge>& upper_vertices, Edge* edge_lower, Edge* edge_upper) {
  //Close previous polygon
  Point_2 location(x_current,calculateVertex(x_current, created_polygons[polygon_nr].back()));
  Edge new_edge = {.loc = location, .dir = created_polygons[polygon_nr].back().dir};
  created_polygons[polygon_nr].push_back(new_edge);

  if (abs(edge_lower->dir.x - location.x)<eps && abs(edge_lower->dir.y - location.y)<eps) {
    if (edge_upper > edge_lower) {
      edge_upper = edge_upper-1;
    }
    edge_list.erase(edge_lower); //Check this
  } else {  
    edge_lower -> loc = location;
  }
  if (upper_vertices.size() == 0) {
    new_edge.loc.y = calculateVertex(x_current, created_polygons[polygon_nr].front()); 
    new_edge.dir = created_polygons[polygon_nr].front().dir;
  } else {
    new_edge.loc.y = calculateVertex(x_current, upper_vertices.back()); 
    new_edge.dir = upper_vertices.back().dir;
  }
  created_polygons[polygon_nr].push_back(new_edge);
  if (abs(edge_upper.dir.x - new_edge.loc.x)<eps && abs(edge_upper.dir.y - new_edge.loc.y)<eps) {
    edge_list.erase(edge_upper); //Check this
  } else {
    edge_upper -> loc = new_edge.loc;
  }
  for (size_t m = upper_vertices.size()-1; m >= 0; --i) {
    created_polygons[polygon_nr].push_back(upper_vertices[m]);
  }
}
   
void BCD::createEvents(Polygon& polygon, std::vector<Event>* events){
  CHECK_NOTNULL(events);
  events -> clear();
  if (polygon.is_counterclockwise_oriented()) { 
    polygon.reverse_orientation();
  } else if (polygon.is_collinear_oriented ()) {
    return false;
  }
  
  std::vector<Vertex> vertices; 
  
  for(Polygon_2::Vertex_const_iterator vi = polygon.vertices_begin();
       vi != polygon.vertices_end(); ++vi ) { 
    VertexConstCirculator vit = vi->vertices_circulator();
    Vertex vertex;
    initVertex(vertex, polygon, vit);
    vertices.push_back(vertex);
  }
  for(size_t i = 0; i < vertices.size(); ++i ) {
    if (!vertices[mod(i-1+vertices.size(),vertices.size())].merge_next) {
      Event event;
      if (vertices[i].merge_next) {
        initEvent(event, vertices[i], vertices[mod(i+1,vertices.size())]);
      } else {
        initEvent(event, vertices[i], vertices[i];
      }
      events -> push_back(event);
    }
  }
}

void BCD::initVertex(Vertex& vertex, VertexConstCirculator orig_vertex) { //Check syntax
  vertex.location = orig_vertex -> point(); 
  vertex.ceiling = std::next(orig_vertex) -> point();
  vertex.floor = std::prev(orig_vertex) -> point();
  vertex.merge_next = false;
  vertex.ceiling.x >= vertex.location.x ? 
    vertex.colour2 = 0 : vertex.colour2 = 1;
  vertex.floor.x > vertex.location.x ? 
    vertex.colour1 = 1 : vertex.colour1 = 0;
  if (abs(vertex.ceiling.x - vertex.location.x) < eps) {
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

bool BCD::find_next_event(Edge edge1, Edge edge2, bool& first_vertex, Event*& next_event, bool &outer) {
  next_event = nullptr;
  if (!edge1.empty()) {
      double x_search = min(edge1.dir.x, edge2.dir.x)+2*eps;
      double x_min = max(vertex1.loc.x, edge2.loc.x)-eps;
  } else {
      double x_search = inf;
      double x_min = -inf;
      double y1 = inf;
      double y2 = -inf;
  }
  int type = -1;
  for (size_t i=0; i<outer_events.size();++i) {
    if (outer_events[i].location.x >= x_min && 
            outer_events[i].location.x <= x_search+eps && 
            !outer_events[i].closed) {
      if (outer_events[i].type > type || 
              outer_events[i].location.x < x_search-eps) {
        if (!edge1.empty()) {
          double y1 = calculateVertex(outer_event[i].location.x, 
                  edge1);
          double y2 = calculateVertex(outer_event[i].location.x, 
                  edge2);
        }
        if ((outer_events[i].location.y<=y1+eps && 
                outer_events[i].location.y>=y2-eps) || 
                (outer_events[i].location2.y<=y1+eps && 
                outer_events[i].location2.y>=y2-eps)) {
          x_search= outer_events[i].location.x;
          type = outer_events[i].type;
          next_event = &outer_events[i];
          if (outer_events[i].location.y<=y1+eps && 
                  outer_events[i].location.y>=y2-eps) {
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
    std::vector<event> inner_events = all_inner_events[a];
    for (size_t i=0; i < inner_events.size();++i) {
      if (inner_events[i].location.x >= x_min && 
              inner_events[i].location.x <= x_search+eps && 
              !inner_events[i].closed) {
        if ((inner_events[i].type > type) || 
                inner_events[i].location.x < x_search-eps ) {
          if (!edge1.empty()) {
            double y1 = calculateVertex(inner_events[i].location.x, 
                    edge1);
            double y2 = calculateVertex(inner_events[i].location.x, 
                    edge2);
          }
          if ((inner_events[i].location.y<=y1+eps && 
                  inner_events[i].location.y>=y2-eps)||
                  (inner_events[i].location2.y<=y1+eps && 
                  inner_events[i].location2.y>=y2-eps)) {
            next_event = &inner_events[i];
            type = inner_events[i].type;
            x_search= inner_events[i].location.x;
            
            if ((inner_events[i].location.y<=y1+eps && 
                    inner_events[i].location.y>=y2-eps)&&
                    (inner_events[i].location2.y<=y1+eps && 
                    inner_events[i].location2.y>=y2-eps)) {
              if (std::min(abs(inner_events[i].location.y-y1),
                      abs(inner_events[i].location.y-y2)) < 
                      min(abs(inner_events[i].location2.y-y1),
                      abs(inner_events[i].location2.y-y2))){
                  first_vertex = true;
              } else {
                  first_vertex = false;
              }
            } else if (inner_events[i].location.y<=y1+eps && 
                    inner_events[i].location.y>=y2-eps) {
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

double BCD::calculateVertex(double x, Edge edge) {
  
  double x_delta = x-edge.loc.x;
  Point_2 vector = edge.dir-edge.loc;
  double frac = x_delta/vector.x;
  double y_res = edge.loc.y+vector.y*frac;
  
  return y_res;
}

void BCD::getEdges(Edge*& edge_upper, Edge*& edge_lower) {
  double inf = std::numeric_limits<double>::max();
  Point_2 value1(inf, inf); 
  Point_2 value2(inf, inf);
  Edge edge = {.loc = value1, .dir = value2};
  
  int edge_lower_number = -1;
  int edge_upper_number = -1;
  for (size_t i = 0; i < edge_list.size(), ++i) {
    if (edge_list[i].loc.x < edge.loc.x-eps) {
      edge.loc.x = edge_list[i].loc.x;
    }
  }
  bool add_more = true;
  while (add_more) {
    Event* next_event;
    bool first_vertex;
    find_next_event([], [], first_vertex, next_event);
    x_now = next_event -> location.x;
    if (next_event.type == 0 && edge.loc.x >= x_now - eps) {
      Edge edge = {.loc = next_event->location, .dir = next_event->floor};
      edge_list.push_back(edge);
      Edge edge = {.loc = next_event->location2, .dir = next_event->ceiling};
      edge_list.push_back(edge);
      next_event->closed = true;
      edge.loc.x = x_now;
    } else { 
        add_more = false;
    }
  }
  for (size_t i= 0; i< edge_list.size(); ++i) {
    if (edge_lower_number >= 0) {
      edge.dir.y = calculateVertex(edge_list[i].dir.x, edge);
    }
    if ((abs(edge_list[i].loc.x-edge.loc.x)<eps && 
            edge.loc.y > edge_list[i].loc.y+eps)|| 
            (abs(edge_list[i].loc.x-edge.loc.x)<eps && 
            abs(edge.loc.y-edge_list[i].loc.y)<eps && 
            edge.dir.y > edge_list[i].dir.y+eps)) {
        edge_lower_number = i;
        edge = edge_list[i];
    }
  }
  
  edge.loc.y = inf;
  edge.dir.x = inf;
  edge.dir.y = inf;
  
  for (size_t i= 0; i< edge_list.size(); ++i) {
    if (edge_upper_number >= 0) {
      edge.dir.y = calculateVertex(edge_list[i].dir.x, edge);
    }
    if ((abs(edge_list[i].loc.x-edge.loc.x)<eps && i ~= edge_lower_number) && (edge.loc.y > edge_list[i].loc.y+eps|| (abs(edge.loc.y-edge_list[i].loc.y)<eps && edge.dir.y > edge_list[i].dir.y+eps))) {
      edge_upper_number = i;
      edge = edge_list[i];
    }
  }
  edge_upper = &edge_list[edge_upper_number];
  edge_lower = &edge_list[edge_lower_number];
}

void BCD::createVertices(bool first_vertex, Event* event, std::vector<Edge>& upper_vertices, bool outer){
  if (!outer) {
    Point_2 direction1 = event -> floor;
    Point_2 direction2 = event -> ceiling;
  } else {
    Point_2 direction1 = event -> ceiling;
    Point_2 direction2 = event -> floor;
  }
  double x_current = event->location.x;
  bool two_vertices = false;
  
  if (!first_vertex && !outer) {
    Point_2 new_point = event -> location2;
    Point_2 new_point2 = event -> location;
  } else {
    Point_2 new_point = event -> location;
    Point_2 new_point2 = event-> location2;
  }
  if (upper_vertices.size() == 0) {
    Point_2 upper_point = calculateVertex(x_current, created_polygons[polygon_nr].front());
  } else {
    Point_2 upper_point = calculateVertex(x_current, upper_vertices.back());
  }
  int upper_number = upper_vertices.size();
  if (abs(event -> location2.y - event -> location.y > eps) {
    two_vertices = true;
  }
  if (abs(upper_point - new_point.y < eps) {
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
  int i = 0;
  while (i <= edge_list.size()) {
    if((abs(edge_list[i].dir.x - event -> location.x) < eps && 
            abs(edge_list[i].dir.y - event -> location.y)<eps) || 
            (abs(edge_list[i].dir.x - event -> location2.x) <eps && 
            abs(edge_list[i].dir.y - event-> location2.y)<eps)) {
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
  Edge edge = {.loc = point1, .dir = directon};
  poly.push_back(edge);
  if (two_vertices) {
    Edge edge2 = {.loc = point2, .dir = directon};
    poly.push_back(edge2);
  }
}





