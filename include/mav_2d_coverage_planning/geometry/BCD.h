#ifndef BCD_H_
#define BCD_H_

#include <mav_coverage_planning_comm/cgal_definitions.h>

namespace mav_coverage_planning {

class BCD {
 public:
  struct Vertex {
    Point_2 location;
    Point_2 ceiling;
    Point_2 floor;
    bool merge_next;
    bool colour1;
    bool colour2;
  };
  
  struct Event {
    Point_2 location;
    Point_2 location2;
    Point_2 ceiling;
    Point_2 floor;
    bool closed;
    bool upper;
    bool lower;
    int type;
  };
  
  struct Edge {
    Point_2 loc;
    Point_2 dir;
  };
  
  BCD(const PolygonWithHoles& polygon);
  bool computeBCDFromPolygonWithHoles(std::vector<Polygon_2>& polygons);
  
 private:
  void removeDublicatedVeritices();
  void innerPolygonEnd(bool first_vertex, Event* event, 
        std::vector<Edge>& upper_vertices, int edge_upper, int edge_lower, 
        bool outer);
  void closeSecondEvent(double x_current, double y_event, bool upper);
  void updateEventStatus(Event& event, bool upper, double x_current, double y_event);
  void closePolygon(double x_current, std::vector<Edge>& upper_vertices, int edge_lower, int edge_upper);
  bool createEvents(Polygon_2& polygon, std::vector<Event>& events);
  void initVertex(Vertex& vertex, VertexConstCirculator orig_vertex);
  void initEvent(Event& event_list, Vertex vertex_now, Vertex vertex_next);
  bool find_next_event(bool& first_vertex, Event*& next_event, bool &outer, Edge edge1, Edge edge2);
  double calculateVertex(double x, Edge edge);
  void getEdges(int& edge_upper_number, int& edge_lower_number);
  void createVertices(bool first_vertex, Event* event, std::vector<Edge>& upper_vertices, bool outer);
  void updateEdgeList(Event* event, Point_2 location, Point_2 direction);
  void addEvent(std::vector<Edge>& poly, bool two_vertices, 
        Point_2 point1, Point_2 direction, Point_2 point2);
  bool find_next_event(bool& first_vertex, Event*& next_event, bool &outer); //Check this in MATLAB
  
  PolygonWithHoles polygon_;
  
  std::vector<Event> outer_events;
  
  std::vector<std::vector<Event>> all_inner_events;
  
  std::vector<Edge> edge_list;
  
  std::vector<std::vector<Edge>> created_polygons;
  
  int polygon_nr;
  double eps = 0.001;
};

}  // namespace mav_coverage_planning
#endif  // BCD_H_
