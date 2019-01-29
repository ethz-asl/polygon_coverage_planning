#ifndef BCD_H_
#define BCD_H_

#include <mav_coverage_planning_comm/cgal_definitions.h>

namespace mav_coverage_planning {

class BCD {
 public:
  struct Vertex {
    Point_2 location; //Location of the vertex
    Point_2 ceiling; //Location of the next vertex, CW
    Point_2 floor; //Location of the next vertex, CCW
    bool merge_next; //Indicates if the next vertex, CW has same x-value
    bool colour1;
    bool colour2;
  };
  
  struct Event {
    Point_2 location; //Location of vertex
    Point_2 location2; //Location of second vertex, if applicable
    Point_2 ceiling; 
    Point_2 floor;
    bool closed; //Indicates if event has been processed
    bool upper; //Indicates if upper side of event has been processed
    bool lower; //Indicates if lower side of event has been processed
    int type; //0: opening, 1:closing, 2:middle
  };
  
  struct Edge {
    Point_2 loc;
    Point_2 dir;
  };
  
  BCD(const PolygonWithHoles& polygon);
  
  // Computes the BCD from a PolygonWithHoles by saving the polyons of the 
  // decomposition in 'polygons'.
  bool computeBCDFromPolygonWithHoles(std::vector<Polygon_2>& polygons);
  
 private:
   
  //Removes dublicated vertices in polygons in created_polygons 
  void removeDublicatedVeritices();
  
  // Closes the current polygon and updates edge list by deleting the edge from
  // the list or updating the first vertex of the edge.
  void innerPolygonEnd(bool first_vertex, Event* event, 
        std::vector<Edge>& upper_vertices, int edge_upper, int edge_lower, 
        bool outer);
  
  // Ensures that also the second event is closed that bounds the newly
  // created polygon if they have the same x-value.
  void closeSecondEvent(double x_current, double y_event, bool upper);
  
  // Updates the event status (closed, upper and lower)
  void updateEventStatus(Event& event, bool upper, 
                          double x_current, double y_event);
  
  // Close the previous polygon at x_current and add all vertices in
  // upper_vertices to the polygon CCW
  void closePolygon(double x_current, std::vector<Edge>& upper_vertices, 
                    int edge_lower, int edge_upper);
  
  // Called once at beginning of algorithm. Sorts all verteces into 
  // events of 'opening', 'closing' and 'middle'
  bool createEvents(Polygon_2& polygon, std::vector<Event>& events);
  
  // Called by createEvents. Initializes all vertex parameters.
  void initVertex(Vertex& vertex, VertexConstCirculator orig_vertex);
  
  // Called by createEvents. Initializes all event parameters.
  void initEvent(Event& event_list, Vertex vertex_now, Vertex vertex_next);
  
  // Finds the vertex with the lowest x-value between edge1 and edge2.
  bool find_next_event(bool& first_vertex, Event*& next_event, bool &outer, 
                      Edge edge1, Edge edge2);
  
  //Find y-value on edge for given x-value
  double calculateVertex(double x, Edge edge);
  
  // Adds all opening events to edge list with lower or equal x-value than the
  // current leftmost vertex. All opening events are added twice, one in each
  // direction. The values edge upper number and edge lower number indicate the
  // edges that open the next polygon. This next polygon is opened between the
  // two vertices with the lowest x- and y-values.
  void getEdges(int& edge_upper_number, int& edge_lower_number);
  
  // Adds the event to the created polygons and updates the edge list such that
  // only the open edges are included.
  void createVertices(bool first_vertex, Event* event, 
                      std::vector<Edge>& upper_vertices, bool outer);
  // Called by createVertices. Updates the edge list such that
  // only the open edges are included.
  
  void updateEdgeList(Event* event, Point_2 location, Point_2 direction);
  
  // Called by createVertices. Adds the event to the created polygons.
  void addEvent(std::vector<Edge>& poly, bool two_vertices, 
        Point_2 point1, Point_2 direction, Point_2 point2);
  
  // Finds the vertex with the lowest x-value.
  bool find_next_event(bool& first_vertex, Event*& next_event, bool &outer); 
  
  // Rotates polygon_ with angle alpha.
  PolygonWithHoles rotPolygon(double alpha);
  
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
