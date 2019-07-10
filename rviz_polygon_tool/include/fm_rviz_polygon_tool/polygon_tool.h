#ifndef FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/connect_holes.h>
#include <OgreColourValue.h>
#include <OgreVector3.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/vector_property.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

namespace mav_polygon_tool {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef std::list<Point> Point_list;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_2_WH;
typedef std::list<Polygon_2_WH> Poly_wh_list;
typedef Polygon_2::Vertex_iterator VertexIterator;

// Declare polygon tool as subclass of rviz::Tool.

/*
This tool allows the user to construct a polygon in 2D in rviz
On start up the user can insert nodes that will form the corners of the polygon
To be considered valid, a polygon must have at least 3 corner points.
The polygon must also be simple, that means it's edges dont intersect.
The user can also add multiple polygons or holes.
To switch between different polygons or holes the toolSelectCallback() is used.
To create a new polygon or hole the newPolyCallback() is used.
To delete an existing polygon the deletePolyCallback() is used.
Please note that the original polygon can't be delted.
              The first object will always be a polygon.
Multiple polygons along with holes can be fused into one global
polygon wiht holes called main_polygon_.
When fusing the following rules apply:
1) All polygons must at some point overlap with an other one such that the
    hull of the combined polygon is simple.
2) When holes overlap with the outer hull, they are integrated into the outer
   hull and deleted
Fusing the polygons is done by calling checkStatusCallback()
Once the fusing has been performed the polygon with holes is published using a
custom defined ros message.
This is done by triggering the polygonPublisherCallback()
The actual publishing is done by the polygon_wh_publisher_ publisher
*/
class PolygonTool : public rviz::Tool {
  Q_OBJECT

 public:
  PolygonTool();
  virtual ~PolygonTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

 private:
  enum PolygonType { kHull = 0, kHole };
  // GUI related
  void leftClicked(rviz::ViewportMouseEvent& event);
  void rightClicked(rviz::ViewportMouseEvent& event);
  void makeVertex(const Ogre::Vector3& position);
  void deletePolygon(size_t index);
  bool checkCGalPolygon();

  bool is_activated_ = false;
  bool check_performed_ = false;
  bool global_planning_ = false;
  size_t current_polygon_ = 0;
  int current_type_;
  std::vector<int> type_of_polygons_;
  std::vector<Polygon_2> polygon_;
  Polygon_2_WH main_polygon_;
  std::vector<std::list<Point>> points_for_poly_;

  // color related
  void makeHoleVertex(const Ogre::Vector3& position);
  void pushBackElements(int new_type);
  void setColor(const Ogre::ColourValue& line_color,
                const Ogre::ColourValue& sphere_color);
  void drawLines(const Ogre::ColourValue& line_color);
  void drawPolyWithHoles(const Polygon_2_WH& to_be_painted,
                         const Ogre::ColourValue color);
  void clearGlobalPlanning();
  void hideIndividualPolygons();
  void showIndividualPolygons();
  void setColorsLeaving();
  void setColorsArriving();

  std::vector<std::vector<rviz::Shape*>> active_spheres_;
  std::vector<std::vector<Ogre::SceneNode*>> vertex_nodes_;
  Ogre::SceneNode* moving_vertex_node_;
  rviz::VectorProperty* current_vertex_property_;
  std::vector<std::vector<rviz::Line*>> active_lines_;
  std::vector<rviz::Shape*> outer_boundary_;
  std::vector<rviz::Line*> outer_boundary_lines_;
  std::vector<std::vector<rviz::Shape*>> inner_pts_;
  std::vector<std::vector<rviz::Line*>> inner_lines_;
  // Point displayed.
  rviz::Shape* vertex_;
  // point scale
  const float kPtScale = 0.8;
  const float kDeleteTol = 0.8;

  // ROS messaging
  ros::NodeHandle nh_;
  ros::Subscriber new_tool_subs_;
  ros::Subscriber delete_poly_subs_;
  ros::Subscriber selector_subs_;
  ros::Subscriber check_poly_subs_;
  ros::Subscriber trigger_polygon_subs_;

  ros::Publisher status_update_publisher_;
  ros::Publisher user_warn_publisher_;
  ros::Publisher polygon_wh_publisher_;

  void checkStatusCallback(const std_msgs::Bool& incomming);
  void toolSelectCallback(const std_msgs::Int8& tool_num);
  void newPolyCallback(const std_msgs::Int8& new_poly_num);
  void deletePolyCallback(const std_msgs::Int8& delete_ind);
  void polygonPublisherCallback(const std_msgs::Bool& incomming);
};

}  // namespace mav_polygon_tool

#endif  // FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
