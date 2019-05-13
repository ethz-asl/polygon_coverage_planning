#ifndef FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <OgreColourValue.h>
#include <OgreVector3.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/vector_property.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

namespace mav_polygon_tool {

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_2_WH;
typedef Polygon_2::Vertex_iterator VertexIterator;

// Declare polygon tool as subclass of rviz::Tool.
class PolygonTool : public rviz::Tool {
  Q_OBJECT

public:
  PolygonTool();
  virtual ~PolygonTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;
  int processKeyEvent(QKeyEvent *event, rviz::RenderPanel *pane) override;
  int processMouseEvent(rviz::ViewportMouseEvent &event) override;

  void load(const rviz::Config &config) override;
  void save(rviz::Config config) const override;

private:
  void setColorsLeaving();
  void setColorsArriving();
  void deletePolygn(size_t index);
  void makeVertex(const Ogre::Vector3 &position);
  void makeHoleVertex(const Ogre::Vector3 &position);
  void leftClicked(rviz::ViewportMouseEvent &event);
  void rightClicked(rviz::ViewportMouseEvent &event);
  bool checkCGalPolygon();

  std::vector<std::vector<rviz::Shape *>> active_spheres_;
  std::vector<std::vector<Ogre::SceneNode *>> vertex_nodes_;
  Ogre::SceneNode *moving_vertex_node_;
  rviz::VectorProperty *current_vertex_property_;
  std::vector<std::vector<rviz::Line *>> active_lines_;
  std::vector<std::list<Point>> points_for_poly_;

  std::vector<Polygon_2> polygon_;
  Polygon_2_WH main_polygon_;
  // Point display.
  rviz::Shape *vertex_;
  // point scale
  const float kPtScale = 0.5;
  const float kDeleteTol = 0.2;

  Ogre::ColourValue red_, blue_, pink_, green_, yellow_, transparent_;
  void pushBackElements(int new_type);
  void setColor(const Ogre::ColourValue &line_color,
                const Ogre::ColourValue &sphere_color);
  void drawLines(const Ogre::ColourValue &line_color);
  size_t current_polygon_ = 0;
  int current_type_;
  std::vector<int> type_of_polygons_;

  bool is_activated_ = false;
  bool check_performed_ = false;

  // ROS messaging
  ros::NodeHandle nh_;
  ros::Subscriber new_tool_subs_;
  ros::Subscriber delete_poly_subs_;
  ros::Subscriber selector_subs_;
  ros::Subscriber check_poly_subs_;
  ros::Publisher status_update_publisher_;
  void checkStatusCallback(const std_msgs::Bool &incomming);
  void toolSelectCallback(const std_msgs::Int8 &tool_num);
  void newPolyCallback(const std_msgs::Int8 &new_poly_num);
  void deletePolyCallback(const std_msgs::Int8 &delete_ind);
};

} // namespace mav_polygon_tool

#endif // FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
