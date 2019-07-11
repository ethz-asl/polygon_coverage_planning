#ifndef RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <polygon_coverage_geometry/cgal_definitions.h>

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

namespace rviz_polygon_tool {

// Declare polygon tool as subclass of rviz::Tool.
// Left click: Insert a new vertex before the selected vertex.
// Right click: Remove a vertex, select the next vertex.
// 'h': Add hole
// 'n': Select next polygon
// 'v': Select next vertex
// 'r': Reset currently selected polygon
// 'c': Clear all
// Enter: Publish polygon if valid
class PolygonTool : public rviz::Tool {
  Q_OBJECT

 public:
  PolygonTool();
  virtual ~PolygonTool();
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

 private:
  // User input.
  void clickLeft(const rviz::ViewportMouseEvent& event);
  void clickRight(const rviz::ViewportMouseEvent& event);

  // Action.
  void createVertex(const Ogre::Vector3& position);
  void deleteVertex(const Ogre::Vector3& position);
  void addHole();
  void nextPolygon();
  void nextVertex();
  void resetPolygon();
  void clearAll();
  void publishPolygon();
  void updateStatus();
  void removeEmptyHoles();
  void increaseAltitude(rviz::ViewportMouseEvent& event);
  void decreaseAltitude(rviz::ViewportMouseEvent& event);

  std::list<Polygon_2> polygons_;
  std::list<Polygon_2>::iterator polygon_selection_;  // 0: hull, 1..N-1: holes
  VertexIterator vertex_selection_;
  double altitude_;

  // Rendering
  void renderPolygon(const Polygon_2& polygon, const Ogre::ColourValue& c);
  void renderPolygons();
  Ogre::SceneNode* polygon_node_;

  // Sphere currently displayed.
  Ogre::SceneNode* moving_vertex_node_;
  rviz::Shape* sphere_;

  // ROS messaging
  ros::NodeHandle nh_;
  ros::Publisher polygon_pub_;
};

}  // namespace rviz_polygon_tool

#endif  // RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
