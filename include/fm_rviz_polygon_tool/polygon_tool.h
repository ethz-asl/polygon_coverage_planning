#ifndef FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/console.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include <OgreVector3.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/vector_property.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

namespace mav_coverage_planning {

// Declare polygon tool as subclass of rviz::Tool.
class PolygonTool : public rviz::Tool {
  Q_OBJECT
public:
  PolygonTool();
  virtual ~PolygonTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz::ViewportMouseEvent &event) override;

  void load(const rviz::Config &config) override;
  void save(rviz::Config config) const override;
  CGAL::Polygon_2<CGAL::Exact_predicates_inexact_constructions_kernel>
  getPolygon();

private:
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2 Point;
  typedef CGAL::Polygon_2<K> Polygon_2;
  typedef Polygon_2::Vertex_iterator VertexIterator;

  void makeVertex(const Ogre::Vector3 &position);
  void leftClicked(rviz::ViewportMouseEvent &event);
  void rightClicked(rviz::ViewportMouseEvent &event);
  void drawLines();
  void checkCGalPolygon();

  std::vector<rviz::Shape *> active_spheres_;
  std::vector<Ogre::SceneNode *> vertex_nodes_;

  Ogre::SceneNode *moving_vertex_node_;
  rviz::VectorProperty *current_vertex_property_;
  rviz::Shape *vertex_;
  std::vector<rviz::Line *> active_lines_;


  std::list<Point> points_for_poly_;
  Polygon_2 polygon_;

  // point scale
  float pt_scale_ = 0.5;
  float delete_tol_ = 0.2;
};

} // namespace mav_coverage_planning

#endif // FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
