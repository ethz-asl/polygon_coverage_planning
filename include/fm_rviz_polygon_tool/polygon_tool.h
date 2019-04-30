#ifndef FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
#define FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_

#include <OGRE/OgreSceneNode.h>
#include <OgreVector3.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/tool.h>
#include <rviz/properties/vector_property.h>

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

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

 private:
  void makeVertex(const Ogre::Vector3& position);

  std::vector<Ogre::SceneNode*> vertex_nodes_;
  Ogre::SceneNode* moving_vertex_node_;
  rviz::VectorProperty* current_vertex_property_;

  // Point display.
  rviz::Shape* vertex_;
};

}  // namespace mav_coverage_planning

#endif  // FM_RVIZ_POLYGON_TOOL_POLYGON_TOOL_H_
