#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <ros/console.h>

#include <rviz/geometry.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/vector_property.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

#include "fm_rviz_polygon_tool/polygon_tool.h"

namespace mav_coverage_planning {

PolygonTool::PolygonTool()
    : Tool(), moving_vertex_node_(nullptr), current_vertex_property_(nullptr) {
  shortcut_key_ = 'p';
}

PolygonTool::~PolygonTool() {
  delete vertex_;

  for (Ogre::SceneNode* vertex_node : vertex_nodes_)
    scene_manager_->destroySceneNode(vertex_node);
}

void PolygonTool::onInitialize() {
  // arrow_ = new Arrow( scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f );
  // arrow_->setColor( 0.0f, 1.0f, 0.0f, 1.0f );
  // arrow_->getSceneNode()->setVisible( false );
  // Create a vertex disc and set it invisible.
  moving_vertex_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  vertex_ =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, moving_vertex_node_);
  moving_vertex_node_->setVisible(false);
}

void PolygonTool::activate() {
  // Make vertex node visible and add property to property container.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(true);

    current_vertex_property_ = new rviz::VectorProperty(
        "Vertex " + QString::number(vertex_nodes_.size()));
    current_vertex_property_->setReadOnly(true);
    getPropertyContainer()->addChild(current_vertex_property_);
  }
}

void PolygonTool::deactivate() {
  // Make moving vertex invisible and delete current flag property.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(false);
    delete current_vertex_property_;
    current_vertex_property_ = nullptr;
  }
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  if (!moving_vertex_node_) {
    return Render;
  }
  // Project mouse pointer on polygon plane.
  Ogre::Vector3 intersection;
  // TODO(rikba): Change to actual polygon plane.
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    moving_vertex_node_->setVisible(true);
    moving_vertex_node_->setPosition(intersection);
    current_vertex_property_->setVector(intersection);

    if (event.leftDown()) {
      makeVertex(intersection);
      // Drop reference so it does not get removed by deactivate().
      current_vertex_property_ = nullptr;
    }
  } else {
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }
  return Render;
}

void PolygonTool::makeVertex(const Ogre::Vector3& position) {
  // Create a new vertex in the Ogre scene and save scene to list.
  Ogre::SceneNode* node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  rviz::Shape(rviz::Shape::Sphere, scene_manager_, node);
  node->setVisible(true);
  node->setPosition(position);
  vertex_nodes_.push_back(node);
}

void PolygonTool::save(rviz::Config config) const {
  config.mapSetValue("Class", getClassId());

  // Create child of map to store list of vertices.
  rviz::Config vertices_config = config.mapMakeChild("Vertices");

  // To read the positions and names of the vertices, we loop over the children
  // of our Property container:
  rviz::Property* container = getPropertyContainer();
  for (int i = 0; i < container->numChildren(); i++) {
    rviz::Property* position_prop = container->childAt(i);
    // For each Property, we create a new Config object representing a
    // single vertex and append it to the Config list.
    rviz::Config vertex_config = vertices_config.listAppendNew();
    // Into the flag's config we store its name:
    vertex_config.mapSetValue("Name", position_prop->getName());
    // ... and its position.
    position_prop->save(vertex_config);
  }
}

void PolygonTool::load(const rviz::Config& config) {
  // Get the vertices sub-config from the tool config and loop over entries.
  rviz::Config vertices_config = config.mapGetChild("Vertices");
  for (int i = 0; i < vertices_config.listLength(); i++) {
    rviz::Config vertex_config = vertices_config.listChildAt(i);
    // Provide a default name in case the name is not in the config file.
    QString name = "Flag " + QString::number(i);
    vertex_config.mapGetString("Name", &name);  // Read name from flag config.
    // Create an rviz::VectorProperty to display the position.
    rviz::VectorProperty* prop = new rviz::VectorProperty(name);
    prop->setReadOnly(true);
    getPropertyContainer()->addChild(prop);
    makeVertex(prop->getVector());  // Make vertex visible.
  }
}

}  // namespace mav_coverage_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_coverage_planning::PolygonTool, rviz::Tool)
