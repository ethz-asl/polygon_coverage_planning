#include "fm_rviz_polygon_tool/polygon_tool.h"

namespace mav_polygon_tool {

PolygonTool::PolygonTool()
    : Tool(), moving_vertex_node_(nullptr), current_vertex_property_(nullptr) {
  shortcut_key_ = 'p';
}

PolygonTool::~PolygonTool() {
  delete vertex_;

  for (Ogre::SceneNode *vertex_node : vertex_nodes_)
    scene_manager_->destroySceneNode(vertex_node);
}

void PolygonTool::onInitialize() {
  moving_vertex_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  vertex_ =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, moving_vertex_node_);
  // or else the ball is visible in the middle of the scene
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
  // Make moving vertex invisible
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(false);
    delete current_vertex_property_;
    current_vertex_property_ = nullptr;
  }
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (!moving_vertex_node_) {
    return Render;
  }
  // Project mouse pointer on polygon plane.
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    moving_vertex_node_->setVisible(true);
    moving_vertex_node_->setPosition(intersection);
    if (event.leftDown()) {
      leftClicked(event);
    } else if (event.rightDown()) {
      rightClicked(event);
    }
  } else {
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }
  return Render;
}

// adding red points to make a polygon
void PolygonTool::makeVertex(const Ogre::Vector3 &position) {
  // Create a new vertex in the Ogre scene and save scene to list.
  Ogre::SceneNode *node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  rviz::Shape *local_sphere =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
  local_sphere->setColor(1.0, 0, 0, 1.0);
  Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
  local_sphere->setScale(scale);
  local_sphere->setPosition(position);
  active_spheres_.push_back(local_sphere);
  node->setPosition(position);
  vertex_nodes_.push_back(node);

  Point local_2d_pt = Point(position[0], position[1]);
  polygon_.push_back(local_2d_pt);

  drawLines();
  checkCGalPolygon();
}

// check geometric propetries of the CGal polygon
void PolygonTool::checkCGalPolygon() {
  std::cout << " Polygon is simple: " << polygon_.is_simple() << std::endl;
  std::cout << " Polygon is convex: " << polygon_.is_convex() << std::endl;
}

void PolygonTool::drawLines() {
  // lines have to be set to invisible before being deleted
  for (size_t j = 0; j < active_lines_.size(); ++j) {
    active_lines_[j]->setVisible(false);
  }
  active_lines_.clear();
  if (vertex_nodes_.size() > 2) {
    // draw lines!
    // last one
    active_lines_.clear();
    rviz::Line *last_line =
        new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
    last_line->setPoints(
        vertex_nodes_[0]->getPosition(),
        vertex_nodes_[vertex_nodes_.size() - 1]->getPosition());
    last_line->setColor(0.0, 1.0, 0.0, 1.0);
    active_lines_.push_back(last_line);

    // all the other lines
    for (size_t i = 1; i < vertex_nodes_.size(); ++i) {
      rviz::Line *local_line =
          new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
      local_line->setColor(0.0, 1.0, 0.0, 1.0);
      local_line->setPoints(vertex_nodes_[i - 1]->getPosition(),
                            vertex_nodes_[i]->getPosition());
      active_lines_.push_back(local_line);
    }
  }
}

// called by the callback when the user clicks the left mouse button
void PolygonTool::leftClicked(rviz::ViewportMouseEvent &event) {
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    makeVertex(intersection);
  }
  std::cout << "size of vertex_nodes_ " << vertex_nodes_.size() << std::endl;
}

// called when the right mouse boutton is clicked
// used to delete nodes within the range of the cursor node
void PolygonTool::rightClicked(rviz::ViewportMouseEvent &event) {
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  VertexIterator vi = polygon_.vertices_begin();
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    for (size_t i = 0; i < vertex_nodes_.size(); ++i) {
      // compare the distance from the center of the nodes already drawn
      Ogre::Vector3 distance_vec =
          intersection - vertex_nodes_[i]->getPosition();
      if (distance_vec.length() < kDeleteTol) {
        vertex_nodes_[i]->setVisible(false);
        active_spheres_[i]->setColor(0.0, 0, 0, 0.0);
        active_spheres_.erase(active_spheres_.begin() + i);
        vertex_nodes_.erase(vertex_nodes_.begin() + i);
        // erase elements in the CGAL polygon
        polygon_.erase(vi + i);
        drawLines();
        break;
      }
    }
  }
}

void PolygonTool::save(rviz::Config config) const {
  config.mapSetValue("Class", getClassId());
  // Create child of map to store list of vertices.
  rviz::Config vertices_config = config.mapMakeChild("Vertices");
  // To read the positions and names of the vertices, we loop over the children
  // of our Property container:
  rviz::Property *container = getPropertyContainer();
  for (int i = 0; i < container->numChildren(); i++) {
    rviz::Property *position_prop = container->childAt(i);
    // For each Property, we create a new Config object representing a
    // single vertex and append it to the Config list.
    rviz::Config vertex_config = vertices_config.listAppendNew();
    vertex_config.mapSetValue("Name", position_prop->getName());
    // ... and its position.
    position_prop->save(vertex_config);
  }
}

void PolygonTool::load(const rviz::Config &config) {
  // Get the vertices sub-config from the tool config and loop over entries.
  rviz::Config vertices_config = config.mapGetChild("Vertices");
  for (int i = 0; i < vertices_config.listLength(); i++) {
    rviz::Config vertex_config = vertices_config.listChildAt(i);
    // Provide a default name in case the name is not in the config file.
    QString name = "Vertex " + QString::number(i);
    vertex_config.mapGetString("Name", &name); // Read name from vertex config.
    // Create an rviz::VectorProperty to display the position.
    rviz::VectorProperty *prop = new rviz::VectorProperty(name);
    // Is this needed?
    // prop->setReadOnly(true);
    getPropertyContainer()->addChild(prop);
    makeVertex(prop->getVector()); // Make vertex visible.
  }
}

Polygon_2 PolygonTool::getPolygon() {
  return polygon_;
}

} // namespace mav_coverage_planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_coverage_planning::PolygonTool, rviz::Tool)
