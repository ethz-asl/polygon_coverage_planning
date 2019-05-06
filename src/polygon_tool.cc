#include "fm_rviz_polygon_tool/polygon_tool.h"

// use this line to launch it programatically
// vis_manager_->getToolManager()->addTool("fm_rviz_polygon_tool/PolygonSelection");
namespace mav_polygon_tool {

PolygonTool::PolygonTool()
    : Tool(), moving_vertex_node_(nullptr), current_vertex_property_(nullptr) {
  shortcut_key_ = 'p';
  red_ = Ogre::ColourValue(1.f,0.f,0.f,1.0);
  green_ = Ogre::ColourValue(0.f,1.f,0.f,1.0);
  blue_ = Ogre::ColourValue(0.f,0.f,1.f,1.0);
  pink_ = Ogre::ColourValue(1.f,0.f,1.f,1.0);
}

PolygonTool::~PolygonTool() {
  std::cout << "called PolygonTool destructor" << std::endl;
  delete vertex_;
  for (Ogre::SceneNode *vertex_node : vertex_nodes_[current_polygon_])
    scene_manager_->destroySceneNode(vertex_node);
}

void PolygonTool::onInitialize() {
  std::cout << "called PolygonTool onInitialize" << std::endl;
  moving_vertex_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  vertex_ =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, moving_vertex_node_);
  // or else the ball is visible in the middle of the scene
  moving_vertex_node_->setVisible(false);
}

void PolygonTool::activate() {
  std::cout << "called PolygonTool activate" << std::endl;
  // Make vertex node visible and add property to property container.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(true);
    current_vertex_property_ = new rviz::VectorProperty(
        "Vertex " + QString::number(vertex_nodes_.size()));
    current_vertex_property_->setReadOnly(true);
    getPropertyContainer()->addChild(current_vertex_property_);
  }
  pushBackElements();
}

void PolygonTool::deactivate() {
  std::cout << "called PolygonTool deactivate" << std::endl;
  // Make moving vertex invisible and delete current flag property.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(false);
    delete current_vertex_property_;
    current_vertex_property_ = nullptr;
  }
}

void PolygonTool::pushBackElements(){
  std::vector<rviz::Shape *> dummy_vec_shapes;
  active_spheres_.push_back(dummy_vec_shapes);
  std::vector<Ogre::SceneNode *> dummy_nodes;
  vertex_nodes_.push_back(dummy_nodes);
  std::vector<rviz::Line *> dummy_lines;
  active_lines_.push_back(dummy_lines);
  std::list<Point> dummy_points;
  points_for_poly_.push_back(dummy_points);
  Polygon_2 dummy_pol;
  polygon_.push_back(dummy_pol);

  }

int PolygonTool::processKeyEvent(QKeyEvent *e, rviz::RenderPanel *pane){
  std::cout<<"processKeyEvent called "<<e->key()<<std::endl;
  std::cout<<"processKeyEvent status of mouse_down_"<<mouse_down_<<std::endl;
  //make this a switch case?
  if(e->key()==78){
    setColor(green_,green_);
    current_polygon_++;
    if(current_polygon_ == polygon_.size()){
      std::cout<<"pushed back elements "<<std::endl;
      pushBackElements();
    }
    else{
      setColor(green_,red_);
    }

  }
  else if(e->key()==66){
    if(current_polygon_>0){
      setColor(green_,green_);
      current_polygon_--;
      setColor(green_,red_);
    }
  }
  else if(e->key()==72){

    std::cout<<"should create a hole..."<<std::endl;
  }
  return 1;
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (!moving_vertex_node_){
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    moving_vertex_node_->setVisible(true);
    moving_vertex_node_->setPosition(intersection);

    if (event.leftUp()) {
      leftClicked(event);
    }
    else if (event.rightUp()) {
      rightClicked(event);
    }
  }
  else {
    std::cout << "called PolygonTool processMouseEvent else" << std::endl;
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }
  return Render;
}

// adding red points to make a polygon
void PolygonTool::makeVertex(const Ogre::Vector3 &position) {
  std::cout << "called PolygonTool makeVertex" << std::endl;
  // Create a new vertex in the Ogre scene and save scene to list.
  Ogre::SceneNode *node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  rviz::Shape *local_sphere =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
  local_sphere->setColor(red_);
  Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
  local_sphere->setScale(scale);
  local_sphere->setPosition(position);
  //std::cout << "debug 1" << std::endl;
  active_spheres_[current_polygon_].push_back(local_sphere);
  node->setPosition(position);
  //std::cout << "debug 2" << std::endl;
  vertex_nodes_[current_polygon_].push_back(node);
  Point local_2d_pt = Point(position[0], position[1]);
  polygon_[current_polygon_].push_back(local_2d_pt);
  std::cout << "debug 4" << std::endl;
  drawLines();
  checkCGalPolygon();
}

// check geometric propetries of the CGal polygon
void PolygonTool::checkCGalPolygon(){
  std::cout << " Polygon is simple: " <<polygon_[current_polygon_].is_simple()<<std::endl;
  std::cout << " Polygon is convex: " <<polygon_[current_polygon_].is_convex()<<std::endl;
}

void PolygonTool::drawLines() {
  // lines have to be set to invisible before being deleted
  for (size_t j = 0; j < active_lines_[current_polygon_].size(); ++j) {
    active_lines_[current_polygon_][j]->setVisible(false);
  }
  active_lines_[current_polygon_].clear();
  // only draw lines if there are 3 or more points,
  // otherwise the polygon is degenerate
  if (vertex_nodes_[current_polygon_].size() > 2) {
    // draw lines!
    // last one is done seperately
    active_lines_[current_polygon_].clear();
    rviz::Line *last_line =
        new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
    last_line->setPoints(
        vertex_nodes_[current_polygon_][0]->getPosition(),
        vertex_nodes_[current_polygon_][vertex_nodes_[current_polygon_].size() - 1]->getPosition());
    last_line->setColor(red_);
    active_lines_[current_polygon_].push_back(last_line);

    // all the other lines
    for (size_t i = 1; i < vertex_nodes_[current_polygon_].size(); ++i) {
      rviz::Line *local_line =
          new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
      local_line->setColor(0.0, 1.0, 0.0, 1.0);
      local_line->setPoints(vertex_nodes_[current_polygon_][i - 1]->getPosition(),
                            vertex_nodes_[current_polygon_][i]->getPosition());
      active_lines_[current_polygon_].push_back(local_line);
    }
  }
}

void PolygonTool::setColor(const Ogre::ColourValue &line_color,
   const Ogre::ColourValue &sphere_color) {
  // lines have to be set to invisible before being deleted
  for (size_t j = 0; j < active_lines_[current_polygon_].size(); ++j) {
    active_lines_[current_polygon_][j]->setVisible(false);
  }
  active_lines_[current_polygon_].clear();
  // only draw lines if there are 3 or more points,
  // otherwise the polygon is degenerate
  if (vertex_nodes_[current_polygon_].size() > 2) {
    // draw lines!
    // last one is done seperately
    active_lines_[current_polygon_].clear();
    rviz::Line *last_line =
        new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
    last_line->setPoints(
        vertex_nodes_[current_polygon_][0]->getPosition(),
        vertex_nodes_[current_polygon_][vertex_nodes_[current_polygon_].size() - 1]->getPosition());
    last_line->setColor(sphere_color);
    active_lines_[current_polygon_].push_back(last_line);

    // all the other lines
    for (size_t i = 1; i < vertex_nodes_[current_polygon_].size(); ++i) {
      rviz::Line *local_line =
          new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
      local_line->setColor(line_color);
      local_line->setPoints(vertex_nodes_[current_polygon_][i - 1]->getPosition(),
                            vertex_nodes_[current_polygon_][i]->getPosition());
      active_lines_[current_polygon_].push_back(local_line);
    }
  }

  // repaint nodes in green
  for(int i=0;i<active_spheres_[current_polygon_].size(); ++i){
    active_spheres_[current_polygon_][i]->setColor(sphere_color);
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

//called when the right mouse boutton is clicked
//used to delete nodes within the range of the cursor node
void PolygonTool::rightClicked(rviz::ViewportMouseEvent &event) {
  // get the x y z coodinates of the click
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  VertexIterator vi = polygon_[current_polygon_].vertices_begin();
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    for (size_t i = 0; i < vertex_nodes_[current_polygon_].size(); ++i) {
      // compare the distance from the center of the nodes already drawn
      Ogre::Vector3 distance_vec =
          intersection - vertex_nodes_[current_polygon_][i]->getPosition();
      if (distance_vec.length() < kDeleteTol) {
        vertex_nodes_[current_polygon_][i]->setVisible(false);
        active_spheres_[current_polygon_][i]->setColor(0.0, 0, 0, 0.0);
        active_spheres_[current_polygon_].erase(active_spheres_[current_polygon_].begin() + i);
        vertex_nodes_[current_polygon_].erase(vertex_nodes_[current_polygon_].begin() + i);
        // erase elements in the CGAL polygon
        polygon_[current_polygon_].erase(vi + i);
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
    // Into the flag's config we store its name:
    vertex_config.mapSetValue("Name", position_prop->getName());
    // ... and its position.
    position_prop->save(vertex_config);
  }
  std::cout << "finished PolygonTool save " << std::endl;
}

void PolygonTool::load(const rviz::Config &config) {
  std::cout << "called PolygonTool load" << std::endl;
  // Get the vertices sub-config from the tool config and loop over entries.
  rviz::Config vertices_config = config.mapGetChild("Vertices");
  for (int i = 0; i < vertices_config.listLength(); i++) {
    rviz::Config vertex_config = vertices_config.listChildAt(i);
    // Provide a default name in case the name is not in the config file.
    QString name = "Flag " + QString::number(i);
    vertex_config.mapGetString("Name", &name); // Read name from flag config.
    // Create an rviz::VectorProperty to display the position.
    rviz::VectorProperty *prop = new rviz::VectorProperty(name);
    // Is this needed?
    // prop->setReadOnly(true);
    getPropertyContainer()->addChild(prop);
    makeVertex(prop->getVector()); // Make vertex visible.
  }
}

std::vector<Polygon_2> PolygonTool::getPolygon(){
  return polygon_;
}

} // namespace mav_polygon_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_polygon_tool::PolygonTool, rviz::Tool)
