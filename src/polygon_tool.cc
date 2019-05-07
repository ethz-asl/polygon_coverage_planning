#include "fm_rviz_polygon_tool/polygon_tool.h"

// use this line to launch it programatically
// vis_manager_->getToolManager()->addTool("fm_rviz_polygon_tool/PolygonSelection");
namespace mav_polygon_tool {

PolygonTool::PolygonTool()
    : Tool(), moving_vertex_node_(nullptr), current_vertex_property_(nullptr) {
  shortcut_key_ = 'p';
  red_ = Ogre::ColourValue(1.f, 0.f, 0.f, 1.0);
  green_ = Ogre::ColourValue(0.f, 1.f, 0.f, 1.0);
  blue_ = Ogre::ColourValue(0.f, 0.f, 1.f, 1.0);
  pink_ = Ogre::ColourValue(1.f, 0.f, 1.f, 1.0);
  yellow_ = Ogre::ColourValue(1.f, 1.f, 0.f, 1.0);
  transparent_ = Ogre::ColourValue(0.f, 0.f, 0.f, 0.0);
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
  selector_subs_ =
      nh_.subscribe("select_poly", 1, &PolygonTool::toolSelectCallback, this);
  new_tool_subs_ =
      nh_.subscribe("new_poly", 1, &PolygonTool::newPolyCallback, this);
  delete_poly_subs_ =
      nh_.subscribe("delete_poly", 1, &PolygonTool::deletePolyCallback, this);
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
  if (!is_activated_) {
    pushBackElements(0);
    is_activated_ = true;
  }
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

void PolygonTool::pushBackElements(int new_type) {
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
  current_type_ = new_type;
  type_of_polygons_.push_back(current_type_);
}

int PolygonTool::processKeyEvent(QKeyEvent *e, rviz::RenderPanel *pane) {
  std::cout << "processKeyEvent called " << e->key() << std::endl;
  // click on n
  if (e->key() == 78) {
    setColorsLeaving();
    current_polygon_++;
    if (current_polygon_ == polygon_.size()) {
      pushBackElements(0);
    }
    // just select the next one
    else {
      current_type_ = type_of_polygons_[current_polygon_];
      setColorsArriving();
    }
  }
  // click on b
  else if (e->key() == 66) {
    if (current_polygon_ > 0) {
      setColorsLeaving();
      current_polygon_--;
      current_type_ = type_of_polygons_[current_polygon_];
      setColorsArriving();
    }
  }
  // click on h
  // for hole
  else if (e->key() == 72) {
    if (current_polygon_ + 1 == polygon_.size()) {
      setColorsLeaving();
      current_polygon_++;
      pushBackElements(1);
    }
  }
  return 1;
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent &event) {
  if (!moving_vertex_node_) {
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
    } else if (event.rightUp()) {
      rightClicked(event);
    }
  } else {
    std::cout << "called PolygonTool processMouseEvent else" << std::endl;
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }
  return Render;
}

//remove all the elements at index
void PolygonTool::deletePolygn(int index){
  if(index<active_spheres_.size() && is_activated_){
    current_polygon_ = index;
    setColor(transparent_, transparent_);
    //delete the vectors
    if(index >=0 && active_spheres_.size()>1){
      active_spheres_.erase(active_spheres_.begin()+index);
      vertex_nodes_.erase(vertex_nodes_.begin()+index);
      active_lines_.erase(active_lines_.begin()+index);
      points_for_poly_.erase(points_for_poly_.begin()+index);
      polygon_.erase(polygon_.begin()+index);
    }
    //only clear the vectors
    else if(index == 0 && active_spheres_.size() == 1){
      active_spheres_[0].clear();
      vertex_nodes_[0].clear();
      active_lines_[0].clear();
      points_for_poly_[0].clear();
      polygon_.clear();
      Polygon_2 dummy_poly;
      polygon_.push_back(dummy_poly);
    }
  }
}
// adding red points to make a polygon
void PolygonTool::makeVertex(const Ogre::Vector3 &position) {
  // std::cout << "called PolygonTool makeVertex" << std::endl;
  // Create a new vertex in the Ogre scene and save scene to list.
  Ogre::SceneNode *node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  rviz::Shape *local_sphere =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
  local_sphere->setColor(red_);
  Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
  local_sphere->setScale(scale);
  local_sphere->setPosition(position);
  active_spheres_[current_polygon_].push_back(local_sphere);
  node->setPosition(position);
  vertex_nodes_[current_polygon_].push_back(node);
  Point local_2d_pt = Point(position[0], position[1]);
  polygon_[current_polygon_].push_back(local_2d_pt);

  if (current_type_ == 0) {
    drawLines(green_);
  } else {
    drawLines(yellow_);
  }
  checkCGalPolygon();
}

// check geometric propetries of the CGal polygon
void PolygonTool::checkCGalPolygon() {
  // std::cout << " Polygon is simple: "
  // <<polygon_[current_polygon_].is_simple()<<std::endl; std::cout << " Polygon
  // is convex: " <<polygon_[current_polygon_].is_convex()<<std::endl;
}

void PolygonTool::drawLines(const Ogre::ColourValue &line_color) {
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
        vertex_nodes_[current_polygon_]
                     [vertex_nodes_[current_polygon_].size() - 1]
                         ->getPosition());
    last_line->setColor(red_);
    active_lines_[current_polygon_].push_back(last_line);

    // all the other lines
    for (size_t i = 1; i < vertex_nodes_[current_polygon_].size(); ++i) {
      rviz::Line *local_line =
          new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
      local_line->setColor(line_color);
      local_line->setPoints(
          vertex_nodes_[current_polygon_][i - 1]->getPosition(),
          vertex_nodes_[current_polygon_][i]->getPosition());
      active_lines_[current_polygon_].push_back(local_line);
    }
  }
}

void PolygonTool::setColorsLeaving() {
  if (current_type_ == 0) {
    setColor(green_, green_);
  } else {
    setColor(pink_, pink_);
  }
}

void PolygonTool::setColorsArriving() {
  if (current_type_ == 0) {
    setColor(green_, red_);
  } else {
    setColor(yellow_, red_);
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
        vertex_nodes_[current_polygon_]
                     [vertex_nodes_[current_polygon_].size() - 1]
                         ->getPosition());
    last_line->setColor(sphere_color);
    active_lines_[current_polygon_].push_back(last_line);

    // all the other lines
    for (size_t i = 1; i < vertex_nodes_[current_polygon_].size(); ++i) {
      rviz::Line *local_line =
          new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
      local_line->setColor(line_color);
      local_line->setPoints(
          vertex_nodes_[current_polygon_][i - 1]->getPosition(),
          vertex_nodes_[current_polygon_][i]->getPosition());
      active_lines_[current_polygon_].push_back(local_line);
    }
  }

  // repaint nodes in green
  for (int i = 0; i < active_spheres_[current_polygon_].size(); ++i) {
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
}

// called when the right mouse boutton is clicked
// used to delete nodes within the range of the cursor node
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
        // make node dissapear
        vertex_nodes_[current_polygon_][i]->setVisible(false);
        active_spheres_[current_polygon_][i]->setColor(0.0, 0, 0, 0.0);
        // re arrange nodes
        std::vector<rviz::Shape *> new_locl_active_sphr;
        std::vector<Ogre::SceneNode *> new_locl_vrtx_nds;
        Polygon_2 new_locl_polygn;
        for (int j = i + 1; j < vertex_nodes_[current_polygon_].size(); ++j) {
          new_locl_active_sphr.push_back(active_spheres_[current_polygon_][j]);
          new_locl_vrtx_nds.push_back(vertex_nodes_[current_polygon_][j]);
          new_locl_polygn.push_back(polygon_[current_polygon_][j]);
        }
        for (int j = 0; j < i; ++j) {
          new_locl_active_sphr.push_back(active_spheres_[current_polygon_][j]);
          new_locl_vrtx_nds.push_back(vertex_nodes_[current_polygon_][j]);
          new_locl_polygn.push_back(polygon_[current_polygon_][j]);
        }
        // swapping the new elements
        swap(active_spheres_[current_polygon_], new_locl_active_sphr);
        swap(vertex_nodes_[current_polygon_], new_locl_vrtx_nds);
        swap(polygon_[current_polygon_], new_locl_polygn);

        if (current_type_ == 0) {
          drawLines(green_);
        } else {
          drawLines(yellow_);
        }
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

std::vector<Polygon_2> PolygonTool::getPolygon() { return polygon_; }

void PolygonTool::toolSelectCallback(const std_msgs::Int8 &tool_num) {
  if (is_activated_) {
    int incomming_num = tool_num.data;
    std::cout << "called set tool " << incomming_num << std::endl;
    if (incomming_num < vertex_nodes_.size()) {
      setColorsLeaving();
      current_polygon_ = incomming_num;
      current_type_ = type_of_polygons_[current_polygon_];
      setColorsArriving();
    }
    // check that its within range and pick tool
  }
}

void PolygonTool::newPolyCallback(const std_msgs::Int8 &new_poly_type) {
  if (is_activated_) {
    int incomming_type = new_poly_type.data;
    int new_current_poly = vertex_nodes_.size();
    setColorsLeaving();
    pushBackElements(incomming_type);
    // needs to be done here using old size (before push back!)
    current_polygon_ = new_current_poly;
    std::cout << "called new poly " << incomming_type << std::endl;
    // if longer than init new tool
    // else pick tool
  }
}

void PolygonTool::deletePolyCallback(const std_msgs::Int8 &delete_ind) {
  if (is_activated_) {
    int delete_location = delete_ind.data;
    std::cout << "delete node nr. " << delete_location << std::endl;
    deletePolygn(delete_location);
  }
}

} // namespace mav_polygon_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_polygon_tool::PolygonTool, rviz::Tool)
