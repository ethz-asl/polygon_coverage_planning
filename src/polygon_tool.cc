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
  std::string node_name = "polygon_tool";
  //ros::init(0, 0, node_name);

  std::cout << "called PolygonTool onInitialize" << std::endl;
  std::cout<<"status "<<ros::isStarted()<<std::endl;
  moving_vertex_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  vertex_ =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, moving_vertex_node_);
  // or else the ball is visible in the middle of the scene
  moving_vertex_node_->setVisible(false);
  //nh_=ros::NodeHandle("/");
  selector_subs_ =
      nh_.subscribe("select_poly", 1, &PolygonTool::toolSelectCallback, this);
  new_tool_subs_ =
      nh_.subscribe("new_poly", 1, &PolygonTool::newPolyCallback, this);
  delete_poly_subs_ =
      nh_.subscribe("delete_poly", 1, &PolygonTool::deletePolyCallback, this);
  check_poly_subs_=
      nh_.subscribe("check_polygon_request", 1, &PolygonTool::checkStatusCallback, this);
  status_update_publisher_ =
      nh_.advertise<std_msgs::Bool>("polygon_status_update", 1, true);

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
//*
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
//*/
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

// remove all the elements at index
void PolygonTool::deletePolygn(size_t index) {
  if (index < active_spheres_.size() && is_activated_) {
    current_polygon_ = index;
    setColor(transparent_, transparent_);
    // delete the vectors
    if (index >= 0 && active_spheres_.size() > 1) {
      active_spheres_.erase(active_spheres_.begin() + index);
      vertex_nodes_.erase(vertex_nodes_.begin() + index);
      active_lines_.erase(active_lines_.begin() + index);
      points_for_poly_.erase(points_for_poly_.begin() + index);
      polygon_.erase(polygon_.begin() + index);
      type_of_polygons_.erase(type_of_polygons_.begin() + index);
      // must be done to avoid out of bounds error when deleting the last one
      current_type_ = type_of_polygons_[index - 1];
      current_polygon_ = index - 1;
    }
    // only clear the vectors, don't delete them
    else if (index == 0 && active_spheres_.size() == 1) {
      active_spheres_[0].clear();
      vertex_nodes_[0].clear();
      active_lines_[0].clear();
      points_for_poly_[0].clear();
      polygon_.clear();
      Polygon_2 dummy_poly;
      polygon_.push_back(dummy_poly);
    }
  }
  setColorsArriving();
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

}

// check geometric propetries of the CGal polygon
bool PolygonTool::checkCGalPolygon() {
  bool to_be_ret = false;

  for(size_t i = 0; i<polygon_.size(); ++i){
    //polygons should be simple and have at least 3 pts!
    if(polygon_[i].size()<3 || !polygon_[i].is_simple()){
      break;
    }
    else if(polygon_[i].is_simple() && i==(polygon_.size()-1)){
      to_be_ret = true;
    }
  }

  //construct polygon with holes
  //Polygon_2 local_combo=polygon_[0];
  std::cout<<"debug 1"<<std::endl;
  if(to_be_ret){
    std::cout<<"debug 2"<<std::endl;

    main_polygon_.clear();
    CGAL::join(polygon_[0],polygon_[0],main_polygon_);
    for(size_t i = 1; i<polygon_.size(); ++i){
      // only interested in the polygons
      if(type_of_polygons_[i] == 0){

        std::cout<<"debug 3"<<std::endl;
        Polygon_2_WH local_copy(main_polygon_);
        Polygon_2_WH local_copy_result(main_polygon_);

        std::cout<<"debug 3.1"<<std::endl;

        for(size_t j=i;j<polygon_.size();++j){
          std::cout<<"debug 3.15"<<std::endl;
          if(CGAL::join(local_copy,polygon_[j],local_copy_result)){
            main_polygon_=local_copy_result;
            std::cout<<"debug 3.2"<<std::endl;
            j=polygon_.size()+1;
          }
          else if(j==(polygon_.size()-1)) {
            std::cout<<"debug 3.3"<<std::endl;
            if(CGAL::join(local_copy,polygon_[j],main_polygon_)) {
              std::cout<<"debug 3.4"<<std::endl;
              }
            else{
              to_be_ret=false;
              i=polygon_.size()+1;
              break;
            }
          }
        }

      }
    }
    if(main_polygon_.is_unbounded()){
      std::cout<<"debug 4"<<std::endl;
    }
    else{
      std::cout<<"debug 5"<<std::endl;
      //to_be_ret = false;
      //std::cout<<"test not passed "<<std::endl;
    }
  }
  return to_be_ret;
}


void PolygonTool::drawLines(const Ogre::ColourValue &line_color) {
  // something has changed, set verification to false
  std_msgs::Bool to_send;
  to_send.data = false;
  status_update_publisher_.publish(to_send);
  // lines have to be set to invisible before being deleted
  for (size_t j = 0; j < active_lines_[current_polygon_].size(); ++j) {
    active_lines_[current_polygon_][j]->setVisible(false);
  }
  active_lines_[current_polygon_].clear();
  // only draw lines if there are 3 or more points,
  // otherwise the polygon is degenerate/a simple line
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
  for (size_t i = 0; i < active_spheres_[current_polygon_].size(); ++i) {
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
        for (size_t j = i + 1; j < vertex_nodes_[current_polygon_].size(); ++j) {
          new_locl_active_sphr.push_back(active_spheres_[current_polygon_][j]);
          new_locl_vrtx_nds.push_back(vertex_nodes_[current_polygon_][j]);
          new_locl_polygn.push_back(polygon_[current_polygon_][j]);
        }
        for (size_t j = 0; j < i; ++j) {
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

void PolygonTool::checkStatusCallback(const std_msgs::Bool &incomming){
  std::cout<<"called PolygonTool::checkStatusCallback "<<std::endl;
  bool to_return = checkCGalPolygon();
  std::cout<<"this is status of polygon :"<<to_return<<std::endl;
  std_msgs::Bool to_send;
  to_send.data = to_return;
  status_update_publisher_.publish(to_send);
}

void PolygonTool::toolSelectCallback(const std_msgs::Int8 &tool_num) {
  if (is_activated_) {
    int incomming_num = tool_num.data;
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
  }
}

void PolygonTool::deletePolyCallback(const std_msgs::Int8 &delete_ind) {
  if (is_activated_) {
    int delete_location = delete_ind.data;
    deletePolygn(delete_location);
  }
}

} // namespace mav_polygon_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_polygon_tool::PolygonTool, rviz::Tool)
