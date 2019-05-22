#include "fm_rviz_polygon_tool/polygon_tool.h"
#include <mav_planning_msgs/Point2D.h>
#include <mav_planning_msgs/Polygon2D.h>
#include <mav_planning_msgs/PolygonWithHoles.h>
namespace mav_polygon_tool {

const Ogre::ColourValue kRed = Ogre::ColourValue(1.f, 0.f, 0.f, 1.0);
const Ogre::ColourValue kGreen = Ogre::ColourValue(0.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kBlue = Ogre::ColourValue(0.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kPink = Ogre::ColourValue(1.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kYellow = Ogre::ColourValue(1.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kTransparent = Ogre::ColourValue(0.f, 0.f, 0.f, 0.0);

PolygonTool::PolygonTool()
    : Tool(), moving_vertex_node_(nullptr), current_vertex_property_(nullptr) {
  shortcut_key_ = 'p';
}

PolygonTool::~PolygonTool() {
  clearGlobalPlanning();
  hideIndividualPolygons();
}

void PolygonTool::onInitialize() {
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
  check_poly_subs_ = nh_.subscribe("check_polygon_request", 1,
                                   &PolygonTool::checkStatusCallback, this);
  trigger_polygon_subs_ =
      nh_.subscribe("trigger_polygon_request", 1,
                    &PolygonTool::polygonPublisherCallback, this);
  polygon_wh_publisher_ = nh_.advertise<mav_planning_msgs::PolygonWithHoles>(
      "polygon_with_holes", 1, true);
  status_update_publisher_ =
      nh_.advertise<std_msgs::Bool>("polygon_status_update", 1, true);
  user_warn_publisher_ =
      nh_.advertise<std_msgs::Bool>("warn_poly_status_user", 1, true);
  polygon_wh_publisher_ = nh_.advertise<mav_planning_msgs::PolygonWithHoles>(
      "polygon_with_holes", 1, true);
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
  if (!is_activated_) {
    pushBackElements(0);
    is_activated_ = true;
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

void PolygonTool::pushBackElements(int new_type) {
  std::vector<rviz::Shape*> dummy_vec_shapes;
  active_spheres_.push_back(dummy_vec_shapes);
  std::vector<Ogre::SceneNode*> dummy_nodes;
  vertex_nodes_.push_back(dummy_nodes);
  std::vector<rviz::Line*> dummy_lines;
  active_lines_.push_back(dummy_lines);
  std::list<Point> dummy_points;
  points_for_poly_.push_back(dummy_points);
  Polygon_2 dummy_pol;
  polygon_.push_back(dummy_pol);
  current_type_ = new_type;
  type_of_polygons_.push_back(current_type_);
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  if (!moving_vertex_node_) {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    moving_vertex_node_->setVisible(true);
    moving_vertex_node_->setPosition(intersection);

    if (event.leftUp() && !global_planning_) {
      leftClicked(event);
    } else if (event.rightUp() && !global_planning_) {
      rightClicked(event);
    }
  } else {
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }
  return Render;
}

// remove all the elements at index
void PolygonTool::deletePolygon(size_t index) {
  clearGlobalPlanning();
  if (index < active_spheres_.size() && is_activated_) {
    current_polygon_ = index;
    setColor(kTransparent, kTransparent);
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
void PolygonTool::makeVertex(const Ogre::Vector3& position) {
  // Create a new vertex in the Ogre scene and save scene to list.
  Ogre::SceneNode* node =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  rviz::Shape* local_sphere =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
  local_sphere->setColor(kRed);
  Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
  local_sphere->setScale(scale);
  local_sphere->setPosition(position);
  active_spheres_[current_polygon_].push_back(local_sphere);
  node->setPosition(position);
  vertex_nodes_[current_polygon_].push_back(node);
  Point local_2d_pt = Point(position[0], position[1]);
  polygon_[current_polygon_].push_back(local_2d_pt);

  if (current_type_ == kHull) {
    drawLines(kGreen);
  } else {
    drawLines(kYellow);
  }
}

// check geometric propetries of the CGal polygon
// this function tries to fuse all the polygons and holes into
// the main_polygon_ object. Overlapping holes are not allowed.
// Holes overlapping with the hull are included into the hull and removed
// Holes inside the hull are added to the holes of the CGAL polygon with holes
// object
// returns false if check failed, true otherwise
bool PolygonTool::checkCGalPolygon() {
  bool ret = false;
  main_polygon_.clear();
  // all polygons and holes should be simple and have at least 3 pts!
  for (size_t i = 0; i < polygon_.size(); ++i) {
    if (polygon_[i].size() < 3 || !polygon_[i].is_simple()) {
      break;
    } else if (polygon_[i].is_simple() && i == (polygon_.size() - 1)) {
      ret = true;
    }
  }
  /*
  check if all polygons (without holes) overlap
  ->as soon as a polygon cannot be joined with any other one
  the polygons are invalid
  */
  if (ret) {
    std::vector<Polygon_2_WH> polygons_no_holes;
    std::vector<Polygon_2> only_the_holes;
    for (size_t i = 0; i < polygon_.size(); ++i) {
      // MAKE SURE THE ORIENTATION IS THE SAME FOR ALL POLYGONS
      if (polygon_[i].is_clockwise_oriented()) {
        polygon_[i].reverse_orientation();
      }
      if (type_of_polygons_[i] == kHull) {
        Polygon_2_WH local_poly(polygon_[i]);
        polygons_no_holes.push_back(local_poly);
      } else if (type_of_polygons_[i] == kHole) {
        only_the_holes.push_back(polygon_[i]);
      }
    }
    for (size_t i = 1; i < polygons_no_holes.size(); ++i) {
      Polygon_2_WH local_copy_result;
      if (CGAL::join(polygons_no_holes[i], polygons_no_holes[0],
                     local_copy_result)) {
        if (local_copy_result.outer_boundary().is_simple()) {
          polygons_no_holes.erase(polygons_no_holes.begin() + i);
          Polygon_2_WH dummy_poly(local_copy_result.outer_boundary());
          polygons_no_holes[0] = dummy_poly;
          i = 0;
        }
      }
    }
    // making sure the holes don't overlap
    // if they do, the polygons are considered invalid
    if (ret) {
      for (size_t h1 = 0; h1 < only_the_holes.size(); h1++) {
        for (size_t h2 = (h1 + 1); h2 < only_the_holes.size(); h2++) {
          if (CGAL::do_intersect(only_the_holes[h1], only_the_holes[h2])) {
            ret = false;
            h1 = h2 = (only_the_holes.size() + 1);
          }
        }
      }
    }
    if (polygons_no_holes.size() == 1 && ret) {
      main_polygon_ = polygons_no_holes[0];
      // checking if the hole overlaps with the outer hull
      // if yes, it becomes part of the outer hull else it
      // will be included as a hole into main_polygon_
      for (size_t h = 0; h < only_the_holes.size(); ++h) {
        if (CGAL::do_intersect(main_polygon_, only_the_holes[h])) {
          Poly_wh_list poly_list;
          Polygon_2_WH local_hole(only_the_holes[h]);
          CGAL::difference(main_polygon_, local_hole,
                           std::back_inserter(poly_list));
          // making sure the hole doesn't split the area into 2 or more parts
          if (poly_list.size() == 1) {
            // checking if the hole is overlapping or completely inside
            if (poly_list.front().outer_boundary().size() >
                main_polygon_.outer_boundary().size()) {
              main_polygon_ = poly_list.front();
            } else {
              main_polygon_.add_hole(only_the_holes[h]);
            }
          } else {
            ret = false;
            break;
          }
        }
      }

      if (ret) drawPolyWithHoles(main_polygon_, kBlue);
    } else {
      ret = false;
    }
  }
  return ret;
}

void PolygonTool::drawPolyWithHoles(const Polygon_2_WH& to_be_painted,
                                    const Ogre::ColourValue color) {
  clearGlobalPlanning();
  hideIndividualPolygons();
  global_planning_ = true;
  Polygon_2 outer_bound = to_be_painted.outer_boundary();
  size_t bndry_size = to_be_painted.outer_boundary().size();
  for (size_t i = 0; i < bndry_size; ++i) {
    Point local_pt = to_be_painted.outer_boundary()[i];
    rviz::Shape* local_sphere =
        new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
    local_sphere->setColor(color);
    outer_boundary_.push_back(local_sphere);
    Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
    local_sphere->setScale(scale);
    Ogre::Vector3 position(local_pt[0], local_pt[1], 0.0);
    local_sphere->setPosition(position);
  }

  for (size_t i = 0; i < bndry_size; ++i) {
    rviz::Line* local_line =
        new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
    local_line->setPoints(outer_boundary_[i]->getPosition(),
                          outer_boundary_[(i + 1) % bndry_size]->getPosition());
    local_line->setColor(color);
    outer_boundary_lines_.push_back(local_line);
  }
  // show the real holes
  for (Polygon_2_WH::Hole_const_iterator hi = to_be_painted.holes_begin();
       hi != to_be_painted.holes_end(); ++hi) {
    std::vector<rviz::Shape*> local_shapes;
    std::vector<rviz::Line*> local_lines;
    size_t pts_size = hi->size();
    // draw the points
    for (size_t i = 0; i < pts_size; ++i) {
      rviz::Shape* local_sphere =
          new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
      local_sphere->setColor(color);
      Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
      local_sphere->setScale(scale);
      Point local_pt = (*hi)[i];
      Ogre::Vector3 position(local_pt[0], local_pt[1], 0.0);
      local_sphere->setPosition(position);
      local_shapes.push_back(local_sphere);
    }
    // draw the lines
    for (size_t i = 0; i < pts_size; ++i) {
      rviz::Line* local_line =
          new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
      local_line->setColor(color);
      local_line->setPoints(local_shapes[i]->getPosition(),
                            local_shapes[(i + 1) % pts_size]->getPosition());
      local_lines.push_back(local_line);
    }
    inner_pts_.push_back(local_shapes);
    inner_lines_.push_back(local_lines);
  }
}

void PolygonTool::clearGlobalPlanning() {
  for (size_t i = 0; i < outer_boundary_.size(); ++i) {
    outer_boundary_[i]->setColor(kTransparent);
    outer_boundary_lines_[i]->setColor(kTransparent);
  }
  outer_boundary_.clear();
  outer_boundary_lines_.clear();

  for (size_t i = 0; i < inner_pts_.size(); ++i) {
    for (size_t j = 0; j < inner_pts_[i].size(); ++j) {
      inner_pts_[i][j]->setColor(kTransparent);
      inner_lines_[i][j]->setColor(kTransparent);
    }
  }
  inner_pts_.clear();
  inner_lines_.clear();
}

// hides the individual polygons
// this needs to be called before showing the main_polygon_
// or when cleaning up
void PolygonTool::hideIndividualPolygons() {
  size_t current_status = current_polygon_;
  size_t loop_stop = type_of_polygons_.size();

  for (size_t i = 0; i < loop_stop; ++i) {
    current_polygon_ = i;
    setColor(kTransparent, kTransparent);
  }
  current_polygon_ = current_status;
}

// shows the individual polygons rather than the main_polygon_
void PolygonTool::showIndividualPolygons() {
  size_t current_status = current_polygon_;
  size_t local_current_type = current_type_;
  size_t loop_stop = type_of_polygons_.size();
  for (size_t i = 0; i < loop_stop; ++i) {
    if (i != current_status) {
      current_polygon_ = i;
      current_type_ = type_of_polygons_[i];
      setColorsLeaving();
    }
  }
  current_polygon_ = current_status;
  current_type_ = local_current_type;
  setColorsArriving();
}
// draws the lines when a new point has been added
// the last line is of a different color
void PolygonTool::drawLines(const Ogre::ColourValue& line_color) {
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
    // last one is done seperately since its color is different
    active_lines_[current_polygon_].clear();
    rviz::Line* last_line =
        new rviz::Line(scene_manager_, scene_manager_->getRootSceneNode());
    last_line->setPoints(
        vertex_nodes_[current_polygon_][0]->getPosition(),
        vertex_nodes_[current_polygon_]
                     [vertex_nodes_[current_polygon_].size() - 1]
                         ->getPosition());
    last_line->setColor(kRed);
    active_lines_[current_polygon_].push_back(last_line);

    // all the other lines
    for (size_t i = 1; i < vertex_nodes_[current_polygon_].size(); ++i) {
      rviz::Line* local_line =
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
  if (current_type_ == kHull) {
    setColor(kGreen, kGreen);
  } else {
    setColor(kPink, kPink);
  }
}

void PolygonTool::setColorsArriving() {
  clearGlobalPlanning();
  if (current_type_ == kHull) {
    setColor(kGreen, kRed);
  } else {
    setColor(kYellow, kRed);
  }
}

// sets the color for the current polygon
// a color
void PolygonTool::setColor(const Ogre::ColourValue& line_color,
                           const Ogre::ColourValue& sphere_color) {
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
    rviz::Line* last_line =
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
      rviz::Line* local_line =
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
void PolygonTool::leftClicked(rviz::ViewportMouseEvent& event) {
  clearGlobalPlanning();
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    makeVertex(intersection);
  }
}

// called when the right mouse boutton is clicked
// used to delete nodes within the range of the cursor node
void PolygonTool::rightClicked(rviz::ViewportMouseEvent& event) {
  // get the x y z coodinates of the click
  clearGlobalPlanning();
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  // VertexIterator vi = polygon_[current_polygon_].vertices_begin();
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
        std::vector<rviz::Shape*> new_locl_active_sphr;
        std::vector<Ogre::SceneNode*> new_locl_vrtx_nds;
        Polygon_2 new_locl_polygn;
        for (size_t j = i + 1; j < vertex_nodes_[current_polygon_].size();
             ++j) {
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

        if (current_type_ == kHull) {
          drawLines(kGreen);
        } else {
          drawLines(kYellow);
        }
        break;
      }
    }
  }
}

void PolygonTool::checkStatusCallback(const std_msgs::Bool& incomming) {
  bool to_return = checkCGalPolygon();
  std_msgs::Bool to_send;
  to_send.data = to_return;
  status_update_publisher_.publish(to_send);
  user_warn_publisher_.publish(to_send);
}

void PolygonTool::toolSelectCallback(const std_msgs::Int8& tool_num) {
  if (is_activated_) {
    if (global_planning_) {
      showIndividualPolygons();
      global_planning_ = false;
    }
    size_t incomming_num = tool_num.data;
    if (incomming_num < vertex_nodes_.size()) {
      setColorsLeaving();
      current_polygon_ = incomming_num;
      current_type_ = type_of_polygons_[current_polygon_];
      setColorsArriving();
    }
  }
}

void PolygonTool::newPolyCallback(const std_msgs::Int8& new_poly_type) {
  if (is_activated_) {
    if (global_planning_) {
      clearGlobalPlanning();
      showIndividualPolygons();
      global_planning_ = false;
    }
    int incomming_type = new_poly_type.data;
    int new_current_poly = vertex_nodes_.size();
    setColorsLeaving();
    pushBackElements(incomming_type);
    // needs to be done here using old size (before push back!)
    current_polygon_ = new_current_poly;
  }
}

void PolygonTool::deletePolyCallback(const std_msgs::Int8& delete_ind) {
  if (is_activated_) {
    if (global_planning_) {
      clearGlobalPlanning();
      showIndividualPolygons();
      global_planning_ = false;
    }
    int delete_location = delete_ind.data;
    deletePolygon(delete_location);
  }
}

void PolygonTool::polygonPublisherCallback(const std_msgs::Bool& incomming) {
  mav_planning_msgs::PolygonWithHoles pwh_msg;

  for (size_t i = 0; i < main_polygon_.outer_boundary().size(); ++i) {
    mav_planning_msgs::Point2D local_pt;
    local_pt.x = main_polygon_.outer_boundary()[i][0];
    local_pt.y = main_polygon_.outer_boundary()[i][1];
    pwh_msg.hull.points.push_back(local_pt);
  }

  for (Polygon_2_WH::Hole_const_iterator hi = main_polygon_.holes_begin();
       hi != main_polygon_.holes_end(); ++hi) {
    size_t pts_size = hi->size();
    mav_planning_msgs::Polygon2D local_poly;
    for (size_t i = 0; i < pts_size; ++i) {
      mav_planning_msgs::Point2D local_pt;
      local_pt.x = (*hi)[i][0];
      local_pt.y = (*hi)[i][1];
      local_poly.points.push_back(local_pt);
    }
    pwh_msg.holes.push_back(local_poly);
  }
  polygon_wh_publisher_.publish(pwh_msg);
}

}  // namespace mav_polygon_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_polygon_tool::PolygonTool, rviz::Tool)
