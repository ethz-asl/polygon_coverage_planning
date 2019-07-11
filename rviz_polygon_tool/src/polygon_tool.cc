#include "rviz_polygon_tool/polygon_tool.h"

#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>

namespace rviz_polygon_tool {

// Colors.
const Ogre::ColourValue kRed = Ogre::ColourValue(1.f, 0.f, 0.f, 1.0);
const Ogre::ColourValue kGreen = Ogre::ColourValue(0.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kBlue = Ogre::ColourValue(0.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kPink = Ogre::ColourValue(1.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kYellow = Ogre::ColourValue(1.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kTransparent = Ogre::ColourValue(0.f, 0.f, 0.f, 0.0);

// Status.
const QString kLeftClick = "<b>Left-Click:</b> Insert a new vertex";
const QString kRightClick = "<b>Right-Click:</b> Remove a vertex";
const QString kH = "<b>h:</b> Add hole";
const QString kP = "<b>n:</b> Select next polygon";
const QString kV = "<b>v:</b> Select next vertex";
const QString kR = "<b>r:</b> Reset polygon";
const QString kC = "<b>c:</b> Clear all";
const QString kEnter = "<b>Enter:</b> Publish polygon";
const QString kSelection = "Current Selection: ";
const QString kStatus = kLeftClick + ", " + kRightClick + ", " + kH + ", " +
                        kP + ", " + kV + ", " + kR + ", " + kC + ", " + kEnter +
                        ", " + kSelection;

// Point scales.
const float kPtScale = 0.5;
const float kDeleteTol = 0.5;

PolygonTool::PolygonTool()
    : Tool(),
      polygons_({Polygon_2()}),
      polygon_selection_(polygons_.begin()),
      vertex_selection_(polygon_selection_->vertices_begin()),
      polygon_node_(nullptr),
      moving_vertex_node_(nullptr),
      sphere_(nullptr) {
  shortcut_key_ = 'p';
}

PolygonTool::~PolygonTool() {}

void PolygonTool::onInitialize() {
  // OGRE nodes.
  moving_vertex_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  sphere_ =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, moving_vertex_node_);
  moving_vertex_node_->setVisible(false);

  polygon_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  polygon_node_->setVisible(true);

  // ROS.
  polygon_pub_ = nh_.advertise<polygon_coverage_msgs::PolygonWithHolesStamped>(
      "polygon", 1, true);

  updateStatus();
}

void PolygonTool::activate() {
  // Make nodes visible.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(true);
  }
  if (polygon_node_) {
    polygon_node_->setVisible(true);
  }
}

void PolygonTool::deactivate() {
  // Make nodes invisible.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(false);
  }
  if (polygon_node_) {
    polygon_node_->setVisible(false);
  }
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  if (!moving_vertex_node_) {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    moving_vertex_node_->setVisible(false);
    moving_vertex_node_->setPosition(intersection);

    if (event.leftUp()) {
      clickLeft(event);
    } else if (event.rightUp()) {
      clickRight(event);
    }
  } else {
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }

  return Render;
}

void PolygonTool::clickLeft(const rviz::ViewportMouseEvent& event) {
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    createVertex(intersection);
  }
  renderPolygons();
}

void PolygonTool::createVertex(const Ogre::Vector3& position) {
  // Add a vertex to the current polygon.
  Point_2 new_point(position.x, position.y);
  vertex_selection_ = polygon_selection_->insert(vertex_selection_, new_point);
}

void PolygonTool::clickRight(const rviz::ViewportMouseEvent& event) {
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    deleteVertex(intersection);
  }
  renderPolygons();
}

void PolygonTool::deleteVertex(const Ogre::Vector3& position) {
  // Select vertex close by.
  for (auto p = polygons_.begin(); p != polygons_.end(); ++p) {
    for (auto v = p->vertices_begin(); v != p->vertices_end(); ++v) {
      Ogre::Vector3 pt(CGAL::to_double(v->x()), CGAL::to_double(v->y()), 0.0);
      if ((position - pt).length() < kDeleteTol) {
        polygon_selection_ = p;
        vertex_selection_ = v;
        vertex_selection_ = polygon_selection_->erase(v);
        if (vertex_selection_ == polygon_selection_->vertices_end())
          vertex_selection_ = polygon_selection_->vertices_begin();
        break;
      }
    }
  }
}

void PolygonTool::renderPolygon(const Polygon_2& polygon,
                                const Ogre::ColourValue& c) {
  // Render vertices and edges.
  for (auto v = polygon.vertices_begin(); v != polygon.vertices_end(); ++v) {
    // Render vertices as spheres.
    rviz::Shape* sphere =
        new rviz::Shape(rviz::Shape::Sphere, scene_manager_, polygon_node_);
    sphere->setColor(c);
    sphere->setScale(Ogre::Vector3(kPtScale));
    Ogre::Vector3 p(CGAL::to_double(v->x()), CGAL::to_double(v->y()), 0.0);
    sphere->setPosition(p);
    if (v == vertex_selection_) sphere->setColor(kGreen);

    // Render edges as lines.
    if (polygon.size() < 2) continue;
    auto v_prev = v == polygon.vertices_begin()
                      ? std::prev(polygon.vertices_end())
                      : std::prev(v);
    Ogre::Vector3 start(CGAL::to_double(v_prev->x()),
                        CGAL::to_double(v_prev->y()), 0.0);
    rviz::Line* line = new rviz::Line(scene_manager_, polygon_node_);
    line->setColor(c);
    line->setPoints(start, p);
    if (v == vertex_selection_) line->setColor(kGreen);
  }
}

void PolygonTool::renderPolygons() {
  ROS_ASSERT(polygon_node_);

  polygon_node_->removeAllChildren();  // Clear polygon visualization.
  for (auto p = polygons_.begin(); p != polygons_.end(); ++p) {
    // Hull is blue, holes are red. Selected polygon is yellow.
    Ogre::ColourValue c = kRed;
    if (p == polygon_selection_) {
      c = kYellow;
    } else if (p == polygons_.begin()) {
      c = kBlue;
    }

    renderPolygon(*p, c);
  }
}

int PolygonTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) {
  if (event->text() == 'h') {
    addHole();
  } else if (event->text() == 'n') {
    nextPolygon();
  } else if (event->text() == 'v') {
    nextVertex();
  } else if (event->text() == 'r') {
    resetPolygon();
  } else if (event->text() == 'c') {
    clearAll();
  } else if (event->matches(QKeySequence::InsertParagraphSeparator)) {
    publishPolygon();
  }

  updateStatus();
  renderPolygons();
  return Render;
}

void PolygonTool::addHole() {
  removeEmptyHoles();
  // Add a new hole.
  polygons_.push_back(Polygon_2());
  polygon_selection_ = std::prev(polygons_.end());
  vertex_selection_ = polygon_selection_->vertices_begin();
}

void PolygonTool::nextPolygon() {
  removeEmptyHoles();
  polygon_selection_ = std::next(polygon_selection_) == polygons_.end()
                           ? polygons_.begin()
                           : std::next(polygon_selection_);
  vertex_selection_ = polygon_selection_->vertices_begin();
}

void PolygonTool::nextVertex() {
  vertex_selection_ = std::next(vertex_selection_);
  if (vertex_selection_ == polygon_selection_->vertices_end())
    vertex_selection_ = polygon_selection_->vertices_begin();
}

void PolygonTool::resetPolygon() {
  polygon_selection_->clear();
  if (polygon_selection_ != polygons_.begin()) {
    // This is a hole. Remove it completely.
    polygon_selection_ = polygons_.erase(polygon_selection_);
    polygon_selection_ = std::prev(polygon_selection_);
  }
  vertex_selection_ = polygon_selection_->vertices_begin();
}
void PolygonTool::clearAll() {
  polygons_ = {Polygon_2()};
  polygon_selection_ = polygons_.begin();
  vertex_selection_ = polygon_selection_->vertices_begin();
}
void PolygonTool::publishPolygon() {}

void PolygonTool::updateStatus() {
  if (polygon_selection_ == polygons_.begin()) {
    setStatus(kStatus + " Hull");
  } else {
    int idx = std::distance(polygons_.begin(), polygon_selection_) - 1;
    setStatus(kStatus + " Hole " + QString::number(idx));
  }
}

void PolygonTool::removeEmptyHoles() {
  // Remove all empty holes.
  auto new_end =
      std::remove_if(std::next(polygons_.begin()), polygons_.end(),
                     [](const Polygon_2& p) { return p.is_empty(); });
  if (new_end == polygons_.end()) return;  // Nothing to do.
  // Erase and update polygon selection.
  polygon_selection_ = polygons_.erase(new_end, polygons_.end());
  polygon_selection_ = polygon_selection_ == polygons_.begin()
                           ? polygons_.begin()
                           : std::prev(polygon_selection_);
  vertex_selection_ = polygon_selection_->vertices_begin();
}

// void PolygonTool::newPolyCallback(const std_msgs::Int8& new_poly_type) {
//   if (is_active_) {
//     if (global_planning_) {
//       showIndividualPolygons();
//       global_planning_ = false;
//     }
//     int incomming_type = new_poly_type.data;
//     int new_current_poly = vertex_nodes_.size();
//     setColorsLeaving();
//     pushBackElements(incomming_type);
//     // needs to be done here using old size (before push back!)
//     current_polygon_ = new_current_poly;
//   }
// }
//
// void PolygonTool::deletePolyCallback(const std_msgs::Int8& delete_ind) {
//   if (is_active_) {
//     if (global_planning_) {
//       showIndividualPolygons();
//       global_planning_ = false;
//     }
//     int delete_location = delete_ind.data;
//     deletePolygon(delete_location);
//   }
// }
//
// void PolygonTool::polygonPublisherCallback(const std_msgs::Bool&
// incomming) {
//   mav_planning_msgs::PolygonWithHoles pwh_msg;
//
//   for (size_t i = 0; i < main_polygon_.outer_boundary().size(); ++i) {
//     mav_planning_msgs::Point2D local_pt;
//     local_pt.x = main_polygon_.outer_boundary()[i][0];
//     local_pt.y = main_polygon_.outer_boundary()[i][1];
//     pwh_msg.hull.points.push_back(local_pt);
//   }
//
//   for (Polygon_2_WH::Hole_const_iterator hi =
//   main_polygon_.holes_begin();
//        hi != main_polygon_.holes_end(); ++hi) {
//     size_t pts_size = hi->size();
//     mav_planning_msgs::Polygon2D local_poly;
//     for (size_t i = 0; i < pts_size; ++i) {
//       mav_planning_msgs::Point2D local_pt;
//       local_pt.x = (*hi)[i][0];
//       local_pt.y = (*hi)[i][1];
//       local_poly.points.push_back(local_pt);
//     }
//     pwh_msg.holes.push_back(local_poly);
//   }
//   polygon_pub_.publish(pwh_msg);
// }

}  // namespace rviz_polygon_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_tool::PolygonTool, rviz::Tool)
