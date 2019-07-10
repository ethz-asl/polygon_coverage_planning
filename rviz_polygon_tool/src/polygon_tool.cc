#include "rviz_polygon_tool/polygon_tool.h"

#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>

namespace rviz_polygon_tool {

const Ogre::ColourValue kRed = Ogre::ColourValue(1.f, 0.f, 0.f, 1.0);
const Ogre::ColourValue kGreen = Ogre::ColourValue(0.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kBlue = Ogre::ColourValue(0.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kPink = Ogre::ColourValue(1.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kYellow = Ogre::ColourValue(1.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kTransparent = Ogre::ColourValue(0.f, 0.f, 0.f, 0.0);

// Point scales.
const float kPtScale = 0.8;
const float kDeleteTol = 0.8;

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

void PolygonTool::clickLeft(rviz::ViewportMouseEvent& event) {
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
  ROS_INFO_STREAM("Polygon: " << *polygon_selection_);
}

void PolygonTool::clickRight(rviz::ViewportMouseEvent& event) {
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
        break;
      }
    }
  }
}

void PolygonTool::renderPolygon(const Polygon_2& polygon,
                                const Ogre::ColourValue& c) {
  // Render vertices.
  for (auto v = polygon.vertices_begin(); v != polygon.vertices_end(); ++v) {
    rviz::Shape* sphere =
        new rviz::Shape(rviz::Shape::Sphere, scene_manager_, polygon_node_);
    sphere->setColor(c);
    sphere->setScale(Ogre::Vector3(kPtScale));
    Ogre::Vector3 p(CGAL::to_double(v->x()), CGAL::to_double(v->y()), 0.0);
    sphere->setPosition(p);
  }
  // Render edges.
  for (auto e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
    rviz::Line* line = new rviz::Line(scene_manager_, polygon_node_);
    line->setColor(c);
    Ogre::Vector3 start(CGAL::to_double(e->source().x()),
                        CGAL::to_double(e->source().y()), 0.0);
    Ogre::Vector3 end(CGAL::to_double(e->target().x()),
                      CGAL::to_double(e->target().y()), 0.0);
    line->setPoints(start, end);
  }
}

void PolygonTool::renderPolygons() {
  ROS_ASSERT(polygon_node_);

  polygon_node_->removeAllChildren();  // Clear polygon visualization.
  for (auto p = polygons_.begin(); p != polygons_.end(); ++p) {
    Ogre::ColourValue c = kBlue;
    renderPolygon(*p, c);
  }
}

//
// void PolygonTool::drawPolyWithHoles(const Polygon_2_WH& to_be_painted,
//                                     const Ogre::ColourValue color) {
//   hideIndividualPolygons();
//   global_planning_ = true;
//   Polygon_2 outer_bound = to_be_painted.outer_boundary();
//   size_t bndry_size = to_be_painted.outer_boundary().size();
//   for (size_t i = 0; i < bndry_size; ++i) {
//     Point local_pt = to_be_painted.outer_boundary()[i];
//     rviz::Shape* local_sphere =
//         new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
//     local_sphere->setColor(color);
//     outer_boundary_.push_back(local_sphere);
//     Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
//     local_sphere->setScale(scale);
//     Ogre::Vector3 position(local_pt[0], local_pt[1], 0.0);
//     local_sphere->setPosition(position);
//   }
//
//   for (size_t i = 0; i < bndry_size; ++i) {
//     rviz::Line* local_line =
//         new rviz::Line(scene_manager_,
//         scene_manager_->getRootSceneNode());
//     local_line->setPoints(outer_boundary_[i]->getPosition(),
//                           outer_boundary_[(i + 1) %
//                           bndry_size]->getPosition());
//     local_line->setColor(color);
//     outer_boundary_lines_.push_back(local_line);
//   }
//   // show the real holes
//   for (Polygon_2_WH::Hole_const_iterator hi =
//   to_be_painted.holes_begin();
//        hi != to_be_painted.holes_end(); ++hi) {
//     std::vector<rviz::Shape*> local_shapes;
//     std::vector<rviz::Line*> local_lines;
//     size_t pts_size = hi->size();
//     // draw the points
//     for (size_t i = 0; i < pts_size; ++i) {
//       rviz::Shape* local_sphere =
//           new rviz::Shape(rviz::Shape::Sphere, scene_manager_);
//       local_sphere->setColor(color);
//       Ogre::Vector3 scale(kPtScale, kPtScale, kPtScale);
//       local_sphere->setScale(scale);
//       Point local_pt = (*hi)[i];
//       Ogre::Vector3 position(local_pt[0], local_pt[1], 0.0);
//       local_sphere->setPosition(position);
//       local_shapes.push_back(local_sphere);
//     }
//     // draw the lines
//     for (size_t i = 0; i < pts_size; ++i) {
//       rviz::Line* local_line =
//           new rviz::Line(scene_manager_,
//           scene_manager_->getRootSceneNode());
//       local_line->setColor(color);
//       local_line->setPoints(local_shapes[i]->getPosition(),
//                             local_shapes[(i + 1) %
//                             pts_size]->getPosition());
//       local_lines.push_back(local_line);
//     }
//     inner_pts_.push_back(local_shapes);
//     inner_lines_.push_back(local_lines);
//   }
// }
//
// // called when the right mouse boutton is clicked
// // used to delete nodes within the range of the cursor node
// void PolygonTool::clickRight(rviz::ViewportMouseEvent& event) {
//   // get the x y z coodinates of the click
//   Ogre::Vector3 intersection;
//   Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
//   // VertexIterator vi = polygon_[current_polygon_].vertices_begin();
//   if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane,
//   event.x,
//                                         event.y, intersection)) {
//     for (size_t i = 0; i < vertex_nodes_[current_polygon_].size(); ++i) {
//       // compare the distance from the center of the nodes already drawn
//       Ogre::Vector3 distance_vec =
//           intersection -
//           vertex_nodes_[current_polygon_][i]->getPosition();
//       if (distance_vec.length() < kDeleteTol) {
//         // make node dissapear
//         vertex_nodes_[current_polygon_][i]->setVisible(false);
//         active_spheres_[current_polygon_][i]->setColor(0.0, 0, 0, 0.0);
//         // re arrange nodes
//         std::vector<rviz::Shape*> new_locl_active_sphr;
//         std::vector<Ogre::SceneNode*> new_locl_vrtx_nds;
//         Polygon_2 new_locl_polygn;
//         for (size_t j = i + 1; j <
//         vertex_nodes_[current_polygon_].size();
//              ++j) {
//           new_locl_active_sphr.push_back(active_spheres_[current_polygon_][j]);
//           new_locl_vrtx_nds.push_back(vertex_nodes_[current_polygon_][j]);
//           new_locl_polygn.push_back(polygon_[current_polygon_][j]);
//         }
//         for (size_t j = 0; j < i; ++j) {
//           new_locl_active_sphr.push_back(active_spheres_[current_polygon_][j]);
//           new_locl_vrtx_nds.push_back(vertex_nodes_[current_polygon_][j]);
//           new_locl_polygn.push_back(polygon_[current_polygon_][j]);
//         }
//         // swapping the new elements
//         swap(active_spheres_[current_polygon_], new_locl_active_sphr);
//         swap(vertex_nodes_[current_polygon_], new_locl_vrtx_nds);
//         swap(polygon_[current_polygon_], new_locl_polygn);
//
//         if (current_type_ == kHull) {
//           drawLines(kGreen);
//         } else {
//           drawLines(kYellow);
//         }
//         break;
//       }
//     }
//   }
// }
//
// void PolygonTool::checkStatusCallback(const std_msgs::Bool& incomming) {
//   bool to_return = checkCGALPolygon();
//   std_msgs::Bool to_send;
//   to_send.data = to_return;
//   status_update_publisher_.publish(to_send);
//   user_warn_publisher_.publish(to_send);
// }
//
// void PolygonTool::toolSelectCallback(const std_msgs::Int8& tool_num) {
//   if (is_active_) {
//     if (global_planning_) {
//       showIndividualPolygons();
//       global_planning_ = false;
//     }
//     size_t incomming_num = tool_num.data;
//     if (incomming_num < vertex_nodes_.size()) {
//       setColorsLeaving();
//       current_polygon_ = incomming_num;
//       current_type_ = type_of_polygons_[current_polygon_];
//       setColorsArriving();
//     }
//   }
// }
//
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
