/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <vector>

#include <ros/assert.h>
#include <ros/console.h>

#include "polygon_coverage_geometry/bcd.h"
#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/pwh_indicator.h"

namespace polygon_coverage_planning {


void calculateSortedVertexIndicators(const PolygonWithHoles & pwh, std::vector<PWHIndicator> & sorted_vertex_indicator) {
    for (int j = 0; j < (int)pwh.outer_boundary().size(); ++j) {
        sorted_vertex_indicator.emplace_back(&pwh, 0, j);
    }
    for (int j = 0; j < (int)pwh.number_of_holes(); ++j) {
        for (int k = 0; k < (int)pwh.holes()[j].size(); ++k) {
            sorted_vertex_indicator.emplace_back(&pwh, j+1, k);
        }
    }
    Polygon_2::Traits::Less_xy_2 less_xy_2;
    std::sort(sorted_vertex_indicator.begin(), sorted_vertex_indicator.end(),
              [&less_xy_2](const PWHIndicator & a,
                           const PWHIndicator & b) -> bool {
                  return less_xy_2(a.getVertex(), b.getVertex());
              });
}



int getIntersectionsWithSweepLine(const std::list<PWHIndicator> & current_edge_list, const PWHIndicator & v,
                                     std::vector<Point_2> & intersection_points) {
    intersection_points.reserve(current_edge_list.size());
    auto v_point = v.getVertex();
    Line_2 l(v_point, Direction_2(0, 1));

    size_t cnt = 0;
    size_t range_index = 0;
    bool range_found_flag = false;
    for (auto & edge_i : current_edge_list) {
        auto edge = edge_i.getEdge();
        auto result = CGAL::intersection(edge, l);
        Point_2 inter_point;
        
        if(result) {
          if (boost::get<Segment_2>(&*result)) {
              inter_point = (edge.target().x() > edge.source().x()) ? (edge.target()) : (edge.source());
          } else {
              inter_point = *(boost::get<Point_2>(&*result));
          }
        } else {
          // It is imposible that the current edge has no intersection with the sweep line.
          return -1;
        }
        intersection_points.push_back(inter_point);
        if(!range_found_flag && inter_point.y() > v_point.y())  {
            range_found_flag = true;
            range_index = cnt;
        }
        cnt++;
    }
    if(range_found_flag) {
        return (int)range_index;
    } else {
        return (int)cnt;
    }
}


void closeOpenPolygon(std::list<Point_2> & open_polygon, Polygon_2 & closed_polygon, const Point_2& up_point,
                      const Point_2& down_point) {
    Segment_2 close_edge(down_point, up_point);
    auto close_line = close_edge.supporting_line();
    if(!close_line.has_on(open_polygon.front())) {
        open_polygon.push_front(up_point);
    }
    if(!close_line.has_on(open_polygon.back())) {
        open_polygon.push_back(down_point);
    }
    for(auto & open_polygon_point : open_polygon) {
        closed_polygon.push_back(open_polygon_point);
    }
}

void closeOpenPolygon(std::list<Point_2> & open_polygon, Polygon_2 & closed_polygon, const Point_2& point) {
    Segment_2 close_edge(open_polygon.front(), open_polygon.back());
    auto close_line = close_edge.supporting_line();
    if(!close_line.has_on(point)) {
        open_polygon.push_front(point);
    }
    for(auto & open_polygon_point : open_polygon) {
        closed_polygon.push_back(open_polygon_point);
    }
}

void openPolygonAddPointUp(std::list<Point_2> & open_polygon, const Point_2 & point) {
    auto begin = open_polygon.begin();
    auto end = open_polygon.end();
    if(begin == end) {
        open_polygon.push_front(point);
    } else {
        auto next_begin = std::next(begin);
        if (next_begin == end) {
            open_polygon.push_front(point);
        } else {
            Line_2 line(*next_begin, *begin);
            if(line.has_on(point)) {
                *begin = point;
            } else {
                open_polygon.push_front(point);
            }
        }
    }
}

void openPolygonAddPointDown(std::list<Point_2> & open_polygon, const Point_2 & point) {
    auto begin = open_polygon.rbegin();
    auto end = open_polygon.rend();
    if(begin == end) {
        open_polygon.push_back(point);
    } else {
        auto next_begin = std::next(begin);
        if (next_begin == end) {
            open_polygon.push_back(point);
        } else {
            Line_2 line(*next_begin, *begin);
            if(line.has_on(point)) {
                *begin = point;
            } else {
                open_polygon.push_back(point);
            }
        }
    }
}


void processEvent(const PWHIndicator & v, std::list<PWHIndicator> & current_edge_list,
                  std::list<std::list<Point_2>>& open_polygons,
                  std::vector<Polygon_2>& closed_polygons) {
    auto v_point = v.getVertex();

    auto next_edge_i = v;
    auto next_edge = next_edge_i.getEdge();
    auto next_edge_end_point = next_edge.target();

    auto pre_edge_i = v.prev();
    auto pre_edge = pre_edge_i.getEdge();
    auto pre_edge_end_point = pre_edge.source();

    bool next_edge_in_list = false;
    bool pre_edge_in_list = false;
    size_t cnt = 0;
    size_t next_edge_index = 0;
    size_t pre_edge_index = 0;
    for(auto & edge_i : current_edge_list) {
        if(edge_i == next_edge_i) {
            next_edge_in_list = true;
            next_edge_index = cnt;
        } else if(edge_i == pre_edge_i) {
            pre_edge_in_list = true;
            pre_edge_index = cnt;
        }
        cnt++;
    }

    if(next_edge_in_list && pre_edge_in_list) {
        size_t min_index = std::min(pre_edge_index, next_edge_index);
        auto start_iter = std::next(current_edge_list.begin(), min_index);
        if(min_index & 0x01) {
            // close two old open polygon and add a new one.
            auto new_up_edge_i = *(std::next(start_iter, 2));
            auto new_down_edge_i = *(std::prev(start_iter, 1));
            std::list<PWHIndicator> new_edge_list;
            new_edge_list.push_back(new_down_edge_i);
            new_edge_list.push_back(new_up_edge_i);
            std::vector<Point_2> intersection_points;
            getIntersectionsWithSweepLine(new_edge_list, v, intersection_points);
            // config old two open polygon.
            auto down_open_polygon_iter = std::next(open_polygons.begin(), (min_index-1)/2);
            auto up_open_polygon_iter = std::next(down_open_polygon_iter);

            closed_polygons.emplace_back();
            closeOpenPolygon(*down_open_polygon_iter, closed_polygons.back(), v_point, intersection_points.front());
            closed_polygons.emplace_back();
            closeOpenPolygon(*up_open_polygon_iter, closed_polygons.back(), intersection_points.back(), v_point);

            // remove from the open polygons.
            down_open_polygon_iter = open_polygons.erase(down_open_polygon_iter);
            down_open_polygon_iter = open_polygons.erase(down_open_polygon_iter);
            // add new open polygon
            std::list<Point_2> new_open_polygon;
            openPolygonAddPointUp(new_open_polygon, intersection_points.back());
            openPolygonAddPointDown(new_open_polygon, intersection_points.front());

            open_polygons.insert(down_open_polygon_iter, new_open_polygon);

        } else {
            // close a open polygon.
            // update old open polygon vertex.
            auto open_polygon_iter = std::next(open_polygons.begin(), min_index/2);
            // add old open polygon to close polygons' container.
            closed_polygons.emplace_back();
            closeOpenPolygon(*open_polygon_iter, closed_polygons.back(), v_point);
            open_polygons.erase(open_polygon_iter);
        }
        start_iter = current_edge_list.erase(start_iter);
        current_edge_list.erase(start_iter);
    } else if(next_edge_in_list) {
        auto start_iter = std::next(current_edge_list.begin(), next_edge_index);
        if(next_edge_index & 0x01) {
            auto open_polygon_iter = std::next(open_polygons.begin(), (next_edge_index-1)/2);
            openPolygonAddPointUp(*open_polygon_iter, v_point);
        } else {
            auto open_polygon_iter = std::next(open_polygons.begin(), next_edge_index/2);
            openPolygonAddPointDown(*open_polygon_iter, v_point);
        }
        *start_iter = pre_edge_i;
    } else if(pre_edge_in_list) {
        auto start_iter = std::next(current_edge_list.begin(), pre_edge_index);
        if(pre_edge_index & 0x01) {
            auto open_polygon_iter = std::next(open_polygons.begin(), (pre_edge_index-1)/2);
            openPolygonAddPointUp(*open_polygon_iter, v_point);
        } else {
            auto open_polygon_iter = std::next(open_polygons.begin(), pre_edge_index/2);
            openPolygonAddPointDown(*open_polygon_iter, v_point);
        }
        *start_iter = next_edge_i;
    } else {
        auto up_edge_i = next_edge_i;
        auto down_edge_i = pre_edge_i;
        Segment_2 pre_edge_(v_point, pre_edge_end_point);
        if(pre_edge_.supporting_line().has_on_negative_side(next_edge_end_point)) {
            up_edge_i = pre_edge_i;
            down_edge_i = next_edge_i;
        }
        std::vector<Point_2> intersection_points;
        size_t range_index = getIntersectionsWithSweepLine(current_edge_list, v, intersection_points);
        auto start_iter = std::next(current_edge_list.begin(), range_index);
        if(range_index & 0x01) {
            // split a open range to two.

            // update old open polygon vertex.
            auto start_point = intersection_points[range_index];
            auto end_point = intersection_points[range_index-1];
            auto open_polygon_iter = std::next(open_polygons.begin(), (range_index-1)/2);
            closed_polygons.emplace_back();
            // add old open polygon to close polygons' container.
            closeOpenPolygon(*open_polygon_iter, closed_polygons.back(), start_point, end_point);
            // remove the old open polygons.
            open_polygon_iter = open_polygons.erase(open_polygon_iter);
            // config two new open polygon
            std::list<Point_2> down_polygon;
            openPolygonAddPointUp(down_polygon, v_point);
            openPolygonAddPointDown(down_polygon, end_point);
            std::list<Point_2> up_polygon;
            openPolygonAddPointUp(up_polygon, start_point);
            openPolygonAddPointDown(up_polygon, v_point);
            // add two new open polygons.
            open_polygon_iter = open_polygons.insert(open_polygon_iter, up_polygon);
            open_polygons.insert(open_polygon_iter, down_polygon);
        } else {
            // add a new open range.
            // config new open polygon.
            auto open_polygon_iter = std::next(open_polygons.begin(), (range_index)/2);
            std::list<Point_2> new_polygon;
            openPolygonAddPointUp(new_polygon, v_point);
            // add new open polygon.
            open_polygons.insert(open_polygon_iter, new_polygon);
        }
        start_iter = current_edge_list.insert(start_iter, up_edge_i);
        current_edge_list.insert(start_iter, down_edge_i);
    }
}


std::vector<Polygon_2> computeBCD(const PolygonWithHoles& polygon_in,
                                  const Direction_2& dir) {
  // Rotate polygon to have direction aligned with x-axis.
  // TODO(rikba): Make this independent of rotation.
  PolygonWithHoles rotated_polygon = rotatePolygon(polygon_in, dir);
  simplifyPolygon(&rotated_polygon);

  // Sort vertices by x value.
  std::vector<PWHIndicator> sorted_vertex_indicators;
  calculateSortedVertexIndicators(rotated_polygon, sorted_vertex_indicators);

  // Initialize edge list.
  std::list<std::list<Point_2>> open_polygons;
  std::vector<Polygon_2> closed_polygons;
  std::list<PWHIndicator> current_line_indicators;

  for (auto & v : sorted_vertex_indicators) {
      processEvent(v, current_line_indicators, open_polygons, closed_polygons);
  }

  // Rotate back all polygons.
  for (Polygon_2& p : closed_polygons) {
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    p = CGAL::transform(rotation, p);
  }

  return closed_polygons;
}


}  // namespace polygon_coverage_planning
