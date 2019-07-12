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

#include "polygon_coverage_geometry/triangulation.h"

#include <ros/assert.h>

namespace polygon_coverage_planning {

void mark_domains(CDT& ct, CDT::Face_handle start, int index,
                  std::list<CDT::Edge>& border) {
  if (start->info().nesting_level != -1) {
    return;
  }
  std::list<CDT::Face_handle> queue;
  queue.push_back(start);
  while (!queue.empty()) {
    CDT::Face_handle fh = queue.front();
    queue.pop_front();
    if (fh->info().nesting_level == -1) {
      fh->info().nesting_level = index;
      for (int i = 0; i < 3; i++) {
        CDT::Edge e(fh, i);
        CDT::Face_handle n = fh->neighbor(i);
        if (n->info().nesting_level == -1) {
          if (ct.is_constrained(e))
            border.push_back(e);
          else
            queue.push_back(n);
        }
      }
    }
  }
}

void mark_domains(CDT& cdt) {
  for (CDT::All_faces_iterator it = cdt.all_faces_begin();
       it != cdt.all_faces_end(); ++it) {
    it->info().nesting_level = -1;
  }
  std::list<CDT::Edge> border;
  mark_domains(cdt, cdt.infinite_face(), 0, border);
  while (!border.empty()) {
    CDT::Edge e = border.front();
    border.pop_front();
    CDT::Face_handle n = e.first->neighbor(e.second);
    if (n->info().nesting_level == -1) {
      mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
    }
  }
}

void triangulatePolygon(const PolygonWithHoles& pwh,
                        std::vector<std::vector<Point_2>>* faces) {
  ROS_ASSERT(faces);
  faces->clear();

  CDT cdt;

  cdt.insert_constraint(pwh.outer_boundary().vertices_begin(),
                        pwh.outer_boundary().vertices_end(), true);

  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    cdt.insert_constraint(hit->vertices_begin(), hit->vertices_end(), true);
  }

  mark_domains(cdt);

  for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
       fit != cdt.finite_faces_end(); ++fit) {
    if (fit->info().in_domain()) {
      std::vector<Point_2> face = {fit->vertex(0)->point(),
                                   fit->vertex(1)->point(),
                                   fit->vertex(2)->point()};
      faces->push_back(face);
    }
  }
}

}  // namespace polygon_coverage_planning
