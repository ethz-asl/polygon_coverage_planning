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

#ifndef POLYGON_COVERAGE_GEOMETRY_TRIANGULATION_H_
#define POLYGON_COVERAGE_GEOMETRY_TRIANGULATION_H_

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include "polygon_coverage_geometry/cgal_definitions.h"

// See
// https://doc.cgal.org/latest/Triangulation_2/index.html#Section_2D_Triangulations_Constrained_Delaunay

namespace polygon_coverage_planning {

struct FaceInfo2 {
  FaceInfo2() {}
  int nesting_level;
  bool in_domain() { return nesting_level % 2 == 1; }
};

typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K> Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;

void mark_domains(CDT& ct, CDT::Face_handle start, int index,
                  std::list<CDT::Edge>& border);

void mark_domains(CDT& cdt);

void triangulatePolygon(const PolygonWithHoles& pwh,
                        std::vector<std::vector<Point_2>>* faces);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_TRIANGULATION_H_
