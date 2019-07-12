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
