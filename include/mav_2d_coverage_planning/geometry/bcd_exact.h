#ifndef MAV_2D_COVERAGE_PLANNING_GEOMETRY_BCD_EXACT_H_
#define MAV_2D_COVERAGE_PLANNING_GEOMETRY_BCD_EXACT_H_

#include <mav_coverage_planning_comm/cgal_definitions.h>

// Choset, Howie. "Coverage of known spaces: The boustrophedon cellular
// decomposition." Autonomous Robots 9.3 (2000): 247-253.
// https://www.cs.cmu.edu/~motionplanning/lecture/Chap6-CellDecomp_howie.pdf
namespace mav_coverage_planning {
std::vector<Polygon_2> computeBCDExact(const PolygonWithHoles& polygon_in,
                                       const Direction_2& dir);
std::vector<VertexConstCirculator> getXSortedVertices(
    const PolygonWithHoles& p);
PolygonWithHoles rotatePolygon(const PolygonWithHoles& polygon_in,
                               const Direction_2& dir);
void processEvent(const VertexConstCirculator& v,
                  const std::vector<Point_2>& processed_vertices,
                  std::list<Segment_2>* L, std::list<Polygon_2>* open_polygons,
                  std::vector<Polygon_2>* closed_polygons);
std::vector<Point_2> getIntersections(
    const std::vector<Point_2>& processed_vertices,
    const std::list<Segment_2>& L, const Line_2& l);
}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_GEOMETRY_BCD_EXACT_H_
