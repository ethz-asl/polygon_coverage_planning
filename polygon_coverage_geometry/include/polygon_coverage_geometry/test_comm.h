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

#ifndef POLYGON_COVERAGE_PLANNING_TEST_COMMON_H_
#define POLYGON_COVERAGE_PLANNING_TEST_COMMON_H_

#include <ros/assert.h>
#include <cstdlib>

#include <CGAL/Kernel/global_functions.h>
#include <CGAL/Random.h>
#include <CGAL/algorithm.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/random_polygon_2.h>

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

double createRandomDouble(double min, double max) {
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

template <class Polygon, class PolygonWithHoles>
PolygonWithHoles createRectangleInRectangle() {
  typedef typename Polygon::Point_2 Point_2;
  Polygon outer;
  outer.push_back(Point_2(0.0, 0.0));
  outer.push_back(Point_2(2.0, 0.0));
  outer.push_back(Point_2(2.0, 2.0));
  outer.push_back(Point_2(0.0, 2.0));

  Polygon hole;
  hole.push_back(Point_2(1.0, 1.25));
  hole.push_back(Point_2(1.0, 1.75));
  hole.push_back(Point_2(0.5, 1.75));
  hole.push_back(Point_2(0.5, 1.25));

  PolygonWithHoles poly_with_holes(outer);
  poly_with_holes.add_hole(hole);
  return poly_with_holes;
}

template <class Polygon>
Polygon createDiamond() {
  typedef typename Polygon::Point_2 Point_2;
  Polygon diamond;
  diamond.push_back(Point_2(1.0, 0.0));
  diamond.push_back(Point_2(2.0, 1.0));
  diamond.push_back(Point_2(1.0, 2.0));
  diamond.push_back(Point_2(0.0, 1.0));

  return diamond;
}

template <class Polygon>
Polygon createBCDCell() {
  typedef typename Polygon::Point_2 Point_2;
  Polygon bcd_cell;

  bcd_cell.push_back(Point_2(-5.5, 9.1));
  bcd_cell.push_back(Point_2(-5.5, 5.9));
  bcd_cell.push_back(Point_2(-0.252786, 5.9));
  bcd_cell.push_back(Point_2(3.5, 4.02361));
  bcd_cell.push_back(Point_2(3.5, 9.1));

  return bcd_cell;
}

template <class Polygon, class PolygonWithHoles>
PolygonWithHoles createUltimateBCDTest() {
  typedef typename Polygon::Point_2 Point_2;

  Polygon outer;
  outer.push_back(Point_2(0.0, 0.0));
  outer.push_back(Point_2(11.0, 0.0));
  outer.push_back(Point_2(11.0, 1.0));
  outer.push_back(Point_2(10.0, 1.0));
  outer.push_back(Point_2(11.0, 2.0));
  outer.push_back(Point_2(10.0, 3.0));
  outer.push_back(Point_2(10.0, 4.0));
  outer.push_back(Point_2(11.0, 4.0));
  outer.push_back(Point_2(11.0, 5.0));
  outer.push_back(Point_2(7.0, 6.0));
  outer.push_back(Point_2(0.0, 5.0));
  outer.push_back(Point_2(0.0, 3.0));
  outer.push_back(Point_2(1.0, 3.0));
  outer.push_back(Point_2(1.0, 2.0));
  outer.push_back(Point_2(0.0, 2.0));
  outer.push_back(Point_2(1.0, 1.0));
  PolygonWithHoles pwh(outer);

  Polygon hole;
  hole.push_back(Point_2(1.0, 5.0));
  hole.push_back(Point_2(2.0, 5.0));
  hole.push_back(Point_2(2.0, 4.0));
  hole.push_back(Point_2(1.0, 4.0));
  pwh.add_hole(hole);

  hole = Polygon();
  hole.push_back(Point_2(3.0, 4.0));
  hole.push_back(Point_2(4.0, 5.0));
  hole.push_back(Point_2(5.0, 4.0));
  hole.push_back(Point_2(5.0, 3.0));
  pwh.add_hole(hole);

  hole = Polygon();
  hole.push_back(Point_2(5.0, 2.0));
  hole.push_back(Point_2(7.0, 2.0));
  hole.push_back(Point_2(7.0, 1.0));
  hole.push_back(Point_2(5.0, 1.0));
  pwh.add_hole(hole);

  return pwh;
}

template <class Polygon, class PolygonWithHoles>
PolygonWithHoles createSophisticatedPolygon() {
  Polygon outer;
  outer.push_back(Point_2(0.0, 0.0));
  outer.push_back(Point_2(10.0, 0.0));
  outer.push_back(Point_2(10.0, 4.5));
  outer.push_back(Point_2(12.0, 4.5));
  outer.push_back(Point_2(12.0, 2.0));
  outer.push_back(Point_2(16.0, 2.0));
  outer.push_back(Point_2(16.0, 8.0));
  outer.push_back(Point_2(12.0, 8.0));
  outer.push_back(Point_2(12.0, 5.5));
  outer.push_back(Point_2(10.0, 5.5));
  outer.push_back(Point_2(10.0, 10.0));
  outer.push_back(Point_2(0.0, 10.0));

  Polygon hole;
  hole.push_back(Point_2(3.0, 3.0));
  hole.push_back(Point_2(3.0, 7.0));
  hole.push_back(Point_2(7.0, 7.0));
  hole.push_back(Point_2(7.0, 3.0));

  PolygonWithHoles poly_with_holes(outer);
  poly_with_holes.add_hole(hole);
  return poly_with_holes;
}

template <class Kernel>
bool checkVerticesIdentical(const typename Kernel::Point_2& a,
                            const typename Kernel::Point_2& b) {
  return a == b;
}

template <class Kernel>
void correctVertices(std::vector<typename Kernel::Point_2>* vertices) {
  ROS_ASSERT(vertices);
  // Delete identical adjacent vertices.
  typename std::vector<typename Kernel::Point_2>::iterator it =
      std::adjacent_find(vertices->begin(), vertices->end(),
                         checkVerticesIdentical<Kernel>);
  while (it != vertices->end()) {
    vertices->erase(it);
    it = std::adjacent_find(vertices->begin(), vertices->end(),
                            checkVerticesIdentical<Kernel>);
  }
  // Check first and last:
  if (checkVerticesIdentical<Kernel>(vertices->front(), vertices->back())) {
    vertices->pop_back();
  }
}

template <class Polygon, class Kernel>
Polygon createRandomConvexPolygon(double x0, double y0, double r) {
  // http://stackoverflow.com/questions/21690008/how-to-generate-random-vertices-to-form-a-convex-polygon-in-c
  double a, x, y;
  std::vector<typename Kernel::Point_2> v;
  for (a = 0.0; a > -2.0 * M_PI;)  // full circle
  {
    x = x0 + (r * std::cos(a));
    y = y0 + (r * std::sin(a));
    // random angle step [20 .. 169] degrees
    a -= (20.0 + double((std::rand() % 150))) * M_PI / 180.0;

    v.push_back(typename Kernel::Point_2(x, y));
  }
  correctVertices<Kernel>(&v);

  return Polygon(Polygon_2(v.begin(), v.end()));
}

template <class Polygon, class Kernel>
Polygon createRandomSimplePolygon(double r, CGAL::Random& random,
                                  int max_poly_size) {
  typedef typename Kernel::Point_2 Point_2;
  Polygon_2 polygon;
  std::list<Point_2> point_set;
  int size = random.get_int(4, max_poly_size);
  // copy size points from the generator, eliminating duplicates, so the
  // polygon will have <= size vertices
  typedef CGAL::Creator_uniform_2<int, Point_2> Creator;
  typedef CGAL::Random_points_in_square_2<Point_2, Creator> Point_generator;
  CGAL::copy_n_unique(Point_generator(r, random), size,
                      std::back_inserter(point_set));
  CGAL::random_polygon_2(point_set.size(), std::back_inserter(polygon),
                         point_set.begin());
  return Polygon(polygon);
}

// Sample 3 random points in cube with side length 2a and generate plane.
template <class Kernel>
typename Kernel::Plane_3 createRandomPlane(double a, CGAL::Random& random) {
  typedef typename Kernel::Point_3 Point_3;
  typedef CGAL::Creator_uniform_3<int, Point_3> Creator;
  typedef CGAL::Random_points_in_cube_3<Point_3, Creator> PointGenerator;
  Point_3 p0 = *PointGenerator(a, random), p1 = p0, p2 = p0;
  while (p0 == p1) p1 = *PointGenerator(a, random);
  while (CGAL::collinear(p0, p1, p2)) p2 = *PointGenerator(a, random);

  return Plane_3(p0, p1, p2);
}

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_PLANNING_TEST_COMMON_H_
