#ifndef MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_
#define MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_

#include <glog/logging.h>
#include <cstdlib>

#include <CGAL/Random.h>
#include <CGAL/algorithm.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

#include "mav_2d_coverage_planning/polygon.h"

namespace mav_coverage_planning {

double createRandomDouble(double min, double max) {
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

Polygon createRectangleInRectangle() {
  Polygon_2 outer;
  outer.push_back(Point_2(0.0, 0.0));
  outer.push_back(Point_2(2.0, 0.0));
  outer.push_back(Point_2(2.0, 2.0));
  outer.push_back(Point_2(0.0, 2.0));

  Polygon_2 hole;
  hole.push_back(Point_2(1.0, 1.25));
  hole.push_back(Point_2(1.0, 1.75));
  hole.push_back(Point_2(0.5, 1.75));
  hole.push_back(Point_2(0.5, 1.25));

  PolygonWithHoles poly_with_holes(outer);
  poly_with_holes.add_hole(hole);
  return Polygon(poly_with_holes);
}

Polygon createDiamond() {
  Polygon_2 outer;
  outer.push_back(Point_2(1.0, 0.0));
  outer.push_back(Point_2(2.0, 1.0));
  outer.push_back(Point_2(1.0, 2.0));
  outer.push_back(Point_2(0.0, 1.0));

  PolygonWithHoles poly(outer);
  return Polygon(poly);
}

Polygon createSophisticatedPolygon() {
  Polygon_2 outer;

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

  Polygon_2 hole;

  hole.push_back(Point_2(3.0, 3.0));
  hole.push_back(Point_2(3.0, 7.0));
  hole.push_back(Point_2(7.0, 7.0));
  hole.push_back(Point_2(7.0, 3.0));

  PolygonWithHoles poly_with_holes(outer);
  poly_with_holes.add_hole(hole);
  return Polygon(poly_with_holes);
}

bool checkVerticesIdentical(const Point_2& a, const Point_2& b) {
  return a == b;
}

void correctVertices(std::vector<Point_2>* vertices) {
  CHECK_NOTNULL(vertices);
  // Delete identical adjacent vertices.
  std::vector<Point_2>::iterator it = std::adjacent_find(
      vertices->begin(), vertices->end(), checkVerticesIdentical);
  while (it != vertices->end()) {
    vertices->erase(it);
    it = std::adjacent_find(vertices->begin(), vertices->end(),
                            checkVerticesIdentical);
  }
  // Check first and last:
  if (checkVerticesIdentical(vertices->front(), vertices->back())) {
    vertices->pop_back();
  }
}

bool createRandomConvexPolygon(double x0, double y0, double r,
                               Polygon* convex_polygon) {
  CHECK_NOTNULL(convex_polygon);
  // http://stackoverflow.com/questions/21690008/how-to-generate-random-vertices-to-form-a-convex-polygon-in-c
  double a, x, y;
  std::vector<Point_2> v;
  for (a = 0.0; a > -2.0 * M_PI;)  // full circle
  {
    x = x0 + (r * cos(a));
    y = y0 + (r * sin(a));
    // random angle step [20 .. 169] degrees
    a -= (20.0 + double((std::rand() % 150))) * M_PI / 180.0;

    v.push_back(Point_2(x, y));
  }
  correctVertices(&v);

  *convex_polygon = Polygon(v.begin(), v.end());
  return v.size() > 2;
}

bool createRandomSimplePolygon(double r, CGAL::Random& random,
                               int max_poly_size, Polygon* simple_polygon) {
  CHECK_NOTNULL(simple_polygon);

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
  *simple_polygon = Polygon(polygon);

  return simple_polygon->getPolygon().outer_boundary().size() > 2;
}

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_
