#ifndef MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_
#define MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_

#include <cstdlib>

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
}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_
