#ifndef MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_
#define MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_

#include <glog/logging.h>
#include <cstdlib>

#include <CGAL/Random.h>
#include <CGAL/algorithm.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>

namespace mav_coverage_planning {

double createRandomDouble(double min, double max) {
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

template <class Polygon, class PolygonWithHoles>
PolygonWithHoles createRectangleInRectangle() {
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
  Polygon diamond;
  diamond.push_back(Point_2(1.0, 0.0));
  diamond.push_back(Point_2(2.0, 1.0));
  diamond.push_back(Point_2(1.0, 2.0));
  diamond.push_back(Point_2(0.0, 1.0));

  return diamond;
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
  CHECK_NOTNULL(vertices);
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
    x = x0 + (r * cos(a));
    y = y0 + (r * sin(a));
    // random angle step [20 .. 169] degrees
    a -= (20.0 + double((std::rand() % 150))) * M_PI / 180.0;

    v.push_back(typename Kernel::Point_2(x, y));
  }
  correctVertices<Kernel>(&v);

  return Polygon(v.begin(), v.end());
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
  return polygon;
}

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_TESTS_TEST_HELPERS_H_
