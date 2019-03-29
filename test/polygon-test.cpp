#include <cstdlib>
#include <fstream>

#include <gtest/gtest.h>
#include <boost/make_shared.hpp>

#include "mav_2d_coverage_planning/geometry/bcd_exact.h"
#include "mav_2d_coverage_planning/geometry/polygon.h"
#include "mav_2d_coverage_planning/tests/test_helpers.h"

#include <CGAL/is_y_monotone_2.h>
#include <mav_coverage_planning_comm/eigen_conversions.h>

using namespace mav_coverage_planning;

TEST(PolygonTest, Offset) {
  Polygon poly(createSophisticatedPolygon<Polygon_2, PolygonWithHoles>());

  for (size_t i = 0; i < 100; ++i) {
    FT max_offset = createRandomDouble(0.0, 10.0);
    Polygon offset_polygon;
    EXPECT_TRUE(poly.computeOffsetPolygon(max_offset, &offset_polygon));
    EXPECT_LE(offset_polygon.computeArea(), poly.computeArea());
  }
}

TEST(PolygonTest, OffsetEdge) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon rectangle(rectangle_in_rectangle.getPolygon().outer_boundary());
  const double kOffset = 0.1;
  double area = CGAL::to_double(rectangle.computeArea());
  for (size_t i = 0; i < 4; ++i) {
    Polygon offsetted_polygon;
    EXPECT_TRUE(rectangle.offsetEdge(i, kOffset, &offsetted_polygon));
    double area_offsetted = CGAL::to_double(offsetted_polygon.computeArea());
    double expected_difference =
        kOffset *
        std::sqrt(CGAL::to_double(
            rectangle.getPolygon().outer_boundary().edge(i).squared_length()));
    const double kPrecision = 1.0e-3;
    EXPECT_LE(area - expected_difference - area_offsetted, kPrecision)
        << offsetted_polygon;
  }
}

TEST(PolygonTest, ConvertPolygonWithHolesToPolygonWithoutHoles) {
  LOG(INFO) << "Create polygon.";
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon poly_without_holes;
  LOG(INFO) << "Run test.";
  EXPECT_TRUE(
      rectangle_in_rectangle.convertPolygonWithHolesToPolygonWithoutHoles(
          &poly_without_holes));
  LOG(INFO) << "Check number of holes.";
  EXPECT_EQ(0, poly_without_holes.getPolygon().number_of_holes());
  EXPECT_EQ(10, poly_without_holes.getPolygon().outer_boundary().size());
}

TEST(PolygonTest, ConvexDecomposition) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  std::vector<Polygon> convex_polygons;
  EXPECT_TRUE(
      rectangle_in_rectangle.computeConvexDecompositionFromPolygonWithHoles(
          &convex_polygons));
  EXPECT_EQ(8, convex_polygons.size());
}

TEST(PolygonTest, BCDecomposition) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  std::vector<Polygon> bc_polygons;
  EXPECT_TRUE(
      rectangle_in_rectangle.computeBCDFromPolygonWithHoles(&bc_polygons));
  EXPECT_EQ(4, bc_polygons.size());
}

TEST(PolygonTest, YMonotoneDecomposition) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  std::vector<Polygon> y_monotone_polygons;
  EXPECT_TRUE(
      rectangle_in_rectangle.computeYMonotoneDecompositionFromPolygonWithHoles(
          &y_monotone_polygons));
  EXPECT_EQ(4, y_monotone_polygons.size());
}

TEST(PolygonTest, pointInOnPolygon) {
  Polygon rect_in_rect(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  // Point on vertex.
  Point_2 p(0.0, 0.0);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(p));

  // Point on edge.
  Point_2 q(1.0, 0.0);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(q));

  // Point in interior.
  Point_2 r(1.0, 1.0);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(r));

  // Point on hole vertex.
  Point_2 s(1.0, 1.25);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(s));

  // Point on hole edge.
  Point_2 t(0.75, 1.75);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(t));

  // Point in hole.
  Point_2 u(0.75, 1.5);
  EXPECT_FALSE(rect_in_rect.pointInPolygon(u));

  // Point outside polygon.
  Point_2 v(100.0, 100.0);
  EXPECT_FALSE(rect_in_rect.pointInPolygon(v));
}

TEST(PolygonTest, computeLineSweepPlan) {
  const double kMaxSweepDistance = 0.1;
  const double kStartVertexIdx = 0;
  const bool kCounterClockwise = true;

  Polygon diamond(createDiamond<Polygon_2>());
  std::vector<Point_2> waypoints;
  EXPECT_TRUE(diamond.computeLineSweepPlan(kMaxSweepDistance, kStartVertexIdx,
                                           kCounterClockwise, &waypoints));
  EXPECT_GE(waypoints.size(), 4);
  for (const Point_2& p : waypoints) EXPECT_TRUE(diamond.pointInPolygon(p));
}

TEST(PolygonText, computeLineSweepPlanBCDCell) {
  const double kMaxSweepDistance = 1.0;

  Polygon polygon(createBCDCell<Polygon_2>());
  std::vector<std::vector<Point_2>> cluster_sweeps;

  bool cc_orientation = true;
  std::vector<EdgeConstIterator> dirs_swept;
  for (size_t start_id = 0;
       start_id < polygon.getPolygon().outer_boundary().size(); ++start_id) {
    // Don't sweep same direction multiple times.
    EdgeConstIterator dir = std::next(
        polygon.getPolygon().outer_boundary().edges_begin(), start_id);
    std::vector<EdgeConstIterator>::iterator it =
        std::find_if(dirs_swept.begin(), dirs_swept.end(),
                     [&dir](const EdgeConstIterator& dir_swept) {
                       return CGAL::parallel(*dir, *dir_swept);
                     });
    if (it != dirs_swept.end()) {
      DLOG(INFO) << "Direction already swept.";
      continue;
    }
    dirs_swept.push_back(dir);

    // Create 4 sweeps. Along direction, along opposite direction and reverse.
    std::vector<Point_2> sweep;
    if (!polygon.computeLineSweepPlan(kMaxSweepDistance, start_id, cc_orientation,
                                      &sweep)) {
      LOG(WARNING)
          << "Could not compute counter clockwise sweep plan for start_id: "
          << start_id << " in polygon: " << polygon;
    } else {
      LOG(INFO) << "Adding cc sweep.";
      for (const Point_2& p : sweep) LOG(INFO) << p;
      cluster_sweeps.push_back(sweep);
      std::reverse(sweep.begin(), sweep.end());
      LOG(INFO) << "Adding cc reverse sweep.";
      for (const Point_2& p : sweep) LOG(INFO) << p;
      cluster_sweeps.push_back(sweep);
    }

    if (!polygon.computeLineSweepPlan(
            kMaxSweepDistance,
            (start_id + 1) % polygon.getPolygon().outer_boundary().size(),
            !cc_orientation, &sweep)) {
      LOG(WARNING) << "Could not compute clockwise sweep plan for start_id: "
                   << start_id << " in polygon: " << polygon;
    } else {
      LOG(INFO) << "Adding cw sweep.";
      for (const Point_2& p : sweep) LOG(INFO) << p;
      cluster_sweeps.push_back(sweep);
      std::reverse(sweep.begin(), sweep.end());
      LOG(INFO) << "Adding cw reverse sweep.";
      for (const Point_2& p : sweep) LOG(INFO) << p;
      cluster_sweeps.push_back(sweep);
    }
  }

  EXPECT_EQ(dirs_swept.size(), 3);
  EXPECT_EQ(cluster_sweeps.size(), dirs_swept.size() * 4);
}

TEST(PolygonTest, computeVisibilityPolygon) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon visibility_polygon;

  // Query on polygon vertex.
  Point_2 query(0.0, 0.0);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  VertexConstIterator vit =
      visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(9, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(0.0, 2.0), *vit++);
  EXPECT_EQ(Point_2(0.0, 0.0), *vit++);
  EXPECT_EQ(Point_2(2.0, 0.0), *vit++);
  EXPECT_EQ(Point_2(2.0, 2.0), *vit++);
  vit++;  // Skip inexact point. (1.6,2)
  EXPECT_EQ(Point_2(1.0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.75), *vit++);
  vit++;  // Skip inexact point. (0.571429,2)

  // Query on hole vertex.
  query = Point_2(1.0, 1.25);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(6, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);

  // Query in face.
  query = Point_2(1.0, 0.5);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(7, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 2), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);

  // Query on polygon halfedge.
  query = Point_2(1.0, 0.0);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(8, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(0, 2), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  // EXPECT_EQ(Point_2(0.2, 2), *vit++); Skip inexact.

  // Query on hole halfedge.
  query = Point_2(0.75, 1.25);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(4, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(2, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);

  // Query outside.
  query = Point_2(100.0, 100.0);
  EXPECT_FALSE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
}

TEST(PolygonTest, projectPointOnHull) {
  Polygon poly(createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  // Point outside closest to vertex.
  Point_2 p(-1.0, -1.0);
  EXPECT_EQ(Point_2(0.0, 0.0), poly.projectPointOnHull(p));

  // Point inside closest to bottom edge.
  p = Point_2(0.1, 0.05);
  EXPECT_EQ(Point_2(0.1, 0.0), poly.projectPointOnHull(p));

  // Point inside hole closest to bottom edge.
  p = Point_2(0.75, 1.3);
  EXPECT_EQ(Point_2(0.75, 1.25), poly.projectPointOnHull(p));

  // Point inside polygon closest to hole bottom edge.
  p = Point_2(0.75, 1.2);
  EXPECT_EQ(Point_2(0.75, 1.25), poly.projectPointOnHull(p));

  // Point on edge.
  p = Point_2(0.5, 0);
  EXPECT_EQ(p, poly.projectPointOnHull(p));

  // Point on vertex.
  p = Point_2(0, 0);
  EXPECT_EQ(p, poly.projectPointOnHull(p));
}

TEST(PolygonTest, toMesh) {
  Polygon poly(createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  Polyhedron_3 mesh = poly.toMesh();
  EXPECT_TRUE(mesh.is_pure_triangle());
  EXPECT_FALSE(mesh.is_closed());

  for (Polyhedron_3::Vertex_iterator it = mesh.vertices_begin();
       it != mesh.vertices_end(); ++it) {
    Point_3 p_3 = convertPoint3<Polyhedron_3::Point_3, Point_3>(it->point());
    Point_2 p_2 = poly.getPlaneTransformation().to2d(p_3);
    EXPECT_TRUE(poly.pointInPolygon(p_2));
  }

  std::ofstream out("/tmp/rectangle_in_rectangle.off");
  out << mesh;
}

TEST(PolygonTest, BCDExact) {
  // Diamond.
  PolygonWithHoles diamond(createDiamond<Polygon_2>());
  FT expected_area = diamond.outer_boundary().area();
  std::vector<Polygon_2> bcd = computeBCDExact(diamond, Direction_2(1, 0));
  EXPECT_EQ(bcd.size(), 1);
  EXPECT_EQ(bcd[0].size(), diamond.outer_boundary().size()) << bcd[0];
  FT area = 0.0;
  for (const Polygon_2& p : bcd) {
    EXPECT_TRUE(CGAL::is_y_monotone_2(p.vertices_begin(), p.vertices_end()));
    EXPECT_EQ(p.size(), 4);
    area += p.area();
  }
  EXPECT_EQ(area, expected_area);

  // Rectangle in rectangle.
  PolygonWithHoles rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  expected_area = rectangle_in_rectangle.outer_boundary().area();
  for (PolygonWithHoles::Hole_const_iterator hit =
           rectangle_in_rectangle.holes_begin();
       hit != rectangle_in_rectangle.holes_end(); ++hit) {
    expected_area -= hit->area();
  }
  bcd = computeBCDExact(rectangle_in_rectangle, Direction_2(1, 0));
  EXPECT_EQ(bcd.size(), 4);
  area = 0.0;
  for (const Polygon_2& p : bcd) {
    EXPECT_TRUE(CGAL::is_y_monotone_2(p.vertices_begin(), p.vertices_end()));
    EXPECT_EQ(p.size(), 4);
    area += p.area();
  }
  EXPECT_EQ(area, expected_area);

  // Ultimate test.
  PolygonWithHoles pwh(createUltimateBCDTest<Polygon_2, PolygonWithHoles>());
  expected_area = pwh.outer_boundary().area();

  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    expected_area += hit->area();
  }
  bcd = computeBCDExact(pwh, Direction_2(1, 0));
  EXPECT_EQ(bcd.size(), 14);
  area = 0.0;
  for (const Polygon_2& p : bcd) {
    Direction_2 dir(0, 1);
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    Polygon_2 rot_p = CGAL::transform(rotation, p);
    EXPECT_TRUE(
        CGAL::is_y_monotone_2(rot_p.vertices_begin(), rot_p.vertices_end()));
    area += p.area();
  }
  EXPECT_EQ(area, expected_area);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
