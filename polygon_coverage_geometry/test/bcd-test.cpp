#include <CGAL/is_y_monotone_2.h>
#include <gtest/gtest.h>

#include "polygon_coverage_geometry/bcd.h"
#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(BctTest, computeBCD) {
  // Diamond.
  PolygonWithHoles diamond(createDiamond<Polygon_2>());
  FT expected_area = diamond.outer_boundary().area();
  std::vector<Polygon_2> bcd = computeBCD(diamond, Direction_2(1, 0));
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
  bcd = computeBCD(rectangle_in_rectangle, Direction_2(1, 0));
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
  bcd = computeBCD(pwh, Direction_2(1, 0));
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
  return RUN_ALL_TESTS();
}
