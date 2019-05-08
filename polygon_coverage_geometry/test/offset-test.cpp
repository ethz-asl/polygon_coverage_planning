#include <gtest/gtest.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/offset.h"
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(OffsetTest, OffsetPolygon) {
  PolygonWithHoles poly =
      createSophisticatedPolygon<Polygon_2, PolygonWithHoles>();

  for (size_t i = 0; i < 100; ++i) {
    FT max_offset = createRandomDouble(0.0, 10.0);
    PolygonWithHoles offset_polygon;
    computeOffsetPolygon(poly, max_offset, &offset_polygon);
    EXPECT_LE(computeArea(offset_polygon), computeArea(poly));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
