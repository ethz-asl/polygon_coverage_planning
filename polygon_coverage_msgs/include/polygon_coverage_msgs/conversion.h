#ifndef POLYGON_COVERAGE_MSGS_CONVERSION_H_
#define POLYGON_COVERAGE_MSGS_CONVERSION_H_

#include <geometry_msgs/Polygon.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_msgs/PolygonWithHoles.h>

namespace polygon_coverage_planning {

void convertPolygonWithHolesToMsg(const PolygonWithHoles& pwh,
                                  const double altitude,
                                  polygon_coverage_msgs::PolygonWithHoles* msg);

void convertPolygonToMsg(const Polygon_2& p, const double altitude,
                         geometry_msgs::Polygon* msg);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_MSGS_CONVERSION_H_
