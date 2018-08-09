#ifndef MAV_2D_COVERAGE_PLANNING_DEFINITIONS_H_
#define MAV_2D_COVERAGE_PLANNING_DEFINITIONS_H_

#include <functional>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_2 Point_2;
typedef K::Vector_2 Vector_2;
typedef K::Line_2 Line_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef Polygon_2::Vertex_const_iterator VertexConstIterator;
typedef Polygon_2::Vertex_const_circulator VertexConstCirculator;
typedef Polygon_2::Edge_const_iterator EdgeConstIterator;
typedef Polygon_2::Edge_const_circulator EdgeConstCirculator;
typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;

typedef std::function<double(const std::vector<Point_2>& path)>
    PathCostFunctionType;
typedef std::function<double(const Point_2& a, const Point_2& b)>
    SegmentCostFunctionType;

#endif  // MAV_2D_COVERAGE_PLANNING_DEFINITIONS_H_
