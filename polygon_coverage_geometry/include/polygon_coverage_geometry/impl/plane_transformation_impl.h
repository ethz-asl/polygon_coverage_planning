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

#ifndef POLYGON_COVERAGE_GEOMETRY_PLANE_TRANSFORMATION_IMPL_H_
#define POLYGON_COVERAGE_GEOMETRY_PLANE_TRANSFORMATION_IMPL_H_

#include <CGAL/number_utils.h>
#include <cmath>

namespace polygon_coverage_planning {

template <class Kernel>
PlaneTransformation<Kernel>::PlaneTransformation(const Plane_3& plane)
    : plane_(plane) {
  // Base vectors.
  const double kZeroDevisionGuard = 1.0e-6;
  if (abs(plane.a()) > kZeroDevisionGuard) {
    projection_type_ = ProjectionType::kYZ;
    b_1_ = Vector_3(-plane.b() / plane.a(), 1.0, 0.0);
    b_2_ = Vector_3(-plane.c() / plane.a(), 0.0, 1.0);
  } else if (abs(plane.c()) > kZeroDevisionGuard) {
    projection_type_ = ProjectionType::kXY;
    b_1_ = Vector_3(1.0, 0.0, -plane.a() / plane.c());
    b_2_ = Vector_3(0.0, 1.0, -plane.b() / plane.c());
  } else {
    projection_type_ = ProjectionType::kZX;
    b_1_ = Vector_3(0.0, -plane.c() / plane.b(), 1.0);
    b_2_ = Vector_3(1.0, -plane.a() / plane.b(), 0.0);
  }
  b_1_ = normalize(b_1_);
  b_2_ = normalize(b_2_);

  // Zero point.
  const FT c = -plane.d() / (plane.a() * plane.a() + plane.b() * plane.b() +
                             plane.c() * plane.c());
  p_0_ = c * Vector_3(plane.a(), plane.b(), plane.c());
}

template <class Kernel>
typename Kernel::Vector_3 PlaneTransformation<Kernel>::normalize(
    const Vector_3& v) const {
  double len = std::sqrt(CGAL::to_double(v.squared_length()));
  return Vector_3(CGAL::to_double(v.x()) / len, CGAL::to_double(v.y()) / len,
                  CGAL::to_double(v.z()) / len);
}

template <class Kernel>
typename Kernel::Point_2 PlaneTransformation<Kernel>::to2d(
    const Point_3& p_3) const {
  FT x, y;
  switch (projection_type_) {
    case ProjectionType::kYZ: {
      x = (p_3.y() - p_0_.y()) / b_1_.y();
      y = (p_3.z() - p_0_.z()) / b_2_.z();
      break;
    }
    case ProjectionType::kXY: {
      x = (p_3.x() - p_0_.x()) / b_1_.x();
      y = (p_3.y() - p_0_.y()) / b_2_.y();
      break;
    }
    case ProjectionType::kZX: {
      x = (p_3.z() - p_0_.z()) / b_1_.z();
      y = (p_3.x() - p_0_.x()) / b_2_.x();
      break;
    }
    default: {
      x = 0;
      y = 0;
      break;
    }
  }
  return Point_2(x, y);
}

template <class Kernel>
typename std::vector<typename Kernel::Point_2>
PlaneTransformation<Kernel>::to2d(const std::vector<Point_3>& p_3) const {
  std::vector<Point_2> p_2(p_3.size());
  typename std::vector<Point_3>::const_iterator p_3it = p_3.begin();
  for (typename std::vector<Point_2>::iterator p_2it = p_2.begin();
       p_2it != p_2.end(); ++p_2it, ++p_3it) {
    *p_2it = to2d(*p_3it);
  }
  return p_2;
}

template <class Kernel>
typename Kernel::Point_3 PlaneTransformation<Kernel>::to3d(
    const Point_2& p_2) const {
  Vector_3 v_3 = p_0_ + p_2.x() * b_1_ + p_2.y() * b_2_;
  return Point_3(v_3.x(), v_3.y(), v_3.z());
}

template <class Kernel>
typename std::vector<typename Kernel::Point_3>
PlaneTransformation<Kernel>::to3d(const std::vector<Point_2>& p_2) const {
  std::vector<Point_3> p_3(p_2.size());
  typename std::vector<Point_2>::const_iterator p_2it = p_2.begin();
  for (typename std::vector<Point_3>::iterator p_3it = p_3.begin();
       p_3it != p_3.end(); ++p_2it, ++p_3it) {
    *p_3it = to3d(*p_2it);
  }
  return p_3;
}

template <class Kernel>
typename Kernel::Point_3 PlaneTransformation<Kernel>::to3dOnPlane(
    const Point_2& p_2) const {
  // Correct inexact vertices to lie in plane.
  return plane_.projection(to3d(p_2));
}

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_PLANE_TRANSFORMATION_IMPL_H_
