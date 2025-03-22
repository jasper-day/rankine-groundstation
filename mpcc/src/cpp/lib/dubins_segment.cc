#include "dubins_segment.h"

#include <algorithm>
#include <cmath>

#include <drake/common/autodiff.h>
#include <drake/common/default_scalars.h>
#include <drake/common/eigen_types.h>
#include <drake/common/trajectories/piecewise_constant_curvature_trajectory.h>
#include <drake/solvers/mathematical_program.h>

namespace mpcc {
namespace dubins {

// Line segment

template <typename T>
T LineSegment<T>::length() const {
  return (end() - start()).norm();
}

template <typename T>
T LineSegment<T>::heading_start() const {
  drake::Vector2<T> dir = end() - start();
  using std::atan2;
  // slope of line with correct sign
  return atan2(dir.y(), dir.x());
}

template <typename T>
T LineSegment<T>::heading_end() const {
  // line segments are straight
  return heading_start();
}

template <typename T>
drake::Vector2<T> LineSegment<T>::eval(T arclength) const {
  drake::Vector2<T> dir = end() - start();
  T total_length = length();
  using std::clamp;
  T t = clamp(arclength / total_length, T(0), T(1));
  return start() + t * dir;
}

template <typename T>
drake::Vector2<T> LineSegment<T>::path_coords(drake::Vector2<T> point) const {
  drake::Vector2<T> to_point = point - start();
  drake::Vector2<T> to_end = end() - start();
  T length = to_end.norm();
  T arclength = to_point.dot(to_end);
  T normal_dist = (to_point - arclength * to_end / length).norm();
  using std::clamp;
  arclength = clamp(arclength, T(0), length);
  return drake::Vector2<T>(arclength, normal_dist);
}

// Circular segment

template <typename T>
drake::Vector2<T> CircularSegment<T>::start() const {
  return eval(T(0));
}

template <typename T>
drake::Vector2<T> CircularSegment<T>::end() const {
  return eval(arclength_);
}

template <typename T>
drake::Vector2<T> CircularSegment<T>::eval(T arclength) const {
  using std::clamp;
  arclength = clamp(arclength, T(0), arclength_);
  T angle = heading_ + dir_ * arclength / radius_;
  using std::cos;
  using std::sin;
  return center_ + radius_ * drake::Vector2<T>(cos(angle), sin(angle));
}

template <typename T>
drake::Vector2<T> CircularSegment<T>::path_coords(
    drake::Vector2<T> point) const {
  // this implementation is incorrect, because we can go around the
  // circle multiple times. We have effectively added a branch cut and are not
  // tracking winding number. In a system, the distance can only increase, and
  // path integration should take care of it.
  drake::Vector2<T> to_point = point - center_;
  T dist = to_point.norm();
  using std::atan2;
  T angle = atan2(to_point.y(), to_point.x()) - heading_start();
  T arclength = angle * radius_ * dir_;
  using std::clamp;
  arclength = clamp(arclength, T(0), arclength_);
  T normal_dist = (dist - radius_) * dir_;
  return drake::Vector2<T>(arclength, normal_dist);
}

}  // namespace dubins
}  // namespace mpcc

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::Segment);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::CircularSegment);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::LineSegment);