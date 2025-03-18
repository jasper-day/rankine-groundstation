#include "dubins.h"
#include <drake/common/eigen_types.h>
#include <drake/common/autodiff.h>
#include <cmath>

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
  return atan2(dir.y(), dir.x());
}

template <typename T>
T LineSegment<T>::heading_end() const {
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
drake::Vector2<T> LineSegment<T>::path_coords(drake::Vector2<T> point) {
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
  using std::cos;
  using std::sin;
  return center_ + radius_ * drake::Vector2<T>(cos(heading_), sin(heading_));
}

template <typename T>
drake::Vector2<T> CircularSegment<T>::end() const {
  using std::cos;
  using std::sin;
  return center_ + radius_ * drake::Vector2<T>(cos(heading_end()), sin(heading_end()));
}




template <typename T>
drake::Vector2<T> CircularSegment<T>::eval(T arclength) const {
  T angle = heading_ + dir_ * arclength / radius_;
  return center_ + radius_ * drake::Vector2<T>(cos(angle), sin(angle));
}

template <typename T>
drake::Vector2<T> CircularSegment<T>::path_coords(drake::Vector2<T> point) {
  // this implementation doesn't really work, because we can go around the
  // circle multiple times. We have effectively added a branch cut and are not
  // tracking winding number. In a system, the distance can only increase, and
  // path integration should take care of it. This will need to be fixed,
  // eventually.
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

// Explicit template instantiations
template class LineSegment<double>;
template class LineSegment<drake::AutoDiffXd>;
template class CircularSegment<double>;
template class CircularSegment<drake::AutoDiffXd>;

}  // namespace dubins
}  // namespace mpcc
