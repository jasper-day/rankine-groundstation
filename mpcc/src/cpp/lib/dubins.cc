#include "dubins.h"

#include <algorithm>
#include <cmath>

#include <drake/common/autodiff.h>
#include <drake/common/eigen_types.h>
#include <drake/common/trajectories/piecewise_constant_curvature_trajectory.h>

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

template <typename T>
std::vector<T> DubinsPath<T>::lengths() {
  std::vector<T> output;
  output.reserve(segments_.size());
  std::transform(segments_.begin(), segments_.end(), std::back_inserter(output),
                 [&](const auto& segment) { return segment->length(); });
  return output;
}

template <typename T>
T DubinsPath<T>::length() {
  return std::accumulate(lengths_.begin(), lengths_.end(), T(0));
}

template <typename T>
drake::Vector2<T> DubinsPath<T>::eval(T arclength) {
  if (segments_.size() == 0)
    throw std::runtime_error("Cannot evaluate empty path - no valid position exists");
  // does not recompute the lengths; instead uses cached value
  int i = 0;
  while (i != segments_.size()) {
    if (arclength < lengths_[i]) break;
    arclength -= lengths_[i];
    ++i;
  }
  if (i == segments_.size()) return segments_.back()->end();  // beyond end
  return segments_[i]->eval(arclength);
}

template <typename T>
drake::trajectories::PiecewiseConstantCurvatureTrajectory<T>
DubinsPath<T>::to_drake() const {
  if (segments_.size() == 0)
    return drake::trajectories::PiecewiseConstantCurvatureTrajectory<T>();
  std::vector<T> breaks;
  breaks.reserve(segments_.size() + 1);
  T total_length = T(0);
  breaks.push_back(total_length);
  for (const auto& segment : segments_) {
    total_length += segment->length();
    breaks.push_back(total_length);
  }
  std::vector<T> turning_rates;
  turning_rates.reserve(segments_.size());
  std::transform(
      segments_.begin(), segments_.end(), std::back_inserter(turning_rates),
      [](const auto& segment) {
        if (segment->type == SegmentType::LINESEGMENT)
          return T(0);
        else if (segment->type == SegmentType::CIRCULARSEGMENT)
          return T(1) / std::dynamic_pointer_cast<CircularSegment<T>>(segment)
                            ->radius_;
        else
          throw std::runtime_error("Unknown segment type" + std::to_string(static_cast<int>(segment->type)));
      });
  using std::cos;
  using std::sin;
  drake::Vector3<T> initial_curve_tangent(cos(segments_[0]->heading_start()),
                                          sin(segments_[0]->heading_start()),
                                          T(0));
  drake::Vector3<T> plane_normal(T(-1) * sin(segments_[0]->heading_start()),
                                 cos(segments_[0]->heading_start()), T(0));
  drake::Vector3<T> initial_position(segments_[0]->start().x(),
                                     segments_[0]->start().y(), T(0));

  return drake::trajectories::PiecewiseConstantCurvatureTrajectory<T>(
      breaks, turning_rates, initial_curve_tangent, plane_normal,
      initial_position, false);
}

// Explicit template instantiations
template class LineSegment<double>;
template class LineSegment<drake::AutoDiffXd>;
template class CircularSegment<double>;
template class CircularSegment<drake::AutoDiffXd>;
template class DubinsPath<double>;
template class DubinsPath<drake::AutoDiffXd>;

}  // namespace dubins
}  // namespace mpcc