// Find the closest suitable Dubins path to the current Dubins path.
#include "dubins_path.h"

#include <algorithm>
#include <cmath>

#include <drake/common/default_scalars.h>

namespace mpcc {
namespace dubins {

template <typename T>
std::vector<T> DubinsPath<T>::lengths() const {
  std::vector<T> output;
  output.reserve(segments_.size());
  std::transform(segments_.begin(), segments_.end(), std::back_inserter(output),
                 [&](const auto& segment) { return segment->length(); });
  return output;
}

template <typename T>
T DubinsPath<T>::length() const {
  return std::accumulate(lengths_.begin(), lengths_.end(), T(0));
}

template <typename T>
drake::Vector2<T> DubinsPath<T>::eval(T arclength) const {
  if (segments_.size() == 0)
    throw std::runtime_error(
        "Cannot evaluate empty path - no valid position exists");
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
          throw std::runtime_error(
              "Unknown segment type" +
              std::to_string(static_cast<int>(segment->type)));
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

template <typename T>
drake::VectorX<T> DubinsPath<T>::get_params() const {
  size_t total_size = 0;
  for (const auto& segment : segments_) {
    total_size += segment->get_params().innerSize();
  }

  drake::VectorX<T> output(total_size);

  // block initialization
  size_t current_index = 0;
  for (const auto& segment : segments_) {
    output.segment(current_index, segment->get_params().innerSize()) =
        segment->get_params();
    current_index += segment->get_params().innerSize();
  }

  return output;
}

template <typename T>
void DubinsPath<T>::set_params(drake::VectorX<T> state) {
  size_t current_index = 0;
  for (const auto& segment : segments_) {
    segment->set_params(
        state.segment(current_index, segment->get_params().innerSize()));
    current_index += segment->get_params().innerSize();
  }
}

template <typename T>
drake::VectorX<T> DubinsPath<T>::get_constraint_residuals(
    drake::VectorX<T> values) {
  // initial values
  drake::VectorX<T> current_parameters = get_params();
  // will set back to initial values
  set_params(values);
  // distances squared
  int n_points = segments_.size() - 1;
  drake::VectorX<T> distance_residuals(n_points);
  drake::Vector2<T> distance;
  // sin squared of headings/2 to allow off-by-2π
  drake::VectorX<T> heading_residuals(n_points);
  T heading_diff;
  using std::pow;
  using std::sin;
  for (int i = 0; i != n_points; ++i) {
    // add distance residual
    distance = segments_[i]->end() - segments_[i + 1]->start();
    distance_residuals(i) = distance.dot(distance);
    heading_diff =
        segments_[i]->heading_end() - segments_[i + 1]->heading_start();
    heading_residuals(i) = pow(sin(heading_diff / 2), T(2));
  }
  drake::VectorX<T> output(n_points * 2);
  output << distance_residuals, heading_residuals;
  set_params(current_parameters);
  return output;
}

template DubinsPath<drake::AutoDiffXd>::DubinsPath(const DubinsPath<double>&);

}  // namespace dubins
}  // namespace mpcc

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::DubinsPath);
