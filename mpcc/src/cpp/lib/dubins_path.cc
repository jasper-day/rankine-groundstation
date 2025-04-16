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

template <typename T>
T DubinsPath<T>::get_closest_arclength(drake::Vector2<T> const& pos,
                                       T const& estimated_arclength) const {
  std::vector<T> arclengths;
  arclengths.reserve(segments_.size());
  std::transform(segments_.begin(), segments_.end(),
                 std::back_inserter(arclengths),
                 [&](std::shared_ptr<Segment<T>> const& segment) {
                   drake::Vector2<T> result;
                   result = segment->path_coords(pos);
                   return result(0);
                 });
  using std::abs;
  // find closest to estimated_arclength
  std::partial_sort(arclengths.begin(), arclengths.begin() + 2,
                    arclengths.end(), [&](T a, T b) {
                      return abs(a - estimated_arclength) <
                             abs(b - estimated_arclength);
                    });
  return arclengths.at(0);
}

template <typename T>
T DubinsPath<T>::get_true_arclength(drake::Vector2<T> const& pos) const {
  std::vector<drake::Vector2<T>> coords;
  coords.reserve(segments_.size());
  std::transform(segments_.begin(), segments_.end(), std::back_inserter(coords),
                 [&](std::shared_ptr<Segment<T>> const& segment) {
                   drake::Vector2<T> result;
                   result = segment->path_coords(pos);
                   return result;
                 });
  using std::abs;
  // find least normal distance
  std::partial_sort(
      coords.begin(), coords.begin() + 2, coords.end(),
      [](drake::Vector2<T> a, drake::Vector2<T> b) { return a(1) < b(1); });
  drake::Vector2<T> first = coords.at(0);
  return first(0);
}

template <typename T>
std::unique_ptr<DubinsPath<T>> DubinsPath<T>::offset_path(T amount) const {
  auto path_offsetter =
      [&](std::shared_ptr<Segment<T>> segment) -> std::shared_ptr<Segment<T>> {
    if (segment->type == SegmentType::LINESEGMENT) {
      // calculate direction along line segment
      drake::Vector2<T> AB = segment->end() - segment->start();
      AB.normalize();
      drake::Vector2<T> offset;
      // offset perpendicular to AB direction
      offset << AB(1) * amount, -AB(0) * amount;
      // make new segment with offset ends
      auto new_segment = std::make_shared<LineSegment<T>>(
          segment->start() + offset, segment->end() + offset);
      // cast into a Segment
      return std::dynamic_pointer_cast<Segment<T>>(new_segment);
    } else if (segment->type == SegmentType::CIRCULARSEGMENT) {
      // cast into CircularSegment to access protected properties
      auto csegment = std::dynamic_pointer_cast<CircularSegment<T>>(segment);
      // new CircularSegment
      auto new_segment = std::make_shared<CircularSegment<T>>(
          csegment->centre_, csegment->radius_ + amount * csegment->dir(),
          csegment->heading_,
          csegment->arclength_ *
              (1 + amount * csegment->dir() / csegment->radius_));
      return std::dynamic_pointer_cast<Segment<T>>(new_segment);
    } else
      throw std::runtime_error("Could not determine type of segment");
  };
  std::vector<std::shared_ptr<Segment<T>>> new_segments;
  new_segments.reserve(segments_.size());
  std::transform(segments_.begin(), segments_.end(),
                 std::back_inserter(new_segments), path_offsetter);
  return std::make_unique<DubinsPath<T>>(new_segments);
}

template DubinsPath<drake::AutoDiffXd>::DubinsPath(const DubinsPath<double>&);

}  // namespace dubins
}  // namespace mpcc

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::DubinsPath);
