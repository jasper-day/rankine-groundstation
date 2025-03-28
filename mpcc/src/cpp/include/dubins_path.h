#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include <drake/common/default_scalars.h>
#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>
#include <drake/common/trajectories/piecewise_constant_curvature_trajectory.h>

#include "dubins_segment.h"

namespace mpcc {
namespace dubins {

/**
 * @brief Dubins path.
 *
 * Dubins paths are made up of straight lines and circular arcs.
 * This implementation uses an ordered list of LineSegments and
 * CircularSegments.
 * A solver is given which enforces location and tangency constraints.
 * This uses the Drake symbolic algebra package.
 *
 * @tparam T the scalar type of the path
 */
template <typename T>
class DubinsPath {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DubinsPath);

  DubinsPath() = default;

  DubinsPath(std::vector<std::shared_ptr<Segment<T>>> segments_in)
      : segments_(segments_in) {
    lengths_ = lengths();
  }

  ~DubinsPath() = default;

  // Scalar converting copy constructor
  template <typename U>
  explicit DubinsPath(const DubinsPath<U>& other)
      : DubinsPath<T>(convert_segments<U, T>(other.segments_)) {}

  // Helper function to convert segments
  template <typename U, typename V>
  static std::vector<std::shared_ptr<Segment<V>>> convert_segments(
      const std::vector<std::shared_ptr<Segment<U>>>& segments) {
    std::vector<std::shared_ptr<Segment<V>>> converted;
    converted.reserve(segments.size());
    std::transform(
        segments.begin(), segments.end(), std::back_inserter(converted),
        [&](const auto& segment) -> std::shared_ptr<Segment<V>> {
          if (segment->type == SegmentType::LINESEGMENT) {
            auto line_segment =
                std::dynamic_pointer_cast<LineSegment<U>>(segment);
            return std::make_shared<LineSegment<V>>(*line_segment);
          } else if (segment->type == SegmentType::CIRCULARSEGMENT) {
            auto circ_segment =
                std::dynamic_pointer_cast<CircularSegment<U>>(segment);
            return std::make_shared<CircularSegment<V>>(*circ_segment);
          }
          throw std::runtime_error("Unknown segment type");
        });
    return converted;
  }

  /// Add a segment to the path
  void add_segment(std::shared_ptr<Segment<T>> segment) {
    segments_.push_back(std::move(segment));
  }

  /// Get the number of segments in the path
  size_t num_segments() const { return segments_.size(); }

  /// Get a segment by index
  const std::shared_ptr<Segment<T>> get_segment(size_t index) const {
    return segments_[index];
  }

  /// Get the associated arclengths
  std::vector<T> lengths() const;
  /// Total length of path
  T length() const;
  /// Find point on path
  drake::Vector2<T> eval(T arclength) const;
  /// Starting point
  drake::Vector2<T> start() const { return eval(T(0)); };
  /// End point
  drake::Vector2<T> end() const { return eval(length()); };

  /// Convert to Drake representation
  drake::trajectories::PiecewiseConstantCurvatureTrajectory<T> to_drake() const;

  /// Get value of all parameters
  drake::VectorX<T> get_params() const;

  /// Get types of all parameters
  std::vector<SegmentType> get_types() const {
    std::vector<SegmentType> output;
    output.reserve(num_segments());
    std::transform(segments_.begin(), segments_.end(),
                   std::back_inserter(output),
                   [](const auto segment) { return segment->type; });
    return output;
  }

  /// Set current path state (value of all parameters)
  void set_params(drake::VectorX<T> params);

  /**
   * @brief Constraint equations for this path
   *
   * If this vector is equal to zero, the path is C1 continuous.
   *
   * @todo See if this can be made const
   */
  drake::VectorX<T> get_constraint_residuals(drake::VectorX<T> values);

  int n_params() const {
    return std::accumulate(segments_.begin(), segments_.end(), 0,
                           [](int sum, const auto& segment) {
                             switch (segment->type) {
                               case SegmentType::LINESEGMENT:
                                 return sum + 4;
                               case SegmentType::CIRCULARSEGMENT:
                                 return sum + 5;
                               default:
                                 return sum;
                             }
                           });
  }

  int n_constraints() const {
    // two constraints for each knot point
    return (segments_.size() - 1) * 2;
  }

  // Declare friendship to enable scalar conversion
  template <typename U>
  friend class DubinsPath;

 private:
  std::vector<std::shared_ptr<Segment<T>>> segments_;
  std::vector<T> lengths_;
};

}  // namespace dubins
}  // namespace mpcc

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::DubinsPath);