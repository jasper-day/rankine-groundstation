#pragma once

#include <memory>
#include <vector>
#include <stdexcept>

#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>
#include <drake/common/trajectories/piecewise_constant_curvature_trajectory.h>

namespace mpcc {
namespace dubins {

enum SegmentType { SEGMENT, LINESEGMENT, CIRCULARSEGMENT };

/**
 * @brief Base class for Dubins path segments
 */
template <typename T>
class Segment {
 public:
  Segment() = default;

  virtual ~Segment() = default;

  /// Path-relative coordinates of a point in NED coordinates
  /// The first coordinate is the arclength and the second is normal distance
  virtual drake::Vector2<T> path_coords(drake::Vector2<T> point) = 0;

  /// The length of the segment
  virtual T length() const = 0;

  /// Starting point
  virtual drake::Vector2<T> start() const = 0;

  /// End point
  virtual drake::Vector2<T> end() const = 0;

  /// Tangent angle at start
  virtual T heading_start() const = 0;

  /// Tangent angle at end
  virtual T heading_end() const = 0;

  /// arclength -> location
  virtual drake::Vector2<T> eval(T arclength) const = 0;

  SegmentType type = SegmentType::SEGMENT;

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Segment);
  // Scalar converting copy constructor
  template <typename U>
  explicit Segment(const Segment<U>& other) : type(other.type) {}
};

/**
 * @brief Straight line segment of Dubins path.
 *
 * Dubins paths are made up of straight lines and circular
 * arcs. The straight lines are given by a start point and an
 * endpoint.
 * @todo we may want to add the height of the airplane later.
 *
 * @tparam T The scalar type of the path.
 */
template <typename T>
class LineSegment final : public Segment<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LineSegment);

  LineSegment() : Segment<T>() {
    this->type = SegmentType::LINESEGMENT;
  }
  ~LineSegment() = default;

  // Constructor with start and end points
  LineSegment(const drake::Vector2<T>& start_in,
              const drake::Vector2<T>& end_in)
      : Segment<T>(), start_(start_in), end_(end_in) {
    this->type = SegmentType::LINESEGMENT;
  }

  // Scalar converting copy constructor
  template <typename U>
  explicit LineSegment(const LineSegment<U>& other)
      : Segment<T>(other), start_(other.start_), end_(other.end_) {
    this->type = SegmentType::LINESEGMENT;
  }

  drake::Vector2<T> path_coords(drake::Vector2<T> point) override;
  T length() const override;
  drake::Vector2<T> start() const override { return start_; }
  drake::Vector2<T> end() const override { return end_; }
  T heading_start() const override;
  T heading_end() const override;
  drake::Vector2<T> eval(T arclength) const override;

  template <typename U>
  friend class LineSegment;

 private:
  drake::Vector2<T> start_;
  drake::Vector2<T> end_;
};

/**
 * @brief Curved segment of Dubins path.
 *
 * Dubins paths are made up of straight lines and circular arcs.
 * The circular arc segments are given by the location of their center,
 * the radius of the circle, the direction (1 for CCW, -1 for CW), the
 * heading of the entry point (from the center of the circle), and the
 * arclength travelled.
 *
 * See: Thomas J. Stastny, Adyasha Dash, and Roland Siegwart, "Nonlinear MPC for
 * Fixed-Wing UAV Trajectory Tracking: Implementation and Flight Experiments,"
 * in AIAA Guidance, Navigation, and Control Conference (AIAA Guidance,
 * Navigation, and Control Conference, Grapevine, Texas: American Institute of
 * Aeronautics and Astronautics, 2017), https://doi.org/10.2514/6.2017-1512.
 *
 * @tparam T the scalar type of the path
 */
template <typename T>
class CircularSegment final : public Segment<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CircularSegment);

  CircularSegment() : Segment<T>() {
    this->type = SegmentType::CIRCULARSEGMENT;
  }
  ~CircularSegment() = default;
  // Constructor with all parameters
  CircularSegment(const drake::Vector2<T>& center_in, T radius_in, T dir_in,
                  T heading_in, T arclength_in)
      : Segment<T>(), center_(center_in),
        radius_(radius_in),
        dir_(dir_in),
        heading_(heading_in),
        arclength_(arclength_in) {
    this->type = SegmentType::CIRCULARSEGMENT;
  }

  // Scalar converting copy constructor
  template <typename U>
  explicit CircularSegment(const CircularSegment<U>& other)
      : Segment<T>(other),
        center_(other.center_),
        radius_(other.radius_),
        dir_(other.dir_),
        heading_(other.heading_),
        arclength_(other.arclength_) {
    this->type = SegmentType::CIRCULARSEGMENT;
  }

  drake::Vector2<T> path_coords(drake::Vector2<T> point) override;
  T length() const override { return arclength_; }
  drake::Vector2<T> start() const override;
  drake::Vector2<T> end() const override;
  T heading_start() const override { return heading_ + dir_ * M_PI / 2; }
  T heading_end() const override {
    return heading_start() + dir_ * arclength_ / radius_;
  }
  drake::Vector2<T> eval(T arclength) const override;

  template <typename U>
  friend class CircularSegment;

  template <typename U>
  friend class DubinsPath;

 private:
  drake::Vector2<T> center_;
  T radius_;
  T dir_;
  T heading_;
  T arclength_;
};

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
  explicit DubinsPath(const DubinsPath<U>& other) {
    segments_.reserve(other.segments_.size());
    std::transform(
        other.segments_.begin(), other.segments_.end(),
        std::back_inserter(segments_),
        [&](const auto& segment) -> std::shared_ptr<Segment<T>> {
          if (segment->type == SegmentType::LINESEGMENT) {
            auto line_segment = std::dynamic_pointer_cast<LineSegment<U>>(segment);
            return std::make_shared<LineSegment<T>>(*line_segment);
          } else if (segment->type == SegmentType::CIRCULARSEGMENT) {
            auto circ_segment = std::dynamic_pointer_cast<CircularSegment<U>>(segment);
            return std::make_shared<CircularSegment<T>>(*circ_segment);
          }
          throw std::runtime_error("Unknown segment type");
        });
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
  std::vector<T> lengths();
  /// Total length of path
  T length();
  /// Find point on path
  drake::Vector2<T> eval(T arclength);
  /// Starting point
  drake::Vector2<T> start() { return eval(T(0)); };
  /// End point
  drake::Vector2<T> end() { return eval(length()); };

  /// Convert to Drake representation
  drake::trajectories::PiecewiseConstantCurvatureTrajectory<T> to_drake() const;

  // Declare friendship to enable scalar conversion
  template <typename U>
  friend class DubinsPath;

 private:
  std::vector<std::shared_ptr<Segment<T>>> segments_;
  std::vector<T> lengths_;
};

}  // namespace dubins
}  // namespace mpcc