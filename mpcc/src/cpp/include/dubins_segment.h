#pragma once

#include <memory>
#include <stdexcept>
#include <vector>

#include <drake/common/default_scalars.h>
#include <drake/common/drake_copyable.h>
#include <drake/common/eigen_types.h>

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
  virtual drake::Vector2<T> path_coords(drake::Vector2<T> point) const = 0;

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

  /// get parameters
  virtual drake::VectorX<T> get_params() const = 0;

  /// set parameters
  virtual void set_params(const drake::VectorX<T>& params) = 0;

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

  LineSegment() : Segment<T>() { this->type = SegmentType::LINESEGMENT; }
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

  drake::Vector2<T> path_coords(drake::Vector2<T> point) const override;
  T length() const override;
  drake::Vector2<T> start() const override { return start_; }
  drake::Vector2<T> end() const override { return end_; }
  T heading_start() const override;
  T heading_end() const override;
  drake::Vector2<T> eval(T arclength) const override;

  drake::VectorX<T> get_params() const override {
    drake::VectorX<T> output(4);
    output << start_, end_;
    return output;
  }

  void set_params(const drake::VectorX<T>& params) override {
    drake::Vector<T, 4> input = params;
    start_ = input.segment(0, 2);
    end_ = input.segment(2, 2);
  }

  template <typename U>
  friend class LineSegment;

  template <typename U>
  friend class DubinsPath;

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
  CircularSegment(const drake::Vector2<T>& center_in, T radius_in, T heading_in,
                  T arclength_in)
      : Segment<T>(),
        centre_(center_in),
        radius_(radius_in),
        heading_(heading_in),
        arclength_(arclength_in) {
    this->type = SegmentType::CIRCULARSEGMENT;
  }

  // Scalar converting copy constructor
  template <typename U>
  explicit CircularSegment(const CircularSegment<U>& other)
      : Segment<T>(other),
        centre_(other.centre_),
        radius_(other.radius_),
        heading_(other.heading_),
        arclength_(other.arclength_) {
    this->type = SegmentType::CIRCULARSEGMENT;
  }

  drake::Vector2<T> path_coords(drake::Vector2<T> point) const override;
  T length() const override {
    using std::abs;
    return abs(arclength_);
  }
  drake::Vector2<T> start() const override;
  drake::Vector2<T> end() const override;
  T heading_start() const override {
    if (arclength_ == T(0))
      throw std::runtime_error("Cannot find heading with zero arclength");
    else if (arclength_ > T(0))
      return heading_ + M_PI / 2;
    else
      return heading_ - M_PI / 2;
  }
  T heading_end() const override {
    return heading_start() + arclength_ / radius_;
  }
  drake::Vector2<T> eval(T arclength) const override;

  drake::VectorX<T> get_params() const override {
    drake::VectorX<T> output(5);
    output << centre_, radius_, heading_, arclength_;
    return output;
  }

  void set_params(const drake::VectorX<T>& params) override {
    drake::Vector<T, 5> input = params;
    centre_ = input.segment(0, 2);
    radius_ = input(2);
    heading_ = input(3);
    arclength_ = input(4);
  }

  template <typename U>
  friend class CircularSegment;

  template <typename U>
  friend class DubinsPath;

 private:
  drake::Vector2<T> centre_;
  T radius_;
  T heading_;
  T arclength_;
  T dir() const { return (arclength_ > T(0)) ? T(1) : T(-1); }
};

}  // namespace dubins
}  // namespace mpcc

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::Segment);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::CircularSegment);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::mpcc::dubins::LineSegment);