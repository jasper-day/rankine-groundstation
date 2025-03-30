#pragma once

#include <drake/common/default_scalars.h>
#include <drake/common/drake_copyable.h>

#include "dubins_path.h"

namespace mpcc {
namespace dubins {

class DubinsSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DubinsSolver);

  DubinsSolver() : DubinsSolver(1e-6, 50, 0) {};
  DubinsSolver(double tolerance, int max_iter, int debug)
      : tolerance_(tolerance), max_iter_(max_iter), debug_(debug) {};
  ~DubinsSolver() = default;

  /**
   * @brief Find the closest valid path to this path
   *
   * Solves C1 continuity constraints in a least-squares sense.
   *
   * dragged_points contains the points currently being dragged.
   * The length of dragged_points must equal the number of independent
   * points in the path (equal to N_segments + 1)
   *
   * @todo Relies on non-const DubinsPath::get_constraint_residuals
   *
   * @returns A converged set of path parameters
   * @throws std::runtime_error for invalid configurations
   */
  drake::VectorX<double> solve(DubinsPath<double>& path,
                               const std::vector<bool>& dragged_points);

 protected:
  double tolerance_;
  int max_iter_;
  int debug_;
};

}  // namespace dubins
}  // namespace mpcc