#include "dubins_solver.h"

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <drake/common/autodiff.h>
#include <drake/math/autodiff_gradient.h>
#include <drake/math/jacobian.h>

#include "dubins_path.h"

drake::VectorX<double> mpcc::dubins::DubinsSolver::solve(
    DubinsPath<double>& path, const std::vector<bool>& dragged_points) {
  /* Based on the solver from SolveSpace.
   * See: src/system.cpp in SolveSpace repository
   *
   * Constraints: zero-order continuity constraints
   * and equality of first derivatives (headings) at endpoints.
   * Free parameters are the protected members of each of the
   * elements, ie, for a LineSegment, the start and end points.
   *
   * Mathematical setup:
   *
   * m constraints f_i
   * n parameters x_j
   *
   * There will be more free parameters than constraints (n > m)
   *
   * A_ij is the mxn matrix of derivatives d f_i / d x_j
   * B_i is the 1xm matrix of values f_i
   *
   * The Jacobian step is
   *
   * J_n (x_{n+1} - x_n) = 0 - F_n
   *
   * or
   *
   * A * dx = -B
   *
   * At each step we solve the least squares version
   *
   * A A^T z = B
   * -dx = A^T z
   *
   * x += dx
   *
   * and iterate until convergence.
   *
   * @note The least squares convergence guarantees that the update dx is in the
   * row space of A, reducing "surprisal"
   *
   * @param dragged_points -- an N_SEGMENTS + 1 sized list of bools indicating
   * whether that control point is currently being dragged.
   *
   */

  using Eigen::Index;
  using Eigen::SparseMatrix;

  int n = path.n_params();
  int m = path.n_constraints();

  SparseMatrix<double> A;
  drake::VectorX<double> B(m);
  drake::VectorX<double> z(m);
  drake::VectorX<double> dx(n);
  drake::VectorX<double> p_curr(n);
  drake::VectorX<double> p_prev(n);
  Index i;
  Index j;
  bool converged = false;
  int iter = 0;
  DubinsPath<drake::AutoDiffXd> differentiable_path{path};

  auto constraint_residuals = [&](const auto& x) {
    using Scalar = typename std::remove_reference_t<decltype(x)>::Scalar;
    if constexpr (std::is_same_v<Scalar, double>) {
      return path.get_constraint_residuals(x);
    } else {
      // cast to AutoDiffXd
      drake::VectorX<drake::AutoDiffXd> x_ad = x.template cast<drake::AutoDiffXd>();
      return differentiable_path.get_constraint_residuals(x_ad);
    }
  };

  p_curr = path.get_params();

  do {
    A.setZero();
    A.resize(m, n);

    // Dense
    auto jac = drake::math::jacobian(constraint_residuals, p_curr);

    B = drake::math::ExtractValue(jac);
    A = drake::math::ExtractGradient(jac).sparseView();

    // TODO
    // // Rescale A according to dragging
    // const int size = A.outerSize();
    // for(int k = 0; k < size; k++) {
    //   for(SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
    //     if (dragged_points[point]) {
    //       it.valueRef() *= T(1/20);
    //     }
    //   }
    // }

    A.makeCompressed();

    // some loop allocation, maybe can remove
    SparseMatrix<double> AAt = A * A.transpose();

    // Solve least squares problem
    Eigen::SparseQR<SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(AAt);
    z = solver.solve(B);

    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("Could not solve constraint system");
    }

    // jacobian step update
    dx = -A.transpose() * z;

    p_prev = p_curr;
    p_curr += dx;

    // Check for convergence
    // All constraint values should be zero
    converged = true;
    if (B.dot(B) >= tolerance_) {
      converged = false;
    }

  } while (
      // copy loop criteria from SolveSpace
      iter++ < max_iter_ && !converged);
  // NEWTON ITERATION LOOP END

  if (!converged) throw std::runtime_error("Could not converge configuration");

  return p_curr;
}