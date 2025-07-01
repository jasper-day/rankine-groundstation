#include <functional>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <autodiff/reverse/var/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>

autodiff::Vector2var ls_start_with(const autodiff::Vector4var& params) {
  return params(Eigen::seq(0, 2));
}

autodiff::Vector2var ls_end_with(const autodiff::Vector4var& params) {
  return params(Eigen::seq(2, 4));
}

autodiff::var ls_heading_start_with(const autodiff::Vector4var& params) {
  const auto direction = ls_end_with(params) - ls_start_with(params);
  using std::atan2;
  return atan2(direction(1), direction(0));
}

autodiff::var ls_heading_end_with(const autodiff::Vector4var& params) {
  return ls_heading_start_with(params);
}

autodiff::Vector2var cs_eval_with(const autodiff::VectorXvar& params, const autodiff::var& arclength) {
  using namespace autodiff;
  const auto center = params(Eigen::seq(0, 2));
  const auto radius = params(2);
  const auto heading = params(3);
  const auto c_arclength = params(4);
  const auto dir = sgn(c_arclength);
  using std::abs;
  const auto arclength_clipped = max(min(arclength, abs(c_arclength)), 0.0);
  const auto angle = heading + arclength_clipped / radius * dir;
  Vector2var angle_direction;
  using std::cos;
  using std::sin;
  angle_direction << cos(angle), sin(angle);
  return center + radius * angle_direction;
}

autodiff::Vector2var cs_start_with(const autodiff::VectorXvar& params) {
  return cs_eval_with(params, 0.0);
}

autodiff::Vector2var cs_end_with(const autodiff::VectorXvar& params) {
  const auto c_arclength = params(4);
  using std::abs;
  return cs_eval_with(params, abs(c_arclength));
}

autodiff::var cs_heading_start_with(const autodiff::VectorXvar& params) {
  using namespace autodiff;
  const auto heading = params(3);
  const auto dir = sgn(params(4));
  return heading + M_PI / 2 * dir;
}

autodiff::var cs_heading_end_with(const autodiff::VectorXvar& params) {
  const auto radius = params(2);
  const auto arclength = params(4);
  return cs_heading_start_with(params) + arclength / radius;
}

enum segment_type {LINE, CIRCLE};

int n_params(segment_type type) {
  if (type == segment_type::LINE) {
    return 4;
  } else if (type == segment_type::CIRCLE) {
    return 5;
  }
}

autodiff::VectorXvar path_get_constraint_residuals(const autodiff::VectorXvar& params, const std::vector<segment_type> & types) {
  using namespace autodiff;
  const auto n_points = types.size() - 1;
  VectorXvar output(n_points * 2);

  Eigen::Index index = 0;
  // some handy loop variables
  Eigen::Index index_i;
  Eigen::Index index_ii;
  segment_type segment_i;
  segment_type segment_ii;
  Vector2var segment_i_end;
  Vector2var segment_ii_start;
  var segment_i_heading_end;
  var segment_ii_heading_start;

  for (int i = 0; i != n_points; ++i) {
    index_i = index + n_params(types.at(i));
    const auto params_i = params(Eigen::seq(index, index_i));
    index_ii = index_i + n_params(types.at(i + 1));
    const auto params_ii = params(Eigen::seq(index_i, index_ii));
    segment_i = types.at(i);
    segment_ii = types.at(i + 1);

    if (segment_i == segment_type::LINE) {
      segment_i_end = ls_end_with(params_i);
      segment_i_heading_end = ls_heading_end_with(params_i);
    } else {
      segment_i_end = cs_end_with(params_i);
      segment_i_heading_end = cs_heading_end_with(params_i);
    }
    if (segment_ii == segment_type::LINE) {
      segment_ii_start = ls_start_with(params_ii);
      segment_ii_heading_start = ls_heading_start_with(params_ii);
    } else {
      segment_ii_start = cs_start_with(params_ii);
      segment_ii_heading_start = cs_heading_start_with(params_ii);
    }

    const auto distance = segment_ii_start - segment_i_end;
    output(i) = distance.dot(distance);
    
    using std::pow;
    using std::sin;
    const auto heading_diff = segment_ii_heading_start - segment_i_heading_end;
    output(i + n_points) = pow(sin(heading_diff / 2.0), 2);

    index = index_i;
  }
  return output;
}

struct SolverResult {
  Eigen::VectorXd const params;
  bool const converged;
};

struct SolverConfig {
  int debug;
  int max_iter;
  bool line_search;
  double tolerance;
};

typedef Eigen::Ref<Eigen::VectorXd const> RefVector;
typedef Eigen::Ref<Eigen::MatrixXd const> RefMatrix;

SolverResult solve(
    SolverConfig const& config, RefVector const& p_start,
    std::function<RefVector const(const Eigen::VectorXd&)> residual_fn,
    std::function<RefMatrix const(const Eigen::VectorXd&)> jacobian_fn) {
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
   * This sets the search direction for a line search.
   *
   * @note The least squares convergence guarantees that the update dx is in the
   * row space of A, reducing "surprisal"
   *
   * @param dragged_points -- an N_SEGMENTS + 1 sized list of bools indicating
   * whether that control point is currently being dragged.
   *
   * @todo add dragged_points
   *
   * @todo don't change circular arc direction parameter
   *
   * @todo debug convergence errors
   *
   */
  using Eigen::Index;
  using Eigen::SparseMatrix;
  using Eigen::VectorXd;

  int n = p_start.size();
  VectorXd B;
  B = residual_fn(p_start);
  int m = B.size();

  if (config.debug >= 1) {
    std::cout << "Solving optimization problem with " << n << " parameters and "
              << m << " constraints." << std::endl;
  }

  SparseMatrix<double> A;
  VectorXd B_new(m);
  VectorXd z(m);
  VectorXd dx(n);
  VectorXd p_curr(n);
  VectorXd p_prev(n);
  VectorXd p_new(n);
  Index i;
  Index j;
  double alpha;
  int line_search_iter;
  bool line_search_step;
  bool converged = false;
  int iter = 0;
  p_curr = p_start;

  do {
    A.setZero();
    A.resize(m, n);

    // Dense
    try {
      B = residual_fn(p_curr);
      A = jacobian_fn(p_curr).sparseView();
    } catch (const std::exception& e) {
      throw std::runtime_error("Error in Python callback: " +
                               std::string(e.what()));
    }

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
    // Line search
    if (config.line_search) {
      alpha = 1.0;
      line_search_step = false;
      for (line_search_iter = 0; line_search_iter < 10; ++line_search_iter) {
        p_new = p_curr + alpha * dx;
        B_new = residual_fn(p_new);
        if (B_new.array().pow(2).sum() < B.array().pow(2).sum()) {
          line_search_step = true;
          p_curr = p_new;
          break;
        }
        alpha *= 0.5;
      }
      if (!line_search_step) {
        p_curr += dx * alpha;
      }
    } else {
      p_curr += dx;
    }

    // Check for convergence
    // All constraint values should be zero
    converged = true;
    if (B.cwiseAbs().sum() >= config.tolerance) {
      converged = false;
    }

    // debug information
    if ((config.debug >= 2) || (config.debug >= 1 && iter % 5 == 0)) {
      using std::cout;
      cout << "Iteration number " << iter << "\n";
      cout << "Current residuals\n" << B << "\n";
      cout << "Parameter values\n" << p_curr << "\n";
      cout << "Change in parameters\n" << dx << std::endl;
    }

  } while (
      // copy loop criteria from SolveSpace
      iter++ < config.max_iter && !converged);
  // NEWTON ITERATION LOOP END

  // if (!converged) {
  //   std::stringstream ss;
  //   ss << "Could not converge configuration after " << iter << "
  //   iterations.\n"; ss << "Final constraint residuals:\n"; ss << B << "\n";
  //   ss << "Final parameter values:\n";
  //   ss << p_curr << "\n";
  //   ss << "Parameter changes in last iteration:\n";
  //   ss << dx << "\n";
  //   throw std::runtime_error(ss.str());
  // }

  SolverResult result{p_curr, converged};
  return result;
}

namespace py = pybind11;

PYBIND11_MODULE(cpp_solve, m) {
  m.doc() = "Python bindings for Dubins path solver";

  py::class_<SolverResult>(m, "SolverResult")
      .def_readonly("params", &SolverResult::params)
      .def_readonly("converged", &SolverResult::converged);

  py::class_<SolverConfig>(m, "SolverConfig")
      .def(py::init<int, int, bool, double>())
      .def_readwrite("debug", &SolverConfig::debug)
      .def_readwrite("max_iter", &SolverConfig::max_iter)
      .def_readwrite("tolerance", &SolverConfig::tolerance)
      .def_readwrite("line_search", &SolverConfig::line_search);

  m.def("solve", &solve, py::arg("config"), py::arg("p_start"),
        py::arg("residual_fn"), py::arg("jacobian_fn"));
}
