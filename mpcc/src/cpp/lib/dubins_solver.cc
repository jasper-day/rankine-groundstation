#include <functional>
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// Type aliases for autodiff
using namespace Eigen;

struct SolverResult {
  Eigen::VectorXd const params;
  bool const converged;
};

struct SolverConfig {
  int debug;
  int max_iter;
  bool line_search;
  double tolerance;
  double epsilon;
};

template <typename T>
T sgn(const T& val) {
  return (T(0) < val) - (val < T(0));
}

VectorXd ls_start_with(const VectorXd& params) { 
VectorXd output(2);
output << params(0), params(1);
return output;
 }

VectorXd ls_end_with(const VectorXd& params) { 
  VectorXd output(2);
  output << params(2), params(3);
  return output;
 }

double ls_heading_start_with(const VectorXd& params) {
  VectorXd direction = ls_end_with(params) - ls_start_with(params);
  return std::atan2(direction(1), direction(0));
}

double ls_heading_end_with(const VectorXd& params) {
  return ls_heading_start_with(params);
}

VectorXd cs_eval_with(const VectorXd& params, const double& arclength) {
  const auto center = params.head<2>();
  const auto radius = params(2);
  const auto heading = params(3);
  const auto c_arclength = params(4);
  const auto dir = sgn(c_arclength);
  const auto arclength_clipped =
      std::max(std::min(arclength, std::abs(c_arclength)), 0.0);
  const auto angle = heading + arclength_clipped / radius * dir;

  Vector2d angle_direction;
  angle_direction << std::cos(angle), std::sin(angle);
  return center + radius * angle_direction;
}

VectorXd cs_start_with(const VectorXd& params) {
  return cs_eval_with(params, double(0.0));
}

VectorXd cs_end_with(const VectorXd& params) {
  const auto c_arclength = params(4);
  return cs_eval_with(params, abs(c_arclength));
}

double cs_heading_start_with(const VectorXd& params) {
  const auto heading = params(3);
  const auto dir = sgn(params(4));
  return heading + M_PI / 2 * dir;
}

double cs_heading_end_with(const VectorXd& params) {
  const auto radius = params(2);
  const auto arclength = params(4);
  return cs_heading_start_with(params) + arclength / radius;
}

enum CPPSegmentType { LINE, CIRCLE };

int n_params(CPPSegmentType type) {
  if (type == CPPSegmentType::LINE) {
    return 4;
  } else if (type == CPPSegmentType::CIRCLE) {
    return 5;
  }
  return 0;  // should never reach here
}

VectorXd path_get_constraint_residuals(
    const VectorXd& params, const std::vector<CPPSegmentType>& types) {
  const auto n_points = types.size() - 1;
  VectorXd output(n_points * 2);

  Eigen::Index index = 0;

  for (int i = 0; i != n_points; ++i) {
    const Eigen::Index index_i = index + n_params(types.at(i));

    const CPPSegmentType segment_i = types.at(i);
    const CPPSegmentType segment_ii = types.at(i + 1);

    const auto params_i = params.segment(index, n_params(segment_i));
    const auto params_ii = params.segment(index_i, n_params(segment_ii));


    Vector2d segment_i_end;
    double segment_i_heading_end;
    Vector2d segment_ii_start;
    double segment_ii_heading_start;

    if (segment_i == CPPSegmentType::LINE) {
      segment_i_end = ls_end_with(params_i);
      segment_i_heading_end = ls_heading_end_with(params_i);
    } else {
      segment_i_end = cs_end_with(params_i);
      segment_i_heading_end = cs_heading_end_with(params_i);
    }

    if (segment_ii == CPPSegmentType::LINE) {
      segment_ii_start = ls_start_with(params_ii);
      segment_ii_heading_start = ls_heading_start_with(params_ii);
    } else {
      segment_ii_start = cs_start_with(params_ii);
      segment_ii_heading_start = cs_heading_start_with(params_ii);
    }

    const auto distance = segment_ii_start - segment_i_end;
    output(i) = distance.dot(distance);

    const auto heading_diff = segment_ii_heading_start - segment_i_heading_end;
    output(n_points + i) = std::pow(std::sin(heading_diff / 2.0), 2);

    index = index_i;
  }
  return output;
}

// jacobian via numerical differentiation
MatrixXd path_jacobian_fn(const VectorXd& params,
                          const std::vector<CPPSegmentType>& types) {
  const int n = params.size();
  const int m = (types.size() - 1) * 2;  // number of constraints

  const double eps = 1e-6;

  const VectorXd f0 = path_get_constraint_residuals(params, types);

  MatrixXd jac(m, n);
  VectorXd curr_params = params;
  VectorXd f_eps(m);
  VectorXd f_eps_neg(m);

  for (int i = 0; i != n; ++i) {
    // centered finite difference
    curr_params[i] += eps;
    f_eps = path_get_constraint_residuals(curr_params, types);
    curr_params[i] -= eps * 2;
    f_eps_neg = path_get_constraint_residuals(curr_params, types);
    curr_params[i] += eps;
    jac.col(i) = (f_eps - f_eps_neg) / eps / 2;
  }

  return jac;
}



SolverResult solve(SolverConfig const& config, VectorXd const& p_start,
                   std::function<VectorXd const(const VectorXd&)> residual_fn,
                   std::function<MatrixXd const(const VectorXd&)> jacobian_fn) {
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

  int n = p_start.size();
  VectorXd B = residual_fn(p_start);
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
  VectorXd p_new(n);
  double alpha;
  int line_search_iter;
  bool line_search_step;
  bool converged = false;
  int iter = 0;
  p_curr = p_start;

  do {
    try {
      B = residual_fn(p_curr);
      A = jacobian_fn(p_curr).sparseView();
    } catch (const std::exception& e) {
      throw std::runtime_error("Error in callback: " + std::string(e.what()));
    }

    A.makeCompressed();
    SparseMatrix<double> AAt = A * A.transpose();

    Eigen::SparseQR<SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(AAt);
    z = solver.solve(B);

    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("Could not solve constraint system");
    }

    dx = -A.transpose() * z;

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

    converged = true;
    if (B.cwiseAbs().sum() >= config.tolerance) {
      converged = false;
    }

    if ((config.debug >= 2) || (config.debug >= 1 && iter % 5 == 0)) {
      using std::cout;
      cout << "Iteration number " << iter << "\n";
      cout << "Current residuals\n" << B << "\n";
      cout << "Parameter values\n" << p_curr << "\n";
      cout << "Change in parameters\n" << dx << std::endl;
    }

  } while (iter++ < config.max_iter && !converged);

  SolverResult result{p_curr, converged};
  return result;
}

// Convenience function for path solving
SolverResult solve_path(const SolverConfig& config,
                        const Ref<VectorXd>& p_start,
                        const std::vector<CPPSegmentType>& types) {
  auto residual_fn = [types](const VectorXd& params) {
    return path_get_constraint_residuals(params, types);
  };

  auto jacobian_fn = [types](const VectorXd& params) {
    return path_jacobian_fn(params, types);
  };

  return solve(config, p_start, residual_fn, jacobian_fn);
}

namespace py = pybind11;

PYBIND11_MODULE(cpp_solve, m) {
  m.doc() = "Python bindings for Dubins path solver";

  py::class_<SolverResult>(m, "SolverResult")
      .def_readonly("params", &SolverResult::params)
      .def_readonly("converged", &SolverResult::converged);

  py::class_<SolverConfig>(m, "SolverConfig")
      .def(py::init<int, int, bool, double, double>(), py::arg("debug"), py::arg("max_iter"), py::arg("line_search"), py::arg("tolerance"), py::arg("epsilon"))
      .def_readwrite("debug", &SolverConfig::debug)
      .def_readwrite("max_iter", &SolverConfig::max_iter)
      .def_readwrite("line_search", &SolverConfig::line_search)
      .def_readwrite("tolerance", &SolverConfig::tolerance)
      .def_readwrite("epsilon", &SolverConfig::epsilon);

  py::enum_<CPPSegmentType>(m, "CPPSegmentType")
      .value("LINE", CPPSegmentType::LINE)
      .value("CIRCLE", CPPSegmentType::CIRCLE);

  // m.def("solve", &solve, py::arg("config"), py::arg("p_start"),
  //       py::arg("residual_fn"), py::arg("jacobian_fn"));

  m.def("solve_path", &solve_path, py::arg("config"), py::arg("p_start"),
        py::arg("types"));
  // Replace the direct function bindings with lambdas
  m.def("ls_start_with",
        [](const Ref<const VectorXd>& params) { return ls_start_with(params); });
  m.def("ls_end_with",
        [](const Ref<const VectorXd>& params) { return ls_end_with(params); });
  m.def("ls_heading_start_with", [](const Ref<const VectorXd>& params) {
    return ls_heading_start_with(params);
  });
  m.def("ls_heading_end_with", [](const Ref<const VectorXd>& params) {
    return ls_heading_end_with(params);
  });

  m.def("cs_start_with",
        [](const Ref<const VectorXd>& params) { 
          return cs_start_with(params); });
  m.def("cs_end_with",
        [](const Ref<const VectorXd>& params) { return cs_end_with(params); });
  m.def("cs_heading_start_with", [](const Ref<const VectorXd>& params) {
    return cs_heading_start_with(params);
  });
  m.def("cs_heading_end_with", [](const Ref<const VectorXd>& params) {
    return cs_heading_end_with(params);
  });

  m.def("path_jacobian_fn", [](const Ref<const VectorXd>& params,
                               const std::vector<CPPSegmentType>& types) {
    return path_jacobian_fn(params, types);
  });
}