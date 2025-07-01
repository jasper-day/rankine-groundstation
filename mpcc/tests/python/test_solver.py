from mpcc.pydubins import DubinsPath, LineSegment, CircularSegment, DubinsSolver
from pytest import approx
import numpy as np
import pytest
from test_pydubins import create_test_segments


ls1, cs1, ls2, cs2, cs3 = create_test_segments()

path = DubinsPath([ls1, cs1, ls2, cs2, cs3])

solver = DubinsSolver(tolerance=1e-6, max_iter=50, debug=1, line_search=True, epsilon=1e-8)

def test_solver_no_perturbation():
    assert solver.solve(
        path, [False] * (int(path.num_segments()) - 1)
    ).params == approx(path.get_params())


@pytest.mark.repeat(2)
def test_solver_random_perturbation():
    original_params = path.get_params().copy()
    perturb = np.random.uniform(-1e-2, 1e-2, size=len(original_params))
    perturbed_params = original_params + perturb

    path.set_params(perturbed_params)
    res = solver.solve(path, [False] * (int(path.num_segments()) - 1))
    assert res.converged

    assert res.params == approx(original_params, abs=1e-1)

    path.set_params(original_params)


def test_solver_debug():
    solver = DubinsSolver(tolerance=1e-6, max_iter=100, debug=1, line_search=False, epsilon = 1e-6[])

    original_params = path.get_params().copy()
    perturb = np.random.uniform(-1e2, 1e2, size=len(original_params))
    perturbed_params = original_params + perturb

    path.set_params(perturbed_params)

    res = solver.solve(path, [False] * (int(path.num_segments()) - 1))

    assert res.converged, "Solution converged"

    assert path.get_constraint_residuals(res.params) == approx(
        np.zeros((path.num_segments() - 1) * 2), abs=1e-6
    ), "Residuals are close to zero"

    path.set_params(original_params)


    

# def test_jacobian():
#     params = path.get_params()
#     assert type(curr_params) is np.ndarray
#     eps = 1e-6
#     f0 = path.get_constraint_residuals(curr_params)
#     n = path.n_params()
#     m = path.n_constraints()
#     # centered finite difference approximation for Jacobian
#     jac = np.zeros((m, n))
#     for i in range(n):
#         curr_params[i] += eps
#         f_eps = path.get_constraint_residuals(curr_params)
#         curr_params[i] -= eps
#         curr_params[i] -= eps
#         f_eps_neg = path.get_constraint_residuals(curr_params)
#         curr_params[i] += eps
#         jac[:, i] = (f_eps - f_eps_neg) / eps / 2
#     jac_analytic = solver.jac(path, curr_params)
#     assert jac_analytic.shape == jac.shape
#     assert jac == approx(jax.device_get(jac_analytic), rel=1e-4)
