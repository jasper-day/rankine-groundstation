from .dubins_path import DubinsPath
import numpy as np
from .cpp_solve import SolverConfig, SolverResult, solve


class DubinsSolver:
    def __init__(self, tolerance: float, max_iter: int, debug: int, line_search: bool):
        self._tolerance = tolerance
        self._max_iter = max_iter
        self._debug = debug
        self._line_search = line_search

    def solve(self, path: DubinsPath, dragged_points: list[bool]) -> SolverResult:
        """Find a suitable configuration of a Dubins path.
        dragged_points currently not implemented
        """
        n = path.n_params()
        m = path.n_constraints()

        if self._debug >= 1:
            print(
                "Solving optimization problem with",
                n,
                "parameters and",
                m,
                "constraints",
            )

        p_curr = np.array(jax.device_get(path.get_params()))
        jac_fn = lambda p: jax.device_get(  # noqa: E731
            jax.jit(jacfwd(path.get_constraint_residuals))(np.array(p))
        )
        resid_fn = lambda p: jax.device_get(  # noqa: E731
            jax.jit(path.get_constraint_residuals)(np.array(p))
        )

        config = SolverConfig(
            self._debug, self._max_iter, self._line_search, self._tolerance
        )

        return solve(config, p_curr, resid_fn, jac_fn)

    def jac(self, path: DubinsPath, params: np.ndarray) -> np.ndarray:
        return jacfwd(path.get_constraint_residuals)(params)
