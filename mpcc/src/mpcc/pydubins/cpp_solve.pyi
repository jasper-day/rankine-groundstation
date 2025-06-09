import numpy as np
from typing import Callable

class SolverResult:
    params: np.ndarray
    converged: bool

class SolverConfig:
    def __init__(debug: int, max_iter: int, line_search: bool, tolerance: float): ...
    debug: int
    max_iter: int
    line_search: bool
    tolerance: float

def solve(
    n: int,
    m: int,
    config: SolverConfig,
    p_start: np.ndarray,
    residual_fn: Callable[[np.ndarray], np.ndarray],
    jacobian_fn: Callable[[np.ndarray], np.ndarray],
) -> SolverResult: ...
