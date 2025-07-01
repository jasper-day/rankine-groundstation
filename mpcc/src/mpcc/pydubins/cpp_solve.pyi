from enum import Enum
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

def solve_path(
    config: SolverConfig,
    p_start: np.ndarray,
    types: list[CPPSegmentType]
) -> SolverResult: ...

class CPPSegmentType(Enum):
    LINE: int
    CIRCLE: int