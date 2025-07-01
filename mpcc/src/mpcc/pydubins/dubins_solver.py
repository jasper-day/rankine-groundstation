from mpcc.pydubins.dubins_path import DubinsPath
from mpcc.pydubins.cpp_solve import SolverConfig, SolverResult, solve_path, CPPSegmentType
from mpcc.pydubins.dubins_segment import SegmentType


class DubinsSolver:
    def __init__(self, tolerance: float, max_iter: int, debug: int, line_search: bool, epsilon: float):
        self._tolerance = tolerance
        self._max_iter = max_iter
        self._debug = debug
        self._line_search = line_search
        self._epsilon = epsilon

    def solve(self, path: DubinsPath, dragged_points: list[bool]) -> SolverResult:
        """Find a suitable configuration of a Dubins path.
        dragged_points currently not implemented
        """
        params = path.get_params()
        config = SolverConfig(self._debug, self._max_iter, self._line_search, self._tolerance, self._epsilon)
        types = map_types(path.get_types())
        return solve_path(config, params, types)

def map_types(types: list[SegmentType]):
    return [CPPSegmentType.CIRCLE if type == SegmentType.CIRCULARSEGMENT else CPPSegmentType.LINE for type in types]