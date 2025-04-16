from numpy import ndarray
from numpy import float64
from typing import TypeAlias, List, Any

Vec2: TypeAlias = ndarray[(2,), float64]

class SegmentType:
    SEGMENT = ...
    LINESEGMENT = ...
    CIRCULARSEGMENT = ...

class Segment:
    "Generic base class for segments"

    type: SegmentType
    def path_coords(self, point: Vec2) -> Vec2: ...
    def length(self) -> float64: ...
    def start(self) -> Vec2: ...
    def end(self) -> Vec2: ...
    def heading_start(self) -> float64: ...
    def heading_end(self) -> float64: ...
    def eval(self, arclength: float64) -> Vec2: ...

class LineSegment(Segment):
    "A straight segment between two points"
    def __init__(self, start: Vec2, end: Vec2): ...
    def get_params(self) -> ndarray[(4,), float64]: ...
    def set_params(self, params: ndarray[(4,), float64]): ...

class CircularSegment(Segment):
    "A curved segment"
    def __init__(
        self, centre: Vec2, radius: float64, heading: float64, arclength: float64
    ): ...
    def get_params(self) -> ndarray[(5,), float64]: ...
    def set_params(self, params: ndarray[(5,), float64]): ...

class DubinsPath:
    def __init__(self, segments: List[Segment]): ...
    def add_segment(self, segment: Segment):
        "Append a segment to the path"
    def num_segments(self) -> int:
        "Number of segments in the path"
    def get_segment(self, index: int) -> Segment:
        "Get segment by index"
    def lengths(self) -> List[float64]:
        "Length of each segment"
    def length(self) -> float64:
        "Total length of the path"
    def eval(self, arclength: float64) -> Vec2:
        "Get the path position at a given arclength"
    def start(self) -> Vec2:
        "Path starting position"
    def end(self) -> Vec2:
        "Path ending position"
    def get_params(self) -> ndarray[float64, Any]: ...
    def set_params(self, params: ndarray[float64, Any]): ...
    def get_constraint_residuals(
        self, params: ndarray[float64, Any]
    ) -> ndarray[float64, Any]: ...
    def n_params(self) -> int: ...
    def n_constraints(self) -> int: ...
    def get_types(self) -> list[SegmentType]: ...

class SolverResult:
    params: ndarray
    converged: bool

class DubinsSolver:
    def __init__(
        self, tolerance: float, max_iter: int, debug: int, line_search: bool
    ): ...
    def solve(self, path: DubinsPath, dragged_points: List[bool]) -> SolverResult:
        """Find a suitable configuration of a Dubins path.
        dragged_points currently not implemented
        """
    def jac(
        self, path: DubinsPath, params: ndarray[float64, Any]
    ) -> ndarray[float64, Any]:
        "Get the jacobian of the constraint residual function"
