from enum import Enum
from .dubins_types import Vec2
import numpy as np

class SegmentType(Enum):
    SEGMENT = 1
    LINESEGMENT = 2
    CIRCULARSEGMENT = 3


class Segment:
    "Generic base class for segments"

    type: SegmentType = SegmentType.SEGMENT
    n_params: int

    def path_coords(self, point: Vec2) -> Vec2:
        "Find the distance along and normal distance to the segment"
        raise NotImplementedError()

    def length(self) -> float:
        "Length of the segment"
        raise NotImplementedError()

    def start(self) -> Vec2:
        "Starting coordinates of the segment"
        return self.eval(0.0)

    @staticmethod
    def start_with(params: np.ndarray) -> Vec2:
        "Starting coordinates with different parameters"
        raise NotImplementedError()

    def end(self) -> Vec2:
        "End coordinates of the segment"
        return self.eval(self.length())

    @staticmethod
    def end_with(params: np.ndarray) -> float:
        "Ending coordinates with different parameters"
        raise NotImplementedError()

    def heading_start(self) -> float:
        "Heading of the start of the segment"
        raise NotImplementedError()

    @staticmethod
    def heading_start_with(params: np.ndarray) -> float:
        raise NotImplementedError()

    def heading_end(self) -> float:
        "Heading of the end of the segment"
        raise NotImplementedError()

    @staticmethod
    def heading_end_with(params: np.ndarray) -> float:
        raise NotImplementedError()

    def eval(self, arclength: float) -> Vec2:
        "Coordinates at given arclength"
        raise NotImplementedError()

    def get_params(self) -> np.ndarray:
        "Parameters defining this segment"
        raise NotImplementedError()

    def set_params(self, params: np.ndarray):
        "Set the parameters defining this segment"
        raise NotImplementedError()

    def from_offset(self, offset: float):
        "Create a new, offset segment"
        raise NotImplementedError()
    
    def ds(self, position: np.ndarray, velocity: np.ndarray) -> float:
        "Arclength velocity tangent to the curve"
        raise NotImplementedError()

    def possible_arclengths(self, position: np.ndarray) -> list[float]:
        raise NotImplementedError()
    


class LineSegment(Segment):
    "A straight segment between two points"

    type = SegmentType.LINESEGMENT
    n_params = 4

    def __init__(self, start: Vec2, end: Vec2):
        self._start = start
        self._end = end

    def length(self) -> float:
        return np.linalg.norm(self._end - self._start)

    def start(self):
        return super().start()

    @staticmethod
    def start_with(params):
        return params[:2]

    def end(self):
        return super().end()

    @staticmethod
    def end_with(params):
        return params[2:]

    def heading_start(self) -> float:
        d = self._end - self._start
        return np.atan2(d[1], d[0])

    @staticmethod
    def heading_start_with(params):
        direction = LineSegment.end_with(params) - LineSegment.start_with(params)
        return np.atan2(direction[1], direction[0])

    def heading_end(self) -> float:
        return self.heading_start()

    @staticmethod
    def heading_end_with(params):
        return LineSegment.heading_start_with(params)

    def eval(self, arclength) -> Vec2:
        direction = self._end - self._start
        total_length = self.length()
        t = np.clip(arclength / total_length, 0, 1)
        return self._start + t * direction

    def path_coords(self, point: Vec2) -> Vec2:
        to_point = point - self._start
        to_end = self._end - self._start
        length = self.length()
        arclength = to_point.dot(to_end) / length
        normal_dist = np.linalg.norm(to_point - to_end * arclength / length)
        arclength_clipped = np.clip(arclength, 0, length)
        return np.array([arclength_clipped, normal_dist])

    def get_params(self) -> np.ndarray:
        return np.concat([self._start, self._end])

    def set_params(self, params: np.ndarray):
        self._start = params[:2]
        self._end = params[2:]

    def from_offset(self, offset):
        AB = self.end() - self.start()
        AB /= np.linalg.norm(AB)
        # perpendicular to the left
        perp = np.array([AB[1] * offset, -AB[0] * offset])
        return LineSegment(self.start() + perp, self.end() + perp)
    
    def ds(self, position, velocity):
        AB = self.end() - self.start()
        AB /= np.linalg.norm(AB)
        return velocity.dot(AB)

    def possible_arclengths(self, position):
        # only one possible arclength for a line segment
        AB = self.end() - self.start()
        AB /= np.linalg.norm(AB)
        return [np.clip(AB.dot(position - self.start()), 0, self.length())]
    
    

class CircularSegment(Segment):
    "A circular arc"

    type = SegmentType.CIRCULARSEGMENT
    n_params = 5

    def __init__(self, centre: Vec2, radius: float, heading: float, arclength: float):
        self._center = centre
        self._radius = radius
        self._heading = heading
        self._arclength = arclength

    def length(self):
        return np.abs(self._arclength)

    def start(self):
        return self.eval(0)

    @staticmethod
    def start_with(params):
        return CircularSegment._eval_with(params, 0)

    def end(self):
        return self.eval(np.abs(self._arclength))

    @staticmethod
    def end_with(params):
        return CircularSegment._eval_with(params, np.abs(params[4]))

    def eval(self, arclength: float):
        arclength_clipped = np.clip(arclength, 0, np.abs(self._arclength))
        angle = self._heading + arclength_clipped / self._radius * self._dir()
        return self._center + self._radius * np.array([np.cos(angle), np.sin(angle)])

    @staticmethod
    def _eval_with(params: np.ndarray, arclength: float) -> np.ndarray:
        center = params[:2]
        radius = params[2]
        heading = params[3]
        c_arclength = params[4]
        _dir = np.where(c_arclength > 0, 1.0, -1.0)
        arclength_clipped = np.clip(arclength, 0, np.abs(c_arclength))
        angle = heading + arclength_clipped / radius * _dir
        return center + radius * np.array([np.cos(angle), np.sin(angle)])

    def heading_start(self):
        return self._heading + np.pi / 2 * self._dir()

    @staticmethod
    def heading_start_with(params):
        heading = params[3]
        _dir = np.where(params[4] > 0, 1.0, -1.0)
        return heading + np.pi / 2 * _dir

    def heading_end(self):
        return self.heading_start() + self._arclength / self._radius

    @staticmethod
    def heading_end_with(params):
        radius = params[2]
        arclength = params[4]
        return CircularSegment.heading_start_with(params) + arclength / radius

    def _dir(self):
        return np.where(self._arclength > 0, 1.0, -1.0)

    def get_params(self):
        return np.array(
            [
                self._center[0],
                self._center[1],
                self._radius,
                self._heading,
                self._arclength,
            ]
        )

    def set_params(self, params: np.ndarray):
        self._center = params[:2]
        self._radius = params[2]
        self._heading = params[3]
        self._arclength = params[4]

    def from_offset(self, offset):
        return CircularSegment(
            self._center,
            self._radius + offset * self._dir(),
            self._heading,
            self._arclength * (1 + offset * self._dir() / self._radius),
        )
    
    def ds(self, position, velocity):
        # nomenclature to match cc_controllers
        circ_r = self._radius
        circ_c = self._center
        circ_xc = position - circ_c
        circ_dir = self._dir()
        return circ_r * circ_dir * np.dot(velocity, np.array([-circ_xc[1], circ_xc[0]])) / circ_xc.dot(circ_xc)

    def possible_arclengths(self, position):
        # family of possible arclengths for a circular segment
        # noting that it could wrap onto itself multiple times...
        R = position - self._center
        phi = np.atan2(R[1], R[0])
        th = phi - self._heading
        arclength = th * self._radius
        # find every possible arclength
        circumference = 2 * np.pi * self._radius
        length = np.abs(self._arclength)
        possibles = []
        ll = arclength - circumference
        while (ll < length):
            if ll > 0:
                possibles.append(float(ll))
            ll += circumference
        return possibles