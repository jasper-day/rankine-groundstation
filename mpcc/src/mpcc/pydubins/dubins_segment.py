import jax.numpy as jnp
from enum import Enum
from .dubins_types import Vec2


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
        raise NotImplementedError()

    @staticmethod
    def start_with(params: jnp.ndarray) -> Vec2:
        "Starting coordinates with different parameters"
        raise NotImplementedError()

    def end(self) -> Vec2:
        "End coordinates of the segment"
        raise NotImplementedError()

    @staticmethod
    def end_with(params: jnp.ndarray) -> float:
        "Ending coordinates with different parameters"
        raise NotImplementedError()

    def heading_start(self) -> float:
        "Heading of the start of the segment"
        raise NotImplementedError()

    @staticmethod
    def heading_start_with(params: jnp.ndarray) -> float:
        raise NotImplementedError()

    def heading_end(self) -> float:
        "Heading of the end of the segment"
        raise NotImplementedError()

    @staticmethod
    def heading_end_with(params: jnp.ndarray) -> float:
        raise NotImplementedError()

    def eval(self, arclength: float) -> Vec2:
        "Coordinates at given arclength"
        raise NotImplementedError()

    def get_params(self) -> jnp.ndarray:
        "Parameters defining this segment"
        raise NotImplementedError()

    def set_params(self, params: jnp.ndarray):
        "Set the parameters defining this segment"
        raise NotImplementedError()

    def from_offset(self, offset: float):
        "Create a new, offset segment"
        raise NotImplementedError()
    
    def possible_arclengths(self, position: jnp.ndarray) -> list[float]:
        raise NotImplementedError()


class LineSegment(Segment):
    "A straight segment between two points"

    type = SegmentType.LINESEGMENT
    n_params = 4

    def __init__(self, start: Vec2, end: Vec2):
        self._start = start
        self._end = end

    def length(self) -> float:
        return jnp.linalg.norm(self._end - self._start)

    def start(self) -> Vec2:
        return self._start

    @staticmethod
    def start_with(params):
        return params[:2]

    def end(self) -> Vec2:
        return self._end

    @staticmethod
    def end_with(params):
        return params[2:]

    def heading_start(self) -> float:
        direction = self._end - self._start
        return jnp.atan2(direction[1], direction[0])

    @staticmethod
    def heading_start_with(params):
        direction = params[2:] - params[:2]
        return jnp.atan2(direction[1], direction[0])

    def heading_end(self) -> float:
        return self.heading_start()

    @staticmethod
    def heading_end_with(params):
        return LineSegment.heading_start_with(params)

    def eval(self, arclength) -> Vec2:
        direction = self._end - self._start
        total_length = self.length()
        t = jnp.clip(arclength / total_length, 0, 1)
        return self._start + t * direction

    def path_coords(self, point: Vec2) -> Vec2:
        to_point = point - self._start
        to_end = self._end - self._start
        length = self.length()
        arclength = to_point.dot(to_end) / length
        normal_dist = jnp.linalg.norm(to_point - to_end * arclength / length)
        arclength_clipped = jnp.clip(arclength, 0, length)
        return jnp.array([arclength_clipped, normal_dist])

    def get_params(self) -> jnp.ndarray:
        return jnp.concat([self._start, self._end])

    def set_params(self, params: jnp.ndarray):
        self._start = params[:2]
        self._end = params[2:]

    def from_offset(self, offset):
        AB = self.end() - self.start()
        AB /= jnp.linalg.norm(AB)
        # perpendicular to the left
        perp = jnp.array([AB[1] * offset, -AB[0] * offset])
        return LineSegment(self.start() + perp, self.end() + perp)
    
    def possible_arclengths(self, position):
        # only one possible arclength for a line segment
        AB = self.end() - self.start()
        AB /= jnp.linalg.norm(AB)
        return [jnp.clip(AB.dot(position - self.start()), 0, self.length())]

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
        return jnp.abs(self._arclength)

    def start(self):
        return self.eval(0)

    @staticmethod
    def start_with(params):
        return CircularSegment._eval_with(params, 0)

    def end(self):
        return self.eval(jnp.abs(self._arclength))

    @staticmethod
    def end_with(params):
        return CircularSegment._eval_with(params, jnp.abs(params[4]))

    def eval(self, arclength: float):
        arclength_clipped = jnp.clip(arclength, 0, jnp.abs(self._arclength))
        angle = self._heading + arclength_clipped / self._radius * self._dir()
        return self._center + self._radius * jnp.array([jnp.cos(angle), jnp.sin(angle)])

    @staticmethod
    def _eval_with(params: jnp.ndarray, arclength: float) -> jnp.ndarray:
        center = params[:2]
        radius = params[2]
        heading = params[3]
        c_arclength = params[4]
        _dir = jnp.where(c_arclength > 0, 1.0, -1.0)
        arclength_clipped = jnp.clip(arclength, 0, jnp.abs(c_arclength))
        angle = heading + arclength_clipped / radius * _dir
        return center + radius * jnp.array([jnp.cos(angle), jnp.sin(angle)])

    def heading_start(self):
        return self._heading + jnp.pi / 2 * self._dir()

    @staticmethod
    def heading_start_with(params):
        heading = params[3]
        _dir = jnp.where(params[4] > 0, 1.0, -1.0)
        return heading + jnp.pi / 2 * _dir

    def heading_end(self):
        return self.heading_start() + self._arclength / self._radius

    @staticmethod
    def heading_end_with(params):
        radius = params[2]
        arclength = params[4]
        return CircularSegment.heading_start_with(params) + arclength / radius

    def _dir(self):
        return jnp.where(self._arclength > 0, 1.0, -1.0)

    def get_params(self):
        return jnp.array(
            [
                self._center[0],
                self._center[1],
                self._radius,
                self._heading,
                self._arclength,
            ]
        )

    def set_params(self, params: jnp.ndarray):
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
    
    def possible_arclengths(self, position):
        # family of possible arclengths for a circular segment
        # noting that it could wrap onto itself multiple times...
        R = position - self._center
        phi = jnp.atan2(R[1], R[0])
        th = phi - self._heading
        arclength = th * self._radius
        # find every possible arclength
        circumference = 2 * jnp.pi * self._radius
        length = jnp.abs(self._arclength)
        possibles = []
        ll = arclength - circumference
        while (ll < length):
            if ll > 0:
                possibles.append(float(ll))
            ll += circumference
        return possibles