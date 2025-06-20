import jax.numpy as jnp
from jax import vmap
from .dubins_segment import Segment, SegmentType
from .dubins_types import Vec2


class DubinsPath:
    def __init__(self, segments: list[Segment]):
        self._segments = segments

    def add_segment(self, segment: Segment):
        "Append a segment to the path"
        self._segments.append(segment)

    def num_segments(self) -> int:
        "Number of segments in the path"
        return len(self._segments)

    def get_segment(self, index: int) -> Segment:
        "Get segment by index"
        return self._segments[index]

    def lengths(self) -> list[float]:
        "Length of each segment"
        return [segment.length() for segment in self._segments]

    def length(self) -> float:
        "Total length of the path"
        return jnp.sum(jnp.array(self.lengths()))

    def eval(self, arclength: float) -> Vec2:
        "Get the path position at a given arclength"
        if self.num_segments() == 0:
            raise Exception("Empty path")
        curr_lengths = self.lengths()
        i = 0
        while i != len(self._segments):
            if arclength < curr_lengths[i]:
                break
            arclength -= curr_lengths[i]
            i += 1
        if i == self.num_segments():
            return self._segments[-1].end()
        return self._segments[i].eval(arclength)

    def start(self) -> Vec2:
        "Path starting position"
        return self._segments[0].start()

    def end(self) -> Vec2:
        "Path ending position"
        return self._segments[-1].end()

    def get_params(self):
        return jnp.concat([segment.get_params() for segment in self._segments])

    def set_params(self, params: jnp.ndarray):
        assert params.shape == self.get_params().shape
        index = 0
        for segment in self._segments:
            segment.set_params(params[index : index + segment.n_params])
            index += segment.n_params

    def get_constraint_residuals(self, params: jnp.ndarray) -> jnp.ndarray:
        n_points = len(self._segments) - 1
        output = jnp.empty((n_points * 2))

        index = 0
        for i in range(n_points):
            index_i = index + self._segments[i].n_params
            params_i = params[index:index_i]
            index_ii = index_i + self._segments[i + 1].n_params
            params_ii = params[index_i:index_ii]
            segment_i = self._segments[i]
            segment_ii = self._segments[i + 1]

            # dist = end - start
            distance = segment_i.end_with(params_i) - segment_ii.start_with(params_ii)
            assert distance.shape == (2,)
            # resid = distance @ distance
            output = output.at[i].set(distance.dot(distance))
            heading_diff = segment_i.heading_end_with(
                params_i
            ) - segment_ii.heading_start_with(params_ii)
            output = output.at[n_points + i].set(
                jnp.pow(jnp.sin(heading_diff / 2.0), 2)
            )
            # move index forward
            index = index_i
        return output

    def n_params(self) -> int:
        return jnp.sum(jnp.array([segment.n_params for segment in self._segments]))

    def n_constraints(self) -> int:
        "Get the number of constraints (two for each knot point)"
        return (len(self._segments) - 1) * 2

    def get_types(self) -> list[SegmentType]:
        return [segment.type for segment in self._segments]

    def get_closest_arclength(self, pos: Vec2, estimated_arclength: float):
        "Find the arclength to a given point closest to the estimated arclength"
        arclength_prev = 0.0
        closest_arclength = 0.0
        closest_delta = jnp.inf
        for segment in self._segments:
            # please lord let it not be O(N^2)
            possibles = segment.possible_arclengths(pos)
            for possible in possibles:
                arclength = arclength_prev + possible
                delta = jnp.abs(estimated_arclength - arclength)
                # 10 meters is chosen to allow the estimated arclength to jump segments
                if delta < closest_delta or delta < 10:
                    closest_arclength = arclength
                    closest_delta = delta
            arclength_prev += segment.length()
        return closest_arclength

    def offset_path(self, offset: float):
        "Return a new path, offset by an amount"
        return DubinsPath([segment.from_offset(offset) for segment in self._segments])