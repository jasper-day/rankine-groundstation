from pydantic import BaseModel, TypeAdapter
from typing import Literal
from mpcc.pydubins import DubinsPath, LineSegment, CircularSegment, SegmentType, DubinsSolver
import numpy as np

class Arc(BaseModel):
    type: Literal["arc"] = "arc"
    center: list[float]
    radius: float
    dir: float
    heading: float
    arclength: float
    def to_dubins(self):
        assert len(self.center) == 2
        assert np.isclose(self.dir, 1) or np.isclose(self.dir, -1)
        return CircularSegment(
            center=np.array(self.center, np.float64),
            radius=self.radius,
            dir=self.dir,
            heading=self.heading,
            arclength=self.arclength,
        )


class Line(BaseModel):
    type: Literal["line"] = "line"
    start: list[float]
    end: list[float]
    def to_dubins(self):
        assert len(self.start) == 2
        assert len(self.end) == 2
        return LineSegment(
            start=np.array(self.start, np.float64),
            end=np.array(self.end, np.float64)
        )

Segment = Arc | Line

Path = list[Segment]

path_adapter = TypeAdapter(Path)

def to_dubins(path: Path) -> DubinsPath:
    segments = [segment.to_dubins() for segment in path]
    return DubinsPath(segments)


def to_path(dubins: DubinsPath, params: np.ndarray[np.float64]) -> Path:
    types = dubins.get_types()
    path = []
    index = 0
    for type in types:
        assert index < params.shape[0]
        match type:
            case SegmentType.LINESEGMENT:
                path.append(
                    Line(
                        start=[params[index], params[index+1]],
                        end=[params[index+2], params[index+3]]
                    )
                )
                index += 4
            case SegmentType.CIRCULARSEGMENT:
                path.append(
                    Arc(
                        center=[params[index], params[index+1]],
                        radius=params[index+2],
                        dir=params[index+3],
                        heading=params[index+4],
                        arclength=params[index+5]
                    )
                )
                index += 6
    return path

def solve(path: Path) -> Path:
    solver = DubinsSolver()
    dubins = to_dubins(path)
    params = solver.solve(dubins, [False])
    return to_path(dubins, params)


def main():
    from pprint import pprint
    pprint(path_adapter.json_schema())

if __name__ == "__main__":
    main()