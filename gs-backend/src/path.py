from pydantic import BaseModel, TypeAdapter, Discriminator
from typing import Literal, Union
from typing_extensions import Annotated
from mpcc.pydubins import DubinsPath, LineSegment, CircularSegment, SegmentType, DubinsSolver
import numpy as np
import matplotlib.pyplot as plt

class Arc(BaseModel):
    type: Literal["arc"] = "arc"
    centre: list[float]
    radius: float
    direction: float
    heading: float
    arclength: float
    def to_dubins(self):
        print("Creating Dubins Circular segment")
        assert len(self.centre) == 2
        assert np.isclose(self.direction, 1) or np.isclose(self.direction, -1)
        return CircularSegment(
            center=np.array(self.centre, np.float64),
            radius=self.radius,
            dir=self.direction,
            heading=self.heading,
            arclength=self.arclength,
        )


class Line(BaseModel):
    type: Literal["line"] = "line"
    start: list[float]
    end: list[float]
    def to_dubins(self):
        print("Creating Dubins Line segment")
        assert len(self.start) == 2
        assert len(self.end) == 2
        return LineSegment(
            start=np.array(self.start, np.float64),
            end=np.array(self.end, np.float64)
        )



path_adapter = TypeAdapter(list[
    Annotated[
    Union[Arc, Line],
    Discriminator('type')
    ]
    ])

Path = list[Union[Arc, Line]]

def to_dubins(path: Path) -> DubinsPath:
    segments = [segment.to_dubins() for segment in path]
    return DubinsPath(segments), segments


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
                        centre=[params[index], params[index+1]],
                        radius=params[index+2],
                        direction=params[index+3],
                        heading=params[index+4],
                        arclength=params[index+5]
                    )
                )
                index += 6
    return path

def solve(path: Path) -> Path:
    print("solving path")
    solver = DubinsSolver(tolerance=1e-6, max_iter=100, debug=1)
    dubins, _ = to_dubins(path)
    # th = np.linspace(0, dubins.length(), 200)
    # locs = np.array([dubins.eval(t) for t in th])
    # plt.plot(locs[:, 1], locs[:, 0])
    # plt.xlabel("Easting, m")
    # plt.ylabel("Northing, m")
    # i = 0
    # for segment in segments:
    #     plt.annotate(f"{i}", xy=np.flip(segment.start()))
    #     plt.annotate(f"{i+1}", xy=np.flip(segment.end()))
    #     i += 2
    # plt.gca().set_aspect("equal")
    # plt.show()
    params = solver.solve(dubins, [False])
    return to_path(dubins, params)


def main():
    from pprint import pprint
    pprint(path_adapter.json_schema())

if __name__ == "__main__":
    main()