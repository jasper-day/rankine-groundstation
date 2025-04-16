from mpcc.pydubins import DubinsPath, CircularSegment
import numpy as np


def append_loiter(path: DubinsPath, radius: float):
    "Append a very long loiter to the end of the path"
    last_segment = path.get_segment(path.num_segments() - 1)
    end = path.end()
    heading = last_segment.heading_end()
    assert np.allclose(end - last_segment.end(), 0), "Sanity check"
    center = end + radius * np.array(
        [np.cos(heading + np.pi / 2), np.sin(heading + np.pi / 2)]
    )
    loiter = CircularSegment(center, radius, heading - np.pi / 2, 1000000)
    path.add_segment(loiter)
