from mpcc.pydubins import DubinsPath, LineSegment, CircularSegment, DubinsSolver
from pytest import approx
import numpy as np

from test_pydubins import create_test_segments

ls1, cs1, ls2, cs2, cs3 = create_test_segments()

path = DubinsPath([ls1, cs1, ls2, cs2, cs3])

solver = DubinsSolver()

def test_solver():
    assert solver.solve(path, [False] * (int(path.num_segments()) - 1 )) == approx(path.get_params())