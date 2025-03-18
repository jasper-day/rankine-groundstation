from mpcc.pydubins import LineSegment, CircularSegment, DubinsPath
from pytest import approx
import numpy as np

ls1 = LineSegment(np.array([10.0, 0.0]), np.array([10.0, 20.0]))
cs1 = CircularSegment(np.array([15, 20]), 5, -1, np.pi, 5 * np.pi/2)
ls2 = LineSegment(np.array([15, 25]), np.array([35, 25]))
cs2 = CircularSegment(np.array([35,35]), 10, 1, -np.pi/2, 10 * np.pi/2)
cs3 = CircularSegment(np.array([50, 35]), 5, -1, np.pi, 5 * np.pi)

sqrthalf = np.sqrt(0.5)

# path = DubinsPath([ls1, cs1, ls2, cs2, cs3])

def test_lengths():
    assert ls1.length() == approx(20.0)
    assert cs1.length() == approx(5.0 * np.pi / 2.0)
    assert ls2.length() == approx(20.0)
    assert cs2.length() == approx(10.0 * np.pi / 2.0)
    assert cs3.length() == approx(5.0 * np.pi)

def test_start_end():
    assert ls1.start() == approx(np.array([10.0, 0.0]))
    assert ls1.end() == approx(np.array([10.0, 20.0]))
    assert cs1.start() == approx(np.array([10.0, 20.0]))
    assert cs1.end() == approx(np.array([15.0, 25.0]))
    assert ls2.start() == approx(np.array([15.0, 25.0]))
    assert ls2.end() == approx(np.array([35.0, 25.0]))
    assert cs2.start() == approx(np.array([35.0, 25.0]))
    assert cs2.end() == approx(np.array([45.0, 35.0]))
    assert cs3.start() == approx(np.array([45.0, 35.0]))
    assert cs3.end() == approx(np.array([55.0, 35.0]))

def test_headings():
    assert ls1.heading_end() == approx(ls1.heading_start())
    assert ls1.heading_end() == approx(np.pi/2)
    assert cs1.heading_start() == approx(np.pi/2)
    assert cs1.heading_end() == approx(0.0)
    assert ls2.heading_end() == approx(ls2.heading_start())
    assert ls2.heading_end() == approx(0.0)
    assert cs2.heading_start() == approx(0.0)
    assert cs2.heading_end() == approx(np.pi / 2)
    assert cs3.heading_start() == approx(np.pi / 2)
    assert cs3.heading_end() == approx(-np.pi / 2)

def test_evals():
    # inside segment
    assert ls1.eval(5) == approx(np.array([10, 5]))
    assert cs1.eval(5 * np.pi / 4) == approx(np.array([15, 20]) + 5 * np.array([-sqrthalf, sqrthalf]))
    assert ls2.eval(10) == approx(np.array([25,25]))
    assert cs2.eval(10 * np.pi/4) == approx(np.array([35, 35]) + 10 * np.array([sqrthalf, -sqrthalf]))
    assert cs3.eval(5 * np.pi/4) == approx(np.array([50, 35]) + 5 * np.array([-sqrthalf, sqrthalf]))
    assert cs3.eval(5 * 3 * np.pi / 4) == approx(np.array([50, 35]) + 5 * np.array([sqrthalf, sqrthalf]))
    # outside segment
    # only need to test line segment and circle
    assert ls1.eval(-1) == approx(ls1.start())
    assert ls1.eval(21) == approx(ls1.end())
    assert cs1.eval(-1) == approx(cs1.start())
    assert cs1.eval(5 * np.pi) == approx(cs1.end())
    # circle in other direction
    assert cs2.eval(-1) == approx(cs2.start())
    assert cs2.eval(10 * np.pi) == approx(cs2.end())