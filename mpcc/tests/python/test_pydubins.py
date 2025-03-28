from mpcc.pydubins import LineSegment, CircularSegment, DubinsPath, SegmentType
from pytest import approx
import numpy as np

def create_test_segments():
    """Creates a test Dubins path with 5 segments:
    1. Vertical line segment from (10, 0) to (10, 20)
    2. Circular segment with center (15, 20), radius 5, direction -1, heading pi, arclength 5pi/2
    3. Horizontal line segment from (15, 25) to (35, 25)
    4. Circular segment with center (35, 35), radius 10, direction 1, heading -pi/2, arclength 10pi/2
    5. Circular segment with center (50, 35), radius 5, direction -1, heading pi, arclength 5pi
    """
    ls1 = LineSegment(np.array([10.0, 0.0]), np.array([10.0, 20.0]))
    cs1 = CircularSegment(np.array([15, 20]), 5, np.pi, -5 * np.pi/2)
    ls2 = LineSegment(np.array([15, 25]), np.array([35, 25]))
    cs2 = CircularSegment(np.array([35,35]), 10, -np.pi/2, 10 * np.pi/2)
    cs3 = CircularSegment(np.array([50, 35]), 5, np.pi, -5 * np.pi)

    return ls1, cs1, ls2, cs2, cs3

# Create the test path for use in this file
ls1, cs1, ls2, cs2, cs3 = create_test_segments()

sqrthalf = np.sqrt(0.5)

path = DubinsPath([ls1, cs1, ls2, cs2, cs3])

def test_get_params():
    assert ls1.get_params().shape == (4,)
    assert cs1.get_params().shape == (5,)
    assert ls1.get_params() == approx(np.array([10.0, 0.0, 10.0, 20.0]))
    assert cs1.get_params() == approx(np.array([15.0, 20.0, 5.0, np.pi, -5.0 * np.pi / 2.0]))
    assert ls2.get_params() == approx(np.array([15.0, 25.0, 35.0, 25.0]))
    assert cs2.get_params() == approx(np.array([35.0, 35.0, 10.0, -np.pi / 2.0, 10.0 * np.pi / 2.0]))
    assert cs3.get_params() == approx(np.array([50.0, 35.0, 5.0, np.pi, -5.0 * np.pi]))

def test_get_path_params():
    assert path.get_params() == approx(
        np.concatenate([ls1.get_params(), cs1.get_params(), ls2.get_params(), cs2.get_params(), cs3.get_params()])
    )

def test_constraint_residuals():
    assert path.get_constraint_residuals(path.get_params()) == approx(np.zeros(8))
    altered_params = path.get_params()
    # change endpoint of first line, now pointing at 45°
    altered_params[2] = 30.0
    expected_residuals = np.zeros(8)
    # location, then heading residuals
    expected_residuals[0] = 20.0**2
    expected_residuals[4] = np.sin(np.pi / 8)**2
    assert path.get_constraint_residuals(altered_params) == approx(expected_residuals)

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

def test_path_start_end():
    assert path.start() == approx(ls1.start())
    assert path.end() == approx(cs3.end())

def test_path_length():
    assert path.length() == approx(ls1.length() + cs1.length() + ls2.length() + cs2.length() + cs3.length())

def test_path_start_end():
    assert path.start() == approx(ls1.start())
    assert path.end() == approx(cs3.end())

def test_path_eval():
    assert path.eval(0) == approx(ls1.start())
    assert path.eval(ls1.length()) == approx(cs1.start())
    assert path.eval(ls1.length() + cs1.length()) == approx(ls2.start())

def test_types():
    assert ls1.type == SegmentType.LINESEGMENT
    assert cs1.type == SegmentType.CIRCULARSEGMENT
    assert path.get_types() == [
        SegmentType.LINESEGMENT,
        SegmentType.CIRCULARSEGMENT,
        SegmentType.LINESEGMENT,
        SegmentType.CIRCULARSEGMENT,
        SegmentType.CIRCULARSEGMENT
    ]