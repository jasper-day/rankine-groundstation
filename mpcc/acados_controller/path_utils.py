from mpcc.pydubins import DubinsPath, CircularSegment
import numpy as np
from mpcc.acados_parameters import get_acados_path_parameters
import scipy.interpolate as ip
from casadi import MX, interpolant


def append_loiter(path: DubinsPath, radius: float):
    "Append a very long loiter to the end of the path"
    last_segment = path.get_segment(path.num_segments() - 1)
    end = path.end()
    heading = last_segment.heading_end()
    assert np.allclose(end - last_segment.end(), 0), "Sanity check"
    center = end + radius * np.array(
        [np.cos(heading + np.pi / 2), np.sin(heading + np.pi / 2)]
    )
    loiter = CircularSegment(center, radius, heading - np.pi / 2, 10000)
    path.add_segment(loiter)


def get_params(wind: np.ndarray, v_A: np.float64, path: DubinsPath, s: float):
    params = np.zeros(14)
    params[:2] = wind
    params[2] = v_A
    params[3:] = get_acados_path_parameters(path, s)
    return params


def curry_params(wind: np.ndarray, v_A: np.float64, path: DubinsPath):
    return lambda s: get_params(wind, v_A, path, s)


def get_spline_MX(path: DubinsPath, n_s: int) -> tuple[MX, MX]:
    "Get a spline with `n_s` breakpoints from a Dubins Path"
    # path is already arclength parameterized
    ts = np.linspace(0, path.length(), n_s)
    locs = np.array([path.eval(t) for t in ts])
    north = locs[:, 0]
    east = locs[:, 1]
    # build coefficients at compilation time
    lut_north = interpolant("LUT_north", "bspline", [ts], north)
    lut_east = interpolant("LUT_east", "bspline", [ts], east)
    return lut_north, lut_east


def get_spline_numeric(
    path: DubinsPath, n_s: int
) -> tuple[ip.InterpolatedUnivariateSpline, ip.InterpolatedUnivariateSpline]:
    "Get Scipy spline with `n_s` breakpoints from a Dubins Path"
    ts = np.linspace(0, path.length(), n_s)
    locs = np.array([path.eval(t) for t in ts])
    north = locs[:, 0]
    east = locs[:, 1]
    lut_north = ip.InterpolatedUnivariateSpline(ts, north)
    lut_east = ip.InterpolatedUnivariateSpline(ts, east)
    return lut_north, lut_east
