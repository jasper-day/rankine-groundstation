from test_pydubins import create_test_segments
from mpcc.pydubins import DubinsPath
from pytest import approx
from mpcc.pydubins.cpp_solve import ls_start_with, ls_end_with, ls_heading_start_with, ls_heading_end_with, cs_start_with, cs_end_with, cs_heading_start_with, cs_heading_end_with
import numpy as np

ls1, cs1, ls2, cs2, cs3 = create_test_segments()

path = DubinsPath([ls1, cs1, ls2, cs2, cs3])

def test_get_set_params():
    # test round trip
    for segment in path._segments:
        params = segment.get_params().copy()
        segment.set_params(params)
        assert params == approx(segment.get_params())

def test_cpp_functions_ls():
    for ls, name in [(ls1, "ls1"), (ls2, "ls2")]:
        params = ls.get_params().copy()

        assert ls.start() == approx(ls.start_with(ls.get_params()))
        assert ls.end() == approx(ls.end_with(ls.get_params()))
        assert ls.heading_start() == approx(ls.heading_start_with(ls.get_params()))
        assert ls.heading_end() == approx(ls.heading_end_with(ls.get_params()))

        assert ls.start() == approx(ls_start_with(ls.get_params()))
        assert ls.end() == approx(ls_end_with(ls.get_params()))
        assert ls.start() == approx(params[:2])
        assert ls.end() == approx(params[2:])
        assert ls.heading_start() == approx(ls_heading_start_with(ls.get_params())), name
        assert ls.heading_end() == approx(ls_heading_end_with(ls.get_params()))

def test_cpp_functions_cs():
    for cs in [cs1, cs2, cs3]:
        params = cs.get_params().copy()

        assert cs.start() == approx(cs.start_with(cs.get_params()))
        assert cs.end() == approx(cs.end_with(cs.get_params()))
        assert cs.heading_start() == approx(cs.heading_start_with(cs.get_params()))
        assert cs.heading_end() == approx(cs.heading_end_with(cs.get_params()))

        assert cs.start() == approx(cs_start_with(cs.get_params()))
        assert cs.end() == approx(cs_end_with(cs.get_params()))
        assert cs.heading_start() == approx(cs_heading_start_with(cs.get_params()))
        assert cs.heading_end() == approx(cs_heading_end_with(cs.get_params()))

        assert params == approx(cs.get_params())