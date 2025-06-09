from pydubins import DubinsPath, SegmentType, Segment
import numpy as np


def find_correct_s(
    path: DubinsPath, north: float, east: float, current_s: float
) -> float:
    "Find the closest arclength on the path to the current arclength"
    lengths = np.array(path.lengths())
    cum_lengths = np.cumsum(lengths)
    curr_segments = np.argwhere(cum_lengths >= current_s)
    assert curr_segments.shape[0] >= 2, "End of path is not yet implemented, sorry!"
    first_segment = path.get_segment(curr_segments[0])
    arclength, _ = first_segment.path_coords(np.array([north, east]))
    return cum_lengths[curr_segments[0]] + arclength


def get_acados_path_parameters(path: DubinsPath, current_s: float) -> np.ndarray:
    """Get parameters of the form
    [
    type_0: 0 | 1
    params_0: float[4]
    s_switch: float
    type_1: 0 | 1
    params_1: float[4] ]
    """
    lengths = np.array(path.lengths())
    cum_lengths = np.cumsum(lengths)
    curr_segments = np.argwhere(cum_lengths >= current_s)
    # get the first and second segments
    first_segment = path.get_segment(curr_segments[0])
    # set the parameters
    parameters = np.zeros(11)
    parameters[:5] = get_acados_segment_parameters(first_segment)
    # cum_lengths[i] is the length at the _end_ of segment i
    parameters[5] = cum_lengths[curr_segments[0]]
    assert parameters[5] >= current_s, "Sanity check"
    if curr_segments.shape[0] >= 2:
        second_segment = path.get_segment(curr_segments[1])
        parameters[6:] = get_acados_segment_parameters(second_segment)
    # otherwise just leave as zeros
    # assuming that the path designer will treat this responsibly!
    return parameters


def get_acados_segment_parameters(segment: Segment) -> np.ndarray[(5,), np.float64]:
    parameters = np.zeros(5)
    segment_p = segment.get_params()
    if segment.type == SegmentType.CIRCULARSEGMENT:
        parameters[0] = 1.0
        parameters[1] = segment_p[0]  # center_n
        parameters[2] = segment_p[1]  # center_e
        parameters[3] = segment_p[2]  # radius
        parameters[4] = np.sign(segment_p[4])  # direction of travel

    else:
        parameters[1:] = segment_p  # start_x, start_y, end_x, end_y
    return parameters
