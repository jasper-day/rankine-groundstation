"""MPCC for fixed-wing UAS.

The global path is given as a series of arclengths and parameters.

Will come back to this once EMPCC and PT have been figured out, soon.
"""

from acados_template import AcadosModel
from fdm2d import export_fdm_2d
from casadi import SX, vertcat, if_else, dot, norm_2, fabs, atan2
from casadi.Function import bspline
import numpy as np
import scipy.linalg


def export_mpcc_controller(model: AcadosModel) -> AcadosModel:
    raise NotImplementedError()
    return model
