"""MPCC for fixed-wing UAS.

The global path is given as a series of arclengths and parameters.

Will come back to this once EMPCC and PT have been figured out, soon.
"""

from acados_template import AcadosModel
from casadi import (
    MX,
    vertcat,
    dot,
    norm_2,
    jacobian,
)
import numpy as np
from mpcc.pydubins import DubinsPath
from typing import Any
from path_utils import get_spline_MX


def export_mpcc_controller(
    model: AcadosModel, path: DubinsPath, controller_config: dict[str, Any]
) -> AcadosModel:
    x = model.x
    u = model.u
    f_expl = model.f_expl_expr

    north = x[0]
    east = x[1]
    phi = x[3]
    dphi = x[4]

    dphi_ref = u[0]

    pos = vertcat(north, east)

    # extra state for progress along path
    s = MX.sym("s")
    # extra control for progress velocity
    ds = MX.sym("ds")

    x_new = vertcat(x, s)
    f_expl_new = vertcat(f_expl, ds)
    u_new = vertcat(u, ds)

    lut_north, lut_east = get_spline_MX(path, controller_config["n_s"])

    pos_spline = vertcat(lut_north(s), lut_east(s))
    tangent_spline = vertcat(jacobian(lut_north(s), s), jacobian(lut_east(s), s))
    tangent_spline_n = tangent_spline / norm_2(tangent_spline)
    normal_spline_n = vertcat(tangent_spline[1], -tangent_spline[0])

    error = pos - pos_spline

    # lag error
    e_l = dot(error, tangent_spline_n)
    # countouring error
    e_c = dot(error, normal_spline_n)

    ny = controller_config["ny"]
    ny_e = controller_config["ny_e"]

    rho = controller_config["rho"]
    W = np.diag(controller_config["Q"] + controller_config["R"])
    W_e = np.diag(controller_config["Q"])

    y = vertcat(-ds * rho, e_c, e_l, phi, dphi, dphi_ref)

    y_e = vertcat(e_c, e_l, phi, dphi)

    ext_cost = (y[1:].T @ W @ y[1:]) + y[0]
    ext_cost_e = y_e.T @ W_e @ y_e

    r = MX.sym("r", ny)
    r_e = MX.sym("r_e", ny_e)

    psi = (r[1:].T @ W @ r[1:]) + r[0]
    psi_e = r_e.T @ W_e @ r_e

    # ---------------------------------------
    # UPDATE MODEL

    model.x = x_new
    model.x_labels.append("s [m]")
    model.u = u_new
    model.u_labels.append(r"$v_\hat{\theta}$ [m/s]")
    model.xdot = MX.sym("xdot", 7)
    model.f_expl_expr = f_expl_new

    model.f_impl_expr = model.xdot - model.f_expl_expr

    # model.cost_y_expr = y
    # model.cost_y_expr_e = y_e

    # model.cost_r_in_psi_expr = r
    # model.cost_r_in_psi_expr_e = r_e

    # model.cost_psi_expr = psi
    # model.cost_psi_expr_e = psi_e

    model.cost_expr_ext_cost = ext_cost
    model.cost_expr_ext_cost_e = ext_cost_e

    # Constraints
    # constrain normal distance
    model.con_h_expr = e_c
    model.con_h_expr_e = e_c

    return model
