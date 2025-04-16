"""Constant-curvature path tracking controllers for fixed-wing UAS

This file defines constant curvature path tracking controllers as described in
_Jasper Day, Exact MPCC for Fixed Wing Flight (2025)_

The path tracking controller (`export_cc_controller(controller: 'pt')`)
minimizes the distance to the path and the difference between the ground heading
$chi$ and the path heading $chi_d$.

The Exact MPCC controller (`export_cc_controller(controller: 'empcc')`)
minimizes the distance to the path and maximizes the progress along the path.

Both controllers require the parameters for the current and next paths.
These parameters are set in the controller. The parameters take the form

Note that x corresponds to North and y corresponds to East.

[
# 0 for a line segment, 1 for a circular segment
type_0: 0 | 1,
# For a line segment, x0, y0, x1, y1
# For a circular segment, c_x, c_y, radius
params_0: float[4],
# arclength at which the controller should switch
# between path segment types
s_switch: float,
# Same as type_0, type of the next segment
type_1: 0 | 1,
# Same as params_0, parameters of the next segment
params_1: float[4],
]
"""

from acados_template import AcadosModel
from typing import Literal
from casadi import (
    SX,
    vertcat,
    if_else,
    dot,
    norm_2,
    fabs,
    atan2,
    fmod,
    remainder,
    sin,
    cos,
)
import numpy as np
import scipy.linalg


def export_cc_controller(
    model: AcadosModel,
    controller_type: Literal["pt", "empcc"],
    Q: np.ndarray,
    R: np.ndarray,
    rho: float | None = None,
) -> AcadosModel:
    x = model.x
    u = model.u
    f_expl = model.f_expl_expr

    north = x[0]
    east = x[1]
    xi = x[2]
    phi = x[3]
    dphi = x[4]
    phi_ref = x[5]

    pos = vertcat(north, east)

    dnorth = f_expl[0]
    deast = f_expl[1]
    dpos = vertcat(dnorth, deast)

    # extra state for distance along path
    s = SX.sym("s")

    # path parameters
    # current path segment
    # = 0 for line segment, = 1 for circular segment
    type_0 = SX.sym("type_curr")
    # for line segment: x0, y0, x1, y1
    # for circular segment: c_x, c_y, +- radius, unused
    params_0 = SX.sym("p_curr", 4)
    # arclength to switch from current to next segment
    s_switch = SX.sym("s_switch")
    # next path segment
    type_1 = SX.sym("type_next")
    # same as params0
    params_1 = SX.sym("p_next", 4)

    # to be appended to model
    extra_params = vertcat(type_0, params_0, s_switch, type_1, params_1)

    # used in calculations
    active_type = if_else(s > s_switch, type_1, type_0)

    active_params = if_else(s > s_switch, params_1, params_0)

    line_a = vertcat(active_params[0], active_params[1])
    line_b = vertcat(active_params[2], active_params[3])
    circ_c = vertcat(active_params[0], active_params[1])
    circ_xc = pos - circ_c
    circ_r = active_params[2]
    line_ba = line_b - line_a
    line_ba_n = line_ba / norm_2(line_ba)
    line_posa = pos - line_a
    x1 = active_params[0]
    y1 = active_params[1]
    x2 = active_params[2]
    y2 = active_params[3]

    # arclength progress
    ds = if_else(
        active_type > 0.5,
        # corresponds to circular segment
        active_params[2]
        * dot(dpos, vertcat(-circ_xc[1], circ_xc[0]))  # BR
        / norm_2(circ_xc) ** 2,
        # corresponds to line segment
        dot(dpos, line_ba_n),
    )

    ny = 5
    ny_e = 4
    y = SX.sym("y", ny)
    y_e = SX.sym("y", ny_e)

    # contouring error

    e_c = if_else(
        active_type > 0.5,
        # circle
        # norm_2((1 - fabs(active_params[2]) / norm_2(circ_xc)) * circ_xc),
        # equal to
        norm_2(circ_xc) - fabs(circ_r),
        # line
        # norm_2(line_posa - line_ba_n * dot(line_ba_n, line_posa)),
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        # seems better able to cope with numerical sensitivity
        ((y2 - y1) * north - (x2 - x1) * east + x2 * y1 - y2 * x1) / norm_2(line_ba),
    )

    # heading
    chi = atan2(dpos[1], dpos[0])

    # path heading
    chi_d = if_else(
        active_type > 0.5,
        # circle
        if_else(
            circ_r > 0,
            atan2(circ_xc[0], -circ_xc[1]),
            atan2(-circ_xc[0], circ_xc[1]),
        ),
        # line
        atan2(line_ba[1], line_ba[0]),
    )
    # wrap to between +- np.pi
    # e_chi = (sin(chi) - sin(chi_d)) ** 2 + (cos(chi) - cos(chi_d)) ** 2
    # e_chi_normalized = remainder(e_chi, 2 * np.pi)
    # e_chi = chi - chi_d

    # Compute heading error directly in sine/cosine space
    # e_chi = (chi_cos - chi_d_cos) ** 2 + (chi_sin - chi_d_sin) ** 2
    e_chi = remainder(chi - chi_d, 2 * np.pi)

    # lagrange cost measurements
    y[0] = e_chi if controller_type == "pt" else -ds * rho
    y[1] = e_c
    y[2] = phi
    y[3] = dphi
    y[4] = u  # control slew rate
    # mayer cost measurements
    # y_e[0] does not get used at all for EMPCC
    y_e[0] = e_chi if controller_type == "pt" else 0
    y_e[1] = e_c
    y_e[2] = phi
    y_e[3] = dphi

    # ---------------------------------------
    # UPDATE MODEL

    model.x = vertcat(model.x, s)
    model.x_labels.append("s [m]")
    model.xdot = SX.sym("xdot", 7)
    model.f_expl_expr = vertcat(f_expl, ds)

    model.f_impl_expr = model.xdot - model.f_expl_expr

    model.p = vertcat(model.p, extra_params)

    model.cost_y_expr = y
    model.cost_y_expr_e = y_e

    r = SX.sym("r", ny)
    r_e = SX.sym("r_e", ny_e)

    model.cost_r_in_psi_expr = r
    model.cost_r_in_psi_expr_e = r_e

    cost_W = scipy.linalg.block_diag(Q, R)

    cost_W_e = Q

    if controller_type == "pt":
        print("Cost function for Path Tracking controller")
        model.cost_psi_expr = 0.5 * (r.T @ cost_W @ r)
        model.cost_psi_expr_e = 0.5 * (r_e.T @ cost_W_e @ r_e)
    elif controller_type == "empcc":
        print("Cost function for EMPCC controller")
        model.cost_psi_expr = 0.5 * (r[1:].T @ cost_W @ r[1:]) + r[0]
        model.cost_psi_expr_e = 0.5 * (r_e[1:].T @ cost_W_e @ r_e[1:])

    # Constraints
    # constrain normal distance
    model.con_h_expr = e_c
    model.con_h_expr_e = e_c

    return model
