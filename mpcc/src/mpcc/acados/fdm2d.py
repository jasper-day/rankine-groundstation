from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan, atan2, fmod, if_else, norm_2, dot, fabs
import numpy as np
import scipy.linalg


def export_fdm_2d(
    b0: float,
    a0: float,
    a1: float,
    g: float = 9.81,
    #    path: Callable[[float], (float, float)]
) -> AcadosModel:
    # TODO: Add path information (estimated arclength)

    model_name = "fdm_2d"

    # states and controls
    phi = SX.sym("phi")

    dphi = SX.sym("dphi")
    xi = SX.sym("xi")
    north = SX.sym("north")
    east = SX.sym("east")
    phi_ref = SX.sym("phi_ref")
    s = SX.sym("s")
    x = vertcat(phi, dphi, xi, north, east, phi_ref, s)

    u = SX.sym("u", 1)  # dphi_ref

    # parameters (wind and airspeed)
    w_n = SX.sym("w_n")
    w_e = SX.sym("w_e")
    v_A = SX.sym("v_A")

    # path parameters
    # current path segment
    # = 0 for line segment, = 1 for circular segment
    type_0 = SX.sym("type_curr")
    # for line segment: x0, y0, x1, y1
    # for circular segment: c0, c1, +- radius
    params_0 = SX.sym("p_curr", 4)
    # arclength to switch from current to next segment
    s_switch = SX.sym("s_switch")
    # next path segment
    type_1 = SX.sym("type_next")
    # same as params0
    params_1 = SX.sym("p_next", 4)

    p = vertcat(w_n, w_e, v_A, type_0, params_0, s_switch, type_1, params_1)

    xdot = SX.sym("xdot", 7)

    dnorth = v_A * cos(xi) + w_n
    deast = v_A * sin(xi) + w_e

    # current type
    type = if_else(s > s_switch, type_1, type_0)

    params = if_else(s > s_switch, params_1, params_0)

    pos = vertcat(north, east)
    dpos = vertcat(dnorth, deast)
    line_a = vertcat(params[0], params[1])
    line_b = vertcat(params[2], 3)
    circ_c = vertcat(params[0], params[1])
    circ_xc = pos - circ_c

    ds = if_else(
        type > 0,
        # corresponds to circular segment
        params[2]
        * dot(dpos, vertcat(-circ_xc[1], circ_xc[0]))  # BR
        / norm_2(circ_xc) ** 2,
        # corresponds to line segment
        dot(dpos, line_b - line_a) / norm_2(line_b - line_a),
    )

    # explicit dynamics
    f_expl = vertcat(
        dphi,
        b0 * phi_ref - a0 * phi - a1 * dphi,  # ddphi
        g * tan(phi) / v_A,
        dnorth,
        deast,
        u[0],  # dphi_ref
        ds,
    )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    # store meta information
    model.x_labels = [
        r"$\phi$ [rad]",
        r"$\dot{\phi}$ [rad/sec]",
        r"$\xi$ [rad]",
        "north [m]",
        "east [m]",
        r"$\phi_\text{ref}$ [rad]",
        "s [m]",
    ]
    model.u_labels = [r"$\Delta \phi_\text{ref}$"]
    model.t_label = "$t$ [s]"
    model.p = p

    # costs

    ny = 6
    ny_e = 5
    y = SX.sym("y", ny)
    y_e = SX.sym("y", ny_e)

    # contouring error
    line_ba = line_b - line_a
    e_c = if_else(
        type > 0,
        # circle
        (1 - fabs(params[2]) / norm_2(circ_xc)) * circ_xc,
        # line
        (pos - line_a)
        - (line_ba)
        * dot(pos - line_a, line_ba)  # BR
        / norm_2(line_ba),
    )
    y[0] = e_c[0]
    y[1] = e_c[1]
    y[2] = phi
    y[3] = dphi
    # control slew rate
    y[4] = u
    y[5] = -ds
    y_e[0] = e_c[0]
    y_e[1] = e_c[1]
    y_e[2] = phi
    y_e[3] = dphi
    y_e[4] = -ds

    model.cost_y_expr = y
    model.cost_y_expr_e = y_e

    r = SX.sym("r", ny)
    r_e = SX.sym("r_e", ny_e)

    model.cost_r_in_psi_expr = r
    model.cost_r_in_psi_expr_e = r_e

    Q = np.diag([0.1, 1, 0.01, 0.1])
    R = np.diag([100])

    cost_W = scipy.linalg.block_diag(Q, R)

    cost_W_e = Q

    model.cost_psi_expr = 0.5 * (r[:5].T @ cost_W @ r[:5]) + r[5]
    model.cost_psi_expr_e = 0.5 * (r_e[:4].T @ cost_W_e @ r_e[:4]) + r[4]

    return model
