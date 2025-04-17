"""
Numeric version of the equations of motion (fdm_2d and cc_controller)
used to check the results from CasADi.
"""

import numpy as np
from numpy import sin, cos, tan, dot, atan2
from math import remainder
from numpy.linalg import norm as norm_2
from mpcc.serialization_schema import path_adapter, to_dubins
import matplotlib.pyplot as plt
import typer


def calc(x, u, p, g: float, b0: float, a0: float, a1: float):
    """State derivative and costs.

    x = [n, e, xi, phi, phidot, phi_ref, s]
    u = [dphi_ref]
    p = [w_n, w_e, v_A, t0, *p0[:4], s_sw, t1, *p1[:4]]

    """

    north, east, xi, phi, dphi, phi_ref, s = x
    w_n, w_e, v_A, t0, p00, p01, p02, p03, s_switch, t1, p10, p11, p12, p13 = p
    p0 = np.array([p00, p01, p02, p03])
    p1 = np.array([p10, p11, p12, p13])

    active_type = t1 if s > s_switch else t0
    active_params = p1 if s > s_switch else p0

    pos = np.array([north, east])

    line_a = np.array([active_params[0], active_params[1]])
    line_b = np.array([active_params[2], active_params[3]])
    circ_c = np.array([active_params[0], active_params[1]])
    circ_xc = pos - circ_c
    circ_r = active_params[2]
    circ_dir = active_params[3]
    line_ba = line_b - line_a
    line_T_n = line_ba / norm_2(line_ba)
    line_N_n = np.array([-line_T_n[1], line_T_n[0]])
    line_posa = pos - line_a

    dnorth = v_A * cos(xi) + w_n
    deast = v_A * sin(xi) + w_e

    dpos = np.array([dnorth, deast])

    ds = (
        active_params[2]
        * dot(dpos, np.array([-circ_xc[1], circ_xc[0]]))
        / norm_2(circ_xc) ** 2
        if active_type > 0.5
        else dot(dpos, line_T_n)
    )

    e_c = (  # contouring error
        # circle
        circ_dir * (norm_2(circ_xc) - circ_r)
        if active_type > 0.5
        # line
        else dot(line_posa, line_N_n)
    )

    xdot = np.array(
        [
            dnorth,
            deast,
            g * tan(phi) / v_A,
            dphi,
            b0 * phi_ref - a0 * phi - a1 * dphi,  # ddphi
            u[0],  # dphi_ref
            ds,
        ]
    )

    # heading
    chi = atan2(dpos[1], dpos[0])

    # path heading
    chi_d = (
        # circle
        (
            atan2(circ_xc[0], -circ_xc[1])
            if circ_dir > 0
            else atan2(-circ_xc[0], circ_xc[1])
        )
        if active_type > 0.5
        # line
        else atan2(line_ba[1], line_ba[0])
    )

    e_chi = remainder(chi - chi_d, 2 * np.pi)

    return xdot, e_c, e_chi, chi, chi_d


def main(filename: str, path: str):
    from recover_data import load_data

    data = load_data(filename)

    x = data["x"]
    u = data["u"]
    params = data["params"]
    costs = data["costs"]
    t = data["t"]
    t = t[:-1]

    Nsim = x.shape[0] - 1
    xdot, errors, angles = np.zeros((Nsim, 7)), np.zeros((Nsim, 2)), np.zeros((Nsim, 2))
    omega = 3.4  # natural frequency of roll transfer function
    zeta = 0.8  # damping ratio of roll transfer function
    b0 = omega**2
    a0 = omega**2
    a1 = 2 * zeta * omega
    for i in range(Nsim):
        xdoti, e_c, e_chi, chi, chi_d = calc(
            x[i, :], u[i], params[i, :], 9.81, b0=b0, a0=a0, a1=a1
        )
        xdot[i, :] = xdoti
        errors[i, 0] = e_c
        errors[i, 1] = e_chi
        angles[i, 0] = chi
        angles[i, 1] = chi_d

    ds = xdot[:, -1]
    dnorth = xdot[:, 0]
    deast = xdot[:, 1]
    s = x[:, -1]
    north = x[:, 0]
    east = x[:, 1]

    # load test path
    with open(path, "r") as f:
        path_pyobj = path_adapter.validate_json(f.read())
        dubins_path, _ = to_dubins(path_pyobj)

    s_actual = np.array(
        [
            dubins_path.get_true_arclength(np.array([north_i, east_i]))
            for north_i, east_i in zip(north, east)
        ]
    )

    length = dubins_path.length()
    ts = np.linspace(0, length, 500)
    locs = np.array([dubins_path.eval(t) for t in ts])
    path_locs = np.array([dubins_path.eval(s_i) for s_i in s])
    offset_path = dubins_path.offset_path(20)
    other_offset_path = dubins_path.offset_path(-20)
    offset_locs = np.array([offset_path.eval(t) for t in ts])
    other_offset_locs = np.array([other_offset_path.eval(t) for t in ts])

    derivs = np.diff(path_locs)
    derivs = derivs / norm_2(derivs, axis=1)
    # plt.subplot(211)
    plt.gca().set_aspect("equal")
    plt.plot(locs[:, 0], locs[:, 1], linestyle="--", label="Reference", color="gray")
    plt.plot(offset_locs[:, 0], offset_locs[:, 1], linestyle=":", color="gray")
    plt.plot(
        other_offset_locs[:, 0], other_offset_locs[:, 1], linestyle=":", color="gray"
    )

    # plt.plot(path_locs[:, 0], path_locs[:, 1])
    plt.plot(x[:, 0], x[:, 1], label="Travelled")

    # scalewidth = dict(scale=30, width=0.003)
    # plt.quiver(
    #     x[:-1:10, 0],
    #     x[:-1:10, 1],
    #     np.cos(angles[:-1:10, 0]),
    #     np.sin(angles[:-1:10, 0]),
    #     color="orange",
    #     **scalewidth,
    # )
    # plt.quiver(
    #     x[:-1:10, 0],
    #     x[:-1:10, 1],
    #     np.cos(angles[:-1:10, 1]),
    #     np.sin(angles[:-1:10, 1]),
    #     color="red",
    #     **scalewidth,
    # )
    plt.xlabel("North")
    plt.ylabel("East")
    plt.show()
    plt.subplot(211)
    plt.plot(t, ds, label="ds")
    plt.plot(t, params[:-1, 3], label="s_switch")
    plt.legend()

    plt.subplot(212)
    plt.plot(t, errors[:, 0], label="e_c")
    plt.axhline(-20, linestyle=":", color="gray")
    plt.axhline(20, linestyle=":", color="gray")
    # plt.plot(t, errors[:, 1], label="e_chi")
    # # plt.plot(t, s[:-1], label="s")
    # # plt.plot(t, s_actual[:-1], label="s_actual")
    plt.grid()
    plt.legend()
    plt.show()


if __name__ == "__main__":
    typer.run(main)
