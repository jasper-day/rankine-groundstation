from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan


def export_fdm_2d(
    b0: float,
    a0: float,
    a1: float,
    g: float = 9.81,
) -> AcadosModel:
    """
    The base flight dynamic model, without measurements, extra parameters, or controls. For details see

    Thomas J. Stastny, Adyasha Dash, and Roland Siegwart, “Nonlinear MPC for Fixed-Wing UAV Trajectory Tracking: Implementation and Flight Experiments,” in AIAA Guidance, Navigation, and Control Conference (AIAA Guidance, Navigation, and Control Conference, Grapevine, Texas: American Institute of Aeronautics and Astronautics, 2017), https://doi.org/10.2514/6.2017-1512.

    or

    Jasper Day, "Exact MPCC for Fixed-Wing Flight"
    """

    model_name = "fdm_2d"

    # states and controls
    north = SX.sym("north")
    east = SX.sym("east")
    xi = SX.sym("xi")
    phi = SX.sym("phi")
    dphi = SX.sym("dphi")
    phi_ref = SX.sym("phi_ref")

    x = vertcat(north, east, xi, phi, dphi, phi_ref)

    u = SX.sym("u", 1)  # dphi_ref

    # parameters (wind and airspeed)
    w_n = SX.sym("w_n")
    w_e = SX.sym("w_e")
    v_A = SX.sym("v_A")

    p = vertcat(w_n, w_e, v_A)

    xdot = SX.sym("xdot", 6)

    dnorth = v_A * cos(xi) + w_n
    deast = v_A * sin(xi) + w_e

    f_expl = vertcat(
        dnorth,
        deast,
        g * tan(phi) / v_A,
        dphi,
        b0 * phi_ref - a0 * phi - a1 * dphi,  # ddphi
        u[0],  # dphi_ref
    )

    # explicit dynamics

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
        "north [m]",
        "east [m]",
        r"$\xi$ [rad]",
        r"$\phi$ [rad]",
        r"$\dot{\phi}$ [rad/sec]",
        r"$\phi_\text{ref}$ [rad]",
    ]
    model.u_labels = [r"$\Delta \phi_\text{ref}$"]
    model.t_label = "$t$ [s]"
    model.p = p

    return model
