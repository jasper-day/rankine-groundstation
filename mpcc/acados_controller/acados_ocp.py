from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from fdm2d import export_fdm_2d
import numpy as np
from cc_controller import export_cc_controller

Q = np.diag(
    [
        # 1,  # e_chi
        0.1,  # e_c
        0.01,  # phi
        0.1,  # dphi
    ]
)
R = np.diag(
    [
        100  # u
    ]
)


def setup_model(x0, phi_max, N_horizon, Tf, RTI=True):
    ocp = AcadosOcp(
        acados_path="/home/jasper/rankine-groundstation/mpcc/external/acados"
    )

    omega = 3.4  # natural frequency of roll transfer function
    zeta = 0.8  # damping ratio of roll transfer function
    fdm_model = export_fdm_2d(b0=omega**2, a0=omega**2, a1=2 * zeta * omega)
    # path tracking controller for now
    model = export_cc_controller(fdm_model, "empcc", Q=Q, R=R, rho=10)
    # nx = model.x.rows()
    # nu = model.u.rows()
    # num_p = model.p.rows()
    ny = 5
    ny_e = 4

    ocp.cost.cost_type = "CONVEX_OVER_NONLINEAR"
    ocp.cost.cost_type_e = "CONVEX_OVER_NONLINEAR"  # terminal node

    ocp.model = model

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # TODO: set constraints

    ocp.constraints.x0 = x0

    ocp.constraints.lbx = np.array([-phi_max])
    ocp.constraints.ubx = np.array([phi_max])
    ocp.constraints.idxbx = np.array([5])

    # TODO: set parameter values automatically from Dubins path
    ocp.parameter_values = np.array(
        [
            0,  # w_n
            0,  # w_e
            14,  # v_A
            1,  # circular segment
            0,  # c_n
            0,  # c_e
            -60,  # radius
            0,  # unused
            50 * 2 * np.pi,  # s_sw, go around circle once
            1,  # circular segment
            0,  # c_n
            0,  # c_e
            60,  # go in the other direction
            0,  # unused
        ]
    )

    # set prediction horizon
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = Tf

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.sim_method_newton_iter = 10

    if RTI:
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.as_rti_iter = 1
        # AS-RTI-D
        ocp.solver_options.as_rti_level = 3
    else:
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.globalization = (
            "MERIT_BACKTRACKING"  # turns on globalization
        )
        ocp.solver_options.nlp_solver_max_iter = 150

    ocp.solver_options.qp_solver_cond_N = N_horizon

    solver_json = "acados_ocp_" + model.name + ".json"
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

    # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file=solver_json)

    return acados_ocp_solver, acados_integrator
