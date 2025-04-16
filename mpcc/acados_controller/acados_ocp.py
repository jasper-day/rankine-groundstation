from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from fdm2d import export_fdm_2d
import numpy as np
from cc_controller import export_cc_controller


def setup_model(
    x0,
    phi_max,
    N_horizon,
    Tf,
    starting_params,
    Q,
    R,
    rho,
    RTI=True,
    controller_type="pt",
):
    ocp = AcadosOcp(
        acados_path="/home/jasper/rankine-groundstation/mpcc/external/acados"
    )

    omega = 3.4  # natural frequency of roll transfer function
    zeta = 0.8  # damping ratio of roll transfer function
    fdm_model = export_fdm_2d(b0=omega**2, a0=omega**2, a1=2 * zeta * omega)
    # path tracking controller for now
    model = export_cc_controller(
        model=fdm_model, controller_type=controller_type, Q=Q, R=R, rho=rho
    )
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

    ocp.constraints.x0 = x0

    ocp.constraints.lbx = np.array([-phi_max])
    ocp.constraints.ubx = np.array([phi_max])
    ocp.constraints.idxbx = np.array([5])

    # cross-wise constraints (will want to be set as a "safety tunnel")
    ocp.constraints.lh = np.array([-20])
    ocp.constraints.uh = np.array([20])
    ocp.constraints.lh_e = np.array([-20])
    ocp.constraints.uh_e = np.array([20])

    # add slacks
    ocp.constraints.idxsh = np.array([0])
    ocp.constraints.idxsh_e = np.array([0])

    Z = np.diag([100])
    z = np.array([0])

    ocp.cost.Zl = Z
    ocp.cost.Zu = Z
    ocp.cost.Zl_e = Z
    ocp.cost.Zu_e = Z
    ocp.cost.zl = z
    ocp.cost.zu = z
    ocp.cost.zl_e = z
    ocp.cost.zu_e = z

    # TODO: set parameter values automatically from Dubins path
    ocp.parameter_values = starting_params

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
