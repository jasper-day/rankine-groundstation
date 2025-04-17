from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from fdm2d import export_fdm_2d
from fdm2d_MX import export_fdm_2d as export_fdm_2d_MX
import numpy as np
from cc_controller import export_cc_controller
from mpcc_controller import export_mpcc_controller
from mpcc.pydubins import DubinsPath


def setup_model(
    x0: np.ndarray,
    phi_max: float,
    N_horizon: int,
    Tf: float,
    starting_params: np.ndarray,
    controller_type: str,
    controller_config: dict,
    model_config: dict,
    constraints_config: dict[str, list[float]],
    path: DubinsPath,
    RTI=True,
):
    ocp = AcadosOcp(
        acados_path="/home/jasper/rankine-groundstation/mpcc/external/acados"
    )

    if controller_type in ["pt", "empcc"]:
        print("Exporting SX model")
        fdm_model = export_fdm_2d(**model_config)
        model = export_cc_controller(
            model=fdm_model,
            controller_type=controller_type,
            controller_config=controller_config,
        )
    else:
        print("Exporting MX model")
        fdm_model = export_fdm_2d_MX(**model_config)
        model = export_mpcc_controller(
            model=fdm_model, path=path, controller_config=controller_config
        )

    ny = controller_config["ny"]
    ny_e = controller_config["ny_e"]

    if controller_type in ["pt", "empcc"]:
        ocp.cost.cost_type = "CONVEX_OVER_NONLINEAR"
        ocp.cost.cost_type_e = "CONVEX_OVER_NONLINEAR"  # terminal node

    else:
        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

    ocp.model = model

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    ocp.constraints.x0 = x0

    ocp.constraints.lbx = np.array([-phi_max])
    ocp.constraints.ubx = np.array([phi_max])
    ocp.constraints.idxbx = np.array([5])

    # cross-wise constraints (will want to be set as a "safety tunnel")
    print("Constraints", constraints_config)

    ocp.constraints.lh = np.array(constraints_config["lh"])
    ocp.constraints.uh = np.array(constraints_config["uh"])
    ocp.constraints.lh_e = np.array(constraints_config["lh_e"])
    ocp.constraints.uh_e = np.array(constraints_config["uh_e"])

    # add slacks
    ocp.constraints.idxsh = np.array([0])
    ocp.constraints.idxsh_e = np.array([0])

    Z = np.diag(constraints_config["Z"])
    z = np.array(constraints_config["z"])

    ocp.cost.Zl = Z
    ocp.cost.Zu = Z
    ocp.cost.Zl_e = Z
    ocp.cost.Zu_e = Z
    ocp.cost.zl = z
    ocp.cost.zu = z
    ocp.cost.zl_e = z
    ocp.cost.zu_e = z

    ocp.parameter_values = starting_params

    # set prediction horizon
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = Tf

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = (
        "EXACT" if controller_type == "mpcc" else "GAUSS_NEWTON"
    )
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

    return ocp, acados_ocp_solver, acados_integrator
