from acados_template import AcadosOcpSolver, AcadosSimSolver, AcadosOcp
from mpcc.pydubins import DubinsPath
from mpcc.path_utils import get_params
from mpcc.controller.mpcc_controller import export_mpcc_controller
from mpcc.controller.cc_controllers import export_cc_controller
from mpcc.controller.fdm2d import export_fdm_2d
from mpcc.controller.fdm2d_MX import export_fdm_2d_MX
from mpcc.controller.setup_ocp import add_constraints
from immutabledict import immutabledict as idict
import numpy as np
from dataclasses import dataclass


@dataclass
class SolverResult:
    preparation_status: int
    feedback_status: int
    t_preparation: float
    t_feedback: float
    cost: float
    simX: np.ndarray | None
    solX: np.ndarray
    solU: np.ndarray


class MPCCController:
    path: DubinsPath | None = None
    config: idict  # immutable
    ocp: AcadosOcp | None = None
    ocp_solver: AcadosOcpSolver | None = None
    ocp_integrator: AcadosSimSolver | None = None
    x_names = ["north", "east", "xi", "phi", "dphi", "phi_ref", "s"]
    u_names = ["dphi_ref"]
    preparation_status: int
    t_preparation: float
    ready: bool = False

    def __init__(self, config: dict[str, any]):
        self.config = idict(config)

    def setup_ocp(self):
        controller_type = self.config["controller"]
        ocp = AcadosOcp()
        model_config = dict(
            b0=self.config["model"]["b0"],
            a0=self.config["model"]["a0"],
            a1=self.config["model"]["a1"],
            g=self.config["model"]["g"],
        )
        # specific configuration for chosen controller (pt, mpcc, empcc)
        controller_config = self.config["controllers"][controller_type]
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
                model=fdm_model, path=self.path, controller_config=controller_config
            )
        ny = controller_config["ny"]
        ny_e = controller_config["ny_e"]
        ocp.model = model

        if controller_type in ["pt", "empcc"]:
            ocp.cost.cost_type = "CONVEX_OVER_NONLINEAR"
            ocp.cost.cost_type_e = "CONVEX_OVER_NONLINEAR"  # terminal node

        else:
            ocp.cost.cost_type = "EXTERNAL"
            ocp.cost.cost_type_e = "EXTERNAL"

        ocp.cost.yref = np.zeros((ny,))
        ocp.cost.yref_e = np.zeros((ny_e,))
        phi_max = np.radians(self.config["constraints"]["state"]["phi_max_deg"])
        ocp.constraints.lbx = np.array([-phi_max])
        ocp.constraints.ubx = np.array([phi_max])
        ocp.constraints.idxbx = np.array([5])

        north0, east0 = self.path.eval(0)
        xi0 = self.path.get_segment(0).heading_start()
        x0 = np.array([north0, east0, xi0, 0, 0, 0, 0])

        ocp.constraints.x0 = x0

        add_constraints(ocp, self.config["constraints"]["path"], controller_type)

        starting_params = self.curr_params(
            0,
            np.array(self.config["model"]["defaults"]["wind"]),
            self.config["model"]["defaults"]["airspeed"],
        )
        ocp.parameter_values = starting_params

        # prediction horizon
        N_horizon = self.config["N_horizon"]
        ocp.solver_options.N_horizon = N_horizon
        tf = (N_horizon + 1) * self.config["dt"]
        ocp.solver_options.tf = tf

        # TODO: Move these to config.toml
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.hessian_approx = (
            "EXACT" if controller_type == "mpcc" else "GAUSS_NEWTON"
        )
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.sim_method_newton_iter = 10

        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.as_rti_iter = 1
        # AS-RTI-D
        ocp.solver_options.as_rti_level = 3

        ocp.solver_options.qp_solver_cond_N = N_horizon

        name = model.name
        assert name is not None
        solver_json = "acados_ocp_" + name + ".json"
        self.ocp_solver = AcadosOcpSolver(ocp, json_file=solver_json)

        # create an integrator with the same settings as used in the OCP solver.
        if self.config["simulator"]:
            self.ocp_integrator = AcadosSimSolver(ocp, json_file=solver_json)
        else:
            self.ocp_integrator = None

        self.preparation_status = self.prepare()
        # end setup

    def set_path(self, path: DubinsPath):
        "Change the path to follow"
        # will need to recompile for MPCC controller
        self.ready = False
        self.path = path
        self.destroy()
        self.setup_ocp()
        self.ready = True

    def update(
        self, arclength: float, wind: np.ndarray, airspeed: float, X: np.ndarray
    ):
        "Run one iteration of solver."
        N_horizon = self.config["N_horizon"]

        path_parameters = self.curr_params(arclength, wind, airspeed)
        for stage in range(N_horizon + 1):
            self.ocp_solver.set(stage, "p", path_parameters)

        # set initial state
        self.ocp_solver.set(0, "lbx", X)
        self.ocp_solver.set(0, "ubx", X)

        # feedback phase
        self.ocp_solver.options_set("rti_phase", 2)
        feedback_status = self.ocp_solver.solve()
        t_feedback = self.ocp_solver.get_stats("time_tot")

        solX = self.ocp_solver.get_flat("x")
        solU = self.ocp_solver.get_flat("u")
        cost = self.ocp_solver.get_cost()

        if self.ocp_integrator is not None:
            self.ocp_integrator.set("p", path_parameters)
            simX = self.ocp_integrator.simulate(x=X, u=solU)
        else:
            simX = None

        return SolverResult(
            preparation_status=self.preparation_status,
            feedback_status=feedback_status,
            t_preparation=self.t_preparation,
            t_feedback=t_feedback,
            cost=cost,
            simX=simX,
            solX=solX,
            solU=solU,
        )

    def prepare(self):
        "Prepare for feedback"
        # preparation phase
        self.ocp_solver.options_set("rti_phase", 1)
        self.preparation_status = self.ocp_solver.solve()
        self.t_preparation = self.ocp_solver.get_stats("time_tot")


    def destroy(self):
        "Release memory held by the controller"
        if self.ocp is not None:
            del self.ocp
        if self.ocp_solver is not None:
            del self.ocp_solver
        if self.ocp_integrator is not None:
            del self.ocp_integrator

    def curr_params(self, arclength: float, wind: np.ndarray, airspeed: float):
        "Get the current & upcoming path parameters"
        if self.config["controller"] in ["pt", "mpcc"]:
            return get_params(wind, airspeed, self.path, arclength)
        else:
            return np.array([*wind, airspeed])