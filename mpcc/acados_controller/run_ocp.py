import numpy as np
from time import perf_counter
from acados_template import AcadosOcpSolver, AcadosSimSolver, AcadosOcp
from dryden_turbulence import TurbulenceSimulator
from typing import Callable

def run_ocp(
    ocp: AcadosOcp,
    ocp_solver: AcadosOcpSolver,
    integrator: AcadosSimSolver,
    Nsim: int,
    param_fn,
    rti_config: dict,
    tsim: TurbulenceSimulator | None = None,
    gust: Callable | None = None,
):
    print("starting_params", param_fn(0))

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu
    N_horizon = ocp.solver_options.N_horizon
    tf = ocp.solver_options.tf
    if N_horizon is None or tf is None:
        raise Exception("Set N_horizon and tf in ocp.solver_options")
    assert nx is not None
    assert nu is not None
    dt = tf / (N_horizon + 1)

    starting_params = param_fn(0)
    n_p = starting_params.shape[0]

    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))
    costs = np.zeros((Nsim, 1))
    params = np.zeros((Nsim + 1, n_p))

    solX = np.zeros((Nsim + 1, nx, N_horizon))

    simX[0, :] = ocp.constraints.x0
    params[0, :] = param_fn(0)

    t_preparation = np.zeros((Nsim))
    t_feedback = np.zeros((Nsim))

    # closed loop
    t0 = perf_counter()
    for i in range(Nsim):
        # set parameters
        current_s = simX[i, -1]
        _current_north = simX[i, 0]
        _current_east = simX[i, 1]
        try:
            path_parameters = param_fn(current_s)
            # simulator and controller both know about gust
            if gust is not None:
                t = dt * i
                path_parameters[:2] += gust(t)
            if i % 10 == 0 and rti_config["verbose"]:
                print("current s", current_s)
                print("path_parameters", path_parameters)
        except AssertionError:
            print("Hit end of path")
            break
        # set parameters in solver
        for stage in range(N_horizon + 1):
            ocp_solver.set(stage, "p", path_parameters)
        # set parameters in simulator (!! very important !!)
        if tsim is not None:
            # append turbulence only in simulation
            tsim.advance(dt)
            xi = simX[i, 2]
            turbulence = tsim.get_wind_vector_2(xi)
            path_parameters[:2] += turbulence

        integrator.set("p", path_parameters)

        # preparation phase
        # print("prepare")
        ocp_solver.options_set("rti_phase", 1)
        status = ocp_solver.solve()
        t_preparation[i] = ocp_solver.get_stats("time_tot")

        if status not in [0, 2, 5] and rti_config["halt_on_error"]:
            raise Exception(f"acados returned status {status}. Exiting.")

        # set initial state
        ocp_solver.set(0, "lbx", simX[i, :])
        ocp_solver.set(0, "ubx", simX[i, :])

        # feedback phase
        # print("feedback")
        ocp_solver.options_set("rti_phase", 2)
        status = ocp_solver.solve()
        t_feedback[i] = ocp_solver.get_stats("time_tot")

        simU[i, :] = ocp_solver.get(0, "u")
        costs[i] = ocp_solver.get_cost()

        # simulate system
        simX[i + 1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :])
    t1 = perf_counter()
    print("Evaluation time", t1 - t0)

    # scale to milliseconds
    t_preparation *= 1000
    t_preparation = np.maximum(t_preparation, 1e-12)
    t_feedback *= 1000
    t_feedback = np.maximum(t_feedback, 1e-12)
    print(
        f"Computation time in preparation phase in ms: \
            min {np.min(t_preparation):.3f} median {np.median(t_preparation):.3f} max {np.max(t_preparation):.3f}"
    )
    print(
        f"Computation time in feedback phase in ms:    \
            min {np.min(t_feedback):.3f} median {np.median(t_feedback):.3f} max {np.max(t_feedback):.3f}"
    )

    del ocp_solver
    return simU, simX, costs, params, t_preparation, t_feedback
