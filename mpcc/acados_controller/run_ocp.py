from acados_ocp import setup_model
import numpy as np
from time import perf_counter
from mpcc.pydubins import DubinsPath
from mpcc.acados_parameters import get_acados_path_parameters, find_correct_s


def run_ocp(
    x0: np.ndarray,
    Tf: float,
    N_horizon: int,
    phi_max: float,
    Nsim: int,
    Q: np.ndarray,
    R: np.ndarray,
    rho: float,
    wind: np.ndarray,
    v_A: float,
    path: DubinsPath,
    use_RTI=True,
    controller_type="pt",
):
    starting_params = np.zeros(14)
    starting_params[:2] = wind
    starting_params[2] = v_A
    starting_params[3:] = get_acados_path_parameters(path, 0)
    print("starting_params", starting_params)

    ocp_solver, integrator = setup_model(
        x0, phi_max, N_horizon, Tf, starting_params, Q, R, rho, use_RTI, controller_type
    )

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))
    costs = np.zeros((Nsim, 1))
    params = np.zeros((Nsim + 1, 14))

    simX[0, :] = x0
    params[0, :] = starting_params

    if use_RTI:
        t_preparation = np.zeros((Nsim))
        t_feedback = np.zeros((Nsim))

    else:
        t = np.zeros((Nsim))

    # closed loop
    t0 = perf_counter()
    for i in range(Nsim):
        if use_RTI:
            # set parameters
            current_s = simX[i, -1]
            _current_north = simX[i, 0]
            _current_east = simX[i, 1]
            try:
                path_parameters = get_acados_path_parameters(path, current_s)
                if i % 10 == 0:
                    print("current s", current_s)
                    print("path_parameters", path_parameters)
            except AssertionError:
                print("Hit end of path")
                break
            # set parameters in solver
            for stage in range(N_horizon + 1):
                ocp_solver.set_params_sparse(stage, np.arange(3, 14), path_parameters)
            # set parameters in simulator (!! very important !!)
            new_params = np.zeros_like(starting_params)
            new_params[:3] = starting_params[:3]
            new_params[3:] = path_parameters
            params[i, :] = new_params
            integrator.set("p", new_params)

            # preparation phase
            # print("prepare")
            ocp_solver.options_set("rti_phase", 1)
            status = ocp_solver.solve()
            t_preparation[i] = ocp_solver.get_stats("time_tot")

            # if status not in [0, 2, 5]:
            #     raise Exception(f'acados returned status {status}. Exiting.')

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

        else:
            # solve ocp and get next control input
            simU[i, :] = ocp_solver.solve_for_x0(x0_bar=simX[i, :])

            t[i] = ocp_solver.get_stats("time_tot")

        # simulate system
        simX[i + 1, :] = integrator.simulate(x=simX[i, :], u=simU[i, :])
    t1 = perf_counter()
    print("Evaluation time", t1 - t0)

    # evaluate timings
    if use_RTI:
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
    else:
        # scale to milliseconds
        t *= 1000
        print(
            f"Computation time in ms: min {np.min(t):.3f} median {np.median(t):.3f} max {np.max(t):.3f}"
        )

    # plot results
    model = ocp_solver.acados_ocp.model
    # plot_pendulum(np.linspace(0, (Tf/N_horizon)*Nsim, Nsim+1), Fmax, simU, simX, latexify=False, time_label=model.t_label, x_labels=model.x_labels, u_labels=model.u_labels)

    ocp_solver = None
    return simU, simX, model, costs, params
