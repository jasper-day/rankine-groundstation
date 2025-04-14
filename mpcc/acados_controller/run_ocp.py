from acados_ocp import setup_model
import numpy as np
from time import perf_counter
from mpcc.pydubins import DubinsPath
from mpcc.acados_parameters import get_acados_path_parameters


def run_ocp(x0, Tf, N_horizon, phi_max, Nsim, path: DubinsPath, use_RTI=True):
    ocp_solver, integrator = setup_model(x0, phi_max, N_horizon, Tf, use_RTI)

    nx = ocp_solver.acados_ocp.dims.nx
    nu = ocp_solver.acados_ocp.dims.nu

    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))

    simX[0, :] = x0

    if use_RTI:
        t_preparation = np.zeros((Nsim))
        t_feedback = np.zeros((Nsim))

    else:
        t = np.zeros((Nsim))

    # closed loop
    t0 = perf_counter()
    for i in range(Nsim):
        if use_RTI:
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

            # set parameters
            current_s = simX[i, -1]
            try:
                path_parameters = get_acados_path_parameters(path, current_s)
            except AssertionError:
                break
            for stage in range(N_horizon):
                ocp_solver.set_params_sparse(stage, np.arange(3, 14), path_parameters)

            # feedback phase
            # print("feedback")
            ocp_solver.options_set("rti_phase", 2)
            status = ocp_solver.solve()
            t_feedback[i] = ocp_solver.get_stats("time_tot")

            simU[i, :] = ocp_solver.get(0, "u")

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
    return simU, simX, model
