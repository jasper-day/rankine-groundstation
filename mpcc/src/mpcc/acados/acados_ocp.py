from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from fdm2d import export_fdm_2d
from plot_fdm import plot_fdm, animate_fdm
import numpy as np
import matplotlib.pyplot as plt
import pickle
from pathlib import Path
import typer
from time import perf_counter
from cc_controller import export_cc_controller

Q = np.diag([0.1, 1, 0.01, 0.1])
R = np.diag([100])


def setup_model(x0, phi_max, N_horizon, Tf, RTI=True):
    ocp = AcadosOcp()

    omega = 3.4  # natural frequency of roll transfer function
    zeta = 0.8  # damping ratio of roll transfer function
    fdm_model = export_fdm_2d(b0=omega**2, a0=omega**2, a1=2 * zeta * omega)
    # path tracking controller for now
    model = export_cc_controller(fdm_model, "pt", Q=Q, R=R)
    nx = model.x.rows()
    nu = model.u.rows()
    num_p = model.p.rows()
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
            -8,  # w_e
            14,  # v_A
            1,  # circular segment
            0,  # c_n
            0,  # c_e
            -50,  # radius
            0,  # unused
            50 * 2 * np.pi,  # s_sw, go around circle once
            1,  # circular segment
            0,  # c_n
            0,  # c_e
            50,  # go in the other direction
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


def run_ocp(x0, Tf, N_horizon, phi_max, Nsim, use_RTI=True):
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

    # # do some initial iterations to start with a good initial guess
    # num_iter_initial = 5
    # for _ in range(num_iter_initial):
    #     ocp_solver.solve_for_x0(x0_bar = x0)

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
        t_feedback *= 1000
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


def main(
    read: bool = False,
    write: bool = True,
    animate: bool = False,
    plot: bool = False,
    Tf: float = 4.1,
    N_horizon: int = 40,
    phi_max_deg: int = 30,
    Nsim: int = 4000,
    extra_name: str = "",
):
    # phi, dphi, xi, north, east, phi_ref, s
    x0 = np.array([0.0, 0.0, 0.0, 50.0, 100.0, 0.0, 0.0])
    phi_max = np.radians(phi_max_deg)

    name = (
        f"fdm_2d_Tf{Tf:.2f}_Nh{N_horizon}_Nsim{Nsim}_phimax{phi_max_deg}_{extra_name}"
    )
    pklfile = Path(f"pickled_data/{name}.pkl")

    if not read:
        simU, simX, model = run_ocp(x0, Tf, N_horizon, phi_max, Nsim)

        t = np.linspace(0, (Tf / N_horizon) * Nsim, Nsim + 1)
        data = {
            "t": t,
            "u": simU,
            "x": simX,
            "u_labels": model.u_labels,
            "x_labels": model.x_labels,
        }

        if write:
            print("Writing to", pklfile)
            with open(pklfile, "wb") as f:
                pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

    if read:
        print("Reading from", pklfile)
        assert pklfile.is_file()
        with open(pklfile, "rb") as f:
            data = pickle.load(f)

    kwargs = data | dict(
        phi_max=phi_max,
    )

    if animate:
        anim = animate_fdm(kwargs=kwargs, num_frames=int(Nsim / 8), interval=50)
        fname = f"figures/{name}.mp4"
        print("Writing animation to", fname)
        anim.save(fname)
    if plot:
        plot_fdm(frame=-1, phi_max=phi_max, latexify=True, plot_legend=True, **data)
        fname = f"figures/{name}.png"
        print("Writing plot to", fname)
        plt.savefig(fname)
        plt.show()


if __name__ == "__main__":
    # main(use_RTI=False)
    typer.run(main)
