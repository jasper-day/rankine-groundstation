import numpy as np
import typer
import pickle
from pathlib import Path
from run_ocp import run_ocp
from acados_ocp import setup_model
from plot_fdm import plot_fdm, animate_fdm
import matplotlib.pyplot as plt
from mpcc.serialization_schema import path_adapter, to_dubins
from path_utils import append_loiter, curry_params
from dryden_turbulence import TurbulenceSimulator
from discrete_gust import get_gust
import toml
import hashlib


def main(
    path: str = "test_path.json",
    config_path: str = "config.toml",
    extra_name: str = "",
    controller: str = "pt",
    read: bool = False,
    write: bool = True,
    animate: bool = False,
    plot: bool = False,
    extra_options=None,
):
    # north, east, xi, phi, dphi, phi_ref, s
    # x0 = np.zeros(7)  # (box_path)

    # load the path
    config = toml.load(config_path)
    with open(config_path, "rb") as f:
        digest = hashlib.sha256(f.read())
    toml_hash = digest.hexdigest()[:10]

    if extra_options is not None:
        config.update(extra_options)

    # extract parameters
    wind = np.array(config["parameters"]["wind"], dtype=np.float64)
    v_A = config["parameters"]["v_A"]

    # sub configurations
    controller_config = config["controllers"][controller]
    model_config = config["model"]
    constraints_config = config["constraints"]["path"]
    rti_config = config["rti"]

    # name for saving data
    path_name = path.split("/")[-1].split(".")[0]
    name = f"{path_name}_{controller}_{extra_name}_{toml_hash}"
    pklfile = Path(f"pickled_data/{name}.pkl")

    # determine simulation parameters with command-line override
    defaults = config["simulation"]["defaults"]
    Tf = defaults["Tf"]
    N_horizon = defaults["N_horizon"]
    Nsim = defaults["Nsim"]
    phi_max = np.radians(config["constraints"]["state"]["phi_max_deg"])

    print(f"""
        Running simulation with {controller} controller
        Tf = {Tf}
        N_horizon = {N_horizon}
        Nsim = {Nsim}
        phi_max = {np.degrees(phi_max)}""")

    # load test path
    print("Loading path", path_name)
    with open(path, "r") as f:
        path_pyobj = path_adapter.validate_json(f.read())
        dubins_path, _ = to_dubins(path_pyobj)
        orig_path, _ = to_dubins(path_pyobj)
        append_loiter(dubins_path, 60)

    # set up turbulence
    if config["simulation"]["turbulence"]["enabled"]:
        h = config["simulation"]["turbulence"]["h"]
        u_h = np.linalg.norm(wind)
        u_6 = TurbulenceSimulator.get_windspeed_6(u_h, h)
        tsim = TurbulenceSimulator(u_6, h, v_A, 0.1)
    else:
        tsim = None

    # set up discrete gust
    if config["simulation"]["gust"]["enabled"]:
        gust = get_gust(config["simulation"]["gust"])
    else:
        gust = None

    x0 = np.array([*dubins_path.eval(0.0), np.pi / 2, 0.0, 0.0, 0.0, 0.0])
    if controller in ["pt", "empcc"]:
        "Append path parameters"
        param_fn = curry_params(wind, v_A, dubins_path)
    else:
        param_fn = lambda s: np.array([*wind, v_A])

    data = None  # default value
    # run simulation
    if not read:
        ocp, solver, integrator = setup_model(
            x0,
            phi_max,
            N_horizon,
            Tf,
            param_fn(0),
            controller,
            controller_config,
            model_config,
            constraints_config,
            orig_path if controller == "mpcc" else dubins_path,
        )

        simU, simX, costs, params, t_preparation, t_feedback = run_ocp(
            ocp,
            solver,
            integrator,
            Nsim,
            param_fn,
            rti_config,
            tsim=tsim,
            gust=gust,
        )

        t = np.linspace(0, (Tf / N_horizon) * Nsim, Nsim + 1)
        data = {
            "t": t,
            "u": simU,
            "x": simX,
            "t_preparation": t_preparation,
            "t_feedback": t_feedback,
            "costs": costs,
            "params": params,
            "u_labels": ocp.model.u_labels,
            "x_labels": ocp.model.x_labels,
            "config": config,
        }

        if write:
            print("Writing to", pklfile)
            with open(pklfile, "wb") as f:
                pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

    if read:
        print("Reading from", pklfile)
        assert pklfile.is_file(), f"{pklfile} must exist for --read"
        with open(pklfile, "rb") as f:
            data = pickle.load(f)

    assert data is not None
    kwargs = data | dict(
        path=orig_path,
        phi_max=phi_max,
        path_constraints=constraints_config,
    )

    if animate:
        anim = animate_fdm(kwargs=kwargs, num_frames=int(Nsim / 8), interval=50)
        fname = f"figures/{name}.mp4"
        print("Writing animation to", fname)
        anim.save(fname)
        plt.close()
    if plot:
        plot_fdm(frame=-1, latexify=True, plot_legend=True, **kwargs)
        fname = f"figures/{name}.png"
        print("Writing plot to", fname)
        plt.savefig(fname)


if __name__ == "__main__":
    # main(use_RTI=False)
    typer.run(main)
