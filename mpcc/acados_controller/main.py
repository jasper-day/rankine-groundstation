import numpy as np
import typer
import pickle
from pathlib import Path
from run_ocp import run_ocp
from plot_fdm import plot_fdm, animate_fdm
import matplotlib.pyplot as plt
from mpcc.pydubins import DubinsPath
from mpcc.serialization_schema import path_adapter, to_dubins
from path_utils import append_loiter
from typing import Optional
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
    Tf: Optional[float] = None,
    N_horizon: Optional[int] = None,
    phi_max_deg: Optional[int] = None,
    Nsim: Optional[int] = None,
):
    # north, east, xi, phi, dphi, phi_ref, s
    # x0 = np.zeros(7)  # (box_path)

    config = toml.load(config_path)
    with open(config_path, "rb") as f:
        digest = hashlib.sha256(f.read())
    toml_hash = digest.hexdigest()[:10]

    wind = np.array(config["parameters"]["wind"], dtype=np.float64)
    v_A = config["parameters"]["v_A"]

    controller_params = config["controllers"][controller]
    model_parameters = config["model"]
    path_constraints = config["constraints"]["path"]

    path_name = path.split("/")[-1].split(".")[0]
    name = f"{path_name}_{controller}_{extra_name}_{toml_hash}"
    pklfile = Path(f"pickled_data/{name}.pkl")

    defaults = config["simulation"]["defaults"]
    if Tf is None:
        Tf = defaults["Tf"]
    if N_horizon is None:
        N_horizon = defaults["N_horizon"]
    if Nsim is None:
        Nsim = defaults["Nsim"]
    if phi_max_deg is None:
        phi_max = np.radians(config["constraints"]["state"]["phi_max_deg"])
    else:
        phi_max = np.radians(phi_max_deg)

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

    x0 = np.array([*dubins_path.eval(0), np.pi / 4, 0.0, 0.0, 0.0, 0.0])

    # run simulation
    if not read:
        simU, simX, model, costs, params = run_ocp(
            x0,
            Tf,
            N_horizon,
            phi_max,
            Nsim,
            wind,
            v_A,
            dubins_path,
            controller_type=controller,
            controller_params=controller_params,
            model_parameters=model_parameters,
            path_constraints=path_constraints,
        )

        t = np.linspace(0, (Tf / N_horizon) * Nsim, Nsim + 1)
        data = {
            "t": t,
            "u": simU,
            "x": simX,
            "costs": costs,
            "params": params,
            "u_labels": model.u_labels,
            "x_labels": model.x_labels,
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

    kwargs = data | dict(
        path=orig_path,
        phi_max=phi_max,
        path_constraints=path_constraints,
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
        plt.show()


if __name__ == "__main__":
    # main(use_RTI=False)
    typer.run(main)
