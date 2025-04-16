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


def main(
    path: str = "test_path.json",
    read: bool = False,
    write: bool = True,
    animate: bool = False,
    plot: bool = False,
    Tf: float = 4.1,
    N_horizon: int = 40,
    phi_max_deg: int = 30,
    Nsim: int = 4000,
    extra_name: str = "",
    controller: str = "pt",
):
    # north, east, xi, phi, dphi, phi_ref, s
    # x0 = np.zeros(7)  # (box_path)

    wind = np.array([0, 0])
    v_A = 14.0

    if controller == "pt":
        Q = np.diag(
            [
                10,  # e_chi
                0.1,  # e_c
                0.01,  # phi
                0.1,  # dphi
            ]
        )
    else:
        Q = np.diag(
            [
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
    rho = 10

    phi_max = np.radians(phi_max_deg)

    path_name = path.split("/")[-1].split(".")[0]
    name = f"{path_name}_{controller}_Tf{Tf:.2f}_Nh{N_horizon}_Nsim{Nsim}_phimax{phi_max_deg}_{extra_name}"
    pklfile = Path(f"pickled_data/{name}.pkl")

    # load test path
    print("Loading path", path_name)
    with open(path, "r") as f:
        path_pyobj = path_adapter.validate_json(f.read())
        dubins_path, _ = to_dubins(path_pyobj)
        orig_path, _ = to_dubins(path_pyobj)
        append_loiter(dubins_path, 60)

    x0 = np.array([*dubins_path.eval(0), np.pi / 4, 0.0, 0.0, 0.0, 0.0])  # (test_path)

    # run simulation
    if not read:
        simU, simX, model, costs, params = run_ocp(
            x0,
            Tf,
            N_horizon,
            phi_max,
            Nsim,
            Q,
            R,
            rho,
            wind,
            v_A,
            dubins_path,
            controller_type=controller,
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
