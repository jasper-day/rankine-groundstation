import numpy as np
import typer
import pickle
from pathlib import Path
from run_ocp import run_ocp
from plot_fdm import plot_fdm, animate_fdm
import matplotlib.pyplot as plt
from mpcc.pydubins import DubinsPath
from mpcc.serialization_schema import path_adapter, to_dubins


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
    # north, east, xi, phi, dphi,  phi_ref, s
    x0 = np.array([-12, -281, 0.0, 0.0, 0.0, 0.0, 0.0])
    phi_max = np.radians(phi_max_deg)

    name = (
        f"fdm_2d_Tf{Tf:.2f}_Nh{N_horizon}_Nsim{Nsim}_phimax{phi_max_deg}_{extra_name}"
    )
    pklfile = Path(f"pickled_data/{name}.pkl")

    with open("test_path.json", "r") as f:
        path_pyobj = path_adapter.validate_json(f.read())
        dubins_path, _ = to_dubins(path_pyobj)

    if not read:
        simU, simX, model = run_ocp(x0, Tf, N_horizon, phi_max, Nsim, dubins_path)

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
        path=dubins_path,
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
