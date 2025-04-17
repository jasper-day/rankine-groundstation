from main import main
import matplotlib.pyplot as plt
import pickle
from mpcc.serialization_schema import path_adapter, to_dubins
import numpy as np
from acados_template import latexify_plot
import hashlib
import toml

latexify_plot()

test_paths = [
    "paths/chicane.json",
    "paths/path_self_intersection.json",
    "paths/test_path.json",
]

controllers = ["pt", "mpcc", "empcc"]


def run_controllers():
    for path in test_paths:
        for controller in controllers:
            main(
                path=path,
                controller=controller,
                plot=True,
            )
            plt.close()


test_cases = [[4.1, 40], [6.1, 60], [8.1, 80], [10.1, 100], [12.1, 120]]


def run_computational_controllers():
    for case in test_cases:
        extra_config = {
            "simulation": {
                "defaults": {
                    "Tf": case[0],
                    "N_horizon": case[1],
                    "Nsim": 3800,
                }
            }
        }
        for controller in controllers:
            print("Running", controller, "with", extra_config)
            main(
                path="paths/twisty_test_path",
                controller=controller,
                plot=True,
                extra_options=extra_config,
            )
            plt.close()


def get_data(path, controller, hsh):
    path_name = path.split("/")[-1].split(".")[0]
    name = f"{path_name}_{controller}__{hsh}"
    with open(f"pickled_data/{name}.pkl", "rb") as f:
        data = pickle.load(f)
    return data


def plot_controller(path, data):
    plt.gca().set_aspect("equal")
    path_constraints = data["config"]["constraints"]["path"]
    with open(path, "r") as f:
        dubins_pyobj = path_adapter.validate_json(f.read())
        dubins, _ = to_dubins(dubins_pyobj)
    th = np.linspace(0, dubins.length(), 1000)
    locs = np.array([dubins.eval(s) for s in th])
    # plot offsets
    offset_lower = path_constraints["lh"][0]
    offset_upper = path_constraints["uh"][0]
    offset_paths = [dubins.offset_path(offset_lower), dubins.offset_path(offset_upper)]
    for o_path in offset_paths:
        o_th = np.linspace(0, o_path.length(), 500)
        o_locs = np.array([o_path.eval(s) for s in o_th])
        plt.plot(o_locs[:, 1], o_locs[:, 0], color="gray", linestyle=":")
    plt.plot(locs[:, 1], locs[:, 0], linestyle="--", color="gray")

    x = data["x"]
    s = x[:, -1]
    in_path = s <= dubins.length()
    north = x[:, 0][in_path]
    east = x[:, 1][in_path]

    print("time:", north.shape[0] * 0.1)

    plt.plot(east, north, color="k")


def get_full_data():
    with open("config.toml", "rb") as f:
        digest = hashlib.sha256(f.read())
    hsh = digest.hexdigest()[:10]
    data = {
        controller: {path: get_data(path, controller, hsh) for path in test_paths}
        for controller in controllers
    }
    return data


def plot_controllers():
    data = get_full_data()
    i = 0
    for controller, paths in data.items():
        for path, data in paths.items():
            plt.subplot(331 + i)
            i += 1
            plot_controller(path, data)
            if i == 1:
                plt.title("Test Path A")
                plt.ylabel("PT Controller")
            elif i == 2:
                plt.title("Test Path B")
            elif i == 3:
                plt.title("Test Path C")
            elif i == 4:
                plt.ylabel("MPCC Controller")
            elif i == 7:
                plt.ylabel("EMPCC Controller")

    plt.gcf().set_dpi(100)
    plt.gcf().suptitle(r"\bf{\large{Controller Performance on Test Paths}}")


if __name__ == "__main__":
    # run_controllers()
    # plot_controllers()
    run_computational_controllers()
    plt.tight_layout()
    plt.show()
