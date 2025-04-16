from mpcc.pydubins import DubinsSolver
from mpcc.serialization_schema import path_adapter, to_dubins
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from time import perf_counter

with open("test_path.json", "r") as f:
    dubins_pyobj = path_adapter.validate_json(f.read())
    dubins_path, _ = to_dubins(dubins_pyobj)

original_params = dubins_path.get_params()


def perturb_path():
    perturb = np.random.uniform(-1e-2, 1e-2, size=len(original_params))
    perturbed_params = original_params + perturb

    dubins_path.set_params(perturbed_params)


def reset_path():
    dubins_path.set_params(original_params)


solver_ls = DubinsSolver(tolerance=1e-10, max_iter=1, debug=False, line_search=True)
solver_nols = DubinsSolver(tolerance=1e-10, max_iter=1, debug=False, line_search=False)

residuals = {
    "solver": [],
    "run": [],
    "iteration": [],
    "log_10 residual": [],
}
default_arg = [False] * (dubins_path.num_segments() - 1)


def test_solver(solver, name):
    t0 = perf_counter()
    for i in range(200):
        perturb_path()
        for j in range(100):
            res = solver.solve(dubins_path, default_arg)
            dubins_path.set_params(res.params)
            residual = np.log10(
                np.linalg.norm(dubins_path.get_constraint_residuals(res.params))
            )
            residuals["solver"].append(name)
            residuals["run"].append(i)
            residuals["iteration"].append(j)
            residuals["log_10 residual"].append(residual)
    t1 = perf_counter()
    print(name, "completed in", t1 - t0, "seconds")


test_solver(solver_nols, "Solver without Line Search")
test_solver(solver_ls, "Solver with Line Search")

sns.set_style(style="whitegrid")
sns.lineplot(residuals, x="iteration", y="log_10 residual", hue="solver")
sns.despine()
plt.title("Convergence with random perturbation")
plt.show()
