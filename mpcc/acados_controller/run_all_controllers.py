from main import main
import matplotlib.pyplot as plt

test_paths = [
    "paths/chicane.json",
    "paths/path_self_intersection.json",
    "paths/test_path.json",
]

controllers = ["mpcc", "empcc", "pt"]

for path in test_paths:
    for controller in controllers:
        main(
            path=path,
            controller=controller,
            plot=True,
        )
        plt.close()
