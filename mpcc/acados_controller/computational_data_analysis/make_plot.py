import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

plt.style.use("../typst.mplstyle")


def read(file):
    with open("./pickled_data/" + file, "rb") as f:
        return pickle.load(f)


datas = [read(file) for file in os.listdir("./pickled_data")]
names = os.listdir("./pickled_data")


def get_info(data):
    t_prepare = data["t_preparation"][500:-1000]
    t_feedback = data["t_feedback"][500:-1000]
    return t_prepare, t_feedback, t_prepare + t_feedback


def get_name_npoints(name):
    words = name.split("_")
    return words[3].upper(), int(words[4])


times = [get_name_npoints(name) + get_info(data) for name, data in zip(names, datas)]

time_df = pd.DataFrame(
    times,
    columns=[
        "Controller",
        "Horizon Length (N)",
        "Preparation Time (ms)",
        "Feedback Time (ms)",
        "Total Time (ms)",
    ],
).explode(["Preparation Time (ms)", "Feedback Time (ms)", "Total Time (ms)"])


MPCC_times = time_df[time_df["Controller"] == "MPCC"]
EMPCC_times = time_df[time_df["Controller"] == "EMPCC"]
print(MPCC_times["Total Time (ms)"].mean() / EMPCC_times["Total Time (ms)"].mean())

MPCC_times_120 = MPCC_times[MPCC_times["Horizon Length (N)"] == 120]
EMPCC_times_120 = EMPCC_times[EMPCC_times["Horizon Length (N)"] == 120]

MPCC_120_sorted = MPCC_times_120.sort_values(by="Total Time (ms)")[
    "Total Time (ms)"
].to_numpy()
EMPCC_120_sorted = EMPCC_times_120.sort_values(by="Total Time (ms)")[
    "Total Time (ms)"
].to_numpy()

N_val = MPCC_120_sorted.shape[0]
median = int(N_val / 2)
qr = int(N_val / 4)

print("MPCC IQR", MPCC_120_sorted[median + qr] - MPCC_120_sorted[median - qr])
print("EMPCC IQR", EMPCC_120_sorted[median + qr] - EMPCC_120_sorted[median - qr])


# print(MPCC_times["Total Time (ms)"].mean() / EMPCC_times["Total Time (ms)"].mean())

ax = plt.gca()

sns.set_context("paper")

sns.set_palette("deep")

sns.pointplot(
    time_df,
    x="Horizon Length (N)",
    y="Total Time (ms)",
    hue="Controller",
    estimator="median",
    errorbar=("pi", 75),
    alpha=1,
    native_scale=True,
    capsize=0.1,
    markers=["^", "o", "s"],
)

plt.title(r"NMPC Solver Timings")

sns.despine()

# sns.stripplot(
#     time_df[::100],
#     x="Horizon Length (N)",
#     y="Total Time (ms)",
#     hue="Controller",
#     alpha=0.25,
#     jitter=True,
#     ax=ax,
# )

plt.grid()
plt.show()
