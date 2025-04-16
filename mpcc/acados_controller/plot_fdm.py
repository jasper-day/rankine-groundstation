import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.animation import FuncAnimation
from typing import Optional
from acados_template import latexify_plot
from mpcc.pydubins import DubinsPath


def plot_fdm(
    frame: int,
    t: np.ndarray,
    phi_max: float,
    u: np.ndarray,
    x: np.ndarray,
    x_labels: list[str],
    u_labels: list[str],
    costs: np.ndarray,
    params: np.ndarray,
    path: Optional[DubinsPath] = None,
    offset: float = 20,
    gs: Optional[gridspec.GridSpec] = None,
    time_label="$t$",
    latexify=True,
    plot_legend: bool = True,
    linestyle: str = "-",
    color: str = "C1",
):
    """
    Params:
        t: time values of the discretization
        phi_max: maximum absolute value of phi
        u: array with shape (N_sim-1, nu) or (N_sim, nu)
        x: array with shape (N_sim, nx)
        x_labels: labels of x
        u_labels: labels of u
        latexify: latex style plots
    """
    if latexify:
        latexify_plot()

    if frame != -1:
        t = t[: frame + 1]
        x = x[: frame + 1, :]
        u = u[:frame, :]
        costs = costs[:frame]

    nx = x.shape[1]
    assert nx == 7
    north = x[:, 0]
    east = x[:, 1]
    xi = x[:, 2]
    phi = x[:, 3]
    phi_ref = x[:, 5]

    if gs is None:
        gs = gridspec.GridSpec(3, 1, plt.gcf(), height_ratios=[1, 1, 5])

    phi_max_deg = np.degrees(phi_max)
    phi_deg = np.degrees(phi)
    phi_ref_deg = np.degrees(phi_ref)
    xi_deg = np.degrees(xi)

    plt.subplot(gs[0])
    if plot_legend:
        plt.axhline(phi_max_deg, t[0], t[-1], color="C0", linestyle="--")
        plt.axhline(-phi_max_deg, t[0], t[-1], color="C0", linestyle="--")
    plt.plot(t, phi_deg, color=color, label=r"$\phi$")
    plt.plot(t, phi_ref_deg, color=color, label=r"$\phi_\text{ref}$", linestyle=":")
    plt.xlabel(time_label)
    if plot_legend:
        plt.legend(loc="right")
    plt.grid()

    plt.subplot(gs[1])
    try:
        plt.plot(t[1:], costs, color=color, label=r"Costs")
    except ValueError as e:
        print(str(e))
    if plot_legend:
        plt.legend(loc="right")
    plt.grid()

    plt.subplot(gs[2])
    plt.gca().set_aspect("equal")
    th = np.linspace(0, 2 * np.pi, 1000, endpoint=True)
    if path is None:
        # plot a circle of radius 60
        r = 60
        cth = np.cos(th) * r
        sth = np.sin(th) * r
        if plot_legend:
            plt.plot(cth, sth, color="C0", linestyle="--")
    else:
        if frame != -1:
            th = np.linspace(0, x[-1, -1], 500)
        else:
            th = np.linspace(0, path.length(), 500)
        locs = np.array([path.eval(s) for s in th])
        # plot offsets
        offset_paths = [path.offset_path(offset), path.offset_path(-offset)]
        for o_path in offset_paths:
            o_th = np.linspace(0, o_path.length(), 500)
            o_locs = np.array([o_path.eval(s) for s in o_th])
            plt.plot(o_locs[:, 1], o_locs[:, 0], color="gray", linestyle=":")

        plt.plot(
            locs[:, 1], locs[:, 0], color="gray", linestyle="--"
        )  # east then north
        if frame != -1:
            # plot contouring error
            plt.plot(
                [locs[-1, 1], east[-1]],
                [locs[-1, 0], north[-1]],
                linestyle=":",
                color="C0",
            )
    plt.plot(east, north, color=color, label=r"$x$", linestyle=linestyle)
    plt.scatter(east[0], north[0], marker="^", c="C1")
    plt.scatter(east[-1], north[-1], marker="s", c="C1")
    plt.annotate("Start", xy=(east[0], north[0]))
    if frame == -1:
        plt.annotate("End", xy=(east[-1], north[-1]))
    if plot_legend:
        plt.legend()
        plt.xlabel("Easting [m]")
        plt.ylabel("Northing [m]")
    plt.grid()
    # plt.tight_layout()
    return gs


def animate_fdm(
    kwargs,
    num_frames=100,
    interval=50,
    latexify=True,
):
    """
    Create an animation of the FDM plot

    Parameters:
        t: time values of the discretization
        phi_max: maximum absolute value of phi
        u: array with shape (N_sim-1, nu) or (N_sim, nu)
        x: array with shape (N_sim, nx)
        x_labels: labels of x
        u_labels: labels of u
        latexify: latex style plots
        num_frames: number of frames for the animation
        interval: delay between frames in milliseconds
    """
    if latexify:
        latexify_plot()

    # Create figure and gridspec
    fig = plt.figure(figsize=(10, 8))
    gs = gridspec.GridSpec(3, 1, fig, height_ratios=[1, 1, 5])

    # Create frames - use every nth point to create smoother animation
    n_points = len(kwargs["t"])
    if num_frames >= n_points:
        frame_indices = range(n_points)
    else:
        frame_indices = np.linspace(0, n_points - 1, num_frames, dtype=int)

    def plotfn(frame):
        plt.clf()
        return plot_fdm(frame, gs=gs, **kwargs)

    # Create animation
    anim = FuncAnimation(
        fig,
        plotfn,
        frames=frame_indices,
        interval=interval,
        blit=False,
    )
    return anim
