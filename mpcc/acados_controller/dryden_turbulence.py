"""
Model turbulence

Status: Verified against Matlab Dryden model
"""

from pydrake.systems.primitives import RandomSource, LinearSystem, Gain
from pydrake.common import RandomDistribution
import scipy.signal as sg
import numpy as np
import matplotlib.pyplot as plt
from acados_template import latexify_plot
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput, Demultiplexer, Multiplexer
from pydrake.systems.drawing import plot_system_graphviz
from typing import Literal
import sys

latexify_plot()


def bode(tf):
    hz = np.logspace(-2, 2, 200, base=10)
    omega = hz * 2 * np.pi
    return sg.bode(tf, w=omega)


def make_simulator(H_u, H_v, H_w, dt):
    H_u_ss = H_u.to_ss()
    H_v_ss = H_v.to_ss()
    H_w_ss = H_w.to_ss()

    builder = DiagramBuilder()
    random = builder.AddSystem(RandomSource(RandomDistribution.kGaussian, 3, dt))
    random_gain = builder.AddSystem(Gain(np.sqrt(np.pi / dt), 3))
    demux = builder.AddSystem(Demultiplexer(3))
    ss_u = builder.AddNamedSystem(
        "Longitudinal gust",
        LinearSystem(H_u_ss.A, H_u_ss.B, H_u_ss.C, H_u_ss.D),
    )
    ss_v = builder.AddNamedSystem(
        "Lateral gust",
        LinearSystem(H_v_ss.A, H_v_ss.B, H_v_ss.C, H_v_ss.D),
    )
    ss_w = builder.AddNamedSystem(
        "Vertical gust",
        LinearSystem(H_w_ss.A, H_w_ss.B, H_w_ss.C, H_w_ss.D),
    )
    mux = builder.AddSystem(Multiplexer(3))

    # demux random output with gain
    builder.Connect(random.get_output_port(), random_gain.get_input_port())
    builder.Connect(random_gain.get_output_port(), demux.get_input_port())

    # connect turbulence filters
    builder.Connect(demux.get_output_port(0), ss_u.get_input_port())
    builder.Connect(demux.get_output_port(1), ss_v.get_input_port())
    builder.Connect(demux.get_output_port(2), ss_w.get_input_port())
    # mux outputs
    builder.Connect(ss_u.get_output_port(), mux.get_input_port(0))
    builder.Connect(ss_v.get_output_port(), mux.get_input_port(1))
    builder.Connect(ss_w.get_output_port(), mux.get_input_port(2))

    builder.ExportOutput(mux.get_output_port(), "gust_vector")
    logger = LogVectorOutput(mux.get_output_port(), builder, publish_period=0.1)
    return builder, logger


def load_turbulence():
    return np.loadtxt("dryden_turbulence_30m.csv", dtype=np.float64, delimiter=",")


class TurbulenceSimulator:
    z0 = 0.6

    def __init__(self, u_6, h, V, dt):
        """Simulates atmospheric turbulence.

        **Only valid below 1750 ft = 533m**


        :param u_6: Wind speed at 6 m AGL
        :param h: The altitude (m) above ground level
        :param V: Airspeed of craft (m/s)
        :param dt: sampling interval

        Source
        https://uk.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html

        and

        https://ntrs.nasa.gov/api/citations/20190000875/downloads/20190000875.pdf
        """
        self.dt = dt
        self.u_6 = u_6
        self.h = h
        self.V = V
        self.H_u, self.H_v, self.H_w = self.get_dryden_tf(u_6, h, V)
        self.builder, self.logger = make_simulator(self.H_u, self.H_v, self.H_w, dt)
        self.logger.set_name("gust")
        self.diagram = self.builder.Build()
        self.diagram.set_name("TurbulenceSimulator")
        self.simulator = Simulator(self.diagram)
        self.context = self.simulator.get_context()
        self.simulator.Initialize()
        self.t = 0

    def advance(self, dt):
        self.t += dt
        self.simulator.AdvanceTo(self.t)

    def advance_to(self, t):
        assert t > self.t
        self.t = t
        self.simulator.AdvanceTo(t)

    def get_gust(self):
        log = self.logger.FindLog(self.context)
        return log.data()[:, -1]

    def get_log(self):
        log = self.logger.FindLog(self.context)
        return log

    def visualize(self):
        plot_system_graphviz(self.diagram, max_depth=2)

    def windspeed(self):
        "Find windspeed assuming a roughness height of 0.6m"
        return self.u_6 * np.log(self.h / self.z0) / np.log(6 / self.z0)

    def get_wind_vector_2(self, xi: float):
        """Get the 2d wind vector with turbulence.

        Make sure to call advance() first.

        :param xi: The heading (from north) of the airplane model
        """
        gust = self.get_gust()
        u = gust[0]
        v = gust[1]
        u_dir = np.array([np.cos(xi), np.sin(xi)])
        v_dir = np.array([-np.sin(xi), np.cos(xi)])
        return u * u_dir + v * v_dir

    @staticmethod
    def get_windspeed_6(windspeed, height):
        "Get the windspeed at 6 m, assuming a logarithmic velocity profile"
        return (
            windspeed
            * np.log(6 / TurbulenceSimulator.z0)
            / np.log(height / TurbulenceSimulator.z0)
        )

    def get_dryden_tf(self, u_6: float, h: float, V: float):
        """Get all transfer functions for Dryden gusts

        **Only valid below 1750 ft = 533m**


        :param u_6: Wind speed at 6 m AGL
        :param h: The altitude (m) above ground level
        :param V: Airspeed of craft (m/s)
        :param dt: sampling interval

        Source
        https://uk.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html

        and

        https://ntrs.nasa.gov/api/citations/20190000875/downloads/20190000875.pdf

        :returns: (lateral_tf, longitudinal_tf, vertical_tf)
        """

        meter2feet = 3.2808398950131235
        feet2meter = 1 / meter2feet

        # turbulent characteristic length in the atmosphere
        L_FA = 1750 * feet2meter
        assert h < L_FA, "Only boundary layer turbulence is supported"

        sigma_w = 0.1 * u_6
        h_ft = h * meter2feet

        # matlab and MIL-F-8785C
        L_u_ft = h_ft / (0.177 + 0.000823 * h_ft) ** 1.2
        sigma_u_ratio = 1 / (0.177 + 0.000823 * h_ft) ** 0.4

        sigma_u = sigma_v = sigma_u_ratio * sigma_w

        L_v_ft = L_u_ft
        L_u = L_u_ft * feet2meter
        L_v = L_v_ft * feet2meter
        L_w = h

        self.L_u = L_u
        self.L_v = L_v
        self.L_w = L_w
        self.sigma_u = sigma_u
        self.sigma_v = sigma_v
        self.sigma_w = sigma_w

        # print("Lengthscales:")
        # print("L_u", L_u)
        # print("L_v", L_v)
        # print("L_w", L_w)

        # print("\nVariances:")
        # print("sigma_u", sigma_u)
        # print("sigma_v", sigma_v)
        # print("sigma_w", sigma_w)

        # Length scales into time periods
        L_u_s = L_u / V
        L_v_s = L_v / V
        L_w_s = L_w / V

        # transfer functions
        H_u_num = sigma_u * np.sqrt(2 / np.pi * L_u_s)
        H_u_den = np.array([L_u_s, 1])

        H_v_num = sigma_v * np.sqrt(L_v_s / np.pi) * np.array([np.sqrt(3) * L_v_s, 1])
        H_v_den_part = np.array([L_v_s, 1])
        H_v_den = np.polymul(H_v_den_part, H_v_den_part)

        H_w_num = sigma_w * np.sqrt(L_w_s / np.pi) * np.array([np.sqrt(3) * L_w_s, 1])
        H_w_den_part = np.array([L_w_s, 1])
        H_w_den = np.polymul(H_w_den_part, H_w_den_part)

        return (
            sg.TransferFunction(H_u_num, H_u_den),
            sg.TransferFunction(H_v_num, H_v_den),
            sg.TransferFunction(H_w_num, H_w_den),
        )

    def get_Pxx(self, axis: Literal["u", "v", "w"], omega):
        if axis == "u":
            Ltu = self.L_u / self.V
            sigma2 = self.sigma_u**2
            return sigma2 * 2 * Ltu / np.pi / (1 + (Ltu * omega) ** 2)
        if axis == "v":
            Ltv = self.L_v / self.V
            sigma2 = self.sigma_v**2
            return (
                sigma2
                * Ltv
                / np.pi
                * (1 + 3 * (Ltv * omega) ** 2)
                / (1 + (Ltv * omega) ** 2) ** 2
            )
        if axis == "w":
            Ltw = self.L_w / self.V
            sigma2 = self.sigma_w**2
            return (
                sigma2
                * Ltw
                / np.pi
                * (1 + 3 * (Ltw * omega) ** 2)
                / (1 + (Ltw * omega) ** 2) ** 2
            )


if __name__ == "__main__":
    matlab_data = load_turbulence().T
    dt = 0.1
    tsim = TurbulenceSimulator(5.885, 30, 14, dt)

    tsim.advance_to(20)

    log_20 = tsim.get_log()
    t_20 = log_20.sample_times()
    N_20 = t_20.shape[0]
    if "block" in sys.argv:
        plt.figure(dpi=400)
        tsim.visualize()
        plt.savefig("figures/block_diagram.png")

    # handy little calculator utility!
    if "calc" in sys.argv:
        print("Calculating windspeed at 6 m.")
        while True:
            try:
                height = float(input("Input a height:\n"))
                speed = float(input("Input the wind speed:\n"))
                print("Windspeed at 6 m:")
                print(tsim.get_windspeed_6(speed, height))
            except Exception as e:
                ans = input(
                    "Sorry, I don't know that. Would you like to try again? [Y/n]\n"
                )
                if "n" in ans.lower():
                    break

    if "plot" not in sys.argv:
        # exceptions for control flow? You betcha!
        raise Exception("Nothing to do.")

    tsim.advance_to(100000)
    log = tsim.get_log()
    t = log.sample_times()

    # basic signal analysis
    print(2**14)
    f, Pxx = sg.welch(log.data(), 1 / dt, nperseg=2**14)
    f_mat, Pxx_mat = sg.welch(matlab_data, 1 / dt, nperseg=512, scaling="density")

    f_omega = f * 2 * np.pi
    omega = np.logspace(np.log10(f_omega[1]), np.log10(f_omega[-1]), 100)
    omega_f = omega / 2 / np.pi
    plt.subplot(311)
    plt.title(r"\bf{Fluctuating velocities}")
    plt.plot(t_20, log_20.data()[0, :N_20], label="$u'$", color="k")
    plt.plot(t_20, log_20.data()[1, :N_20], label="$v'$", color="grey")
    # plt.plot(t_20, matlab_data[0, :N_20], label="matlab $u'$")
    # plt.plot(t_20, matlab_data[1, :N_20], label="matlab $v'$")

    plt.xlabel("Time $t$ (s)")
    plt.ylabel(r"$\|u'\|$ (m/s)")
    plt.legend()
    plt.grid()
    plt.subplot(312)
    plt.semilogx()
    plt.title(r"\bf{PSD for $u'$ component}")
    plt.plot(f, Pxx[0, :], label="Measured spectrum", color="k")
    # plt.plot(f_mat, Pxx_mat[0, :], label="Matlab spectrum")
    plt.plot(
        omega_f,
        tsim.get_Pxx("u", omega) * 2 * np.pi,
        label="Dryden spectrum",
        color="grey",
    )
    plt.xlabel("Frequency $f$ (Hz)")
    plt.ylabel(r"$\Phi \, (\frac{\text{m}^2}{\text{s}^2 \text{Hz}})$")
    plt.legend()
    plt.subplot(313)
    plt.semilogx()
    plt.title(r"\bf{PSD for $v'$ component}")
    plt.plot(f, Pxx[1, :], label="Measured spectrum", color="k")
    # plt.plot(f_mat, Pxx_mat[1, :], label="Matlab spectrum")
    plt.plot(
        omega_f,
        tsim.get_Pxx("v", omega) * 2 * np.pi,
        label="Dryden spectrum",
        color="grey",
    )
    plt.xlabel("Frequency $f$ (Hz)")
    plt.ylabel(r"$\Phi \, (\frac{\text{m}^2}{\text{s}^2 \text{Hz}})$")
    plt.legend()
    # plt.subplot(313)
    # plt.title("W")
    # plt.plot(f, Pxx[2, :], alpha=0.5)
    plt.tight_layout()
    plt.show()
    plt.show()
