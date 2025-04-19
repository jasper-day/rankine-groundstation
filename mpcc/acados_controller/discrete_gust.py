"Cosinusoidal discrete gust, according to mil spec 8785C"

import numpy as np
from numpy import cos, pi


def get_gust(gust_parameters):
    t_start = gust_parameters["t_start"]
    t_rampup = gust_parameters["t_rampup"]
    v_max = np.array(gust_parameters["v_max"])
    t_rampdown = gust_parameters["t_rampdown"]
    t_end = gust_parameters["t_end"]
    d_m_up = t_rampup - t_start
    d_m_down = t_end - t_rampdown

    def gust(t):
        shape_fn_up = v_max / 2 * (1 - cos(pi * t / d_m_up))
        shape_fn_down = -v_max / 2 * (1 - cos(pi * (t - t_rampdown) / d_m_down))
        if t < t_start:
            return np.array([0.0, 0.0])
        elif t < t_rampup:
            return shape_fn_up
        elif t < t_rampdown:
            return v_max
        elif t < t_end:
            return v_max + shape_fn_down
        else:
            return np.array([0.0, 0.0])

    return gust


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from dryden_turbulence import TurbulenceSimulator

    gust_parameters = dict(
        t_start=10, t_rampup=15, t_rampdown=20, t_end=30, v_max=np.array([10, -5])
    )
    tsim = TurbulenceSimulator(6.55, 30, 14, 0.1)
    ts = np.linspace(0, 40, 100)
    gust = get_gust(gust_parameters)
    gust_v = []
    for t in ts:
        tsim.advance(0.4)
        gust_v.append(gust(t) + tsim.get_wind_vector_2(0.0))
    gust_v = np.array(gust_v)
    plt.subplot(211)
    plt.title("N component")
    plt.plot(ts, gust_v[:, 0])
    plt.subplot(212)
    plt.title("E component")
    plt.plot(ts, gust_v[:, 1])
    plt.tight_layout()
    plt.show()
