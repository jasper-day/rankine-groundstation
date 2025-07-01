"""
Second-order ARX system identification for roll dynamics.
"""

import numpy as np
from typing import Literal
from enum import Enum, auto
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from mpcc_interfaces.msg import SystemID
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
from scipy import signal

def second_order_estimation(x, u, dt, method: Literal["Euler", "Tustin"] = "Euler"):
    """Estimate the second-order coefficients a0, a1, b0 using linear least squares
    
    :param x: roll data
    :param u: roll commands    

    :returns (a0, a1, b0): Continuous time transfer function coefficients
    """

    if method == "Tustin":
        raise NotImplementedError("Tustin has not been implemented")
    
    assert x.shape == u.shape and len(x.shape) == 1, "Shapes must match"

    # autoregressive estimation

    def phi(n):
        return np.array([x[n-1], x[n-2], u[n]])
    
    N = x.shape[0] - 2
    
    phi_phiT_N = np.sum(np.array([np.linalg.outer(phi(n), phi(n)) for n in range(2, N + 2)]), axis=0)
    
    phiT_y_N = np.sum(np.array([phi(n) * x[n] for n in range(2, N + 2)]), axis=0)
    

    # discrete time coefficients
    alpha_0, alpha_1, beta_0 = np.linalg.solve(phi_phiT_N, phiT_y_N)

    if method == "Euler":
        # Euler estimation method.
        # alpha_1 = -(2 + a_1 dt) * alpha_2
        # alpha_2 = -1 / (1 + a_1 dt + a_0 dt^2)
        # beta_0 = - b0 * dt^2 * alpha_2
        a_1 = -(alpha_0 / alpha_1 + 2) / (dt)
        b_0 = - beta_0 / alpha_1 / (dt**2)
        a_0 = (-1 / alpha_1 - 1 - a_1 * dt)/(dt**2)
    
    return a_0, a_1, b_0, alpha_0, alpha_1, beta_0

class SystemIdMode(Enum):
    TWO_UP = auto()
    ONE_DOWN = auto()
    ONE_UP = auto()
    NEUTRAL = auto()

class SystemIdController:
    inverse: bool
    mode: SystemIdMode = SystemIdMode.NEUTRAL
    angle: float
    time_unit: float
    t_start: Time

    def __init__(self, t_start, angle=np.radians(30), inverse=False, time_unit=1.0):
        """
        :param t_start: Time of initialization
        :param angle: Maximum angle in radians
        :param inverse: Do in reverse?
        :param time_unit: Length of 1 leg of the 2-1-1
        """
        self.inverse = inverse
        self.angle = angle
        self.time_unit = time_unit
        self.t_start = t_start
    
    def elapsed_time_s(self, t: Time) -> float:
        return (t - self.t_start).nanoseconds * 1e-9

    def transition(self, mode: SystemIdMode, t: Time):
        self.mode = mode
        self.t_start = t

    def up(self):
        return self.angle if not self.inverse else -self.angle
    
    def down(self):
        return -self.angle if not self.inverse else self.angle

    def update(self, t: Time):
        """
        :param t: Current time
        
        :returns angle: Commanded angle in radians
        """
        if self.mode == SystemIdMode.TWO_UP:
            if self.elapsed_time_s(t) > 2.0:
                self.transition(SystemIdMode.ONE_DOWN, t)
                return self.down()
            else:
                return self.up()
        if self.mode == SystemIdMode.ONE_DOWN:
            if self.elapsed_time_s(t) > 1.0:
                self.transition(SystemIdMode.ONE_UP, t)
                return self.up()
            else:
                return self.down()
        if self.mode == SystemIdMode.ONE_UP:
            if self.elapsed_time_s(t) > 1.0:
                self.transition(SystemIdMode.NEUTRAL, t)
                return 0.0
            else:
                return self.up()
        if self.mode == SystemIdMode.NEUTRAL:
            if self.elapsed_time_s(t) > 1.0:
                self.transition(SystemIdMode.TWO_UP, t)
                return self.up()
            else:
                return 0.0


class SystemIdNode(Node):

    measurements: list[float] = []
    commands: list[float] = []
    seconds: list[float] = []
    dt: float

    def __init__(self, dt = 0.05):
        super().__init__("system_id_py")


        self.system_id_sub = self.create_subscription(
            SystemID, "/gs/system_id", self.recv_system_id, 1
        )

        self.do_system_id_client = self.create_client(
            SetBool, "/gs/do_system_id"
        )

        self.print_results_srv = self.create_service(
            Trigger, "/sysid/print_results", self.print_results
        )

        self.stop_sysid_srv = self.create_service(
            Trigger, "/sysid/stop_sysid", self.stop_sysid_cb
        )



        while not self.do_system_id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service")

        msg = SetBool.Request()
        msg.data = True
        future = self.do_system_id_client.call_async(msg)
        future.add_done_callback(lambda future: self.get_logger().info(future.result().message))

        self.dt = dt

        self.plot_timer = self.create_timer(0.2, self.update_plot)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        plt.xlabel("Time (s)")
        plt.ylabel("Bank angle (deg)")


        self.get_logger().info("Starting interactive plot")
        plt.ion()

    
    def recv_system_id(self, msg: SystemID):
        self.measurements.append(msg.roll_angle)
        self.commands.append(msg.commanded_roll)
        self.seconds.append(self.get_clock().now().nanoseconds * 1e-9)

    def update_plot(self):

        if len(self.measurements) > 100:
            a_0, a_1, b_0, alf_0, alf_1, bet_0 = second_order_estimation(
                np.array(self.measurements),
                np.array(self.commands),
                0.05
            )

            alpha_1 = - 1 / (1 + a_1  * self.dt + a_0 * self.dt**2 )
            alpha_0 = (-2 - a_1 * self.dt) * alpha_1
            beta_0 = -b_0 * self.dt**2 * alpha_1

            assert np.abs(alpha_0 - alf_0) < 1e-6
            assert np.abs(alpha_1 - alf_1) < 1e-6
            assert np.abs(bet_0 - beta_0) < 1e-6

            theoretical_response = signal.lfilter(
                np.array([beta_0]),
                np.array([1, -alpha_1, -alpha_0]),
                np.array(self.measurements)
            )
            self.ax.cla()

            plt.plot(self.seconds, np.degrees(self.measurements), color="black")
            plt.plot(self.seconds, np.degrees(self.commands), color="black", linestyle="--")
            # plt.plot(self.seconds, theoretical_response, color="green")
            plt.pause(0.1)

    def stop_sysid(self):
        msg = SetBool.Request()
        msg.data = False
        future = self.do_system_id_client.call_async(msg)
        return future
    
    def print_results(self, req: Trigger.Request, res: Trigger.Response):
        with open("results.csv", "a") as f:
            np.array(self.measurements).tofile(f, sep=",")
            f.write('\n')
            np.array(self.commands).tofile(f, sep=",")
            f.write('\n')
            np.array(self.seconds).tofile(f, sep=",")
            f.write('\n')
        res.message = ""
        res.success = True
        return res

    def stop_sysid_cb(self, req: Trigger.Request, res: Trigger.Response):
        future = self.stop_sysid()
        future.add_done_callback(lambda future: self.get_logger().info(f"Cancelled sysid: {future.result().success}"))
        res.message = ""
        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)
    node = SystemIdNode()
    rclpy.spin(node)
    # once spin ends
    node.stop_sysid()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()