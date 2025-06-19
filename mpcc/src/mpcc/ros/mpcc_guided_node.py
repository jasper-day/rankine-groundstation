import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import State, AttitudeTarget, StatusText
from mavros_msgs.srv import SetMode
from mavros.command import CommandBool, CommandTOL, CommandInt
from mpcc_interfaces.srv import SetPath, GetPath
from mpcc_interfaces.msg import Status, TrajectoryPlan
from mpcc_interfaces.msg import State as State_mpcc_msg
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt32, Float64
from std_srvs.srv import SetBool
from mpcc.ros.state_machine import StateMachine, SMState
from mpcc.pydubins import DubinsPath
from mpcc.serialization_schema import to_path, to_dubins, path_adapter
from pygeodesy.ltp import LocalCartesian
from pygeodesy.ecef import EcefKarney
from mpcc.quaternion import quaternion_to_RPY
import math
import numpy as np
from mpcc.controller import MPCCController
import toml


class MPCCGuidedNode(Node):
    dubins: DubinsPath | None = None
    origin: GeoPoint | None
    local_cartesian_converter: LocalCartesian | None = None
    pose: PoseStamped | None = None
    velocity: TwistStamped | None = None
    gps_fix: NavSatFix | None = None
    satellites: int | None = None
    arm_ready: bool = False
    controller: MPCCController
    heading: Float64 | None = None
    phi_ref: float | None
    arclength: float | None
    wind: TwistWithCovarianceStamped | None

    def __init__(self, mpcc_config: dict[str, any]):
        super().__init__("mpcc_guided_py")

        # Low-latency QoS profile
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # high refresh rate commands
        attitude_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # low refresh rate commands
        altitude_speed_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # flight termination commands (must be reliable)
        fts_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
        )

        self.current_state = State()
        print(self.current_state)

        # Subscribers

        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.recv_state, state_qos
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.recv_pose, 1
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_body", self.recv_velocity, 1
        )

        self.gps_fix_sub = self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.recv_gps_fix, 1
        )

        self.satellites_sub = self.create_subscription(
            UInt32, "/mavros/global/raw/satellites", self.recv_satellites, 1
        )

        self.heading_sub = self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self.recv_heading, 1
        )

        self.wind_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/mavros/wind_estimation", self.recv_wind, 1
        )

        # Publishers

        self.trajectory_pub = self.create_publisher(
            TrajectoryPlan, "gs/trajectory_plan", qos_profile=attitude_qos
        )

        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            "/mavros/setpoint_raw/target_attitude",
            qos_profile=state_qos,
        )

        self.altitude_pub = self.create_client(
            CommandInt, "/mavros/cmd/command_int", qos_profile=altitude_speed_qos
        )

        self.velocity_pub = self.create_client(
            CommandInt, "/mavros/cmd/command_int", qos_profile=altitude_speed_qos
        )

        self.status_pub = self.create_publisher(Status, "/gs/status", 10)

        self.message_pub = self.create_publisher(
            StatusText, "/gs/statustext/recv", 10
        )

        # Clients

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")

        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.tol_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        # Services

        self.set_path_srv = self.create_service(
            SetPath, "/gs/set_path", callback=self.set_path
        )

        self.get_path_srv = self.create_service(
            GetPath, "/gs/get_path", callback=self.get_path
        )

        self.try_arm_srv = self.create_service(
            SetBool, "/gs/try_arm", callback=self.set_arm_ready
        )

        self.state_machine = StateMachine(self)
        self.controller = MPCCController(mpcc_config)

        # Start clients
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for mode set service")
        while not self.tol_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff landing service")

        # start timers

        # self.mpcc_timer = self.create_timer(0.1, self.mpcc_update)
        self.status_timer = self.create_timer(0.1, self.pub_status)
        self.state_timer = self.create_timer(0.1, self.state_machine.update)

    def recv_state(self, msg):
        self.current_state = msg

    def recv_pose(self, msg):
        self.pose = msg

    def recv_velocity(self, msg):
        self.velocity = msg

    def recv_satellites(self, msg):
        self.satellites = msg

    def recv_gps_fix(self, msg):
        if self.local_cartesian_converter is not None and self.dubins is not None:
            enu = self.ltp_Enu(msg)
            north = enu[1]
            east = enu[0]
            self.arclength = self.dubins.get_closest_arclength(
                np.array([north, east]), self.arclength
            )
        self.gps_fix = msg

    def recv_heading(self, msg):
        self.heading = msg

    def recv_wind(self, msg):
        self.wind = msg

    def get_status(self):
        res = Status()
        res.armed = self.current_state.armed
        res.connected = self.current_state.connected
        res.mode = self.current_state.mode
        res.state = State_mpcc_msg()
        res.state.state = self.state_machine.state.value
        # TODO: Figure out the time stuff (probably with the help of Claude)
        # if self.state_machine.last_transition_time is not None:
        #     res.last_transition_time = Time()
        #     Time.nanosec = self.state_machine.last_transition_time.nanoseconds
        return res

    def pub_status(self):
        msg = self.get_status()
        self.status_pub.publish(msg)

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        return self.arming_client.call_async(req)

    def _set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        return self.set_mode_client.call_async(req)

    def set_auto(self):
        return self._set_mode("AUTO")

    def set_guided(self):
        return self._set_mode("GUIDED")

    def takeoff(self, altitude: float):
        req = CommandTOL.Request()
        req.altitude = 20.0
        return self.tol_client.call_async(req)

    def cmd_altitude(self, altitude: float, altitude_rate: float):
        req = CommandInt.Request()
        req.command = 43001
        req.z = altitude
        req.param3 = altitude_rate
        return self.altitude_pub.call_async(req)

    def cmd_speed(self, speed: float, speed_rate: float):
        req = CommandInt.Request()
        req.command = 43000
        req.param1 = 0.0  # airspeed, not ground speed
        req.param2 = speed
        req.param3 = speed_rate
        return self.velocity_pub.call_async(req)

    def cmd_roll(self, roll_angle_rad: float):
        req = AttitudeTarget()
        req.type_mask = 0b01111110
        # just rotation about the x-axis
        req.orientation.w = math.cos(roll_angle_rad / 2)
        req.orientation.x = math.sin(roll_angle_rad / 2)
        self.attitude_pub.publish(req)
        return None

    def set_path(self, req: SetPath.Request, res: SetPath.Response):
        curr_state = self.state_machine.state
        # Cannot change path after arming the vehicle (!)
        if curr_state != SMState.CONNECTED:
            res.success = False
            res.error_msg = f"Cannot set state in mode {curr_state.name}"
            return res
        try:
            self.dubins = to_dubins(path_adapter.validate_json(req.newpath.newpath, strict=True))[0]
            self.origin = req.newpath.origin
            self.local_cartesian_converter = LocalCartesian(
                latlonh0=self.origin.latitude,
                lon0=self.origin.longitude,
                height0=self.origin.altitude,
                ecef=EcefKarney(),  # default WGS-84 ellipsoid
            )
            res.success = True
            res.error_msg = ""
            self.controller.set_path(self.dubins)

        except Exception as e:
            # atomic transaction
            self.dubins = None
            self.origin = None
            self.local_cartesian_converter = None
            res.success = False
            res.error_msg = traceback.format_exc()
        return res

    def get_path(self, req: SetPath.Request, res: SetPath.Response):
        if self.dubins is None:
            res.path = ""
            res.error_msg = "No path set"
        else:
            try:
                res.path = path_adapter.dump_json(to_path(self.dubins))
                res.error_msg = ""
            except Exception as e:
                res.path = ""
                res.error_msg = str(e)
        return res

    def set_arm_ready(self, req: SetBool.Request, res: SetBool.Response):
        self.arm_ready = req.data
        res.success = True
        res.message = "Will try to arm" if req.data else "Stop trying to arm"
        return res

    # state machine transition checkers

    def arming_ready(self) -> bool:
        return (
            self.arm_ready and (not self.current_state.armed) #and self.satellites is not None and self.satellites > 3
            # other checks...?
        )

    def mpcc_ready(self) -> bool:
        if (
            self.gps_fix is None
            or self.dubins is None
            or self.local_cartesian_converter is None
        ):
            return False
        local_coords = self.ltp_Enu(self.gps_fix)
        p = np.array([local_coords[1], local_coords[0]])
        distance_to_path_start = np.linalg.norm(p - self.dubins.eval(0))

        return (
            self.velocity is not None
            and self.pose is not None
            and self.heading is not None
            and self.controller.ready
            and self.arclength > 0.0
            and distance_to_path_start
            < 40  # assume we can get within 40 m of the path start
            # other checks...?
        )

    def ltp_Enu(self, gps_fix: NavSatFix):
        "translate current location into local tangent space"
        return self.local_cartesian_converter.forward(
            latlonh=gps_fix.latitude, lon=gps_fix.longitude, height=gps_fix.altitude
        ).toEnu()

    def mpcc_update(self):
        "Run MPCC loop"
        local_coords = self.ltp_Enu
        orientation = self.pose.pose.orientation
        rpy = quaternion_to_RPY(orientation)
        # get full current state vector
        # north, east, xi, phi, dphi, phi_ref, s
        north = local_coords[1]
        east = local_coords[0]
        xi = self.heading
        phi = rpy["roll"]
        dphi = self.velocity.twist.angular.x
        phi_ref = self.phi_ref if self.phi_ref is not None else 0.0
        s = self.arclength
        X = np.array([north, east, xi, phi, dphi, phi_ref, s])
        # TODO: fix wind (check if negative sign is needed; frame generally confusing)
        wind = np.array([self.wind.twist.twist.linear.x, self.wind.twist.twist.linear.y])
        # TODO: fix airspeed (only correct in wind frame)
        airspeed = self.velocity.twist.linear.x
        # TODO: Make this asynchronous, ideally
        res = self.controller.update(s, wind, airspeed, X)

        trajectory_plan = TrajectoryPlan()
        trajectory_plan.norths = res.solX[0::7]
        trajectory_plan.easts = res.solX[1::7]
        trajectory_plan.headings = res.solX[2::7]
        trajectory_plan.bank_angles = res.solX[3::7]
        self.trajectory_pub.publish(trajectory_plan)

        self.cmd_roll(res.solX[3])


def main(args=None):
    config_file = "controller_config.toml"
    config = toml.load(config_file)
    rclpy.init(args=args)
    node = MPCCGuidedNode(config)
    rclpy.spin(node)
    # once spin ends
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
