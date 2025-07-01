import traceback
import rclpy
from rclpy.node import Node, Timer, Subscription
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterValue, ParameterType
from geometry_msgs.msg import PoseStamped, TwistStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import State, AttitudeTarget, StatusText, Waypoint, CommandCode
from mavros_msgs.srv import (
    SetMode,
    WaypointClear,
    WaypointPush,
    MessageInterval,
    ParamPush,
    ParamSetV2,
)
from mavros.system import SENSOR_QOS
from mavros.command import CommandBool, CommandTOL, CommandInt
from mpcc_interfaces.srv import SetPath, GetPath
from mpcc_interfaces.msg import Status, TrajectoryPlan, SystemID
from mpcc_interfaces.msg import State as State_mpcc_msg
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt32, Float64
from std_srvs.srv import SetBool
from mpcc.ros.state_machine import StateMachine, SMState
from mpcc.pydubins import DubinsPath
from mpcc.serialization_schema import to_path, to_dubins, path_adapter
from mpcc.quaternion import quaternion_to_RPY
from mpcc.ros.sysid import SystemIdController
import math
import numpy as np
from mpcc.controller import MPCCController
import toml
import asyncio
from mpcc.projection import geodetic_to_enu, enu_to_geodetic, Enu, Cartographic


class MPCCGuidedNode(Node):
    dubins: DubinsPath | None = None
    origin: GeoPoint | None = None
    pose: PoseStamped | None = None
    velocity: TwistStamped | None = None
    gps_fix: NavSatFix | None = None
    satellites: int | None = None
    arm_ready: bool = False
    controller: MPCCController
    heading: Float64 | None = None
    phi_ref: float | None = None
    wind: TwistWithCovarianceStamped | None
    altitude: float | None = None
    arclength: float = 0.0
    mpcc_timer: Timer | None = None
    cb_group: ReentrantCallbackGroup
    do_system_id: bool = False
    system_id_timer: Timer | None = None
    system_id_controller: SystemIdController | None = None
    state_sub: Subscription
    pose_sub: Subscription
    velocity_sub: Subscription
    gps_fix_sub: Subscription
    satellites_sub: Subscription
    wind_sub: Subscription
    heading_sub: Subscription

    def __init__(self, mpcc_config: dict[str, any]):
        super().__init__("mpcc_guided_py")
        cb_group = ReentrantCallbackGroup()
        self.cb_group = cb_group

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

        for o, t, name, cb, qos in [
            ("state_sub", State, "/mavros/state", self.recv_state, state_qos),
            ("pose_sub", PoseStamped, "/mavros/local_position/pose", self.recv_pose, SENSOR_QOS,),
            ("velocity_sub", TwistStamped, "/mavros/local_position/velocity_body", self.recv_velocity, SENSOR_QOS,),
            ("gps_fix_sub", NavSatFix, "/mavros/global_position/global", self.recv_gps_fix, SENSOR_QOS,),
            ("satellites_sub", UInt32, "/mavros/global_position/raw/satellites", self.recv_satellites, SENSOR_QOS,),
            ("heading_sub", Float64, "/mavros/global_position/compass_hdg", self.recv_heading, SENSOR_QOS,),
            ("wind_sub", TwistWithCovarianceStamped, "/mavros/wind_estimation", self.recv_wind, SENSOR_QOS,)
        ]:
            setattr(self, o, self.create_subscription(t, name, cb, qos_profile = qos))


        # Publishers

        self.trajectory_pub = self.create_publisher(
            TrajectoryPlan, "gs/trajectory_plan", 1
        )

        self.attitude_pub = self.create_publisher(
            AttitudeTarget,
            "/mavros/setpoint_raw/attitude",
            qos_profile=state_qos,
        )

        self.system_id_pub = self.create_publisher(SystemID, "/gs/system_id", 1)

        # Clients

        # for o, t, name, cb, qos, cb_group in [
            
        # ]

        self.altitude_client = self.create_client(
            CommandInt,
            "/mavros/cmd/command_int",
            qos_profile=altitude_speed_qos,
            callback_group=cb_group,
        )

        self.velocity_client = self.create_client(
            CommandInt,
            "/mavros/cmd/command_int",
            qos_profile=altitude_speed_qos,
            callback_group=cb_group,
        )

        self.status_pub = self.create_publisher(Status, "/gs/status", 10)

        self.message_pub = self.create_publisher(StatusText, "/gs/statustext/recv", 10)

        self.arming_client = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=cb_group
        )

        self.set_mode_client = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cb_group
        )

        self.tol_client = self.create_client(
            CommandTOL, "/mavros/cmd/takeoff", callback_group=cb_group
        )

        self.mission_clear_client = self.create_client(
            WaypointClear, "/mavros/mission/clear", callback_group=cb_group
        )

        self.mission_push_client = self.create_client(
            WaypointPush, "/mavros/mission/push", callback_group=cb_group
        )

        self.message_interval_client = self.create_client(
            MessageInterval, "/mavros/set_message_interval", callback_group=cb_group
        )

        self.param_set_client = self.create_client(
            ParamSetV2, "/mavros/param/set", callback_group=cb_group
        )

        self.param_pull_client = self.create_client(
            ParamPush, "/mavros/param/push", callback_group=cb_group
        )

        # Services

        self.set_path_srv = self.create_service(
            SetPath, "/gs/set_path", callback=self.set_path, callback_group=cb_group
        )

        self.get_path_srv = self.create_service(
            GetPath, "/gs/get_path", callback=self.get_path, callback_group=cb_group
        )

        self.try_arm_srv = self.create_service(
            SetBool, "/gs/try_arm", callback=self.set_arm_ready, callback_group=cb_group
        )

        self.do_system_id_srv = self.create_service(
            SetBool,
            "/gs/do_system_id",
            callback=self.set_do_system_id,
            callback_group=cb_group,
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
        while not self.message_interval_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for message interval service")

        self.get_logger().info("Setting rates")
        for message_id, message_rate, message_name in [
            (2, 20, "SYS_TIME"),
            (30, 40, "ATTITUDE"),
            (31, 40, "ATTITUDE_QUATERNION"),
            (32, 40, "LOCAL_POSITION_NED"),
            (33, 40, "GLOBAL_POSITION_INT"),
            # (61, 20, "ATTITUDE_QUATERNION_COV"),
            (62, 10, "NAV_CONTROLLER_OUTPUT"),
            # (63, 10, "GLOBAL_POSITION_NED_COV"),
            # (64, 10, "LOCAL_POSITION_NED_COV"),
            (141, 10, "ALTITUDE"),
            # (231, 10, "WIND_COV"),
        ]:
            self.get_logger().info(
                f"Set {message_name} ({message_id})to rate {message_rate}"
            )
            req = MessageInterval.Request()
            req.message_id = message_id
            req.message_rate = float(message_rate)
            future = self.message_interval_client.call_async(req)
            future.message_name = message_name
            future.add_done_callback(
                lambda future: self.get_logger().info(
                    "Set " + future.message_name + f": {future.result().success}"
                )
            )

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
        if self.origin is not None and self.dubins is not None:
            pos_enu = self.ltp_Enu(msg)
            pos_curr = np.array([pos_enu.n, pos_enu.e])
            if self.gps_fix is not None:
                pos_prev_enu = self.ltp_Enu(self.gps_fix)
                pos_prev = np.array([pos_prev_enu.n, pos_prev_enu.e])
            else:
                pos_prev = pos_curr

            self.arclength = self.dubins.get_closest_arclength(
                pos_prev, pos_curr, self.arclength
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

    async def disarm_drone(self):
        req = CommandBool.Request()
        req.value = True
        await self.arming_client.call_async(req)
        return self.set_manual()

    def _set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        return self.set_mode_client.call_async(req)

    def set_auto(self):
        return self._set_mode("AUTO")

    def set_guided(self):
        return self._set_mode("GUIDED")

    def set_manual(self):
        return self._set_mode("MANUAL")

    def set_takeoff(self):
        return self._set_mode("TAKEOFF")

    def takeoff(self):
        req = CommandTOL.Request()
        req.altitude = 20.0
        return self.tol_client.call_async(req)

    def cmd_altitude(self, altitude: float, altitude_rate: float):
        req = CommandInt.Request()
        req.command = 43001
        req.z = altitude
        req.param3 = altitude_rate
        return self.altitude_client.call_async(req)

    def cmd_speed(self, speed: float, speed_rate: float):
        req = CommandInt.Request()
        req.command = 43000
        req.param1 = 0.0  # airspeed, not ground speed
        req.param2 = speed
        req.param3 = speed_rate
        return self.velocity_client.call_async(req)

    def cmd_roll(self, roll_angle_rad: float):
        req = AttitudeTarget()
        req.type_mask = 126
        # just rotation about the x-axis, but mavlink rotates everything 180 deg...?
        req.orientation.w = math.cos(roll_angle_rad / 2)
        req.orientation.x = math.sin(roll_angle_rad / 2)
        self.attitude_pub.publish(req)
        return None

    async def set_path(self, req: SetPath.Request, res: SetPath.Response):
        curr_state = self.state_machine.state
        # Cannot change path after arming the vehicle (!)
        if curr_state != SMState.CONNECTED:
            res.success = False
            res.error_msg = f"Cannot set state in mode {curr_state.name}"
            return res
        try:
            self.dubins = to_dubins(
                path_adapter.validate_json(req.newpath.newpath, strict=True)
            )[0]
            self.origin = req.newpath.origin
            self.get_logger().info(
                f"Origin: lat {self.origin.latitude}, lon {self.origin.longitude}"
            )
            # TODO: self.altitude = req.newpath.altitude
            res.success = True
            res.error_msg = ""
            # self.controller.set_path(self.dubins)

        except Exception as e:
            # atomic transaction
            self.dubins = None
            self.origin = None
            res.success = False
            res.error_msg = traceback.format_exc()
            return res

        # new waypoint list evenly spaced along path (50m apart)
        path_length = self.dubins.length()
        new_waypoints = np.arange(0, path_length, 50)
        waypoint_locs_NE = [self.dubins.eval(waypoint) for waypoint in new_waypoints]
        waypoint_locs_latlon = [
            enu_to_geodetic(
                Enu(e=waypoint[1], n=waypoint[0], u=100.0),  # TODO: fixup
                origin=Cartographic(
                    latitude=self.origin.latitude,
                    longitude=self.origin.longitude,
                    altitude=self.origin.altitude,
                ),
            )
            for waypoint in waypoint_locs_NE
        ]
        self.get_logger().info(str(waypoint_locs_latlon))
        waypoint_list = [
            Waypoint(
                frame=Waypoint.FRAME_GLOBAL_REL_ALT,
                command=CommandCode.NAV_WAYPOINT,
                is_current=False,
                autocontinue=True,
                param2=20.0,  # accept radius
                param3=0.0,  # pass through waypoint
                x_lat=waypoint.latitude,
                y_long=waypoint.longitude,
                z_alt=40.0,  # TODO: fixup
            )
            for (waypoint) in waypoint_locs_latlon
        ]
        waypoint_list.insert(
            0,
            Waypoint(
                command=CommandCode.NAV_TAKEOFF,
                is_current=True,
                autocontinue=True,
                param1=10.0,  # minimum pitch, deg
                z_alt=40.0,
            ),
        )
        waypoint_list[-1].command = CommandCode.NAV_LAND
        waypoint_list[-1].z_alt = 0.0
        self.get_logger().info(repr(waypoint_list))

        wp_clear_req = WaypointClear.Request()
        wp_set_req = WaypointPush.Request()
        wp_set_req.waypoints = waypoint_list

        self.get_logger().info("Clearing preexisting waypoints")

        future = self.mission_clear_client.call_async(wp_clear_req)
        res: WaypointClear.Response = await future

        if not res.success:
            error_msg = "Failed to clear waypoints"
            self.get_logger().error(error_msg)
            # TODO: Handle bad error case (retry)
            setpath_res = SetPath.Response()
            setpath_res.success = False
            setpath_res.error_msg = error_msg
            return setpath_res
        self.get_logger().info("Waypoints cleared; setting new waypoint list")

        future = self.mission_push_client.call_async(wp_set_req)

        res: WaypointPush.Response = await future
        if not res.success or res.wp_transfered != len(waypoint_list):
            error_msg = f"Waypoints not set successfully. Successful: {res.success}; {res.wp_transfered} / {len(waypoint_list)} waypoints transferred"
            self.get_logger().error(error_msg)
            # TODO: Handle error case
            setpath_res = SetPath.Response()
            setpath_res.success = False
            setpath_res.error_msg = error_msg
            return setpath_res
        self.get_logger().info("Waypoints set successfully")

        setpath_res = SetPath.Response()
        setpath_res.success = True
        setpath_res.error_msg = ""
        return setpath_res

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
            self.arm_ready and self.satellites is not None and self.satellites.data > 3
            # other checks...?
        )

    def takeoff_altitude_reached(self) -> bool:
        return (
            self.gps_fix is not None and self.gps_fix.altitude > 130  # TODO: fixup
        )

    def mpcc_ready(self) -> bool:
        if self.gps_fix is None or self.dubins is None:
            return False
        local_coords = self.ltp_Enu(self.gps_fix)
        p = np.array([local_coords.n, local_coords.e])
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
        return geodetic_to_enu(
            geodetic=Cartographic(
                latitude=gps_fix.latitude,
                longitude=gps_fix.longitude,
                altitude=gps_fix.altitude,
            ),
            origin=Cartographic(
                latitude=self.origin.latitude,
                longitude=self.origin.longitude,
                altitude=self.origin.altitude,
            ),
        )

    def start_mpcc(self):
        self.get_logger().info("Starting MPCC")
        # _res = await self.set_guided()
        self.mpcc_timer = self.create_timer(
            self.controller.config["dt"], self.mpcc_update
        )

    def stop_mpcc(self):
        if self.mpcc_timer is not None:
            self.mpcc_timer.destroy()
        return self.set_auto()

    def mpcc_update(self):
        "Run MPCC loop"
        local_coords = self.ltp_Enu(self.gps_fix)
        orientation = self.pose.pose.orientation
        rpy = quaternion_to_RPY(orientation)
        # get full current state vector
        # north, east, xi, phi, dphi, phi_ref, s
        north = local_coords.n
        east = local_coords.e
        xi = np.radians(self.heading.data)
        phi = rpy["roll"]
        dphi = self.velocity.twist.angular.x
        phi_ref = self.phi_ref if self.phi_ref is not None else 0.0
        s = self.arclength
        X = np.array([north, east, xi, phi, dphi, phi_ref, s], dtype=np.float64)
        # TODO: fix wind (check if negative sign is needed; frame generally confusing)
        wind = np.array(
            [self.wind.twist.twist.linear.x, self.wind.twist.twist.linear.y]
        )
        # TODO: fix airspeed (only correct in wind frame)
        airspeed = self.velocity.twist.linear.x
        # TODO: Make this asynchronous, ideally
        res = self.controller.update(s, wind, airspeed, X)
        trajectory_plan = TrajectoryPlan()
        trajectory_plan.norths = res.solX[0::7].tolist()
        trajectory_plan.easts = res.solX[1::7].tolist()
        trajectory_plan.headings = res.solX[2::7].tolist()
        trajectory_plan.bank_angles = res.solX[3::7].tolist()
        self.trajectory_pub.publish(trajectory_plan)

        target_bank = phi_ref + res.solU[0]

        self.get_logger().info(
            f"Updating MPCC with target bank angle {np.degrees(target_bank):.2f} deg (current: {np.degrees(phi):.2f} deg); arclength {s:.2f}; heading {xi:.2f}"
        )
        self.cmd_roll(target_bank)

        self.controller.prepare()

    def set_do_system_id(self, req: SetBool.Request, res: SetBool.Response):
        self.do_system_id = req.data
        res.success = True
        res.message = (
            "Will do system identification"
            if req.data
            else "Will stop system identification"
        )
        return res

    # System identification functions
    def system_id_ready(self):
        "Ready to commence system identification from mode SET_AUTO"
        return self.do_system_id

    def system_id_finished(self):
        "Ready to end system identification"
        return not self.do_system_id

    def finish_system_id(self):
        self.system_id_timer.destroy()
        self.system_id_timer = None
        return self.set_auto()

    def start_system_id(self):
        "Perform 2-1-1 system identification"
        future = self.set_guided()
        future.add_done_callback(
            lambda future: self.get_logger().info(
                f"Set mode guided: {future.result().mode_sent}"
            )
        )
        self.system_id_controller = SystemIdController(t_start=self.get_clock().now())
        self.system_id_timer = self.create_timer(
            0.05, self.system_id_update, callback_group=self.cb_group
        )

    def system_id_update(self):
        "Timer callback for syste identification"
        commanded_angle = self.system_id_controller.update(self.get_clock().now())
        self.cmd_roll(commanded_angle)
        # publish information
        sysid = SystemID()
        sysid.commanded_roll = commanded_angle
        orientation = self.pose.pose.orientation
        rpy = quaternion_to_RPY(orientation)
        sysid.roll_angle = rpy["roll"]
        self.system_id_pub.publish(sysid)

    def terminate(self):
        req = ParamSetV2.Request()
        req.force_set = True
        req.param_id = "AFS_TERMINATE"
        req.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=1
        )
        return self.param_set_client.call_async(req)


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
