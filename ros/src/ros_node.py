import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros.command import CommandBool, CommandTOL, CommandLong, CommandInt
from mavros import mavlink

# CommandLong used for attitude, CommandInt used for Speed and Altitude

MODE_MANUAL = 0
MODE_AUTO = 10
MODE_TAKEOFF = 13
MODE_GUIDED = 15


class MPCCGuidedNode(Node):
    def __init__(self):
        super().__init__('mpcc_guided_py')

        # Low-latency QoS profile
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # high refresh rate commands
        attitude_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # low refresh rate commands
        altitude_speed_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # flight termination commands (must be reliable)
        fts_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
        )

        self.current_state=State()
        print(self.current_state)

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.read_state_callback, state_qos
        )

        self.attitude_pub = self.create_client(
            CommandLong, '/mavros/cmd/command', qos_profile=state_qos
        )

        self.altitude_pub = self.create_client(
            CommandInt, '/mavros/cmd/command_int', qos_profile=altitude_speed_qos
        )

        self.velocity_pub = self.create_client(
            CommandInt, '/mavros/cmd/command_int', qos_profile=altitude_speed_qos
        )

        self.diagnostics = self.create_publisher(
            String, 'gs/diagnostics', 10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.tol_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mode set service')
        while not self.tol_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for takeoff landing service')

        self.state_machine()
        #self.mpcc_timer = self.create_timer(0.1, self.mpcc_timer_callback)
        self.last_req = self.get_clock().now()

    def read_state_callback(self, msg):
        self.current_state = msg

    def state_machine(self):
        

    
    def mpcc_timer_callback(self):
        # command to current MPCC path
        # for now we'll just arm the plane
        current_time = self.get_clock().now()

        if (not self.current_state.armed and 
            (current_time - self.last_req).nanoseconds > 1e9):
            print("Arming")
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            
            self.last_req = current_time
        
        if (self.current_state.armed and self.current_state.mode == "MANUAL"):
            print("Current mode", self.current_state.mode)
            print("Set mode AUTO")
            req = SetMode.Request()
            req.custom_mode = "AUTO"
            future = self.set_mode_client.call_async(req)
            self.last_req = current_time
        

        if (self.current_state.armed and self.current_state.mode == "AUTO"):
            print("Takeoff")
            req = CommandTOL.Request()
            req.altitude = 20.0
            future = self.tol_client.call_async(req)
            self.last_req = current_time




def main(args=None):
    rclpy.init(args=args)
    node = MPCCGuidedNode()

    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()







