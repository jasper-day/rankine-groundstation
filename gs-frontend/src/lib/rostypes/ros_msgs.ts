/* eslint-disable */
// These files were generated using "ros-typescript-generator"
export namespace GeographicMsgs {
  export interface BoundingBox {
    min_pt: GeographicMsgs.GeoPoint;
    max_pt: GeographicMsgs.GeoPoint;
  }

  export interface GeoPath {
    header: StdMsgs.Header;
    poses: GeographicMsgs.GeoPoseStamped[];
  }

  export interface GeoPoint {
    latitude: number;
    longitude: number;
    altitude: number;
  }

  export interface GeoPointStamped {
    header: StdMsgs.Header;
    position: GeographicMsgs.GeoPoint;
  }

  export interface GeoPose {
    position: GeographicMsgs.GeoPoint;
    orientation: GeometryMsgs.Quaternion;
  }

  export interface GeoPoseStamped {
    header: StdMsgs.Header;
    pose: GeographicMsgs.GeoPose;
  }

  export interface GeoPoseWithCovariance {
    pose: GeographicMsgs.GeoPose;
    covariance: number[];
  }

  export interface GeoPoseWithCovarianceStamped {
    header: StdMsgs.Header;
    pose: GeographicMsgs.GeoPoseWithCovariance;
  }

  export interface GeographicMap {
    header: StdMsgs.Header;
    id: UniqueIdentifierMsgs.Uuid;
    bounds: GeographicMsgs.BoundingBox;
    points: GeographicMsgs.WayPoint[];
    features: GeographicMsgs.MapFeature[];
    props: GeographicMsgs.KeyValue[];
  }

  export interface GeographicMapChanges {
    header: StdMsgs.Header;
    diffs: GeographicMsgs.GeographicMap;
    deletes: UniqueIdentifierMsgs.Uuid[];
  }

  export interface GetGeoPath {
    request: GeographicMsgs.GetGeoPathRequest;
    response: GeographicMsgs.GetGeoPathResponse;
  }

  export interface GetGeoPathRequest {
    start: GeographicMsgs.GeoPoint;
    goal: GeographicMsgs.GeoPoint;
  }

  export interface GetGeoPathResponse {
    success: boolean;
    status: string;
    plan: GeographicMsgs.GeoPath;
    network: UniqueIdentifierMsgs.Uuid;
    start_seg: UniqueIdentifierMsgs.Uuid;
    goal_seg: UniqueIdentifierMsgs.Uuid;
    distance: number;
  }

  export interface GetGeographicMap {
    request: GeographicMsgs.GetGeographicMapRequest;
    response: GeographicMsgs.GetGeographicMapResponse;
  }

  export interface GetGeographicMapRequest {
    url: string;
    bounds: GeographicMsgs.BoundingBox;
  }

  export interface GetGeographicMapResponse {
    success: boolean;
    status: string;
    map: GeographicMsgs.GeographicMap;
  }

  export interface GetRoutePlan {
    request: GeographicMsgs.GetRoutePlanRequest;
    response: GeographicMsgs.GetRoutePlanResponse;
  }

  export interface GetRoutePlanRequest {
    network: UniqueIdentifierMsgs.Uuid;
    start: UniqueIdentifierMsgs.Uuid;
    goal: UniqueIdentifierMsgs.Uuid;
  }

  export interface GetRoutePlanResponse {
    success: boolean;
    status: string;
    plan: GeographicMsgs.RoutePath;
  }

  export interface KeyValue {
    key: string;
    value: string;
  }

  export interface MapFeature {
    id: UniqueIdentifierMsgs.Uuid;
    components: UniqueIdentifierMsgs.Uuid[];
    props: GeographicMsgs.KeyValue[];
  }

  export interface RouteNetwork {
    header: StdMsgs.Header;
    id: UniqueIdentifierMsgs.Uuid;
    bounds: GeographicMsgs.BoundingBox;
    points: GeographicMsgs.WayPoint[];
    segments: GeographicMsgs.RouteSegment[];
    props: GeographicMsgs.KeyValue[];
  }

  export interface RoutePath {
    header: StdMsgs.Header;
    network: UniqueIdentifierMsgs.Uuid;
    segments: UniqueIdentifierMsgs.Uuid[];
    props: GeographicMsgs.KeyValue[];
  }

  export interface RouteSegment {
    id: UniqueIdentifierMsgs.Uuid;
    start: UniqueIdentifierMsgs.Uuid;
    end: UniqueIdentifierMsgs.Uuid;
    props: GeographicMsgs.KeyValue[];
  }

  export interface UpdateGeographicMap {
    request: GeographicMsgs.UpdateGeographicMapRequest;
    response: GeographicMsgs.UpdateGeographicMapResponse;
  }

  export interface UpdateGeographicMapRequest {
    updates: GeographicMsgs.GeographicMapChanges;
  }

  export interface UpdateGeographicMapResponse {
    success: boolean;
    status: string;
  }

  export interface WayPoint {
    id: UniqueIdentifierMsgs.Uuid;
    position: GeographicMsgs.GeoPoint;
    props: GeographicMsgs.KeyValue[];
  }
}

export namespace GeometryMsgs {
  export interface Accel {
    linear: GeometryMsgs.Vector3;
    angular: GeometryMsgs.Vector3;
  }

  export interface AccelStamped {
    header: StdMsgs.Header;
    accel: GeometryMsgs.Accel;
  }

  export interface AccelWithCovariance {
    accel: GeometryMsgs.Accel;
    covariance: number[];
  }

  export interface AccelWithCovarianceStamped {
    header: StdMsgs.Header;
    accel: GeometryMsgs.AccelWithCovariance;
  }

  export interface Inertia {
    m: number;
    com: GeometryMsgs.Vector3;
    ixx: number;
    ixy: number;
    ixz: number;
    iyy: number;
    iyz: number;
    izz: number;
  }

  export interface InertiaStamped {
    header: StdMsgs.Header;
    inertia: GeometryMsgs.Inertia;
  }

  export interface Point {
    x: number;
    y: number;
    z: number;
  }

  export interface Point32 {
    x: number;
    y: number;
    z: number;
  }

  export interface PointStamped {
    header: StdMsgs.Header;
    point: GeometryMsgs.Point;
  }

  export interface Polygon {
    points: GeometryMsgs.Point32[];
  }

  export interface PolygonStamped {
    header: StdMsgs.Header;
    polygon: GeometryMsgs.Polygon;
  }

  export interface Pose {
    position: GeometryMsgs.Point;
    orientation: GeometryMsgs.Quaternion;
  }

  export interface Pose2D {
    x: number;
    y: number;
    theta: number;
  }

  export interface PoseArray {
    header: StdMsgs.Header;
    poses: GeometryMsgs.Pose[];
  }

  export interface PoseStamped {
    header: StdMsgs.Header;
    pose: GeometryMsgs.Pose;
  }

  export interface PoseWithCovariance {
    pose: GeometryMsgs.Pose;
    covariance: number[];
  }

  export interface PoseWithCovarianceStamped {
    header: StdMsgs.Header;
    pose: GeometryMsgs.PoseWithCovariance;
  }

  export interface Quaternion {
    x: number;
    y: number;
    z: number;
    w: number;
  }

  export interface QuaternionStamped {
    header: StdMsgs.Header;
    quaternion: GeometryMsgs.Quaternion;
  }

  export interface Transform {
    translation: GeometryMsgs.Vector3;
    rotation: GeometryMsgs.Quaternion;
  }

  export interface TransformStamped {
    header: StdMsgs.Header;
    child_frame_id: string;
    transform: GeometryMsgs.Transform;
  }

  export interface Twist {
    linear: GeometryMsgs.Vector3;
    angular: GeometryMsgs.Vector3;
  }

  export interface TwistStamped {
    header: StdMsgs.Header;
    twist: GeometryMsgs.Twist;
  }

  export interface TwistWithCovariance {
    twist: GeometryMsgs.Twist;
    covariance: number[];
  }

  export interface TwistWithCovarianceStamped {
    header: StdMsgs.Header;
    twist: GeometryMsgs.TwistWithCovariance;
  }

  export interface Vector3 {
    x: number;
    y: number;
    z: number;
  }

  export interface Vector3Stamped {
    header: StdMsgs.Header;
    vector: GeometryMsgs.Vector3;
  }

  export interface VelocityStamped {
    header: StdMsgs.Header;
    body_frame_id: string;
    reference_frame_id: string;
    velocity: GeometryMsgs.Twist;
  }

  export interface Wrench {
    force: GeometryMsgs.Vector3;
    torque: GeometryMsgs.Vector3;
  }

  export interface WrenchStamped {
    header: StdMsgs.Header;
    wrench: GeometryMsgs.Wrench;
  }
}

export namespace MavrosMsgs {
  export enum CommandCodeConst {
    AIRFRAME_CONFIGURATION = 2520,
    ARM_AUTHORIZATION_REQUEST = 3001,
    CAMERA_TRACK_POINT = 2004,
    CAMERA_TRACK_RECTANGLE = 2005,
    CAMERA_STOP_TRACKING = 2010,
    CAN_FORWARD = 32000,
    COMPONENT_ARM_DISARM = 400,
    CONDITION_DELAY = 112,
    CONDITION_CHANGE_ALT = 113,
    CONDITION_DISTANCE = 114,
    CONDITION_YAW = 115,
    CONDITION_LAST = 159,
    CONTROL_HIGH_LATENCY = 2600,
    DO_FOLLOW = 32,
    DO_FOLLOW_REPOSITION = 33,
    DO_SET_MODE = 176,
    DO_JUMP = 177,
    DO_CHANGE_SPEED = 178,
    DO_SET_HOME = 179,
    DO_SET_PARAMETER = 180,
    DO_SET_RELAY = 181,
    DO_REPEAT_RELAY = 182,
    DO_SET_SERVO = 183,
    DO_REPEAT_SERVO = 184,
    DO_FLIGHTTERMINATION = 185,
    DO_CHANGE_ALTITUDE = 186,
    DO_RETURN_PATH_START = 188,
    DO_LAND_START = 189,
    DO_RALLY_LAND = 190,
    DO_GO_AROUND = 191,
    DO_REPOSITION = 192,
    DO_PAUSE_CONTINUE = 193,
    DO_SET_REVERSE = 194,
    DO_SET_ROI_LOCATION = 195,
    DO_SET_ROI_WPNEXT_OFFSET = 196,
    DO_SET_ROI_NONE = 197,
    DO_SET_ROI_SYSID = 198,
    DO_CONTROL_VIDEO = 200,
    DO_SET_ROI = 201,
    DO_DIGICAM_CONFIGURE = 202,
    DO_DIGICAM_CONTROL = 203,
    DO_MOUNT_CONFIGURE = 204,
    DO_MOUNT_CONTROL = 205,
    DO_SET_CAM_TRIGG_DIST = 206,
    DO_FENCE_ENABLE = 207,
    DO_PARACHUTE = 208,
    DO_MOTOR_TEST = 209,
    DO_INVERTED_FLIGHT = 210,
    DO_GRIPPER = 211,
    DO_AUTOTUNE_ENABLE = 212,
    DO_SET_CAM_TRIGG_INTERVAL = 214,
    DO_MOUNT_CONTROL_QUAT = 220,
    DO_GUIDED_MASTER = 221,
    DO_GUIDED_LIMITS = 222,
    DO_ENGINE_CONTROL = 223,
    DO_SET_MISSION_CURRENT = 224,
    DO_LAST = 240,
    DO_JUMP_TAG = 601,
    DO_GIMBAL_MANAGER_PITCHYAW = 1000,
    DO_GIMBAL_MANAGER_CONFIGURE = 1001,
    DO_TRIGGER_CONTROL = 2003,
    DO_VTOL_TRANSITION = 3000,
    DO_SET_SAFETY_SWITCH_STATE = 5300,
    DO_ADSB_OUT_IDENT = 10001,
    DO_WINCH = 42600,
    FIXED_MAG_CAL_YAW = 42006,
    GET_HOME_POSITION = 410,
    GET_MESSAGE_INTERVAL = 510,
    IMAGE_START_CAPTURE = 2000,
    IMAGE_STOP_CAPTURE = 2001,
    JUMP_TAG = 600,
    LOGGING_START = 2510,
    LOGGING_STOP = 2511,
    MISSION_START = 300,
    NAV_WAYPOINT = 16,
    NAV_LOITER_UNLIM = 17,
    NAV_LOITER_TURNS = 18,
    NAV_LOITER_TIME = 19,
    NAV_RETURN_TO_LAUNCH = 20,
    NAV_LAND = 21,
    NAV_TAKEOFF = 22,
    NAV_LAND_LOCAL = 23,
    NAV_TAKEOFF_LOCAL = 24,
    NAV_FOLLOW = 25,
    NAV_CONTINUE_AND_CHANGE_ALT = 30,
    NAV_LOITER_TO_ALT = 31,
    NAV_ROI = 80,
    NAV_PATHPLANNING = 81,
    NAV_SPLINE_WAYPOINT = 82,
    NAV_VTOL_TAKEOFF = 84,
    NAV_VTOL_LAND = 85,
    NAV_GUIDED_ENABLE = 92,
    NAV_DELAY = 93,
    NAV_PAYLOAD_PLACE = 94,
    NAV_LAST = 95,
    NAV_SET_YAW_SPEED = 213,
    NAV_FENCE_RETURN_POINT = 5000,
    NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
    NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
    NAV_FENCE_CIRCLE_INCLUSION = 5003,
    NAV_FENCE_CIRCLE_EXCLUSION = 5004,
    NAV_RALLY_POINT = 5100,
    OBLIQUE_SURVEY = 260,
    OVERRIDE_GOTO = 252,
    PANORAMA_CREATE = 2800,
    PAYLOAD_PREPARE_DEPLOY = 30001,
    PAYLOAD_CONTROL_DEPLOY = 30002,
    PREFLIGHT_CALIBRATION = 241,
    PREFLIGHT_SET_SENSOR_OFFSETS = 242,
    PREFLIGHT_UAVCAN = 243,
    PREFLIGHT_STORAGE = 245,
    PREFLIGHT_REBOOT_SHUTDOWN = 246,
    REQUEST_MESSAGE = 512,
    REQUEST_PROTOCOL_VERSION = 519,
    REQUEST_AUTOPILOT_CAPABILITIES = 520,
    REQUEST_CAMERA_INFORMATION = 521,
    REQUEST_CAMERA_SETTINGS = 522,
    REQUEST_STORAGE_INFORMATION = 525,
    REQUEST_CAMERA_CAPTURE_STATUS = 527,
    REQUEST_FLIGHT_INFORMATION = 528,
    REQUEST_VIDEO_STREAM_INFORMATION = 2504,
    REQUEST_VIDEO_STREAM_STATUS = 2505,
    RESET_CAMERA_SETTINGS = 529,
    RUN_PREARM_CHECKS = 401,
    SET_MESSAGE_INTERVAL = 511,
    SET_CAMERA_MODE = 530,
    SET_CAMERA_ZOOM = 531,
    SET_CAMERA_FOCUS = 532,
    SET_STORAGE_USAGE = 533,
    SET_CAMERA_SOURCE = 534,
    SET_GUIDED_SUBMODE_STANDARD = 4000,
    SET_GUIDED_SUBMODE_CIRCLE = 4001,
    START_RX_PAIR = 500,
    STORAGE_FORMAT = 526,
    UAVCAN_GET_NODE_INFO = 5200,
    VIDEO_START_CAPTURE = 2500,
    VIDEO_STOP_CAPTURE = 2501,
    VIDEO_START_STREAMING = 2502,
    VIDEO_STOP_STREAMING = 2503,
  }

  export interface ActuatorControl {
    header: StdMsgs.Header;
    group_mix: ActuatorControlGroupMix;
    controls: number[];
  }
  
  export enum ActuatorControlGroupMix {
    PX4_MIX_FLIGHT_CONTROL = 0,
    PX4_MIX_FLIGHT_CONTROL_VTOL_ALT = 1,
    PX4_MIX_PAYLOAD = 2,
    PX4_MIX_MANUAL_PASSTHROUGH = 3,
  }

  export interface AdsbVehicle {
    header: StdMsgs.Header;
    icao_address: number;
    callsign: string;
    latitude: number;
    longitude: number;
    altitude: number;
    heading: number;
    hor_velocity: number;
    ver_velocity: number;
    altitude_type: number;
    emitter_type: number;
    tslc: { sec: number, nanosec: number };
    flags: number;
    squawk: number;
  }
  
  export enum AdsbVehicleConst {
    ALT_PRESSURE_QNH = 0,
    ALT_GEOMETRIC = 1,
    EMITTER_NO_INFO = 0,
    EMITTER_LIGHT = 1,
    EMITTER_SMALL = 2,
    EMITTER_LARGE = 3,
    EMITTER_HIGH_VORTEX_LARGE = 4,
    EMITTER_HEAVY = 5,
    EMITTER_HIGHLY_MANUV = 6,
    EMITTER_ROTOCRAFT = 7,
    EMITTER_UNASSIGNED = 8,
    EMITTER_GLIDER = 9,
    EMITTER_LIGHTER_AIR = 10,
    EMITTER_PARACHUTE = 11,
    EMITTER_ULTRA_LIGHT = 12,
    EMITTER_UNASSIGNED2 = 13,
    EMITTER_UAV = 14,
    EMITTER_SPACE = 15,
    EMITTER_UNASSGINED3 = 16,
    EMITTER_EMERGENCY_SURFACE = 17,
    EMITTER_SERVICE_SURFACE = 18,
    EMITTER_POINT_OBSTACLE = 19,
    FLAG_VALID_COORDS = 1,
    FLAG_VALID_ALTITUDE = 2,
    FLAG_VALID_HEADING = 4,
    FLAG_VALID_VELOCITY = 8,
    FLAG_VALID_CALLSIGN = 16,
    FLAG_VALID_SQUAWK = 32,
    FLAG_SIMULATED = 64,
    FLAG_VERTICAL_VELOCITY_VALID = 128,
    FLAG_BARO_VALID = 256,
    FLAG_SOURCE_UAT = 32768,
  }

  export interface Altitude {
    header: StdMsgs.Header;
    monotonic: number;
    amsl: number;
    local: number;
    relative: number;
    terrain: number;
    bottom_clearance: number;
  }

  export interface AttitudeTarget {
    header: StdMsgs.Header;
    type_mask: AttitudeTargetTypeMask;
    orientation: GeometryMsgs.Quaternion;
    body_rate: GeometryMsgs.Vector3;
    thrust: number;
  }
  
  export enum AttitudeTargetTypeMask {
    IGNORE_ROLL_RATE = 1,
    IGNORE_PITCH_RATE = 2,
    IGNORE_YAW_RATE = 4,
    IGNORE_THRUST = 64,
    IGNORE_ATTITUDE = 128,
  }

  export interface CamImuStamp {
    frame_stamp: { sec: number, nanosec: number };
    frame_seq_id: number;
  }

  export interface CameraImageCaptured {
    header: StdMsgs.Header;
    orientation: GeometryMsgs.Quaternion;
    geo: GeographicMsgs.GeoPoint;
    relative_alt: number;
    image_index: number;
    capture_result: number;
    file_url: string;
  }

  export interface CellularStatus {
    status: number;
    failure_reason: number;
    type: number;
    quality: number;
    mcc: number;
    mnc: number;
    lac: number;
  }

  export interface CommandAck {
    request: MavrosMsgs.CommandAckRequest;
    response: MavrosMsgs.CommandAckResponse;
  }

  export interface CommandAckRequest {
    command: number;
    result: number;
    progress: number;
    result_param2: number;
  }

  export interface CommandAckResponse {
    success: boolean;
    result: number;
  }

  export interface CommandBool {
    request: MavrosMsgs.CommandBoolRequest;
    response: MavrosMsgs.CommandBoolResponse;
  }

  export interface CommandBoolRequest {
    value: boolean;
  }

  export interface CommandBoolResponse {
    success: boolean;
    result: number;
  }

  export interface CommandHome {
    request: MavrosMsgs.CommandHomeRequest;
    response: MavrosMsgs.CommandHomeResponse;
  }

  export interface CommandHomeRequest {
    current_gps: boolean;
    yaw: number;
    latitude: number;
    longitude: number;
    altitude: number;
  }

  export interface CommandHomeResponse {
    success: boolean;
    result: number;
  }

  export interface CommandInt {
    request: MavrosMsgs.CommandIntRequest;
    response: MavrosMsgs.CommandIntResponse;
  }

  export interface CommandIntRequest {
    broadcast: boolean;
    frame: number;
    command: number;
    current: number;
    autocontinue: number;
    param1: number;
    param2: number;
    param3: number;
    param4: number;
    x: number;
    y: number;
    z: number;
  }

  export interface CommandIntResponse {
    success: boolean;
  }

  export interface CommandLong {
    request: MavrosMsgs.CommandLongRequest;
    response: MavrosMsgs.CommandLongResponse;
  }

  export interface CommandLongRequest {
    broadcast: boolean;
    command: number;
    confirmation: number;
    param1: number;
    param2: number;
    param3: number;
    param4: number;
    param5: number;
    param6: number;
    param7: number;
  }

  export interface CommandLongResponse {
    success: boolean;
    result: number;
  }

  export interface CommandTol {
    request: MavrosMsgs.CommandTolRequest;
    response: MavrosMsgs.CommandTolResponse;
  }

  export interface CommandTolLocal {
    request: MavrosMsgs.CommandTolLocalRequest;
    response: MavrosMsgs.CommandTolLocalResponse;
  }

  export interface CommandTolLocalRequest {
    min_pitch: number;
    offset: number;
    rate: number;
    yaw: number;
    position: GeometryMsgs.Vector3;
  }

  export interface CommandTolLocalResponse {
    success: boolean;
    result: number;
  }

  export interface CommandTolRequest {
    min_pitch: number;
    yaw: number;
    latitude: number;
    longitude: number;
    altitude: number;
  }

  export interface CommandTolResponse {
    success: boolean;
    result: number;
  }

  export interface CommandTriggerControl {
    request: MavrosMsgs.CommandTriggerControlRequest;
    response: MavrosMsgs.CommandTriggerControlResponse;
  }

  export interface CommandTriggerControlRequest {
    trigger_enable: boolean;
    sequence_reset: boolean;
    trigger_pause: boolean;
  }

  export interface CommandTriggerControlResponse {
    success: boolean;
    result: number;
  }

  export interface CommandTriggerInterval {
    request: MavrosMsgs.CommandTriggerIntervalRequest;
    response: MavrosMsgs.CommandTriggerIntervalResponse;
  }

  export interface CommandTriggerIntervalRequest {
    cycle_time: number;
    integration_time: number;
  }

  export interface CommandTriggerIntervalResponse {
    success: boolean;
    result: number;
  }

  export interface CommandVtolTransition {
    request: MavrosMsgs.CommandVtolTransitionRequest;
    response: MavrosMsgs.CommandVtolTransitionResponse;
  }

  export interface CommandVtolTransitionRequest {
    header: StdMsgs.Header;
    state: CommandVtolTransitionRequestState;
  }
  
  export enum CommandVtolTransitionRequestState {
    STATE_MC = 3,
    STATE_FW = 4,
  }

  export interface CommandVtolTransitionResponse {
    success: boolean;
    result: number;
  }

  export interface CompanionProcessStatus {
    header: StdMsgs.Header;
    state: number;
    component: number;
  }
  
  export enum CompanionProcessStatusConst {
    MAV_STATE_UNINIT = 0,
    MAV_STATE_BOOT = 1,
    MAV_STATE_CALIBRATING = 2,
    MAV_STATE_STANDBY = 3,
    MAV_STATE_ACTIVE = 4,
    MAV_STATE_CRITICAL = 5,
    MAV_STATE_EMERGENCY = 6,
    MAV_STATE_POWEROFF = 7,
    MAV_STATE_FLIGHT_TERMINATION = 8,
    MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196,
    MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197,
  }

  export interface DebugValue {
    header: StdMsgs.Header;
    index: number;
    array_id: number;
    name: string;
    value_float: number;
    value_int: number;
    data: number[];
    type: DebugValueType;
  }
  
  export enum DebugValueType {
    TYPE_DEBUG = 0,
    TYPE_DEBUG_VECT = 1,
    TYPE_DEBUG_FLOAT_ARRAY = 2,
    TYPE_NAMED_VALUE_FLOAT = 3,
    TYPE_NAMED_VALUE_INT = 4,
  }

  export interface EndpointAdd {
    request: MavrosMsgs.EndpointAddRequest;
    response: MavrosMsgs.EndpointAddResponse;
  }

  export interface EndpointAddRequest {
    url: string;
    type: EndpointAddRequestType;
  }
  
  export enum EndpointAddRequestType {
    TYPE_FCU = 0,
    TYPE_GCS = 1,
    TYPE_UAS = 2,
  }

  export interface EndpointAddResponse {
    successful: boolean;
    reason: string;
    id: number;
  }

  export interface EndpointDel {
    request: MavrosMsgs.EndpointDelRequest;
    response: MavrosMsgs.EndpointDelResponse;
  }

  export interface EndpointDelRequest {
    id: number;
    url: string;
    type: EndpointDelRequestType;
  }
  
  export enum EndpointDelRequestType {
    TYPE_FCU = 0,
    TYPE_GCS = 1,
    TYPE_UAS = 2,
  }

  export interface EndpointDelResponse {
    successful: boolean;
  }

  export interface EscInfo {
    header: StdMsgs.Header;
    counter: number;
    count: number;
    connection_type: number;
    info: number;
    esc_info: MavrosMsgs.EscInfoItem[];
  }

  export interface EscInfoItem {
    header: StdMsgs.Header;
    failure_flags: number;
    error_count: number;
    temperature: number;
  }

  export interface EscStatus {
    header: StdMsgs.Header;
    esc_status: MavrosMsgs.EscStatusItem[];
  }

  export interface EscStatusItem {
    header: StdMsgs.Header;
    rpm: number;
    voltage: number;
    current: number;
  }

  export interface EscTelemetry {
    header: StdMsgs.Header;
    esc_telemetry: MavrosMsgs.EscTelemetryItem[];
  }

  export interface EscTelemetryItem {
    header: StdMsgs.Header;
    temperature: number;
    voltage: number;
    current: number;
    totalcurrent: number;
    rpm: number;
    count: number;
  }

  export interface EstimatorStatus {
    header: StdMsgs.Header;
    attitude_status_flag: boolean;
    velocity_horiz_status_flag: boolean;
    velocity_vert_status_flag: boolean;
    pos_horiz_rel_status_flag: boolean;
    pos_horiz_abs_status_flag: boolean;
    pos_vert_abs_status_flag: boolean;
    pos_vert_agl_status_flag: boolean;
    const_pos_mode_status_flag: boolean;
    pred_pos_horiz_rel_status_flag: boolean;
    pred_pos_horiz_abs_status_flag: boolean;
    gps_glitch_status_flag: boolean;
    accel_error_status_flag: boolean;
  }

  export interface ExtendedState {
    header: StdMsgs.Header;
    vtol_state: ExtendedStateVtolState;
    landed_state: ExtendedStateLandedState;
  }
  
  export enum ExtendedStateVtolState {
    UNDEFINED = 0,
    TRANSITION_TO_FW = 1,
    TRANSITION_TO_MC = 2,
    MC = 3,
    FW = 4,
  }
  
  export enum ExtendedStateLandedState {
    UNDEFINED = 0,
    ON_GROUND = 1,
    IN_AIR = 2,
    TAKEOFF = 3,
    LANDING = 4,
  }

  export interface FileChecksum {
    request: MavrosMsgs.FileChecksumRequest;
    response: MavrosMsgs.FileChecksumResponse;
  }

  export interface FileChecksumRequest {
    file_path: string;
  }

  export interface FileChecksumResponse {
    crc32: number;
    success: boolean;
    r_errno: number;
  }

  export interface FileClose {
    request: MavrosMsgs.FileCloseRequest;
    response: MavrosMsgs.FileCloseResponse;
  }

  export interface FileCloseRequest {
    file_path: string;
  }

  export interface FileCloseResponse {
    success: boolean;
    r_errno: number;
  }

  export interface FileEntry {
    name: string;
    type: FileEntryType;
    size: number;
  }
  
  export enum FileEntryType {
    TYPE_FILE = 0,
    TYPE_DIRECTORY = 1,
  }

  export interface FileList {
    request: MavrosMsgs.FileListRequest;
    response: MavrosMsgs.FileListResponse;
  }

  export interface FileListRequest {
    dir_path: string;
  }

  export interface FileListResponse {
    list: MavrosMsgs.FileEntry[];
    success: boolean;
    r_errno: number;
  }

  export interface FileMakeDir {
    request: MavrosMsgs.FileMakeDirRequest;
    response: MavrosMsgs.FileMakeDirResponse;
  }

  export interface FileMakeDirRequest {
    dir_path: string;
  }

  export interface FileMakeDirResponse {
    success: boolean;
    r_errno: number;
  }

  export interface FileOpen {
    request: MavrosMsgs.FileOpenRequest;
    response: MavrosMsgs.FileOpenResponse;
  }

  export interface FileOpenRequest {
    file_path: string;
    mode: FileOpenRequestMode;
  }
  
  export enum FileOpenRequestMode {
    MODE_READ = 0,
    MODE_WRITE = 1,
    MODE_CREATE = 2,
  }

  export interface FileOpenResponse {
    size: number;
    success: boolean;
    r_errno: number;
  }

  export interface FileRead {
    request: MavrosMsgs.FileReadRequest;
    response: MavrosMsgs.FileReadResponse;
  }

  export interface FileReadRequest {
    file_path: string;
    offset: number;
    size: number;
  }

  export interface FileReadResponse {
    data: number[];
    success: boolean;
    r_errno: number;
  }

  export interface FileRemove {
    request: MavrosMsgs.FileRemoveRequest;
    response: MavrosMsgs.FileRemoveResponse;
  }

  export interface FileRemoveDir {
    request: MavrosMsgs.FileRemoveDirRequest;
    response: MavrosMsgs.FileRemoveDirResponse;
  }

  export interface FileRemoveDirRequest {
    dir_path: string;
  }

  export interface FileRemoveDirResponse {
    success: boolean;
    r_errno: number;
  }

  export interface FileRemoveRequest {
    file_path: string;
  }

  export interface FileRemoveResponse {
    success: boolean;
    r_errno: number;
  }

  export interface FileRename {
    request: MavrosMsgs.FileRenameRequest;
    response: MavrosMsgs.FileRenameResponse;
  }

  export interface FileRenameRequest {
    old_path: string;
    new_path: string;
  }

  export interface FileRenameResponse {
    success: boolean;
    r_errno: number;
  }

  export interface FileTruncate {
    request: MavrosMsgs.FileTruncateRequest;
    response: MavrosMsgs.FileTruncateResponse;
  }

  export interface FileTruncateRequest {
    file_path: string;
    length: number;
  }

  export interface FileTruncateResponse {
    success: boolean;
    r_errno: number;
  }

  export interface FileWrite {
    request: MavrosMsgs.FileWriteRequest;
    response: MavrosMsgs.FileWriteResponse;
  }

  export interface FileWriteRequest {
    file_path: string;
    offset: number;
    data: number[];
  }

  export interface FileWriteResponse {
    success: boolean;
    r_errno: number;
  }

  export interface GimbalDeviceAttitudeStatus {
    header: StdMsgs.Header;
    target_system: number;
    target_component: number;
    flags: GimbalDeviceAttitudeStatusFlags;
    q: GeometryMsgs.Quaternion;
    angular_velocity_x: number;
    angular_velocity_y: number;
    angular_velocity_z: number;
    failure_flags: GimbalDeviceAttitudeStatusFailureFlags;
  }
  
  export enum GimbalDeviceAttitudeStatusFlags {
    FLAGS_RETRACT = 1,
    FLAGS_NEUTRAL = 2,
    FLAGS_ROLL_LOCK = 4,
    FLAGS_PITCH_LOCK = 8,
    FLAGS_YAW_LOCK = 16,
  }
  
  export enum GimbalDeviceAttitudeStatusFailureFlags {
    ERROR_FLAGS_AT_ROLL_LIMIT = 1,
    ERROR_FLAGS_AT_PITCH_LIMIT = 2,
    ERROR_FLAGS_AT_YAW_LIMIT = 4,
    ERROR_FLAGS_ENCODER_ERROR = 8,
    ERROR_FLAGS_POWER_ERROR = 16,
    ERROR_FLAGS_MOTOR_ERROR = 32,
    ERROR_FLAGS_SOFTWARE_ERROR = 64,
    ERROR_FLAGS_COMMS_ERROR = 128,
    ERROR_FLAGS_CALIBRATION_RUNNING = 256,
  }

  export interface GimbalDeviceInformation {
    header: StdMsgs.Header;
    vendor_name: string;
    model_name: string;
    custom_name: string;
    firmware_version: number;
    hardware_version: number;
    uid: number;
    cap_flags: GimbalDeviceInformationCapFlags;
    custom_cap_flags: number;
    roll_min: number;
    roll_max: number;
    pitch_min: number;
    pitch_max: number;
    yaw_min: number;
    yaw_max: number;
  }
  
  export enum GimbalDeviceInformationCapFlags {
    HAS_RETRACT = 1,
    HAS_NEUTRAL = 2,
    HAS_ROLL_AXIS = 4,
    HAS_ROLL_FOLLOW = 8,
    HAS_ROLL_LOCK = 16,
    HAS_PITCH_AXIS = 32,
    HAS_PITCH_FOLLOW = 64,
    HAS_PITCH_LOCK = 128,
    HAS_YAW_AXIS = 256,
    HAS_YAW_FOLLOW = 512,
    HAS_YAW_LOCK = 1024,
    SUPPORTS_INFINITE_YAW = 2048,
  }

  export interface GimbalDeviceSetAttitude {
    target_system: number;
    target_component: number;
    flags: GimbalDeviceSetAttitudeFlags;
    q: GeometryMsgs.Quaternion;
    angular_velocity_x: number;
    angular_velocity_y: number;
    angular_velocity_z: number;
  }
  
  export enum GimbalDeviceSetAttitudeFlags {
    FLAGS_RETRACT = 1,
    FLAGS_NEUTRAL = 2,
    FLAGS_ROLL_LOCK = 4,
    FLAGS_PITCH_LOCK = 8,
    FLAGS_YAW_LOCK = 16,
  }

  export interface GimbalGetInformation {
    response: MavrosMsgs.GimbalGetInformationResponse;
  }

  export interface GimbalGetInformationResponse {
    success: boolean;
    result: number;
  }

  export interface GimbalManagerCameraTrack {
    request: MavrosMsgs.GimbalManagerCameraTrackRequest;
    response: MavrosMsgs.GimbalManagerCameraTrackResponse;
  }

  export interface GimbalManagerCameraTrackRequest {
    mode: GimbalManagerCameraTrackRequestMode;
    x: number;
    y: number;
    radius: number;
    top_left_x: number;
    top_left_y: number;
    bottom_right_x: number;
    bottom_right_y: number;
  }
  
  export enum GimbalManagerCameraTrackRequestMode {
    CAMERA_TRACK_MODE_POINT = 0,
    CAMERA_TRACK_MODE_RECTANGLE = 1,
    CAMERA_TRACK_MODE_STOP_TRACKING = 2,
  }

  export interface GimbalManagerCameraTrackResponse {
    success: boolean;
    result: number;
  }

  export interface GimbalManagerConfigure {
    request: MavrosMsgs.GimbalManagerConfigureRequest;
    response: MavrosMsgs.GimbalManagerConfigureResponse;
  }

  export interface GimbalManagerConfigureRequest {
    sysid_primary: number;
    compid_primary: number;
    sysid_secondary: number;
    compid_secondary: number;
    gimbal_device_id: number;
  }

  export interface GimbalManagerConfigureResponse {
    success: boolean;
    result: number;
  }

  export interface GimbalManagerInformation {
    header: StdMsgs.Header;
    cap_flags: GimbalManagerInformationCapFlags;
    gimbal_device_id: number;
    roll_min: number;
    roll_max: number;
    pitch_min: number;
    pitch_max: number;
    yaw_min: number;
    yaw_max: number;
  }
  
  export enum GimbalManagerInformationCapFlags {
    CAP_FLAGS_HAS_RETRACT = 1,
    CAP_FLAGS_HAS_NEUTRAL = 2,
    CAP_FLAGS_HAS_ROLL_AXIS = 4,
    CAP_FLAGS_HAS_ROLL_FOLLOW = 8,
    CAP_FLAGS_HAS_ROLL_LOCK = 16,
    CAP_FLAGS_HAS_PITCH_AXIS = 32,
    CAP_FLAGS_HAS_PITCH_FOLLOW = 64,
    CAP_FLAGS_HAS_PITCH_LOCK = 128,
    CAP_FLAGS_HAS_YAW_AXIS = 256,
    CAP_FLAGS_HAS_YAW_FOLLOW = 512,
    CAP_FLAGS_HAS_YAW_LOCK = 1024,
    CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048,
    CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536,
    CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072,
  }

  export interface GimbalManagerPitchyaw {
    request: MavrosMsgs.GimbalManagerPitchyawRequest;
    response: MavrosMsgs.GimbalManagerPitchyawResponse;
  }

  export interface GimbalManagerPitchyawRequest {
    pitch: number;
    yaw: number;
    pitch_rate: number;
    yaw_rate: number;
    flags: GimbalManagerPitchyawRequestFlags;
    gimbal_device_id: number;
  }
  
  export enum GimbalManagerPitchyawRequestFlags {
    GIMBAL_MANAGER_FLAGS_RETRACT = 1,
    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2,
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4,
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8,
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16,
  }

  export interface GimbalManagerPitchyawResponse {
    success: boolean;
    result: number;
  }

  export interface GimbalManagerSetAttitude {
    target_system: number;
    target_component: number;
    flags: GimbalManagerSetAttitudeFlags;
    gimbal_device_id: number;
    q: GeometryMsgs.Quaternion;
    angular_velocity_x: number;
    angular_velocity_y: number;
    angular_velocity_z: number;
  }
  
  export enum GimbalManagerSetAttitudeFlags {
    GIMBAL_MANAGER_FLAGS_RETRACT = 1,
    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2,
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4,
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8,
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16,
  }

  export interface GimbalManagerSetPitchyaw {
    target_system: number;
    target_component: number;
    flags: GimbalManagerSetPitchyawFlags;
    gimbal_device_id: number;
    pitch: number;
    yaw: number;
    pitch_rate: number;
    yaw_rate: number;
  }
  
  export enum GimbalManagerSetPitchyawFlags {
    GIMBAL_MANAGER_FLAGS_RETRACT = 1,
    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2,
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4,
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8,
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16,
  }

  export interface GimbalManagerSetRoi {
    request: MavrosMsgs.GimbalManagerSetRoiRequest;
    response: MavrosMsgs.GimbalManagerSetRoiResponse;
  }

  export interface GimbalManagerSetRoiRequest {
    mode: number;
    gimbal_device_id: number;
    latitude: number;
    longitude: number;
    altitude: number;
    pitch_offset: number;
    roll_offset: number;
    yaw_offset: number;
    sysid: number;
  }
  
  export enum GimbalManagerSetRoiRequestConst {
    ROI_MODE_LOCATION = 0,
    ROI_MODE_WP_NEXT_OFFSET = 1,
    ROI_MODE_SYSID = 2,
    ROI_MODE_NONE = 3,
  }

  export interface GimbalManagerSetRoiResponse {
    success: boolean;
    result: number;
  }

  export interface GimbalManagerStatus {
    header: StdMsgs.Header;
    flags: GimbalManagerStatusFlags;
    gimbal_device_id: number;
    sysid_primary: number;
    compid_primary: number;
    sysid_secondary: number;
    compid_secondary: number;
  }
  
  export enum GimbalManagerStatusFlags {
    GIMBAL_MANAGER_FLAGS_RETRACT = 1,
    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2,
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4,
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8,
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16,
  }

  export interface GlobalPositionTarget {
    header: StdMsgs.Header;
    coordinate_frame: GlobalPositionTargetCoordinateFrame;
    type_mask: GlobalPositionTargetTypeMask;
    latitude: number;
    longitude: number;
    altitude: number;
    velocity: GeometryMsgs.Vector3;
    acceleration_or_force: GeometryMsgs.Vector3;
    yaw: number;
    yaw_rate: number;
  }
  
  export enum GlobalPositionTargetCoordinateFrame {
    FRAME_GLOBAL_INT = 5,
    FRAME_GLOBAL_REL_ALT = 6,
    FRAME_GLOBAL_TERRAIN_ALT = 11,
  }
  
  export enum GlobalPositionTargetTypeMask {
    IGNORE_LATITUDE = 1,
    IGNORE_LONGITUDE = 2,
    IGNORE_ALTITUDE = 4,
    IGNORE_VX = 8,
    IGNORE_VY = 16,
    IGNORE_VZ = 32,
    IGNORE_AFX = 64,
    IGNORE_AFY = 128,
    IGNORE_AFZ = 256,
    FORCE = 512,
    IGNORE_YAW = 1024,
    IGNORE_YAW_RATE = 2048,
  }

  export interface Gpsinput {
    header: StdMsgs.Header;
    fix_type: number;
    gps_id: number;
    ignore_flags: number;
    time_week_ms: number;
    time_week: number;
    lat: number;
    lon: number;
    alt: number;
    hdop: number;
    vdop: number;
    vn: number;
    ve: number;
    vd: number;
    speed_accuracy: number;
    horiz_accuracy: number;
    vert_accuracy: number;
    satellites_visible: number;
    yaw: number;
  }
  
  export enum GpsinputConst {
    GPS_FIX_TYPE_NO_GPS = 0,
    GPS_FIX_TYPE_NO_FIX = 1,
    GPS_FIX_TYPE_2D_FIX = 2,
    GPS_FIX_TYPE_3D_FIX = 3,
    GPS_FIX_TYPE_DGPS = 4,
    GPS_FIX_TYPE_RTK_FLOATR = 5,
    GPS_FIX_TYPE_RTK_FIXEDR = 6,
    GPS_FIX_TYPE_STATIC = 7,
    GPS_FIX_TYPE_PPP = 8,
  }

  export interface Gpsraw {
    header: StdMsgs.Header;
    fix_type: number;
    lat: number;
    lon: number;
    alt: number;
    eph: number;
    epv: number;
    vel: number;
    cog: number;
    satellites_visible: number;
    alt_ellipsoid: number;
    h_acc: number;
    v_acc: number;
    vel_acc: number;
    hdg_acc: number;
    yaw: number;
    dgps_numch: number;
    dgps_age: number;
  }
  
  export enum GpsrawConst {
    GPS_FIX_TYPE_NO_GPS = 0,
    GPS_FIX_TYPE_NO_FIX = 1,
    GPS_FIX_TYPE_2D_FIX = 2,
    GPS_FIX_TYPE_3D_FIX = 3,
    GPS_FIX_TYPE_DGPS = 4,
    GPS_FIX_TYPE_RTK_FLOAT = 5,
    GPS_FIX_TYPE_RTK_FIXED = 6,
    GPS_FIX_TYPE_STATIC = 7,
    GPS_FIX_TYPE_PPP = 8,
  }

  export interface Gpsrtk {
    header: StdMsgs.Header;
    rtk_receiver_id: number;
    wn: number;
    tow: number;
    rtk_health: number;
    rtk_rate: number;
    nsats: number;
    baseline_a: number;
    baseline_b: number;
    baseline_c: number;
    accuracy: number;
    iar_num_hypotheses: number;
  }

  export interface HilActuatorControls {
    header: StdMsgs.Header;
    controls: number[];
    mode: number;
    flags: number;
  }

  export interface HilControls {
    header: StdMsgs.Header;
    roll_ailerons: number;
    pitch_elevator: number;
    yaw_rudder: number;
    throttle: number;
    aux1: number;
    aux2: number;
    aux3: number;
    aux4: number;
    mode: number;
    nav_mode: number;
  }

  export interface HilGps {
    header: StdMsgs.Header;
    fix_type: number;
    geo: GeographicMsgs.GeoPoint;
    eph: number;
    epv: number;
    vel: number;
    vn: number;
    ve: number;
    vd: number;
    cog: number;
    satellites_visible: number;
  }

  export interface HilSensor {
    header: StdMsgs.Header;
    acc: GeometryMsgs.Vector3;
    gyro: GeometryMsgs.Vector3;
    mag: GeometryMsgs.Vector3;
    abs_pressure: number;
    diff_pressure: number;
    pressure_alt: number;
    temperature: number;
    fields_updated: number;
  }

  export interface HilStateQuaternion {
    header: StdMsgs.Header;
    orientation: GeometryMsgs.Quaternion;
    angular_velocity: GeometryMsgs.Vector3;
    linear_acceleration: GeometryMsgs.Vector3;
    linear_velocity: GeometryMsgs.Vector3;
    geo: GeographicMsgs.GeoPoint;
    ind_airspeed: number;
    true_airspeed: number;
  }

  export interface HomePosition {
    header: StdMsgs.Header;
    geo: GeographicMsgs.GeoPoint;
    position: GeometryMsgs.Point;
    orientation: GeometryMsgs.Quaternion;
    approach: GeometryMsgs.Vector3;
  }

  export interface LandingTarget {
    header: StdMsgs.Header;
    target_num: number;
    frame: number;
    angle: number[];
    distance: number;
    size: number[];
    pose: GeometryMsgs.Pose;
    type: number;
  }
  
  export enum LandingTargetConst {
    GLOBAL = 0,
    LOCAL_NED = 2,
    MISSION = 3,
    GLOBAL_RELATIVE_ALT = 4,
    LOCAL_ENU = 5,
    GLOBAL_INT = 6,
    GLOBAL_RELATIVE_ALT_INT = 7,
    LOCAL_OFFSET_NED = 8,
    BODY_NED = 9,
    BODY_OFFSET_NED = 10,
    GLOBAL_TERRAIN_ALT = 11,
    GLOBAL_TERRAIN_ALT_INT = 12,
    LIGHT_BEACON = 0,
    RADIO_BEACON = 1,
    VISION_FIDUCIAL = 2,
    VISION_OTHER = 3,
  }

  export interface LogData {
    header: StdMsgs.Header;
    id: number;
    offset: number;
    data: number[];
  }

  export interface LogEntry {
    header: StdMsgs.Header;
    id: number;
    num_logs: number;
    last_log_num: number;
    time_utc: { sec: number, nanosec: number };
    size: number;
  }

  export interface LogRequestData {
    request: MavrosMsgs.LogRequestDataRequest;
    response: MavrosMsgs.LogRequestDataResponse;
  }

  export interface LogRequestDataRequest {
    id: number;
    offset: number;
    count: number;
  }

  export interface LogRequestDataResponse {
    success: boolean;
  }

  export interface LogRequestEnd {
    response: MavrosMsgs.LogRequestEndResponse;
  }

  export interface LogRequestEndResponse {
    success: boolean;
  }

  export interface LogRequestList {
    request: MavrosMsgs.LogRequestListRequest;
    response: MavrosMsgs.LogRequestListResponse;
  }

  export interface LogRequestListRequest {
    start: number;
    end: number;
  }

  export interface LogRequestListResponse {
    success: boolean;
  }

  export interface MagnetometerReporter {
    header: StdMsgs.Header;
    report: number;
    confidence: number;
    compass_id: number;
    cal_mask: number;
    cal_status: number;
    autosaved: number;
    fitness: number;
    ofs_x: number;
    ofs_y: number;
    ofs_z: number;
    diag_x: number;
    diag_y: number;
    diag_z: number;
    offdiag_x: number;
    offdiag_y: number;
    offdiag_z: number;
    orientation_confidence: number;
    old_orientation: number;
    new_orientation: number;
    scale_factor: number;
  }

  export interface ManualControl {
    header: StdMsgs.Header;
    x: number;
    y: number;
    z: number;
    r: number;
    buttons: number;
    buttons2: number;
    enabled_extensions: number;
    s: number;
    t: number;
    aux1: number;
    aux2: number;
    aux3: number;
    aux4: number;
    aux5: number;
    aux6: number;
  }

  export interface Mavlink {
    header: StdMsgs.Header;
    framing_status: number;
    magic: number;
    len: number;
    incompat_flags: number;
    compat_flags: number;
    seq: number;
    sysid: number;
    compid: number;
    msgid: number;
    checksum: number;
    payload64: number[];
    signature: number[];
  }
  
  export enum MavlinkConst {
    FRAMING_OK = 1,
    FRAMING_BAD_CRC = 2,
    FRAMING_BAD_SIGNATURE = 3,
    MAVLINK_V10 = 254,
    MAVLINK_V20 = 253,
  }

  export interface MessageInterval {
    request: MavrosMsgs.MessageIntervalRequest;
    response: MavrosMsgs.MessageIntervalResponse;
  }

  export interface MessageIntervalRequest {
    message_id: number;
    message_rate: number;
  }

  export interface MessageIntervalResponse {
    success: boolean;
  }

  export interface MountConfigure {
    request: MavrosMsgs.MountConfigureRequest;
    response: MavrosMsgs.MountConfigureResponse;
  }

  export interface MountConfigureRequest {
    header: StdMsgs.Header;
    mode: MountConfigureRequestMode;
    stabilize_roll: boolean;
    stabilize_pitch: boolean;
    stabilize_yaw: boolean;
    roll_input: number;
    pitch_input: number;
    yaw_input: number;
  }
  
  export enum MountConfigureRequestMode {
    RETRACT = 0,
    NEUTRAL = 1,
    MAVLINK_TARGETING = 2,
    RC_TARGETING = 3,
    GPS_POINT = 4,
  }
  
  export enum MountConfigureRequestConst {
    INPUT_ANGLE_BODY_FRAME = 0,
    INPUT_ANGULAR_RATE = 1,
    INPUT_ANGLE_ABSOLUTE_FRAME = 2,
  }

  export interface MountConfigureResponse {
    success: boolean;
    result: number;
  }

  export interface MountControl {
    header: StdMsgs.Header;
    mode: MountControlMode;
    pitch: number;
    roll: number;
    yaw: number;
    altitude: number;
    latitude: number;
    longitude: number;
  }
  
  export enum MountControlMode {
    MAV_MOUNT_MODE_RETRACT = 0,
    MAV_MOUNT_MODE_NEUTRAL = 1,
    MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,
    MAV_MOUNT_MODE_RC_TARGETING = 3,
    MAV_MOUNT_MODE_GPS_POINT = 4,
  }

  export interface NavControllerOutput {
    header: StdMsgs.Header;
    nav_roll: number;
    nav_pitch: number;
    nav_bearing: number;
    target_bearing: number;
    wp_dist: number;
    alt_error: number;
    aspd_error: number;
    xtrack_error: number;
  }

  export interface OnboardComputerStatus {
    header: StdMsgs.Header;
    component: number;
    uptime: number;
    type: number;
    cpu_cores: number[];
    cpu_combined: number[];
    gpu_cores: number[];
    gpu_combined: number[];
    temperature_board: number;
    temperature_core: number[];
    fan_speed: number[];
    ram_usage: number;
    ram_total: number;
    storage_type: number[];
    storage_usage: number[];
    storage_total: number[];
    link_type: number[];
    link_tx_rate: number[];
    link_rx_rate: number[];
    link_tx_max: number[];
    link_rx_max: number[];
  }

  export interface OpenDroneIdBasicId {
    header: StdMsgs.Header;
    id_or_mac: string;
    id_type: OpenDroneIdBasicIdIdType;
    ua_type: OpenDroneIdBasicIdUaType;
    uas_id: string;
  }
  
  export enum OpenDroneIdBasicIdIdType {
    NONE = 0,
    SERIAL_NUMBER = 1,
    CAA_REGISTRATION_ID = 2,
    UTM_ASSIGNED_UUID = 3,
    SPECIFIC_SESSION_ID = 4,
  }
  
  export enum OpenDroneIdBasicIdUaType {
    NONE = 0,
    AEROPLANE = 1,
    HELICOPTER_OR_MULTIROTOR = 2,
    GYROPLANE = 3,
    HYBRID_LIFT = 4,
    ORNITHOPTER = 5,
    GLIDER = 6,
    KITE = 7,
    FREE_BALLOON = 8,
    CAPTIVE_BALLOON = 9,
    AIRSHIP = 10,
    FREE_FALL_PARACHUTE = 11,
    ROCKET = 12,
    TETHERED_POWERED_AIRCRAFT = 13,
    GROUND_OBSTACLE = 14,
    OTHER = 15,
  }

  export interface OpenDroneIdOperatorId {
    header: StdMsgs.Header;
    id_or_mac: string;
    operator_id_type: number;
    operator_id: string;
  }
  
  export enum OpenDroneIdOperatorIdConst {
    ID_TYPE_CAA = 0,
  }

  export interface OpenDroneIdSelfId {
    header: StdMsgs.Header;
    id_or_mac: string;
    description_type: OpenDroneIdSelfIdDescriptionType;
    description: string;
  }
  
  export enum OpenDroneIdSelfIdDescriptionType {
    DESC_TYPE_TEXT = 0,
    DESC_TYPE_EMERGENCY = 1,
    DESC_TYPE_EXTENDED_STATUS = 2,
  }

  export interface OpenDroneIdSystem {
    header: StdMsgs.Header;
    id_or_mac: string;
    operator_location_type: number;
    classification_type: OpenDroneIdSystemClassificationType;
    operator_latitude: number;
    operator_longitude: number;
    area_count: number;
    area_radius: number;
    area_ceiling: number;
    area_floor: number;
    category_eu: number;
    class_eu: number;
    operator_altitude_geo: number;
  }
  
  export enum OpenDroneIdSystemClassificationType {
    UNDECLARED = 0,
    EU = 1,
  }
  
  export enum OpenDroneIdSystemConst {
    LOCATION_TYPE_TAKEOFF = 0,
    LOCATION_TYPE_LIVE_GNSS = 1,
    LOCATION_TYPE_FIXED = 2,
  }

  export interface OpenDroneIdSystemUpdate {
    header: StdMsgs.Header;
    operator_latitude: number;
    operator_longitude: number;
    operator_altitude_geo: number;
  }

  export interface OpticalFlow {
    header: StdMsgs.Header;
    flow: GeometryMsgs.Vector3;
    flow_comp_m: GeometryMsgs.Vector3;
    quality: number;
    ground_distance: number;
    flow_rate: GeometryMsgs.Vector3;
  }

  export interface OpticalFlowRad {
    header: StdMsgs.Header;
    integration_time_us: number;
    integrated_x: number;
    integrated_y: number;
    integrated_xgyro: number;
    integrated_ygyro: number;
    integrated_zgyro: number;
    temperature: number;
    quality: number;
    time_delta_distance_us: number;
    distance: number;
  }

  export interface OverrideRcIn {
    channels: number[];
  }
  
  export enum OverrideRcInConst {
    CHAN_RELEASE = 0,
    CHAN_NOCHANGE = 65535,
  }

  export interface Param {
    header: StdMsgs.Header;
    param_id: string;
    value: MavrosMsgs.ParamValue;
    param_index: number;
    param_count: number;
  }

  export interface ParamEvent {
    header: StdMsgs.Header;
    param_id: string;
    value: RclInterfaces.ParameterValue;
    param_index: number;
    param_count: number;
  }

  export interface ParamGet {
    request: MavrosMsgs.ParamGetRequest;
    response: MavrosMsgs.ParamGetResponse;
  }

  export interface ParamGetRequest {
    param_id: string;
  }

  export interface ParamGetResponse {
    success: boolean;
    value: MavrosMsgs.ParamValue;
  }

  export interface ParamPull {
    request: MavrosMsgs.ParamPullRequest;
    response: MavrosMsgs.ParamPullResponse;
  }

  export interface ParamPullRequest {
    force_pull: boolean;
  }

  export interface ParamPullResponse {
    success: boolean;
    param_received: number;
  }

  export interface ParamPush {
    response: MavrosMsgs.ParamPushResponse;
  }

  export interface ParamPushResponse {
    success: boolean;
    param_transfered: number;
  }

  export interface ParamSet {
    request: MavrosMsgs.ParamSetRequest;
    response: MavrosMsgs.ParamSetResponse;
  }

  export interface ParamSetRequest {
    param_id: string;
    value: MavrosMsgs.ParamValue;
  }

  export interface ParamSetResponse {
    success: boolean;
    value: MavrosMsgs.ParamValue;
  }

  export interface ParamSetV2 {
    request: MavrosMsgs.ParamSetV2Request;
    response: MavrosMsgs.ParamSetV2Response;
  }

  export interface ParamSetV2Request {
    force_set: boolean;
    param_id: string;
    value: RclInterfaces.ParameterValue;
  }

  export interface ParamSetV2Response {
    success: boolean;
    value: RclInterfaces.ParameterValue;
  }

  export interface ParamValue {
    integer: number;
    real: number;
  }

  export interface PlayTuneV2 {
    format: PlayTuneV2Format;
    tune: string;
  }
  
  export enum PlayTuneV2Format {
    QBASIC1_1 = 1,
    MML_MODERN = 2,
  }

  export interface PositionTarget {
    header: StdMsgs.Header;
    coordinate_frame: PositionTargetCoordinateFrame;
    type_mask: PositionTargetTypeMask;
    position: GeometryMsgs.Point;
    velocity: GeometryMsgs.Vector3;
    acceleration_or_force: GeometryMsgs.Vector3;
    yaw: number;
    yaw_rate: number;
  }
  
  export enum PositionTargetCoordinateFrame {
    FRAME_LOCAL_NED = 1,
    FRAME_LOCAL_OFFSET_NED = 7,
    FRAME_BODY_NED = 8,
    FRAME_BODY_OFFSET_NED = 9,
  }
  
  export enum PositionTargetTypeMask {
    IGNORE_PX = 1,
    IGNORE_PY = 2,
    IGNORE_PZ = 4,
    IGNORE_VX = 8,
    IGNORE_VY = 16,
    IGNORE_VZ = 32,
    IGNORE_AFX = 64,
    IGNORE_AFY = 128,
    IGNORE_AFZ = 256,
    FORCE = 512,
    IGNORE_YAW = 1024,
    IGNORE_YAW_RATE = 2048,
  }

  export interface RadioStatus {
    header: StdMsgs.Header;
    rssi: number;
    remrssi: number;
    txbuf: number;
    noise: number;
    remnoise: number;
    rxerrors: number;
    fixed: number;
    rssi_dbm: number;
    remrssi_dbm: number;
  }

  export interface RcIn {
    header: StdMsgs.Header;
    rssi: number;
    channels: number[];
  }

  export interface RcOut {
    header: StdMsgs.Header;
    channels: number[];
  }

  export interface Rtcm {
    header: StdMsgs.Header;
    data: number[];
  }

  export interface RtkBaseline {
    header: StdMsgs.Header;
    time_last_baseline_ms: number;
    rtk_receiver_id: number;
    wn: number;
    tow: number;
    rtk_health: number;
    rtk_rate: number;
    nsats: number;
    baseline_coords_type: number;
    baseline_a_mm: number;
    baseline_b_mm: number;
    baseline_c_mm: number;
    accuracy: number;
    iar_num_hypotheses: number;
  }
  
  export enum RtkBaselineConst {
    RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0,
    RTK_BASELINE_COORDINATE_SYSTEM_NED = 1,
  }

  export interface SetMavFrame {
    request: MavrosMsgs.SetMavFrameRequest;
    response: MavrosMsgs.SetMavFrameResponse;
  }

  export interface SetMavFrameRequest {
    mav_frame: SetMavFrameRequestMavFrame;
  }
  
  export enum SetMavFrameRequestMavFrame {
    FRAME_GLOBAL = 0,
    FRAME_LOCAL_NED = 1,
    FRAME_MISSION = 2,
    FRAME_GLOBAL_RELATIVE_ALT = 3,
    FRAME_LOCAL_ENU = 4,
    FRAME_GLOBAL_INT = 5,
    FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    FRAME_LOCAL_OFFSET_NED = 7,
    FRAME_BODY_NED = 8,
    FRAME_BODY_OFFSET_NED = 9,
    FRAME_GLOBAL_TERRAIN_ALT = 10,
    FRAME_GLOBAL_TERRAIN_ALT_INT = 11,
    FRAME_BODY_FRD = 12,
    FRAME_RESERVED_13 = 13,
    FRAME_RESERVED_14 = 14,
    FRAME_RESERVED_15 = 15,
    FRAME_RESERVED_16 = 16,
    FRAME_RESERVED_17 = 17,
    FRAME_RESERVED_18 = 18,
    FRAME_RESERVED_19 = 19,
    FRAME_LOCAL_FRD = 20,
    FRAME_LOCAL_FLU = 21,
  }

  export interface SetMavFrameResponse {
    success: boolean;
  }

  export interface SetMode {
    request: MavrosMsgs.SetModeRequest;
    response: MavrosMsgs.SetModeResponse;
  }

  export interface SetModeRequest {
    base_mode: SetModeRequestBaseMode;
    custom_mode: string;
  }
  
  export enum SetModeRequestBaseMode {
    MAV_MODE_PREFLIGHT = 0,
    MAV_MODE_STABILIZE_DISARMED = 80,
    MAV_MODE_STABILIZE_ARMED = 208,
    MAV_MODE_MANUAL_DISARMED = 64,
    MAV_MODE_MANUAL_ARMED = 192,
    MAV_MODE_GUIDED_DISARMED = 88,
    MAV_MODE_GUIDED_ARMED = 216,
    MAV_MODE_AUTO_DISARMED = 92,
    MAV_MODE_AUTO_ARMED = 220,
    MAV_MODE_TEST_DISARMED = 66,
    MAV_MODE_TEST_ARMED = 194,
  }

  export interface SetModeResponse {
    mode_sent: boolean;
  }

  export interface State {
    header: StdMsgs.Header;
    connected: boolean;
    armed: boolean;
    guided: boolean;
    manual_input: boolean;
    mode: StateMode;
    system_status: number;
  }
  
  export enum StateMode {
    MODE_APM_PLANE_MANUAL = 'MANUAL',
    MODE_APM_PLANE_CIRCLE = 'CIRCLE',
    MODE_APM_PLANE_STABILIZE = 'STABILIZE',
    MODE_APM_PLANE_TRAINING = 'TRAINING',
    MODE_APM_PLANE_ACRO = 'ACRO',
    MODE_APM_PLANE_FBWA = 'FBWA',
    MODE_APM_PLANE_FBWB = 'FBWB',
    MODE_APM_PLANE_CRUISE = 'CRUISE',
    MODE_APM_PLANE_AUTOTUNE = 'AUTOTUNE',
    MODE_APM_PLANE_AUTO = 'AUTO',
    MODE_APM_PLANE_RTL = 'RTL',
    MODE_APM_PLANE_LOITER = 'LOITER',
    MODE_APM_PLANE_LAND = 'LAND',
    MODE_APM_PLANE_GUIDED = 'GUIDED',
    MODE_APM_PLANE_INITIALISING = 'INITIALISING',
    MODE_APM_PLANE_QSTABILIZE = 'QSTABILIZE',
    MODE_APM_PLANE_QHOVER = 'QHOVER',
    MODE_APM_PLANE_QLOITER = 'QLOITER',
    MODE_APM_PLANE_QLAND = 'QLAND',
    MODE_APM_PLANE_QRTL = 'QRTL',
    MODE_APM_COPTER_STABILIZE = 'STABILIZE',
    MODE_APM_COPTER_ACRO = 'ACRO',
    MODE_APM_COPTER_ALT_HOLD = 'ALT_HOLD',
    MODE_APM_COPTER_AUTO = 'AUTO',
    MODE_APM_COPTER_GUIDED = 'GUIDED',
    MODE_APM_COPTER_LOITER = 'LOITER',
    MODE_APM_COPTER_RTL = 'RTL',
    MODE_APM_COPTER_CIRCLE = 'CIRCLE',
    MODE_APM_COPTER_POSITION = 'POSITION',
    MODE_APM_COPTER_LAND = 'LAND',
    MODE_APM_COPTER_OF_LOITER = 'OF_LOITER',
    MODE_APM_COPTER_DRIFT = 'DRIFT',
    MODE_APM_COPTER_SPORT = 'SPORT',
    MODE_APM_COPTER_FLIP = 'FLIP',
    MODE_APM_COPTER_AUTOTUNE = 'AUTOTUNE',
    MODE_APM_COPTER_POSHOLD = 'POSHOLD',
    MODE_APM_COPTER_BRAKE = 'BRAKE',
    MODE_APM_COPTER_THROW = 'THROW',
    MODE_APM_COPTER_AVOID_ADSB = 'AVOID_ADSB',
    MODE_APM_COPTER_GUIDED_NOGPS = 'GUIDED_NOGPS',
    MODE_APM_ROVER_MANUAL = 'MANUAL',
    MODE_APM_ROVER_LEARNING = 'LEARNING',
    MODE_APM_ROVER_STEERING = 'STEERING',
    MODE_APM_ROVER_HOLD = 'HOLD',
    MODE_APM_ROVER_AUTO = 'AUTO',
    MODE_APM_ROVER_RTL = 'RTL',
    MODE_APM_ROVER_GUIDED = 'GUIDED',
    MODE_APM_ROVER_INITIALISING = 'INITIALISING',
    MODE_PX4_MANUAL = 'MANUAL',
    MODE_PX4_ACRO = 'ACRO',
    MODE_PX4_ALTITUDE = 'ALTCTL',
    MODE_PX4_POSITION = 'POSCTL',
    MODE_PX4_OFFBOARD = 'OFFBOARD',
    MODE_PX4_STABILIZED = 'STABILIZED',
    MODE_PX4_RATTITUDE = 'RATTITUDE',
    MODE_PX4_MISSION = 'AUTO.MISSION',
    MODE_PX4_LOITER = 'AUTO.LOITER',
    MODE_PX4_RTL = 'AUTO.RTL',
    MODE_PX4_LAND = 'AUTO.LAND',
    MODE_PX4_RTGS = 'AUTO.RTGS',
    MODE_PX4_READY = 'AUTO.READY',
    MODE_PX4_TAKEOFF = 'AUTO.TAKEOFF',
  }

  export interface StatusEvent {
    header: StdMsgs.Header;
    severity: StatusEventSeverity;
    px4_id: number;
    arguments: number[];
    sequence: number;
  }
  
  export enum StatusEventSeverity {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7,
  }

  export interface StatusText {
    header: StdMsgs.Header;
    severity: StatusTextSeverity;
    text: string;
  }
  
  export enum StatusTextSeverity {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7,
  }

  export interface StreamRate {
    request: MavrosMsgs.StreamRateRequest;
  }

  export interface StreamRateRequest {
    stream_id: StreamRateRequestStreamId;
    message_rate: number;
    on_off: boolean;
  }
  
  export enum StreamRateRequestStreamId {
    STREAM_ALL = 0,
    STREAM_RAW_SENSORS = 1,
    STREAM_EXTENDED_STATUS = 2,
    STREAM_RC_CHANNELS = 3,
    STREAM_RAW_CONTROLLER = 4,
    STREAM_POSITION = 6,
    STREAM_EXTRA1 = 10,
    STREAM_EXTRA2 = 11,
    STREAM_EXTRA3 = 12,
  }

  export interface SysStatus {
    header: StdMsgs.Header;
    sensors_present: number;
    sensors_enabled: number;
    sensors_health: number;
    load: number;
    voltage_battery: number;
    current_battery: number;
    battery_remaining: number;
    drop_rate_comm: number;
    errors_comm: number;
    errors_count1: number;
    errors_count2: number;
    errors_count3: number;
    errors_count4: number;
  }

  export interface TerrainReport {
    header: StdMsgs.Header;
    latitude: number;
    longitude: number;
    spacing: number;
    terrain_height: number;
    current_height: number;
    pending: number;
    loaded: number;
  }

  export interface Thrust {
    header: StdMsgs.Header;
    thrust: number;
  }

  export interface TimesyncStatus {
    header: StdMsgs.Header;
    remote_timestamp_ns: number;
    observed_offset_ns: number;
    estimated_offset_ns: number;
    round_trip_time_ms: number;
  }

  export interface Trajectory {
    header: StdMsgs.Header;
    type: TrajectoryType;
    point_1: MavrosMsgs.PositionTarget;
    point_2: MavrosMsgs.PositionTarget;
    point_3: MavrosMsgs.PositionTarget;
    point_4: MavrosMsgs.PositionTarget;
    point_5: MavrosMsgs.PositionTarget;
    point_valid: number[];
    command: number[];
    time_horizon: number[];
  }
  
  export enum TrajectoryType {
    MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0,
    MAV_TRAJECTORY_REPRESENTATION_BEZIER = 1,
  }

  export interface Tunnel {
    target_system: number;
    target_component: number;
    payload_type: TunnelPayloadType;
    payload_length: number;
    payload: number[];
  }
  
  export enum TunnelPayloadType {
    PAYLOAD_TYPE_UNKNOWN = 0,
    PAYLOAD_TYPE_STORM32_RESERVED0 = 200,
    PAYLOAD_TYPE_STORM32_RESERVED1 = 201,
    PAYLOAD_TYPE_STORM32_RESERVED2 = 202,
    PAYLOAD_TYPE_STORM32_RESERVED3 = 203,
    PAYLOAD_TYPE_STORM32_RESERVED4 = 204,
    PAYLOAD_TYPE_STORM32_RESERVED5 = 205,
    PAYLOAD_TYPE_STORM32_RESERVED6 = 206,
    PAYLOAD_TYPE_STORM32_RESERVED7 = 207,
    PAYLOAD_TYPE_STORM32_RESERVED8 = 208,
    PAYLOAD_TYPE_STORM32_RESERVED9 = 209,
  }

  export interface VehicleInfo {
    header: StdMsgs.Header;
    available_info: number;
    sysid: number;
    compid: number;
    autopilot: number;
    type: number;
    system_status: number;
    base_mode: number;
    custom_mode: number;
    mode: string;
    mode_id: number;
    capabilities: number;
    flight_sw_version: number;
    middleware_sw_version: number;
    os_sw_version: number;
    board_version: number;
    flight_custom_version: string;
    vendor_id: number;
    product_id: number;
    uid: number;
  }
  
  export enum VehicleInfoConst {
    HAVE_INFO_HEARTBEAT = 1,
    HAVE_INFO_AUTOPILOT_VERSION = 2,
  }

  export interface VehicleInfoGet {
    request: MavrosMsgs.VehicleInfoGetRequest;
    response: MavrosMsgs.VehicleInfoGetResponse;
  }

  export interface VehicleInfoGetRequest {
    sysid: number;
    compid: number;
    get_all: boolean;
  }
  
  export enum VehicleInfoGetRequestConst {
    GET_MY_SYSID = 0,
    GET_MY_COMPID = 0,
  }

  export interface VehicleInfoGetResponse {
    success: boolean;
    vehicles: MavrosMsgs.VehicleInfo[];
  }

  export interface VfrHud {
    header: StdMsgs.Header;
    airspeed: number;
    groundspeed: number;
    heading: number;
    throttle: number;
    altitude: number;
    climb: number;
  }

  export interface Vibration {
    header: StdMsgs.Header;
    vibration: GeometryMsgs.Vector3;
    clipping: number[];
  }

  export interface Waypoint {
    frame: WaypointFrame;
    command: number;
    is_current: boolean;
    autocontinue: boolean;
    param1: number;
    param2: number;
    param3: number;
    param4: number;
    x_lat: number;
    y_long: number;
    z_alt: number;
  }
  
  export enum WaypointFrame {
    FRAME_GLOBAL = 0,
    FRAME_LOCAL_NED = 1,
    FRAME_MISSION = 2,
    FRAME_GLOBAL_REL_ALT = 3,
    FRAME_LOCAL_ENU = 4,
    FRAME_GLOBAL_INT = 5,
    FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    FRAME_LOCAL_OFFSET_NED = 7,
    FRAME_BODY_NED = 8,
    FRAME_BODY_OFFSET_NED = 9,
    FRAME_GLOBAL_TERRAIN_ALT = 10,
    FRAME_GLOBAL_TERRAIN_ALT_INT = 11,
    FRAME_BODY_FRD = 12,
    FRAME_RESERVED_13 = 13,
    FRAME_RESERVED_14 = 14,
    FRAME_RESERVED_15 = 15,
    FRAME_RESERVED_16 = 16,
    FRAME_RESERVED_17 = 17,
    FRAME_RESERVED_18 = 18,
    FRAME_RESERVED_19 = 19,
    FRAME_LOCAL_FRD = 20,
    FRAME_LOCAL_FLU = 21,
  }

  export interface WaypointClear {
    response: MavrosMsgs.WaypointClearResponse;
  }

  export interface WaypointClearResponse {
    success: boolean;
  }

  export interface WaypointList {
    current_seq: number;
    waypoints: MavrosMsgs.Waypoint[];
  }

  export interface WaypointPull {
    response: MavrosMsgs.WaypointPullResponse;
  }

  export interface WaypointPullResponse {
    success: boolean;
    wp_received: number;
  }

  export interface WaypointPush {
    request: MavrosMsgs.WaypointPushRequest;
    response: MavrosMsgs.WaypointPushResponse;
  }

  export interface WaypointPushRequest {
    start_index: number;
    waypoints: MavrosMsgs.Waypoint[];
  }

  export interface WaypointPushResponse {
    success: boolean;
    wp_transfered: number;
  }

  export interface WaypointReached {
    header: StdMsgs.Header;
    wp_seq: number;
  }

  export interface WaypointSetCurrent {
    request: MavrosMsgs.WaypointSetCurrentRequest;
    response: MavrosMsgs.WaypointSetCurrentResponse;
  }

  export interface WaypointSetCurrentRequest {
    wp_seq: number;
  }

  export interface WaypointSetCurrentResponse {
    success: boolean;
  }

  export interface WheelOdomStamped {
    header: StdMsgs.Header;
    data: number[];
  }
}

export namespace RclInterfaces {
  export enum ParameterTypeConst {
    PARAMETER_NOT_SET = 0,
    PARAMETER_BOOL = 1,
    PARAMETER_INTEGER = 2,
    PARAMETER_DOUBLE = 3,
    PARAMETER_STRING = 4,
    PARAMETER_BYTE_ARRAY = 5,
    PARAMETER_BOOL_ARRAY = 6,
    PARAMETER_INTEGER_ARRAY = 7,
    PARAMETER_DOUBLE_ARRAY = 8,
    PARAMETER_STRING_ARRAY = 9,
  }

  export interface DescribeParameters {
    request: RclInterfaces.DescribeParametersRequest;
    response: RclInterfaces.DescribeParametersResponse;
  }

  export interface DescribeParametersRequest {
    names: string[];
  }

  export interface DescribeParametersResponse {
    descriptors: RclInterfaces.ParameterDescriptor[];
  }

  export interface FloatingPointRange {
    from_value: number;
    to_value: number;
    step: number;
  }

  export interface GetParameterTypes {
    request: RclInterfaces.GetParameterTypesRequest;
    response: RclInterfaces.GetParameterTypesResponse;
  }

  export interface GetParameterTypesRequest {
    names: string[];
  }

  export interface GetParameterTypesResponse {
    types: number[];
  }

  export interface GetParameters {
    request: RclInterfaces.GetParametersRequest;
    response: RclInterfaces.GetParametersResponse;
  }

  export interface GetParametersRequest {
    names: string[];
  }

  export interface GetParametersResponse {
    values: RclInterfaces.ParameterValue[];
  }

  export interface IntegerRange {
    from_value: number;
    to_value: number;
    step: number;
  }

  export interface ListParameters {
    request: RclInterfaces.ListParametersRequest;
    response: RclInterfaces.ListParametersResponse;
  }

  export interface ListParametersRequest {
    prefixes: string[];
    depth: number;
  }
  
  export enum ListParametersRequestConst {
    DEPTH_RECURSIVE = 0,
  }

  export interface ListParametersResponse {
    result: RclInterfaces.ListParametersResult;
  }

  export interface ListParametersResult {
    names: string[];
    prefixes: string[];
  }

  export interface Log {
    stamp: { sec: number, nanosec: number };
    level: LogLevel;
    name: string;
    msg: string;
    file: string;
    function: string;
    line: number;
  }
  
  export enum LogLevel {
    DEBUG = 10,
    INFO = 20,
    WARN = 30,
    ERROR = 40,
    FATAL = 50,
  }

  export interface Parameter {
    name: string;
    value: RclInterfaces.ParameterValue;
  }

  export interface ParameterDescriptor {
    name: string;
    type: number;
    description: string;
    additional_constraints: string;
    read_only: boolean;
    dynamic_typing: boolean;
    floating_point_range: RclInterfaces.FloatingPointRange[];
    integer_range: RclInterfaces.IntegerRange[];
  }

  export interface ParameterEvent {
    stamp: { sec: number, nanosec: number };
    node: string;
    new_parameters: RclInterfaces.Parameter[];
    changed_parameters: RclInterfaces.Parameter[];
    deleted_parameters: RclInterfaces.Parameter[];
  }

  export interface ParameterEventDescriptors {
    new_parameters: RclInterfaces.ParameterDescriptor[];
    changed_parameters: RclInterfaces.ParameterDescriptor[];
    deleted_parameters: RclInterfaces.ParameterDescriptor[];
  }

  export interface ParameterValue {
    type: number;
    bool_value: boolean;
    integer_value: number;
    double_value: number;
    string_value: string;
    byte_array_value: number[];
    bool_array_value: boolean[];
    integer_array_value: number[];
    double_array_value: number[];
    string_array_value: string[];
  }

  export interface SetParameters {
    request: RclInterfaces.SetParametersRequest;
    response: RclInterfaces.SetParametersResponse;
  }

  export interface SetParametersAtomically {
    request: RclInterfaces.SetParametersAtomicallyRequest;
    response: RclInterfaces.SetParametersAtomicallyResponse;
  }

  export interface SetParametersAtomicallyRequest {
    parameters: RclInterfaces.Parameter[];
  }

  export interface SetParametersAtomicallyResponse {
    result: RclInterfaces.SetParametersResult;
  }

  export interface SetParametersRequest {
    parameters: RclInterfaces.Parameter[];
  }

  export interface SetParametersResponse {
    results: RclInterfaces.SetParametersResult[];
  }

  export interface SetParametersResult {
    successful: boolean;
    reason: string;
  }
}

export namespace SensorMsgs {
  export interface BatteryState {
    header: StdMsgs.Header;
    voltage: number;
    temperature: number;
    current: number;
    charge: number;
    capacity: number;
    design_capacity: number;
    percentage: number;
    power_supply_status: BatteryStatePowerSupplyStatus;
    power_supply_health: BatteryStatePowerSupplyHealth;
    power_supply_technology: BatteryStatePowerSupplyTechnology;
    present: boolean;
    cell_voltage: number[];
    cell_temperature: number[];
    location: string;
    serial_number: string;
  }
  
  export enum BatteryStatePowerSupplyStatus {
    UNKNOWN = 0,
    CHARGING = 1,
    DISCHARGING = 2,
    NOT_CHARGING = 3,
    FULL = 4,
  }
  
  export enum BatteryStatePowerSupplyHealth {
    UNKNOWN = 0,
    GOOD = 1,
    OVERHEAT = 2,
    DEAD = 3,
    OVERVOLTAGE = 4,
    UNSPEC_FAILURE = 5,
    COLD = 6,
    WATCHDOG_TIMER_EXPIRE = 7,
    SAFETY_TIMER_EXPIRE = 8,
  }
  
  export enum BatteryStatePowerSupplyTechnology {
    UNKNOWN = 0,
    NIMH = 1,
    LION = 2,
    LIPO = 3,
    LIFE = 4,
    NICD = 5,
    LIMN = 6,
  }

  export interface CameraInfo {
    header: StdMsgs.Header;
    height: number;
    width: number;
    distortion_model: string;
    d: number[];
    k: number[];
    r: number[];
    p: number[];
    binning_x: number;
    binning_y: number;
    roi: SensorMsgs.RegionOfInterest;
  }

  export interface ChannelFloat32 {
    name: string;
    values: number[];
  }

  export interface CompressedImage {
    header: StdMsgs.Header;
    format: string;
    data: number[];
  }

  export interface FluidPressure {
    header: StdMsgs.Header;
    fluid_pressure: number;
    variance: number;
  }

  export interface Illuminance {
    header: StdMsgs.Header;
    illuminance: number;
    variance: number;
  }

  export interface Image {
    header: StdMsgs.Header;
    height: number;
    width: number;
    encoding: string;
    is_bigendian: number;
    step: number;
    data: number[];
  }

  export interface Imu {
    header: StdMsgs.Header;
    orientation: GeometryMsgs.Quaternion;
    orientation_covariance: number[];
    angular_velocity: GeometryMsgs.Vector3;
    angular_velocity_covariance: number[];
    linear_acceleration: GeometryMsgs.Vector3;
    linear_acceleration_covariance: number[];
  }

  export interface JointState {
    header: StdMsgs.Header;
    name: string[];
    position: number[];
    velocity: number[];
    effort: number[];
  }

  export interface Joy {
    header: StdMsgs.Header;
    axes: number[];
    buttons: number[];
  }

  export interface JoyFeedback {
    type: JoyFeedbackType;
    id: number;
    intensity: number;
  }
  
  export enum JoyFeedbackType {
    LED = 0,
    RUMBLE = 1,
    BUZZER = 2,
  }

  export interface JoyFeedbackArray {
    array: SensorMsgs.JoyFeedback[];
  }

  export interface LaserEcho {
    echoes: number[];
  }

  export interface LaserScan {
    header: StdMsgs.Header;
    angle_min: number;
    angle_max: number;
    angle_increment: number;
    time_increment: number;
    scan_time: number;
    range_min: number;
    range_max: number;
    ranges: number[];
    intensities: number[];
  }

  export interface MagneticField {
    header: StdMsgs.Header;
    magnetic_field: GeometryMsgs.Vector3;
    magnetic_field_covariance: number[];
  }

  export interface MultiDofJointState {
    header: StdMsgs.Header;
    joint_names: string[];
    transforms: GeometryMsgs.Transform[];
    twist: GeometryMsgs.Twist[];
    wrench: GeometryMsgs.Wrench[];
  }

  export interface MultiEchoLaserScan {
    header: StdMsgs.Header;
    angle_min: number;
    angle_max: number;
    angle_increment: number;
    time_increment: number;
    scan_time: number;
    range_min: number;
    range_max: number;
    ranges: SensorMsgs.LaserEcho[];
    intensities: SensorMsgs.LaserEcho[];
  }

  export interface NavSatFix {
    header: StdMsgs.Header;
    status: SensorMsgs.NavSatStatus;
    latitude: number;
    longitude: number;
    altitude: number;
    position_covariance: number[];
    position_covariance_type: NavSatFixPositionCovarianceType;
  }
  
  export enum NavSatFixPositionCovarianceType {
    COVARIANCE_TYPE_UNKNOWN = 0,
    COVARIANCE_TYPE_APPROXIMATED = 1,
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
    COVARIANCE_TYPE_KNOWN = 3,
  }

  export interface NavSatStatus {
    status: NavSatStatusStatus;
    service: NavSatStatusService;
  }
  
  export enum NavSatStatusStatus {
    STATUS_NO_FIX = -1,
    STATUS_FIX = 0,
    STATUS_SBAS_FIX = 1,
    STATUS_GBAS_FIX = 2,
  }
  
  export enum NavSatStatusService {
    SERVICE_GPS = 1,
    SERVICE_GLONASS = 2,
    SERVICE_COMPASS = 4,
    SERVICE_GALILEO = 8,
  }

  export interface PointCloud {
    header: StdMsgs.Header;
    points: GeometryMsgs.Point32[];
    channels: SensorMsgs.ChannelFloat32[];
  }

  export interface PointCloud2 {
    header: StdMsgs.Header;
    height: number;
    width: number;
    fields: SensorMsgs.PointField[];
    is_bigendian: boolean;
    point_step: number;
    row_step: number;
    data: number[];
    is_dense: boolean;
  }

  export interface PointField {
    name: string;
    offset: number;
    datatype: PointFieldDatatype;
    count: number;
  }
  
  export enum PointFieldDatatype {
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8,
  }

  export interface Range {
    header: StdMsgs.Header;
    radiation_type: RangeRadiationType;
    field_of_view: number;
    min_range: number;
    max_range: number;
    range: number;
  }
  
  export enum RangeRadiationType {
    ULTRASOUND = 0,
    INFRARED = 1,
  }

  export interface RegionOfInterest {
    x_offset: number;
    y_offset: number;
    height: number;
    width: number;
    do_rectify: boolean;
  }

  export interface RelativeHumidity {
    header: StdMsgs.Header;
    relative_humidity: number;
    variance: number;
  }

  export interface SetCameraInfo {
    request: SensorMsgs.SetCameraInfoRequest;
    response: SensorMsgs.SetCameraInfoResponse;
  }

  export interface SetCameraInfoRequest {
    camera_info: SensorMsgs.CameraInfo;
  }

  export interface SetCameraInfoResponse {
    success: boolean;
    status_message: string;
  }

  export interface Temperature {
    header: StdMsgs.Header;
    temperature: number;
    variance: number;
  }

  export interface TimeReference {
    header: StdMsgs.Header;
    time_ref: { sec: number, nanosec: number };
    source: string;
  }
}

export namespace StdMsgs {
  export interface Bool {
    data: boolean;
  }

  export interface Byte {
    data: number;
  }

  export interface ByteMultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface Char {
    data: number;
  }

  export interface ColorRgba {
    r: number;
    g: number;
    b: number;
    a: number;
  }

  export interface Float32 {
    data: number;
  }

  export interface Float32MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface Float64 {
    data: number;
  }

  export interface Float64MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface Header {
    stamp: { sec: number, nanosec: number };
    frame_id: string;
  }

  export interface Int16 {
    data: number;
  }

  export interface Int16MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface Int32 {
    data: number;
  }

  export interface Int32MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface Int64 {
    data: number;
  }

  export interface Int64MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface Int8 {
    data: number;
  }

  export interface Int8MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface MultiArrayDimension {
    label: string;
    size: number;
    stride: number;
  }

  export interface MultiArrayLayout {
    dim: StdMsgs.MultiArrayDimension[];
    data_offset: number;
  }

  export interface String {
    data: string;
  }

  export interface UInt16 {
    data: number;
  }

  export interface UInt16MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface UInt32 {
    data: number;
  }

  export interface UInt32MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface UInt64 {
    data: number;
  }

  export interface UInt64MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }

  export interface UInt8 {
    data: number;
  }

  export interface UInt8MultiArray {
    layout: StdMsgs.MultiArrayLayout;
    data: number[];
  }
}

export namespace UniqueIdentifierMsgs {
  export interface Uuid {
    uuid: number[];
  }
}