import ROSLIB from "roslib";
import {
    MavrosMsgs,
    MpccInterfaces,
    RclInterfaces,
    SensorMsgs,
    type GeographicMsgs,
    type GeometryMsgs,
    type StdMsgs,
    type StdSrvs
} from "$lib/rostypes/ros_msgs";
import { Arc, deserialise_path, Line, ORIGIN, serialise_path, type DubinsPath } from "./geometry";
import type { BMFA_Coords } from "./waypoints";
import { get_dubins_wps, get_geofence_wps } from "./mission";

export function open_ros(url: string, onconnect: () => void, onerror: () => void, onclose: () => void) {
    var ros = new ROSLIB.Ros(
        { url: url }
    );
    ros.on("connection", onconnect);
    ros.on("error", onerror);
    ros.on("close", onclose);
    return ros;
}

export interface IMavData {
    batteryData?: SensorMsgs.BatteryState;
    state?: MavrosMsgs.State;
    wind?: GeometryMsgs.TwistWithCovarianceStamped;
    estimatorStatus?: MavrosMsgs.EstimatorStatus;
    compassHeading?: StdMsgs.Float64;
    relAltitude?: StdMsgs.Float64;
    gpsFix?: SensorMsgs.NavSatFix;
    localPose?: GeometryMsgs.PoseStamped;
    imuData?: SensorMsgs.Imu;
};

const mav_sources: { name: string; interface: string; target: string }[] = [
    { name: "/mavros/battery", interface: "sensor_msgs/msg/BatteryState", target: "batteryData" },
    { name: "/mavros/state", interface: "mavros_msgs/msg/State", target: "state" },
    {
        name: "/mavros/wind_estimation",
        interface: "geometry_msgs/msg/TwistWithCovarianceStamped",
        target: "wind"
    },
    // { This one is not super duper helpful
    //     name: "/mavros/estimator_status",
    //     interface: "mavros_msgs/msg/EstimatorStatus",
    //     target: "estimatorStatus"
    // },
    {
        name: "/mavros/local_position/pose",
        interface: "geometry_msgs/msg/PoseStamped",
        target: "localPose"
    },
    {
        name: "/mavros/global_position/compass_hdg",
        interface: "std_msgs/msg/Float64",
        target: "compassHeading"
    },
    { name: "/mavros/global_position/rel_alt", interface: "std_msgs/msg/Float64", target: "relAltitude" },
    { name: "/mavros/global_position/global", interface: "sensor_msgs/msg/NavSatFix", target: "gpsFix" },
    { name: "/mavros/imu/data", interface: "sensor_msgs/msg/Imu", target: "imuData" }
];

function updateStatusText(message: MavrosMsgs.StatusText) {
    const statusTextContainer = document.getElementById("status-message-container");
    // check if at bottom; scroll down if so
    if (statusTextContainer) {
        const isAtBottom =
            statusTextContainer.scrollHeight - statusTextContainer.clientHeight <=
            statusTextContainer.scrollTop + 1;
        const messagePar = document.createElement("p");
        let classes: string[] = ["text-xs"];
        switch (message.severity) {
            case MavrosMsgs.StatusTextSeverity.EMERGENCY:
                classes.push("text-red-500", "font-bold");
                break;
            case MavrosMsgs.StatusTextSeverity.ALERT:
                classes.push("text-red-600");
                break;
            case MavrosMsgs.StatusTextSeverity.CRITICAL:
                classes.push("text-red-700", "font-bold");
                break;
            case MavrosMsgs.StatusTextSeverity.ERROR:
                classes.push("text-red-500");
                break;
            case MavrosMsgs.StatusTextSeverity.WARNING:
                classes.push("text-orange-500");
                break;
            case MavrosMsgs.StatusTextSeverity.NOTICE:
                classes.push("text-yellow-500");
                break;
            case MavrosMsgs.StatusTextSeverity.INFO:
                classes.push("text-white/90");
                break;
            case MavrosMsgs.StatusTextSeverity.DEBUG:
                classes.push("text-white/90");
                break;
        }
        messagePar.classList.add(...classes);
        messagePar.innerText = message.text;
        statusTextContainer.appendChild(messagePar);
        if (isAtBottom) {
            // scroll to bottom of element
            statusTextContainer.scrollTop = statusTextContainer.scrollHeight;
        }
    }
}

function onRosConnect() {
    console.log("roslib connected");
    // var preData = document.getElementById("pre-data");

    mav_sources.forEach((element) => {
        const listener = new ROSLIB.Topic({ ros: ros, name: element.name, messageType: element.interface });
        listener.subscribe(function (message) {
            mav_data[element.target] = message;
        });
    });

    // periodically record history
    setInterval(function () {
        mav_data_history.push(structuredClone(mav_data));
        if (mav_data_history.length > 4096) {
            mav_data_history.unshift();
        }
    }, 20);

    status_listener = new ROSLIB.Topic({
        ros: ros,
        name: "mavros/statustext/recv",
        messageType: "mavros_msgs/msg/StatusText"
    });
    status_listener.subscribe(updateStatusText);

    trajectory_plan_listener = new ROSLIB.Topic({
        ros: ros,
        name: "gs/trajectory_plan",
        messageType: "mpcc_interfaces/msg/TrajectoryPlan"
    });
    trajectory_plan_listener.subscribe(update_trajectory_plan);

    arm_vehicle = new ROSLIB.Service({
        ros: ros,
        name: "/mavros/cmd/arming",
        serviceType: "mavros_msgs/srv/CommandBool"
    });

    set_path_client = new ROSLIB.Service({
        ros: ros,
        name: "gs/set_path",
        serviceType: "mpcc_interfaces/srv/SetPath"
    });

    set_parameter_client = new ROSLIB.Service({
        ros: ros,
        name: "mavros/param/set",
        serviceType: "mavros_msgs/srv/ParamSetV2"
    });

    path_drag_client = new ROSLIB.Service({
        ros: ros,
        name: "gs/converge_path",
        serviceType: "mpcc_interfaces/srv/ConvergePath"
    });

    // mission and geofence clients

    mission_clear_client = new ROSLIB.Service({
        ros: ros,
        name: "/mavros/mission/clear",
        serviceType: "mavros_msgs/srv/WaypointClear"
    });

    mission_push_client = new ROSLIB.Service({
        ros: ros,
        name: "/mavros/mission/push",
        serviceType: "mavros_msgs/srv/WaypointPush"
    });

    mission_set_current_client = new ROSLIB.Service({
        ros: ros,
        name: "/mavros/mission/set_current",
        serviceType: "mavros_msgs/srv/WaypointSetCurrent"
    });

    geofence_clear_client = new ROSLIB.Service({
        ros: ros,
        name: "/mavros/geofence/clear",
        serviceType: "mavros_msgs/srv/WaypointClear"
    });

    geofence_push_client = new ROSLIB.Service({
        ros: ros,
        name: "/mavros/geofence/push",
        serviceType: "mavros_msgs/srv/WaypointPush"
    })
}

export function enable_arm() {
    if (arm_vehicle) {
        const req: MavrosMsgs.CommandBoolRequest = {
            value: true
        };
        arm_vehicle.callService(req, function (result: MavrosMsgs.CommandBoolResponse) {
            console.log("Attempting arm");
            console.log("Success", result.success);
        });
    }
}

export function cancel_arm() {
    if (arm_vehicle) {
        const req: MavrosMsgs.CommandBoolRequest = {
            value: false
        };
        arm_vehicle.callService(req, function (result: MavrosMsgs.CommandBoolResponse) {
            console.log("Attempting disarm");
            console.log("Success", result.success);
        });
    }
}

export function terminate() {
    if (set_parameter_client) {
        const req: MavrosMsgs.ParamSetV2Request = {
            force_set: true,
            param_id: "AFS_TERMINATE",
            value: {
                type: RclInterfaces.ParameterTypeConst.PARAMETER_INTEGER,
                integer_value: 1
            }
        }
        set_parameter_client.callService(req, (res: MavrosMsgs.ParamSetV2Response) => {
            console.log("Flight terminated", res.success);
        })

    } else {
        throw new Error("Parameter set service not found")
    }
}

export function send_path(shapes: (Line | Arc)[]) {
    if (set_path_client) {
        const origin: GeographicMsgs.GeoPoint = {
            latitude: (ORIGIN.latitude * 180) / Math.PI, // must be in degrees
            longitude: (ORIGIN.longitude * 180) / Math.PI,
            altitude: ORIGIN.height
        };
        let s_shapes = shapes.map((sh) => sh.serialise());
        const path: MpccInterfaces.Path = {
            newpath: JSON.stringify(s_shapes),
            origin: origin
        };
        const req: MpccInterfaces.SetPathRequest = {
            newpath: path
        };
        set_path_client.callService(req, function (res: MpccInterfaces.SetPathResponse) {
            console.log("Sent path");
            console.log("Success", res.success);
            console.log(res.error_msg);
        });
    }
}

export function send_waypoints(shapes: DubinsPath, altitude: number) {
    let wps = get_dubins_wps(shapes, altitude);
    set_mission(wps)
}

function update_trajectory_plan(msg: MpccInterfaces.TrajectoryPlan) {
    trajectory_plan = msg;
}

const ros = open_ros(
    // "ws://126.158.118:9090",
    // "ws://192.168.67.63:9090",
    "ws://localhost:9090",

    onRosConnect,

    function () {
        console.log("Connection error.");
    },

    function () {
        console.log("Connection closed");
    }
);

export function get_current_mav_data(): IMavData {
    return mav_data;
}
export function get_mav_data_history(): IMavData[] {
    return mav_data_history;
}
export function get_trajectory_plan(): MpccInterfaces.TrajectoryPlan | null {
    return trajectory_plan;
}

export function converge_path(shapes: DubinsPath, callback: (path: DubinsPath) => void) {
    if (!path_drag_client) return;
    let req: MpccInterfaces.ConvergePathRequest = {
        path: serialise_path(shapes)
    }
    path_drag_client?.callService(req, (res: MpccInterfaces.ConvergePathResponse) => {
        console.log(res.converged)
        callback(deserialise_path(res.path) as DubinsPath)
    });

}

// TODO: better handling of return values

export function clear_geofence() {
    if (!geofence_clear_client) return new Error("Geofence push client not connected");
    geofence_clear_client.callService({}, (res: MavrosMsgs.WaypointClearResponse) => {
        console.log(res);
    })
}

export function set_geofence(geofence: BMFA_Coords[]) {
    if (!geofence_push_client || !set_parameter_client) throw new Error("Client not connected (geofence, parameter)");
    let waypoints = get_geofence_wps(geofence);
    let req: MavrosMsgs.WaypointPushRequest = {
        start_index: 0,
        waypoints: waypoints
    };
    geofence_push_client.callService(req, (res: MavrosMsgs.WaypointPushResponse) => {
        console.log(res.success);
    })
    
    // enable fence (FENCE_ENABLE = 1)
    let enable_geofence_req: MavrosMsgs.ParamSetV2Request = {
        force_set: true,
        param_id: "FENCE_ENABLE",
        value: {
            type: RclInterfaces.ParameterTypeConst.PARAMETER_INTEGER,
            integer_value: 1
        } as RclInterfaces.ParameterValue
    }
    set_parameter_client.callService(enable_geofence_req, (res: MavrosMsgs.ParamSetV2Response) => {
        console.log(res);
    })
}

export function clear_mission() {
    if (!mission_clear_client) throw new Error("Mission clear client not connected");
    mission_clear_client.callService({}, (res: MavrosMsgs.WaypointClearResponse) => {
        console.log(res);
    });
}

export function set_mission(waypoints: MavrosMsgs.Waypoint[]) {
    if (!mission_push_client) throw new Error("Mission push client not connected");
    let req: MavrosMsgs.WaypointPushRequest = {
        start_index: 0,
        waypoints: waypoints
    }
    mission_push_client.callService(req, (res: MavrosMsgs.WaypointPushResponse) => {
        console.log(res);
    })
}

let payload_toggle_value = false;

export function drop_payload() {
    payload_toggle_value = !payload_toggle_value;
    if (!set_parameter_client) throw new Error("Parameter client not connected");
    let req: MavrosMsgs.ParamSetV2Request = {
        force_set: true,
        param_id: "SERVO8_TRIM",
        value: {
            type: RclInterfaces.ParameterTypeConst.PARAMETER_INTEGER,
            integer_value: payload_toggle_value ? 2000 : 1000,
        }
    }
    set_parameter_client.callService(req, (res: MavrosMsgs.ParamSetV2Response) => {
        console.log(res);
    })
}

let status_listener: ROSLIB.Topic | null = null,
    arm_vehicle: ROSLIB.Service | null = null,
    set_path_client: ROSLIB.Service | null = null,
    set_parameter_client: ROSLIB.Service | null = null,
    trajectory_plan_listener: ROSLIB.Topic | null = null,
    trajectory_plan: MpccInterfaces.TrajectoryPlan | null = null;

let path_drag_client: ROSLIB.Service | null = null,
    geofence_clear_client: ROSLIB.Service | null = null,
    geofence_push_client: ROSLIB.Service | null = null,
    mission_clear_client: ROSLIB.Service | null = null,
    mission_push_client: ROSLIB.Service | null = null,
    mission_set_current_client: ROSLIB.Service | null = null;

let mav_data: IMavData = {};
let mav_data_history: IMavData[] = [];

