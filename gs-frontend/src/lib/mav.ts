import ROSLIB from "roslib";
import {
    MavrosMsgs,
    MpccInterfaces,
    SensorMsgs,
    type GeographicMsgs,
    type GeometryMsgs,
    type StdMsgs,
    type StdSrvs
} from "$lib/rostypes/ros_msgs";
import { open_ros } from "$lib/mavros";
import { ORIGIN, type Arc, type Line } from "./geometry";

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
                classes.push("text-red", "font-bold");
                break;
            case MavrosMsgs.StatusTextSeverity.ALERT:
                classes.push("text-red");
                break;
            case MavrosMsgs.StatusTextSeverity.CRITICAL:
                classes.push("text-red", "font-bold");
                break;
            case MavrosMsgs.StatusTextSeverity.ERROR:
                classes.push("text-red");
                break;
            case MavrosMsgs.StatusTextSeverity.WARNING:
                classes.push("text-orange");
                break;
            case MavrosMsgs.StatusTextSeverity.NOTICE:
                classes.push("text-yellow");
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
        if (mav_data_history.length > 40960) {
            mav_data_history.unshift();
        }
    }, 20);

    statusListener = new ROSLIB.Topic({
        ros: ros,
        name: "mavros/statustext/recv",
        messageType: "mavros_msgs/msg/StatusText"
    });
    statusListener.subscribe(updateStatusText);

    armVehicle = new ROSLIB.Service({
        ros: ros,
        name: "gs/try_arm",
        serviceType: "std_srvs/srv/SetBool"
    });

    setPathService = new ROSLIB.Service({
        ros: ros,
        name: "gs/set_path",
        serviceType: "mpcc_interfaces/srv/SetPath"
    });

    trajectory_plan_listener = new ROSLIB.Topic({
        ros: ros,
        name: "gs/trajectory_plan",
        messageType: "mpcc_interfaces/msg/TrajectoryPlan"
    });
    trajectory_plan_listener.subscribe(updateTrajectoryPlan);
}

export function enableArm() {
    if (armVehicle) {
        const req: StdSrvs.SetBoolRequest = {
            data: true
        };
        armVehicle.callService(req, function (result: StdSrvs.SetBoolResponse) {
            console.log("Attempting arm");
            console.log("Success", result.success);
            console.log("Response", result.message);
        });
    }
}

export function cancelArm() {
    if (armVehicle) {
        const req: StdSrvs.SetBoolRequest = {
            data: false
        };
        armVehicle.callService(req, function (result: StdSrvs.SetBoolResponse) {
            console.log("Attempting disarm");
            console.log("Success", result.success);
            console.log("Response", result.message);
        });
    }
}

export function sendPath(shapes: (Line | Arc)[]) {
    if (setPathService) {
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
        setPathService.callService(req, function (res: MpccInterfaces.SetPathResponse) {
            console.log("Sent path");
            console.log("Success", res.success);
            console.log(res.error_msg);
        });
    }
}

function updateTrajectoryPlan(msg: MpccInterfaces.TrajectoryPlan) {
    trajectory_plan = msg;
}

const ros = open_ros(
    "ws://126.158.118:9090",

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

let statusListener: ROSLIB.Topic | null = null,
    armVehicle: ROSLIB.Service | null = null,
    setPathService: ROSLIB.Service | null = null,
    trajectory_plan_listener: ROSLIB.Topic | null = null,
    trajectory_plan: MpccInterfaces.TrajectoryPlan | null = null;

let mav_data: IMavData = {};
let mav_data_history: IMavData[] = [];
