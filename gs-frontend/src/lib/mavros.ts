import ROSLIB from "roslib"

export function open_ros(url: string, onconnect: ()=>void, onerror: ()=>void, onclose: ()=>void) {
    var ros = new ROSLIB.Ros(
        { url: url }
    );
    ros.on("connection", onconnect);
    ros.on("error", onerror);
    ros.on("close", onclose);
    return ros;
}

