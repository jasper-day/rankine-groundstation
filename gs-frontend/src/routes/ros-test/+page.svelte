<script lang="ts">
    import ROSLIB, { Topic } from "roslib";
    import { open_ros } from "$lib/mavros";
    import { type MavrosMsgs, type SensorMsgs, type GeometryMsgs, type StdMsgs, type MpccInterfaces  } from "$lib/rostypes/ros_msgs";
    import { onMount } from "svelte";

    onMount(() => {

        var ros = open_ros(
            "ws://localhost:9090",

            onRosConnect,

            function () {
                console.log("Connection error.");
            },

            function () {
                console.log("Connection closed");
            }
        );

        function onRosConnect() {
        var preData = document.getElementById("pre-data");

        let data: Partial<{
            batteryData: SensorMsgs.BatteryState;
            state: MavrosMsgs.State;
            wind: GeometryMsgs.TwistWithCovarianceStamped;
            estimatorStatus: MavrosMsgs.EstimatorStatus;
            compassHeading: StdMsgs.Float64;
            rel_altitude: StdMsgs.Float64;
            gps_fix: SensorMsgs.NavSatFix;
        }> = {};

        let sources: { name: string; interface: string; target: string }[] = [
            { name: "/mavros/battery", interface: "sensor_msgs/msg/BatteryState", target: "batteryData" },
            { name: "/mavros/state", interface: "mavros_msgs/msg/State", target: "state" },
            {
                name: "/mavros/wind_estimation",
                interface: "geometry_msgs/msg/TwistWithCovarianceStamped",
                target: "wind"
            },
            {
                name: "/mavros/estimator_status",
                interface: "mavros_msgs/msg/EstimatorStatus",
                target: "estimatorStatus"
            },
            {
                name: "/mavros/global_position/compass_hdg",
                interface: "std_msgs/msg/Float64",
                target: "compassHeading"
            },
            { name: "/mavros/global_position/rel_alt", interface: "std_msgs/msg/Float64", target: "rel_altitude" },
            { name: "/mavros/global_position/global", interface: "sensor_msgs/msg/NavSatFix", target: "gps_fix" }
        ];

        sources.forEach((element) => {
            const listener = new ROSLIB.Topic({ ros: ros, name: element.name, messageType: element.interface });
            listener.subscribe(function (message) {
                data[element.target] = message
                preData.innerText = `
                Connected: ${data.state?.connected}
                Armed: ${data.state?.armed}
                Mode: ${data.state?.mode}
                Battery percentage: ${data.batteryData?.percentage}
                Wind estimation: (${data.wind?.twist.twist.linear.x}, ${data.wind?.twist.twist.linear.y}, ${data.wind?.twist.twist.linear.z})
                GPS glitch flag: ${data.estimatorStatus?.gps_glitch_status_flag}
                Compass heading: ${data.compassHeading?.data}
                Relative altitude: ${data.rel_altitude?.data}
                Latitude: ${data.gps_fix?.latitude}
                Longitude: ${data.gps_fix?.longitude}
                WGS-84 Altitude MSL: ${data.gps_fix?.altitude}
                `
            })
        });
    };
    });

    

</script>

<div class="m-auto w-1/2 p-10">
    <h1>Showing data from Ros2 with rosbridge</h1>

    <pre id="pre-data" class="p-1">
    Data goes here....
</pre>
</div>
