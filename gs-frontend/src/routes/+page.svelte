<script lang="ts">
    // TODO !IMPORTANT
    // undo
    // delete
    // look into pointer events in chrome (no drag?)
    // canvas resizing
    // split path
    // reconnect to backend + show error
    // fix arrow rendering on arcs when in 3d mode
    // fix plane indicator same problem as above

    // drag plane??
    // Export to json
    // mode annunciator display
    // scale markings
    // distance from mouse to nearest waypoint for positioning
    // export / save paths

    import "cesium/Build/Cesium/Widgets/widgets.css";

    import {
        Cartesian3,
        Ion,
        Math as CesiumMath,
        Terrain,
        Viewer,
        Cartesian2,
        Ellipsoid,
        UrlTemplateImageryProvider,
        CzmlDataSource,
        Cartographic,
        viewerCesium3DTilesInspectorMixin
    } from "cesium";
    import {
        Arc,
        HANDLE_POINT_RADIUS,
        Line,
        Local2,
        ORIGIN,
        quaternion_to_RPY,
        make_line,
        make_arc,
        serialise_path,
        deserialise_path,
        path_from_points
    } from "$lib/geometry";
    import { draw_horizon } from "$lib/pfd";
    import "cesium/Build/Cesium/Widgets/widgets.css";
    import { onMount } from "svelte";
    import SensorView from "$lib/SensorView.svelte";
    import {
        cancel_arm,
        clear_geofence,
        converge_path,
        drop_payload,
        enable_arm,
        get_current_mav_data,
        get_mav_data_history,
        get_trajectory_plan,
        send_waypoints,
        set_geofence,
        terminate
    } from "$lib/mav";
    import { Coord_Type, to_czml, type BMFA_Coords } from "$lib/waypoints";
    import { draw_coordinates, draw_mav_history, draw_waypoint_distances } from "$lib/graphics";
    import {
        draw_path,
        get_path,
        get_path_points,
        keypress,
        mousedown,
        mousemove,
        mouseup,
        mouseX,
        mouseY,
        set_path,
        set_path_points
    } from "$lib/edit_path";
    import { download_string, export_waypoints, get_dubins_wps } from "$lib/mission";

    Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

    let shapes: (Arc | Line)[] = [];

    let viewer: Viewer | undefined;
    let ctx: CanvasRenderingContext2D | null = null;
    let pfd_ctx: CanvasRenderingContext2D | null = null;
    const plane_points: Local2[] = [];
    let data:
        | {
              t: number[];
              u: number[];
              x: [number, number, number, number, number, number, number][];
              solX: number[][][];
          }
        | undefined = undefined;

    let roll_command = 0;
    let curr_heading = 0;
    let coordinates: BMFA_Coords[] | null = null;
    let geofence: BMFA_Coords[] | null = null;
    let waypoints: BMFA_Coords[] | null = null;
    let payload: BMFA_Coords | null = null;

    // to avoid instantiating objects continuously
    // may be premature optimisation but cesium does it so i will too
    const scratchc3_a: Cartesian3 = new Cartesian3();
    const scratchc3_b: Cartesian3 = new Cartesian3();

    onMount(() => {
        // Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
        viewer = new Viewer("cesiumContainer", {
            terrain: Terrain.fromWorldTerrain(),
            fullscreenButton: false,
            homeButton: false,
            sceneModePicker: false,
            timeline: false,
            animation: false,
            // selectionIndicator: false,
            navigationInstructionsInitiallyVisible: false
        });
        ctx = canvas.getContext("2d");
        pfd_ctx = pfd.getContext("2d");

        fetch("/coordinates.json").then((res) => {
            if (res.ok) {
                res.json().then((res: { name: string; latitude: number; longitude: number }[]) => {
                    coordinates = res.map((item) => {
                        let coordinate: Partial<BMFA_Coords> = item;
                        if (item.name.startsWith("R")) {
                            coordinate.type = Coord_Type.RUNWAY;
                        } else if (item.name.startsWith("G")) {
                            coordinate.type = Coord_Type.GEOFENCE;
                        } else if (item.name.startsWith("W")) {
                            coordinate.type = Coord_Type.WAYPOINT;
                        } else if (item.name.startsWith("P")) {
                            coordinate.type = Coord_Type.PAYLOAD;
                        } else {
                            console.error("Could not parse coordinate", item);
                        }
                        return coordinate;
                    }) as BMFA_Coords[];
                    geofence = coordinates.filter((coord) => coord.type == Coord_Type.GEOFENCE);
                    waypoints = coordinates.filter((coord) => coord.type == Coord_Type.WAYPOINT);
                    payload = coordinates.find((coord) => coord.type == Coord_Type.PAYLOAD) ?? null;
                    const waypoint_czml: object[] = waypoints?.map((wp) => to_czml(wp));
                    waypoint_czml?.unshift({
                        id: "document",
                        name: "Waypoints",
                        version: "1.0"
                    });
                    console.log(waypoint_czml);

                    viewer?.dataSources.add(CzmlDataSource.load(waypoint_czml));
                });
            } else {
                console.log("Asset not found: coordinates.json");
            }
        });

        // Fly the camera to the origin longitude, latitude, and height.
        viewer.camera.flyTo({
            destination: Cartesian3.fromRadians(ORIGIN.longitude, ORIGIN.latitude, 1000),
            orientation: {
                heading: CesiumMath.toRadians(0.0),
                pitch: CesiumMath.toRadians(-90.0)
            },
            duration: 0
        });
        viewer.camera.switchToOrthographicFrustum();

        viewer.clock.onTick.addEventListener((_: any) => {
            const mav_data = get_current_mav_data();
            if (viewer && ctx) {
                if (canvas.width != viewer.canvas.width) {
                    canvas.width = viewer.canvas.width;
                }
                if (canvas.height != viewer.canvas.height) {
                    canvas.height = viewer.canvas.height;
                }
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                draw_path(ctx, viewer);
                const mav_history = get_mav_data_history();
                const plan = get_trajectory_plan();
                draw_mav_history(mav_history, mav_data, plan, viewer, ctx, curr_heading);
                coordinates && geofence && draw_coordinates(coordinates, ctx, viewer, geofence);
                waypoints && draw_waypoint_distances(ctx, viewer, waypoints, mouseX, mouseY);
            }
            if (pfd_ctx) {
                let pitch = 0;
                let roll = 0;
                let alt = 0;
                let connected = false;
                if (mav_data.imuData /*&& mav_data.relAltitude*/) {
                    const rpy = quaternion_to_RPY(mav_data.imuData?.orientation);
                    pitch = -rpy.pitch;
                    roll = -rpy.roll;
                    // alt = mav_data.relAltitude?.data;
                    connected = true;
                }
                draw_horizon(pfd, pfd_ctx, {
                    pitch,
                    roll,
                    ias: 10.135,
                    alt,
                    command_roll: roll_command,
                    gs: 10.3513,
                    ws: 314.3,
                    heading: (curr_heading * 180) / Math.PI,
                    connected,
                    armed: false
                });
            }

            // current, voltage, power, rpm, throttle, vibration, ground speed, wind speed, gps altitude, rssi
            sensors[0][1].push(mav_data.batteryData?.current || 0.0);
            sensors[1][1].push(mav_data.batteryData?.voltage || 0.0);
            sensors[2][1].push(mav_data.batteryData?.voltage || 0.0 * (mav_data.batteryData?.current || 0.0));
            sensors[7][1].push(
                Math.sqrt(
                    (mav_data.wind?.twist.twist.linear.x || 0.0) ** 2 +
                        (mav_data.wind?.twist.twist.linear.y || 0.0) ** 2
                )
            );
            sensors[8][1].push(mav_data.gpsFix?.altitude || 0.0);
            draw_graphs();
        });

        // Chrome doesn't support mouse events
        function on_mousedown(e: MouseEvent) {
            if (viewer == undefined) return;

            mousedown(viewer, e);
        }

        function on_mouseup(e: MouseEvent) {
            if (viewer == undefined) return;
            mouseup(viewer, e);
        }

        function on_mousemove(e: MouseEvent) {
            if (viewer == undefined || !ctx) return;
            mousemove(viewer, ctx, e);
        }

        function on_keypress(e: KeyboardEvent) {
            if (viewer == undefined || !ctx) return;
            keypress(viewer, ctx, e);
        }

        viewer.cesiumWidget.canvas.addEventListener("pointerdown", on_mousedown);
        viewer.cesiumWidget.canvas.addEventListener("pointerup", on_mouseup);
        window.addEventListener("mousemove", on_mousemove);
        window.addEventListener("keypress", on_keypress);

        let path_points = localStorage.getItem("path_points");
        if (path_points) {
            set_path_points(JSON.parse(path_points).map((l: { x: number; y: number }) => new Local2(l.x, l.y)));
        }

        setInterval(() => {
            localStorage.setItem(
                "path_points",
                JSON.stringify(
                    get_path_points().map((l) => {
                        return { x: l.x, y: l.y };
                    })
                )
            );
            console.log(JSON.stringify(get_path_points()));
        }, 5000);
    });

    function gen_data(x: number): number[] {
        let datas = [x];
        for (let i = 0; i < Math.random() * 200; i++) {
            const prev = datas[datas.length - 1];
            datas.push(prev + x * 0.05 * (Math.random() - 0.5));
        }
        return datas;
    }

    function export_waypoints_gs() {
        let path_points = get_path_points();
        let path = path_from_points(path_points);
        let waypoints = get_dubins_wps(path, 40);
        let ex = export_waypoints(waypoints);
        download_string(ex, "text/csv", ".waypoints");
    }

    let sensors: [string, number[], string][] = [
        ["Current", gen_data(13.3), "A"],
        ["Voltage", gen_data(44.4), "V"],
        ["Power", gen_data(1353), "W"],
        ["RPM", gen_data(1389), "rpm"],
        ["Throttle", gen_data(10), "%"],
        ["Vibration", gen_data(113), ""],
        ["Ground speed", gen_data(10), "m/s"],
        ["Wind speed", gen_data(10.13), "m/s"],
        ["GPS altitude", gen_data(35), "m"],
        ["RSSI", gen_data(10), "dBm"]
    ];

    let draw_graphs: () => void;
    let canvas: HTMLCanvasElement;
    let pfd: HTMLCanvasElement;
</script>

<div id="data">
    <canvas id="pfd" bind:this={pfd}></canvas>
    <SensorView {sensors} bind:draw_graphs />
</div>
<canvas id="canvas" bind:this={canvas}></canvas>
<div id="cesiumContainer"></div>
<div class="fixed right-4 bottom-4 z-50 h-94 w-80 overflow-hidden">
    <div class="rounded bg-black/70" style="height: 100%">
        <div id="arm-button" class="grid-columns-3 border-b border-white/20">
            <button class="btn btn-gray" on:click={enable_arm}>Arm</button>
            <button class="btn btn-gray" on:click={cancel_arm}>Cancel&nbsp;Arm</button>
            <button class="btn btn-gray" on:click={() => send_waypoints(shapes, 40)}>Send&nbsp;path</button>
            <button
                class="btn btn-gray"
                on:click={() => {
                    if (!geofence) return;
                    clear_geofence;
                    console.log(geofence);
                    set_geofence(geofence);
                }}>Send&nbsp;Geofence</button
            >
            <button class="btn btn-gray" on:click={terminate}>FTS</button>
            <button class="btn btn-gray" on:click={drop_payload}>Drop payload</button>
            <button class="btn btn-gray" on:click={export_waypoints_gs}>Export</button>
        </div>
        <div class="border-b border-white/20 p-3"></div>
        <div class="h-full space-y-2 overflow-y-auto p-3" id="status-message-container"></div>
    </div>
</div>

<style lang="postcss">
    @import "tailwindcss";
    #canvas {
        z-index: 2;
        position: absolute;
        pointer-events: none;
        width: 70%;
        height: 100%;
        top: 0;
        left: 30%;
    }

    #cesiumContainer {
        height: 100%;
        width: 70%;
        z-index: 1;
        position: absolute;
        top: 0;
        left: 30%;
    }
    #pfd {
        height: 52%;
        min-height: 52%;
        width: 100%;
    }

    #data {
        display: flex;
        flex-direction: column;
        position: absolute;
        width: 30%;
        height: 100%;
        top: 0;
        left: 0;
    }

    .btn {
        @apply mx-2 my-2 rounded px-2 py-1 font-bold;
        flex: 1;
    }

    .btn-gray {
        border: 1px solid #3c424f;
        border-radius: 6px;
        color: #d3d3d3;
        background-color: #272e39;
    }

    .btn-gray:hover {
        @apply bg-gray-400;
    }
</style>
