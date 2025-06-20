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
    // Jasper Day: Controller / frontend integration:
    // Frontend sends information to the controller in the form of a Dubins path
    // Controller sends information to the frontend in the form of
    // - list of positions over the next 6 seconds (continually updated)
    // - Current (simulated) position of the airplane
    // Jasper Day: So we need:
    // - A schema to pass this data back and forth (just some long arrays)

    // Jasper Day: idea (not final)
    // {
    // curr_pos: number[2],
    // projected_pos: number[60],
    // // other things
    // heading: number
    // commanded roll angle: number
    // actual roll angle: number
    // timestep: number
    // currentArclengthAlongPath: number
    // ...
    // }
    //
    //
    // Export to json
    // soft disable regions
    // mode annunciator display
    import "cesium/Build/Cesium/Widgets/widgets.css";

    import {
        Cartesian3,
        Ion,
        Math as CesiumMath,
        Terrain,
        Viewer,
        Cartesian2,
        Ellipsoid,
    } from "cesium";
    import { Arc, HANDLE_POINT_RADIUS, Line, Local3, ORIGIN, quaternion_to_RPY, make_line, make_arc } from "$lib/geometry";
    import { draw_horizon } from "$lib/pfd";
    import "cesium/Build/Cesium/Widgets/widgets.css";
    import { onMount } from "svelte";
    import SensorView from "$lib/SensorView.svelte";
    import { cancelArm, enableArm, get_current_mav_data, get_mav_data_history, get_trajectory_plan, sendPath, type IMavData } from "$lib/mav";
    import { MpccInterfaces } from "$lib/rostypes/ros_msgs";

    Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

    let shapes: (Arc | Line)[] = [];
    // async function close_path() {
    //     let s_shapes = shapes.map((sh) => sh.serialise());
    //     let response = await n.request({ op: "path:solve", data: { path: s_shapes } });
    //     console.log("Got response", response);
    //     if (response instanceof ConnectionError) {
    //         // idk how to handle this TODO
    //     } else if (response instanceof ServerError) {
    //         console.log(
    //             `The server responded with error message '${response.message}' for packet '${JSON.stringify(response.request)}'`
    //         );
    //     } else {
    //         shapes = response.path.map((ds: any) => {
    //             if (ds.type == "arc") {
    //                 return Arc.deserialise(ds);
    //             } else if (ds.type == "line") {
    //                 return Line.deserialise(ds);
    //             }
    //         });
    //     }
    // }

    let viewer: Viewer | undefined;
    let ctx: CanvasRenderingContext2D | null = null;
    let pfd_ctx: CanvasRenderingContext2D | null = null;
    let tool: "Line" | "Arc" | "Empty" = "Empty";
    let editing_mode: "Create" | "Edit" = "Create";
    let intermediate_point: Local3 | undefined = undefined;
    const plane_points: Local3[] = [];
    let data:
        | {
              t: number[];
              u: number[];
              x: [number, number, number, number, number, number, number][];
              solX: number[][][];
          }
        | undefined = undefined;


    function draw_mav_history(history: IMavData[], current: IMavData, plan: MpccInterfaces.TrajectoryPlan | null) {
        if (!viewer || !ctx) return;
        const degrees_heights_array = 
            history
                .map((element) => {
                    return element.gpsFix !== undefined && element.gpsFix.latitude !== 0 && element.gpsFix.longitude !== 0
                        ? [element.gpsFix?.longitude, element.gpsFix?.latitude, element.gpsFix?.altitude]
                        : [];
                })
                .flat();

        if (degrees_heights_array.length < 2 * 3) return;

        const cartesians = Cartesian3.fromDegreesArrayHeights(
            degrees_heights_array,
            Ellipsoid.WGS84
        );

        let a = viewer.scene.cartesianToCanvasCoordinates(cartesians[0], scratchc3_a);

        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.strokeStyle = "red";
        ctx.fillStyle = "#ffd040";
        for (let i = 1; i < cartesians.length; i += 1) {
            if (cartesians[i].x === cartesians[i - 1].x) {
                continue;
            }
            const b = viewer.scene.cartesianToCanvasCoordinates(cartesians[i], scratchc3_b);
            ctx.lineTo(b.x, b.y);
        }
        a = viewer.scene.cartesianToCanvasCoordinates(cartesians[cartesians.length - 1], scratchc3_a);
        ctx.lineTo(a.x, a.y);
        ctx.stroke();

        if (plan) {
            ctx.strokeStyle = "blue";
            ctx.fillStyle = "#ffd040";
            ctx.beginPath();
            let local: Local3 | null = null;
            for (let i = 0; i != plan.norths.length; i++) {
                local = new Local3(plan.easts[i], plan.norths[i], 100);
                let b = viewer.scene.cartesianToCanvasCoordinates(local.toCartesian(), scratchc3_b);
                i == 0 ? ctx.moveTo(a.x, a.y) : ctx.lineTo(b.x, b.y);
            }
            ctx.stroke();
        }

        // draw that pointer thing
        if (!current.compassHeading) return;
        curr_heading = ((current.compassHeading.data || 0.0) * Math.PI) / 180;
        ctx.strokeStyle = "#f00000";
        ctx.fillStyle = "#ff4030";
        ctx.save();
        ctx.translate(a.x, a.y);
        ctx.rotate(curr_heading - Math.PI / 2);
        ctx.beginPath();
        const l = 10;
        const w = 8;
        ctx.moveTo(l, 0);
        ctx.lineTo(-l, w);
        ctx.lineTo(-l / 2, 0);
        ctx.lineTo(-l, -w);
        ctx.lineTo(l, 0);
        ctx.fill();
        ctx.stroke();
        ctx.restore();
    }

    // to avoid instantiating objects continuously
    // may be premature optimisation but cesium does it so i will too
    const scratchc3_a: Cartesian3 = new Cartesian3();
    const scratchc3_b: Cartesian3 = new Cartesian3();

    function draw_intermediate_shape(viewer: Viewer, intermediate_point: Local3, ctx: CanvasRenderingContext2D) {
        // TODO costly operation to undo later on...
        const mouse_cartesian = viewer.camera.pickEllipsoid(new Cartesian3(mouseX, mouseY), viewer.scene.ellipsoid);
        if (!mouse_cartesian) return;
        const mouse_local = Local3.fromCartesian(mouse_cartesian);
        mouse_local.z = 100; // for now, we define the path always at 100m above surface

        const c = viewer.scene.cartesianToCanvasCoordinates(intermediate_point.toCartesian(), scratchc3_a);
        if (Cartesian2.distance(c, new Cartesian2(mouseX, mouseY)) < 5) {
            if (has_moved_away) {
                tool = tool == "Line" ? "Arc" : "Line";
                has_moved_away = false;
                return;
            }
        } else {
            has_moved_away = true;
        }

        if (tool == "Line") {
            make_line(intermediate_point, shapes[shapes.length - 1], mouse_local).draw(ctx, viewer);
        }
        if (tool == "Arc" && intermediate_point !== undefined) {
            const p1 = intermediate_point;
            const p2 = mouse_local;
            // Points are too close, can't make an arc
            if (Local3.distance(p1, p2) < 0.01) return;

            const line = shapes[shapes.length - 1];
            let tangent;
            if (line instanceof Line) {
                tangent = line.end.sub(line.start);
            } else {
                tangent = line.tangent_at_endpoint();
            }

            Arc.from_tangent_and_points(tangent, p1, p2).draw(ctx, viewer);
        }
    }
    let roll_command = 0;
    let curr_heading = 0;

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
                draw_tooltip(mouseX, mouseY);
                for (const s of shapes) {
                    s.draw(ctx, viewer);
                }

                if (intermediate_point !== undefined) {
                    // some shape is being defined!! draw it
                    draw_intermediate_shape(viewer, intermediate_point, ctx);
                }
                const mav_history = get_mav_data_history();
                const plan = get_trajectory_plan();
                draw_mav_history(mav_history, mav_data, plan);
            }
            if (pfd_ctx) {
                let pitch = 0;
                let roll = 0;
                let alt = 0;
                let connected = false;
                if (mav_data.localPose && mav_data.relAltitude) {
                    const rpy = quaternion_to_RPY(mav_data.localPose?.pose.orientation);
                    pitch = -rpy.pitch;
                    roll = -rpy.roll;
                    alt = mav_data.relAltitude?.data;
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
                    armed: false,
                });
            }

            // current, voltage, power, rpm, throttle, vibration, ground speed, wind speed, gps altitude, rssi
            sensors[0][1].push(mav_data.batteryData?.current || 0.0);
            sensors[1][1].push(mav_data.batteryData?.voltage || 0.0);
            sensors[2][1].push(mav_data.batteryData?.voltage || 0.0 * (mav_data.batteryData?.current || 0.0));
            sensors[7][1].push(
                Math.sqrt(
                    (mav_data.wind?.twist.twist.linear.x || 0.0) ** 2 + (mav_data.wind?.twist.twist.linear.y || 0.0) ** 2
                )
            );
            sensors[8][1].push(mav_data.gpsFix?.altitude || 0.0);
            draw_graphs();
        });

        // Chrome doesn't support mouse events
        viewer.cesiumWidget.canvas.addEventListener("pointerdown", mousedown);
        viewer.cesiumWidget.canvas.addEventListener("pointerup", mouseup);
    });

    let drag_object: { shape_index: number; point_index: number } | undefined = undefined;
    let has_moved_away = false;
    function mousedown(event: MouseEvent) {
        if (viewer !== undefined) {
            const cartesian = viewer.camera.pickEllipsoid(
                new Cartesian3(event.layerX, event.layerY),
                viewer.scene.ellipsoid
            );
            if (!cartesian) return; // just ignore an invalid position
            const mouse_local = Local3.fromCartesian(cartesian);
            mouse_local.z = 100; // for now, we define the path always at 100m above surface

            if (tool == "Line") {
                if (intermediate_point === undefined) {
                    // add first point
                    intermediate_point = mouse_local;
                } else {
                    // finish the line
                    const line = make_line(intermediate_point, shapes[shapes.length - 1], mouse_local);
                    shapes.push(line);
                    intermediate_point = line.end;
                    tool = "Arc";
                    has_moved_away = false;
                }
            } else if (tool == "Arc") {
                if (intermediate_point === undefined) {
                    // TODO maybe include old code for defining first arc?
                } else {
                    const val = make_arc(intermediate_point, shapes[shapes.length - 1], mouse_local);

                    if (val !== undefined) {
                        const [arc, endpoint] = val;
                        shapes.push(arc);
                        intermediate_point = endpoint;
                        tool = "Line";
                        has_moved_away = false;
                    }
                }
            } else if (tool == "Empty") {
                // check for dragging
                const mouse = new Cartesian2(event.layerX, event.layerY);
                let idx = 0;
                const max_sq_dist = HANDLE_POINT_RADIUS * HANDLE_POINT_RADIUS;
                for (const s of shapes) {
                    if (s instanceof Line) {
                        const a = viewer.scene.cartesianToCanvasCoordinates(s.start.toCartesian(), scratchc3_a);
                        const b = viewer.scene.cartesianToCanvasCoordinates(s.end.toCartesian(), scratchc3_b);

                        if (Cartesian2.distanceSquared(a, mouse) < max_sq_dist) {
                            drag_object = { shape_index: idx, point_index: 0 };
                        }
                        if (Cartesian2.distanceSquared(b, mouse) < max_sq_dist) {
                            drag_object = { shape_index: idx, point_index: 1 };
                        }
                        const handle_points = s.allowed_area_handle_points(viewer);
                        let i = 0;
                        for (const p of handle_points) {
                            if (Cartesian2.distanceSquared(p, mouse) < max_sq_dist) {
                                drag_object = { shape_index: idx, point_index: i + 2 };
                            }
                            i += 1;
                        }
                    } else if (s instanceof Arc) {
                        // TODO not needed? just do s.get_endpoint & convert it to screenspace
                        let points = [s.centre, s.get_endpoint_local("Start"), s.get_endpoint_local("End")];
                        let i = 0;
                        for (const c of points) {
                            if (
                                Cartesian2.distanceSquared(
                                    viewer.scene.cartesianToCanvasCoordinates(c.toCartesian()),
                                    mouse
                                ) < max_sq_dist
                            ) {
                                drag_object = { shape_index: idx, point_index: i };
                                break;
                            }
                            i += 1;
                        }

                        // const handle_points = s.allowed_region_handle_points(viewer);
                        // for (const p of handle_points) {
                        //     if (Cartesian2.distanceSquared(p, mouse) < max_sq_dist) {
                        //         drag_object = { shape_index: idx, point_index: i };
                        //         break;
                        //     }
                        //     i += 1;
                        // }
                    }
                    idx += 1;
                }
                if (drag_object !== undefined) {
                    viewer.scene.screenSpaceCameraController.enableInputs = false;
                }
            }
        }
    }

    function mouseup(_: MouseEvent) {
        if (drag_object !== undefined) {
            if (viewer) viewer.scene.screenSpaceCameraController.enableInputs = true;
            drag_object = undefined;
        }
    }

    function keypress(event: KeyboardEvent) {
        if (event.key == "l") {
            tool = "Line";
            // Make next segment start at the end of the previous, if it exists
            if (shapes.length > 0) {
                const shape = shapes[shapes.length - 1];
                if (shape instanceof Line) {
                    intermediate_point = shape.end;
                } else if (shape instanceof Arc) {
                    intermediate_point = new Local3(
                        shape.centre.x + shape.radius * Math.cos(-Arc.NEtoXY(shape.theta0 + shape.dangle)),
                        shape.centre.y + shape.radius * Math.sin(-Arc.NEtoXY(shape.theta0 + shape.dangle)),
                        shape.centre.z
                    );
                }
            }
        } else if (event.key == "Escape") {
            tool = "Empty";
            intermediate_point = undefined;
            has_moved_away = false;
        } else if (event.key == "c") {
            editing_mode = "Create";
        } else if (event.key == "e") {
            close_path();
            editing_mode = "Edit";
        } else if (event.key == "1" && viewer) {
            viewer.camera.cancelFlight();
            viewer.camera.flyTo({
                destination: Cartesian3.fromRadians(ORIGIN.longitude, ORIGIN.latitude, 1000),
                orientation: {
                    heading: CesiumMath.toRadians(0.0),
                    pitch: CesiumMath.toRadians(-90.0)
                },
                duration: 0
            });
        }
        draw_tooltip(mouseX, mouseY);
    }

    let mouseX: number, mouseY: number;
    function mousemove(event: MouseEvent) {
        mouseX = event.layerX;
        mouseY = event.layerY;
        if (drag_object !== undefined && viewer !== undefined) {
            const ellipsoid = viewer.scene.ellipsoid;
            const cartesian = viewer.camera.pickEllipsoid(new Cartesian3(mouseX, mouseY), ellipsoid);
            if (!cartesian) return; // just ignore an invalid position
            let local = Local3.fromCartesian(cartesian);
            local.z = 0;
            let s = shapes[drag_object.shape_index];
            if (s instanceof Line) {
                if (drag_object.point_index == 0) {
                    s.start = local;
                } else if (drag_object.point_index == 1) {
                    s.end = local;
                } else {
                    let index = drag_object.point_index - 2;
                    let point;
                    if (index < 2) {
                        point = s.start;
                    } else {
                        point = s.end;
                    }
                    // TODO actually project this onto a line normal to the line...
                    s.width[index] = Local3.distance(point, local);
                }
            } else if (s instanceof Arc) {
                if (drag_object.point_index == 0) {
                    // move centre
                    s.centre = local;
                } else if (drag_object.point_index <= 2) {
                    // const p = s.get_screenspace_params(viewer);
                    // TODO maybe like idk, sqitch direcqtuion simetimes?
                    let a, b;
                    if (drag_object.point_index == 1) {
                        a = local;
                        b = s.get_endpoint_local("End");
                    } else {
                        a = s.get_endpoint_local("Start");
                        b = local;
                    }
                    const r = Local3.distance(local, s.centre);
                    const new_s = Arc.from_centre_and_points(s.centre, a, b, s.dangle < 0 ? -1 : 1, r);
                    shapes[drag_object.shape_index] = new_s;
                    shapes[drag_object.shape_index].width = s.width;
                } else {
                    let index = drag_object.point_index - 3;
                    let point;
                    if (index < 2) {
                        point = s.get_endpoint_local("Start");
                    } else {
                        point = s.get_endpoint_local("End");
                    }
                    // TODO actually project this onto a line normal to the arc...
                    s.width[index] = Local3.distance(point, local);
                }
            }
            // update adjacent widths to maintain C0 continuity
            let index;
            if (s instanceof Line) index = drag_object.point_index - 2;
            else index = drag_object.point_index - 3;
            if (index >= 0) {
                if (index < 2 && drag_object.shape_index > 0) {
                    shapes[drag_object.shape_index - 1].width[index + 2] = s.width[index];
                }
                if (index >= 2 && drag_object.shape_index < shapes.length - 1) {
                    shapes[drag_object.shape_index + 1].width[index - 2] = s.width[index];
                }
            }

            if (editing_mode == "Edit") {
                close_path();
            }
        }
    }

    function draw_tooltip(nx: number, ny: number) {
        if (ctx && editing_mode == "Create") {
            ctx.strokeStyle = "red";
            const x = nx;
            const y = ny - 16;
            let d = 6;
            ctx.beginPath();
            ctx.moveTo(x - d, y);
            ctx.lineTo(x + d, y);
            ctx.moveTo(x, y + d);
            ctx.lineTo(x, y - d);
            ctx.stroke();
        }
        if (tool === "Empty") {
            return;
        }
        if (ctx) {
            const x = nx - 16;
            const y = ny + 16;
            ctx.strokeStyle = "red";
            if (tool == "Arc") {
                ctx.beginPath();
                ctx.arc(x, y, 10, Math.PI / 2, (2 * Math.PI) / 3 + Math.PI / 2);
            } else if (tool == "Line") {
                ctx.beginPath();
                ctx.moveTo(x, y);
                ctx.lineTo(x + 16, y + 16);
            }
            ctx.stroke();
        }
    }

    function gen_data(x: number): number[] {
        let datas = [x];
        for (let i = 0; i < Math.random() * 200; i++) {
            const prev = datas[datas.length - 1];
            datas.push(prev + x * 0.05 * (Math.random() - 0.5));
        }
        return datas;
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
    <SensorView {sensors} bind:draw_graphs={draw_graphs} />
</div>
<canvas id="canvas" bind:this={canvas}></canvas>
<div id="cesiumContainer"></div>
<div class="fixed bottom-4 right-4 z-50 h-64 w-80 overflow-hidden">
    <div class="rounded bg-black/70" style="height: 100%">
        <div id="arm-button" class="flex-column flex border-b border-white/20">
            <button class="btn btn-gray" on:click={enableArm}>Arm</button>
            <button class="btn btn-gray" on:click={cancelArm}>Cancel&nbsp;Arm</button>
            <button class="btn btn-gray" on:click={() => sendPath(shapes)}>Send&nbsp;path</button>
        </div>
        <div class="border-b border-white/20 p-3">
            <h3 class="text-sm font-medium text-white">Messages</h3>
        </div>
        <div class="h-full space-y-2 overflow-y-auto p-3" id="status-message-container"></div>
    </div>
</div>
<svelte:window on:keydown={keypress} on:mousemove|preventDefault={mousemove} />

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
