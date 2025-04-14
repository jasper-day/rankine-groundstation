<script lang="ts">
    // TODO !IMPORTANT
    // undo
    // delete
    // look into pointer events in chrome (no drag?)
    // canvas resizing
    // split path
    // reconnect to backend + show error
    // line tangent to previous arc if applicable
    // make regions C0 continuous
    import "cesium/Build/Cesium/Widgets/widgets.css";

    import { Cartesian3, Ion, Math as CesiumMath, Terrain, Viewer, Cartesian2 } from "cesium";
    import { Arc, HANDLE_POINT_RADIUS, Line, Local3, ORIGIN } from "$lib/geometry";
    import { ConnectionError, Network, ServerError } from "$lib/network";
    import "cesium/Build/Cesium/Widgets/widgets.css";
    import { onMount } from "svelte";

    Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN;

    class Shapes {
        shapes: (Arc | Line)[] = [];
        constructor() {}

        add_shape(s: Arc | Line) {
            this.shapes.push(s);
        }
        async close_path() {
            let s_shapes = this.shapes.map((sh) => sh.serialise());
            let response = await n.request({ op: "path:solve", data: { path: s_shapes } });
            console.log("Got response", response);
            if (response instanceof ConnectionError) {
                // idk how to handle this TODO
            } else if (response instanceof ServerError) {
                console.log(
                    `The server responded with error message '${response.message}' for packet '${JSON.stringify(response.request)}'`
                );
            } else {
                this.shapes = response.path.map((ds: any) => {
                    if (ds.type == "arc") {
                        return Arc.deserialise(ds);
                    } else if (ds.type == "line") {
                        return Line.deserialise(ds);
                    }
                });
            }
        }
    }

    const shapes = new Shapes();

    let viewer: Viewer | undefined;
    let ctx: CanvasRenderingContext2D | null = null;
    let tool: "Line" | "Arc" | "Empty" = "Empty";
    let editing_mode: "Create" | "Edit" = "Create";
    let intermediate_points: Local3[] = [];

    // to avoid instantiating objects continuously
    // may be premature optimisation but cesium does it so i will too
    const scratchc3_a: Cartesian3 = new Cartesian3();
    const scratchc3_b: Cartesian3 = new Cartesian3();

    let n = new Network();
    n.connect().then(console.log); // TODO handle errors

    onMount(() => {
        // Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
        viewer = new Viewer("cesiumContainer", {
            terrain: Terrain.fromWorldTerrain()
        });
        ctx = canvas.getContext("2d");
        // TODO update this when window resizes
        canvas.width = viewer.canvas.width;
        canvas.height = viewer.canvas.height;

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
            if (viewer && ctx) {
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                draw_tooltip(mouseX, mouseY);
                for (const s of shapes.shapes) {
                    s.draw(ctx, viewer);
                }

                if (intermediate_points.length > 0) {
                    // some shape is being defined!! draw it
                    // TODO costly operation to undo later on...
                    const mouse_cartesian = viewer.camera.pickEllipsoid(
                        new Cartesian3(mouseX, mouseY),
                        viewer.scene.ellipsoid
                    );
                    if (!mouse_cartesian) return;
                    const mouse_local = Local3.fromCartesian(mouse_cartesian);
                    mouse_local.z = 100; // for now, we define the path always at 100m above surface

                    const c = viewer.scene.cartesianToCanvasCoordinates(
                        intermediate_points[0].toCartesian(),
                        scratchc3_a
                    );
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
                        let end;
                        if (shapes.shapes.length > 0) {
                            const dist = Local3.distance(intermediate_points[0], mouse_local);
                            const s = shapes.shapes[shapes.shapes.length - 1];
                            let dir;
                            if (s instanceof Line) {
                                dir = s.end.sub(s.start);
                                dir = dir.mul(1.0 / dir.mag());
                            } else {
                                dir = s.tangent_at_endpoint();
                            }
                            end = dir.mul(dist);
                        } else {
                            end = mouse_local;
                        }
                        new Line(intermediate_points[0], end).draw(ctx, viewer);
                    }
                    if (tool == "Arc" && intermediate_points.length > 0) {
                        const p1 = intermediate_points[0];
                        const p2 = mouse_local;
                        // Points are too close, can't make an arc
                        if (Local3.distance(p1, p2) < 0.01) return;

                        const line = shapes.shapes[shapes.shapes.length - 1];
                        let tangent;
                        if (line instanceof Line) {
                            tangent = line.end.sub(line.start);
                        } else {
                            // uhh find tangent of arc
                            tangent = line.tangent_at_endpoint();
                        }

                        Arc.from_tangent_and_points(tangent, p1, p2).draw(ctx, viewer);
                    }
                }
            }
        });

        // Add Cesium OSM Buildings, a global 3D buildings layer.
        // createOsmBuildingsAsync().then((buildingTileset) => viewer.scene.primitives.add(buildingTileset));

        // Chrome doesn't support mouse events
        viewer.cesiumWidget.canvas.addEventListener("pointerdown", mousedown);
        viewer.cesiumWidget.canvas.addEventListener("pointerup", mouseup);
    });

    let drag_object: { shape_index: number; point_index: number } | undefined = undefined;
    let has_moved_away = false;
    function mousedown(event: MouseEvent) {
        // console.log(event);
        if (viewer !== undefined) {
            const cartesian = viewer.camera.pickEllipsoid(
                new Cartesian3(event.clientX, event.clientY),
                viewer.scene.ellipsoid
            );
            if (!cartesian) return; // just ignore an invalid position
            const mouse_local = Local3.fromCartesian(cartesian);
            mouse_local.z = 100; // for now, we define the path always at 100m above surface

            if (tool == "Line") {
                if (intermediate_points.length == 0) {
                    // add first point
                    intermediate_points.push(mouse_local);
                } else {
                    const p1 = intermediate_points[0];
                    const p2 = mouse_local;
                    shapes.add_shape(new Line(p1, p2));
                    intermediate_points = [mouse_local];
                    tool = "Arc";
                    has_moved_away = false;
                }
            } else if (tool == "Arc") {
                // TODO maybe include old code for defining first arc?
                const p1 = intermediate_points[0];
                const p2 = mouse_local;
                // Points are too close, can't make an arc
                if (Local3.distance(p1, p2) < 0.01) return;

                const line = shapes.shapes[shapes.shapes.length - 1];
                let tangent;
                if (line instanceof Line) {
                    tangent = line.end.sub(line.start);
                } else {
                    // uhh find tangent of arc
                    tangent = line.tangent_at_endpoint();
                }

                shapes.add_shape(Arc.from_tangent_and_points(tangent, p1, p2));
                intermediate_points = [p2];
                tool = "Line";
                has_moved_away = false;
            } else if (tool == "Empty") {
                // check for dragging
                const mouse = new Cartesian2(event.clientX, event.clientY);
                let idx = 0;
                const max_sq_dist = HANDLE_POINT_RADIUS * HANDLE_POINT_RADIUS;
                for (const s of shapes.shapes) {
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
                        const p = s.get_screenspace_params(viewer);
                        let points = [p.centre, s.get_endpoint_screenspace(p, "Start"), s.get_endpoint_screenspace(p, "End")];
                        let i = 0;
                        for (const c of points) {
                            if (Cartesian2.distanceSquared(c, mouse) < max_sq_dist) {
                                drag_object = { shape_index: idx, point_index: i };
                                break;
                            }
                            i += 1;
                        }

                        const handle_points = s.allowed_region_handle_points(viewer);
                        for (const p of handle_points) {
                            if (Cartesian2.distanceSquared(p, mouse) < max_sq_dist) {
                                drag_object = { shape_index: idx, point_index: i };
                                break;
                            }
                            i += 1;
                        }
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
            if (shapes.shapes.length > 0) {
                const shape = shapes.shapes[shapes.shapes.length - 1];
                if (shape instanceof Line) {
                    intermediate_points = [shape.end];
                } else if (shape instanceof Arc) {
                    intermediate_points = [
                        new Local3(
                            shape.centre.x + shape.radius * Math.cos(-Arc.NEtoXY(shape.theta0 + shape.dangle)),
                            shape.centre.y + shape.radius * Math.sin(-Arc.NEtoXY(shape.theta0 + shape.dangle)),
                            shape.centre.z
                        )
                    ];
                }
            }
        } else if (event.key == "Escape") {
            tool = "Empty";
            intermediate_points = [];
            has_moved_away = false;
        } else if (event.key == "c") {
            editing_mode = "Create";
        } else if (event.key == "e") {
            shapes.close_path();
            editing_mode = "Edit";
        }
        draw_tooltip(mouseX, mouseY);
    }

    let mouseX: number, mouseY: number;
    function mousemove(event: MouseEvent) {
        mouseX = event.clientX;
        mouseY = event.clientY;
        if (drag_object !== undefined && viewer !== undefined) {
            const ellipsoid = viewer.scene.ellipsoid;
            const cartesian = viewer.camera.pickEllipsoid(new Cartesian3(event.clientX, event.clientY), ellipsoid);
            if (!cartesian) return; // just ignore an invalid position
            let local = Local3.fromCartesian(cartesian);
            let s = shapes.shapes[drag_object.shape_index];
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
                    if (drag_object.point_index == 1) a = local, b = s.get_endpoint_local("End");
                    else b = local, a = s.get_endpoint_local("Start");
                    const r = Local3.distance(local, s.centre);
                    const new_s = Arc.from_centre_and_points(s.centre, a, b, s.dangle < 0 ? -1 : 1, r);
                    shapes.shapes[drag_object.shape_index] = new_s;
                    shapes.shapes[drag_object.shape_index].width = s.width;
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

            if (editing_mode == "Edit") {
                shapes.close_path();
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
    let canvas: HTMLCanvasElement;
</script>

<canvas id="canvas" style="z-index: 2; position:absolute; pointer-events: none;" bind:this={canvas}></canvas>
<div id="cesiumContainer" style="height:max-content; z-index: 1;position:relative;"></div>
<svelte:window on:keydown={keypress} on:mousemove|preventDefault={mousemove} />
