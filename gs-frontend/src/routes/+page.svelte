<script lang="ts">
    // TODO !IMPORTANT
    // you can make the path segments out of order
    // move arcs
    // undo
    // delete
    // look into pointer events in chrome (no drag?)
    // canvas resizing
    // split path
    // enforce order somehow
    // draw + change region around path
    // reconnect to backend + show error
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
    let arc_direction_guess: 1 | -1 | undefined;
    let prelim_arc_direction_guess: 1 | -1 | undefined; // this is for rendering the arc preview when we are not really confident in the direction guess yet

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
                ctx.strokeStyle = "yellow";
                ctx.fillStyle = "#ffd040";
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

                    if (tool == "Line") {
                        new Line(intermediate_points[0], mouse_local).draw(ctx, viewer);
                    }
                    if (tool == "Arc" && intermediate_points.length > 0) {
                        if (intermediate_points.length == 1) {
                            let centre = intermediate_points[0];
                            let outer_point = mouse_local;
                            let r = Local3.distance(centre, outer_point);
                            new Arc(centre, r, 0, Math.PI * 2 - 0.0001).draw(ctx, viewer, true);
                        } else if (intermediate_points.length == 2) {
                            let dir: 1 | -1;
                            if (arc_direction_guess === undefined) {
                                if (prelim_arc_direction_guess !== undefined) {
                                    dir = prelim_arc_direction_guess;
                                } else {
                                    dir = 1;
                                }
                            } else {
                                dir = arc_direction_guess;
                            }
                            Arc.from_centre_and_points(
                                intermediate_points[0],
                                intermediate_points[1],
                                mouse_local,
                                dir
                            ).draw(ctx, viewer);
                        }
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
    function mousedown(event: MouseEvent) {
        // console.log(event);
        if (viewer !== undefined) {
            const cartesian = viewer.camera.pickEllipsoid(
                new Cartesian3(event.clientX, event.clientY),
                viewer.scene.ellipsoid
            );
            if (!cartesian) return; // just ignore an invalid position

            if (tool == "Line") {
                if (intermediate_points.length == 0) {
                    // add first point
                    intermediate_points.push(Local3.fromCartesian(cartesian));
                } else {
                    shapes.add_shape(new Line(intermediate_points[0], Local3.fromCartesian(cartesian)));
                    intermediate_points = [];
                }
            } else if (tool == "Arc") {
                if (intermediate_points.length < 2) {
                    intermediate_points.push(Local3.fromCartesian(cartesian));
                } else {
                    // we really should have a guess for the direction of the arc by now. Try to use a preliminary guess if we have one
                    if (arc_direction_guess === undefined) {
                        if (prelim_arc_direction_guess !== undefined) {
                            arc_direction_guess = prelim_arc_direction_guess;
                        } else {
                            arc_direction_guess = 1;
                        }
                    }
                    shapes.add_shape(
                        Arc.from_centre_and_points(
                            intermediate_points[0],
                            intermediate_points[1],
                            Local3.fromCartesian(cartesian),
                            arc_direction_guess
                        )
                    );
                    arc_direction_guess = undefined;
                    prelim_arc_direction_guess = undefined;
                    intermediate_points = [];
                }
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
                    } else if (s instanceof Arc) {
                        const c = viewer.scene.cartesianToCanvasCoordinates(s.centre.toCartesian(), scratchc3_a);
                        if (Cartesian2.distanceSquared(c, mouse) < max_sq_dist) {
                            drag_object = { shape_index: idx, point_index: 0 };
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
        } else if (event.key == "a") {
            tool = "Arc";
        } else if (event.key == "Escape") {
            tool = "Empty";
            intermediate_points = [];
            arc_direction_guess = undefined;
            prelim_arc_direction_guess = undefined;
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
                }
                if (drag_object.point_index == 1) {
                    s.end = local;
                }
            } else if (s instanceof Arc) {
                if (drag_object.point_index == 0) {
                    // move centre
                    s.centre = local;
                } else {
                    // const { centre, rad, theta0, theta1 } = s.get_screenspace_params(viewer);
                    // let keep_point;
                    // if (drag_object.point_index == 1) keep_point = s.get_endpoint(centre, rad, theta1);
                    // else keep_point = s.get_endpoint(centre, rad, theta0);
                    // const new_s = Arc.from_centre_and_points()
                }
            }

            if (editing_mode == "Edit") {
                shapes.close_path();
            }
        }
        // we are almost done making the arc, defining the last point, but we haven't figured out what direction it's going in yet (TODO inaccurate comment maybe)
        if (tool == "Arc" && intermediate_points.length == 2 && viewer !== undefined) {
            // guess
            const centre = intermediate_points[0];
            const a = intermediate_points[1];
            const cartesian = viewer.camera.pickEllipsoid(
                new Cartesian3(event.clientX, event.clientY),
                viewer.scene.ellipsoid
            );
            // ignore invalid position...
            if (cartesian === undefined) return;
            const b = Local3.fromCartesian(cartesian);

            // // b = location of mouse pointer
            // // centre = circle centre
            // // a = starting point of arc
            // // project location of mouse pointer onto line CA
            // const lineCA = a.sub(centre),
            // // b length in CA direction
            // 	  lineCB = b.sub(centre),
            // // distance along CA
            // 	  distCAtoB = lineCA.mul(lineCA.dot(lineCB) / lineCA.dot(lineCA)),
            // // normal distance from CA to B
            // 	  distBtoCA = lineCB.sub(distCAtoB),
            // // right hand rotation of CA
            // 	  lineCA_rot90_RH = new Local3(- lineCA.y, lineCA.x, lineCA.z),
            // // which side are we on?
            // 	  side = lineCA_rot90_RH.dot(distBtoCA),
            // // 1 for CCW, -1 for CW
            // 	  dir = Math.sign(side);
            // console.log(dir, side);
            //       prelim_arc_direction_guess = dir == 1 ? 1 : -1;
            //
            const theta0 = -Math.atan2(a.y - centre.y, a.x - centre.x);
            const theta1 = -Math.atan2(b.y - centre.y, b.x - centre.x);

            // this is awful and slow
            // why is it so hard to find the signed difference between two angles??
            let delta = Math.atan2(Math.sin(theta0 - theta1), Math.cos(theta0 - theta1));

            prelim_arc_direction_guess = delta > 0 ? -1 : 1;

            // don't guess for very tiny (possibly zero) difference
            // in fact, reset so we can guess again
            if (Math.abs(delta) < 0.15) {
                arc_direction_guess = undefined;
                return;
            }

            // do not overwrite an existing guess
            if (arc_direction_guess === undefined) {
                arc_direction_guess = prelim_arc_direction_guess;
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
