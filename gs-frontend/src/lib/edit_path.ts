import { Cartesian3, type Viewer, Math as CesiumMath, Ellipsoid, Cartesian2 } from "cesium";
import { Arc, HANDLE_POINT_RADIUS, Line, Local2, make_arc, make_line, ORIGIN, path_from_points, path_make_continuous, type DubinsPath } from "./geometry";
import type { BMFA_Coords } from "./waypoints";
import { draw_distance_dashed, draw_point } from "./graphics";

export enum Tool { DRAW, EMPTY }
enum PointType { LINESTART, LINEEND, CENTER }

let path_points: Local2[] = [];
let tool: Tool = Tool.EMPTY;
let point_type: PointType = PointType.LINESTART;

function next_point_type(point_type: PointType) {
    switch (point_type) {
        case PointType.LINESTART:
            return PointType.LINEEND;
        case PointType.LINEEND:
            return PointType.CENTER;
        case PointType.CENTER:
            return PointType.LINEEND;
    }
}

function get_point_type(idx: number) {
    if (idx == 0) return PointType.LINESTART;
    if (idx % 2 == 0) return PointType.CENTER;
    return PointType.LINEEND;
}

enum Endpoint {
    START, END
}

let drag_point: number | null = null;
export let mouseX: number, mouseY: number;
let scratchc3_a = new Cartesian3();
let scratchc3_b = new Cartesian3();

export function get_path() {
    return path_from_points(path_points);
}

export function set_path(new_path: DubinsPath) {
    if (new_path.length == 0) path_points = [];
    path_points = new_path.map((s) => {
        if (s instanceof Line) {
            return [s.start, s.end]
        } else if (s instanceof Arc) {
            return s.centre
        }
    }).filter((o) => o != undefined).flat()
    let final = new_path.at(-1) ?? null;
    if (final && final instanceof Arc) {
        path_points.push(final.get_endpoint_local("End"));
    }
}

export function mousedown(viewer: Viewer, e: MouseEvent) {
    if (viewer !== undefined) {
        const cartesian = viewer.camera.pickEllipsoid(
            new Cartesian3(e.layerX, e.layerY),
            viewer.scene.ellipsoid
        );
        if (!cartesian) return; // just ignore an invalid position
        const mouse_local = Local2.fromCartesian(cartesian);

        if (tool == Tool.DRAW) {
            let next_point = get_next_point(mouse_local, point_type);
            if (next_point) {
                path_points.push(next_point);
                point_type = next_point_type(point_type);
            }
        } else if (tool == Tool.EMPTY) {
            // check for dragging
            const max_sq_dist = HANDLE_POINT_RADIUS * HANDLE_POINT_RADIUS;
            for (let idx = 0; idx != path_points.length; ++idx) {
                const m_a = mouse_local.sub(path_points[idx]);
                if (m_a.mag2() < max_sq_dist) {
                    drag_point = idx;
                }
            }
            if (drag_point) {
                viewer.scene.screenSpaceCameraController.enableInputs = false;
            }
        }

    }
}

export function mouseup(viewer: Viewer, _: MouseEvent) {
    if (drag_point) {
        if (viewer) viewer.scene.screenSpaceCameraController.enableInputs = true;
        drag_point = null;
    }
}

export function mousemove(viewer: Viewer, ctx: CanvasRenderingContext2D, event: MouseEvent) {
    mouseX = event.layerX;
    mouseY = event.layerY;
    const cartesian = viewer.camera.pickEllipsoid(new Cartesian3(mouseX, mouseY), Ellipsoid.WGS84);
    if (cartesian == undefined) return;
    const mouse_local = Local2.fromCartesian(cartesian);
    if (drag_point) {
        // we are dragging
        // need to constrain to available drag points
        const p_type = get_point_type(drag_point);
        let next_point = get_next_point(mouse_local, p_type, drag_point);
        if (next_point) {
            const old_point = path_points[drag_point];
            path_points[drag_point] = next_point;
            if (drag_point < path_points.length - 1 && p_type == PointType.LINEEND) {
                path_points[drag_point + 1] = path_points[drag_point + 1].add(next_point.sub(old_point));
            }
            for (let i = drag_point; i != path_points.length; ++i) {
                const p_type = get_point_type(i);
                if (p_type == PointType.CENTER) {
                    const new_point = get_next_point(path_points[i], p_type, i);
                    if (new_point) {
                        path_points[i] = new_point;
                    }
                }
            }

        }
        
    }
    
}

export function keypress(viewer: Viewer, ctx: CanvasRenderingContext2D, event: KeyboardEvent) {
    switch (event.key) {
        case "e":
            tool = Tool.DRAW;
            break;
        case "r":
            path_points = [];
            point_type = PointType.LINESTART;
        // fall through
        case "Escape":
        case "q":
            tool = Tool.EMPTY;
            break;
        case "1":
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
    draw_tooltip(ctx, mouseX, mouseY);
}

function draw_tooltip(ctx: CanvasRenderingContext2D, nx: number, ny: number) {
    if (tool === Tool.EMPTY) {
        return;
    }
    const x = nx - 16;
    const y = ny + 16;
    ctx.strokeStyle = "red";
    if (point_type == PointType.CENTER) {
        // place center
        ctx.beginPath();
        ctx.arc(x, y, 10, Math.PI / 2, (2 * Math.PI) / 3 + Math.PI / 2);
    } else {
        // line
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + 16, y + 16);
    }
    ctx.stroke();
}

export function draw_path(ctx: CanvasRenderingContext2D, viewer: Viewer) {
    let points = path_points;
    draw_tooltip(ctx, mouseX, mouseY);
    if (tool == Tool.DRAW) {
        const cartesian = viewer.camera.pickEllipsoid(new Cartesian3(mouseX, mouseY), Ellipsoid.WGS84);
        if (cartesian == undefined) return;
        const mouse_local = Local2.fromCartesian(cartesian);
        let next_point = get_next_point(mouse_local, point_type);
        if (next_point) {
            points = [...points, next_point];
        }
    }
    let path = path_from_points(points);
    for (const s of path) {
        s.draw(ctx, viewer);
    }
    for (const p of points) {
        draw_point(p, ctx, viewer);
    }
    for (let i = 1; i < points.length - 1; i += 2) {
        const end = points[i].toCartesian();
        const center = points[i + 1].toCartesian();
        ctx.fillStyle = "yellow";
        ctx.strokeStyle = ctx.fillStyle;
        draw_distance_dashed(ctx, viewer, end, center);
    }
}

function get_next_point(mouse: Local2, point_type: PointType, idx?: number): Local2 | null {
    if (idx == 0) return mouse; // special case for LINESTART
    const points = idx != undefined ? path_points.slice(0, idx) : path_points;
    switch (point_type) {
        case PointType.LINESTART:
            return mouse
        case PointType.LINEEND:
            // needs to be outside of previous circle
            if (points.length < 3) return mouse;
            const p_center = points.at(-1) as Local2,
                p_endpoint = points.at(-2) as Local2,
                r2 = p_endpoint.sub(p_center.mul(1)).mag2();
            if (mouse.sub(p_center).mag2() < r2) return null;
            else return mouse;
        case PointType.CENTER:
            // project onto previous line segment
            const path = path_from_points(points),
                ls = path.at(-1);
            if (!(ls instanceof Line)) throw new Error("Last segment should be a line");
            let ab = ls.end.sub(ls.start),
                r90 = new Local2(-ab.y, ab.x);
            r90.normalise();
            let projected = r90.mul(mouse.sub(ls.end).dot(r90)).add(ls.end);
            return projected;
    }
}

