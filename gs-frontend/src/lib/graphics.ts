import { Cartesian2, Cartesian3, Ellipsoid, type Viewer } from "cesium";
import type { IMavData } from "./mav";
import type { MpccInterfaces } from "./rostypes/ros_msgs";
import { Arc, HANDLE_POINT_RADIUS, Line, Local2, make_line, type DubinsPath } from "./geometry";
import { Coord_Type, get_cartesians, type BMFA_Coords } from "./waypoints";

const scratchc3_a = new Cartesian3();
const scratchc3_b = new Cartesian3();

export function draw_mav_history(history: IMavData[], current: IMavData, plan: MpccInterfaces.TrajectoryPlan | null, viewer: Viewer, ctx: CanvasRenderingContext2D, curr_heading: number) {
    const degrees_heights_array = history
        .map((element) => {
            return element.gpsFix !== undefined && element.gpsFix.latitude !== 0 && element.gpsFix.longitude !== 0
                ? [element.gpsFix?.longitude, element.gpsFix?.latitude, element.gpsFix?.altitude]
                : [];
        })
        .flat();

    if (degrees_heights_array.length < 2 * 3) return;

    const cartesians = Cartesian3.fromDegreesArrayHeights(degrees_heights_array, Ellipsoid.WGS84);

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
        let local: Local2 | null = null;
        for (let i = 0; i != plan.north.length; i++) {
            local = new Local2(plan.east[i], plan.north[i]);
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

export function draw_intermediate_shape(viewer: Viewer, intermediate_point: Local2, ctx: CanvasRenderingContext2D, mouseX: number, mouseY: number, has_moved_away: boolean, tool: string, shapes: DubinsPath) {
    // TODO costly operation to undo later on...
    const mouse_cartesian = viewer.camera.pickEllipsoid(new Cartesian3(mouseX, mouseY), viewer.scene.ellipsoid);
    if (!mouse_cartesian) return;
    const mouse_local = Local2.fromCartesian(mouse_cartesian);

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
        if (Local2.distance(p1, p2) < 0.01) return;

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

export function draw_coordinates(coordinates: BMFA_Coords[], ctx: CanvasRenderingContext2D, viewer: Viewer, geofence: BMFA_Coords[], mouseX: number, mouseY: number) {
    if (!viewer || !ctx || !coordinates || !geofence) return;
    const cartesians = get_cartesians(coordinates);
    ctx.save();
    for (let i = 0; i < coordinates.length; i += 1) {
        let a = viewer.scene.cartesianToCanvasCoordinates(cartesians[i], scratchc3_a);
        let mouse_inside = false;
        if (
            Math.pow(mouseX - a.x, 2) + Math.pow(mouseY - a.y, 2) <
            HANDLE_POINT_RADIUS * HANDLE_POINT_RADIUS + 150
        ) {
            mouse_inside = true;
        }
        let lightness = mouse_inside ? "50%" : "90%";
        let hue: string = "0deg";
        switch (coordinates[i].type) {
            case Coord_Type.RUNWAY:
                hue = "150deg";
                break;
            case Coord_Type.GEOFENCE:
                hue = "30deg";
                break;
            case Coord_Type.WAYPOINT:
                hue = "230deg";
                break;
            case Coord_Type.PAYLOAD:
                hue = "300deg";
                break;
        }
        ctx.fillStyle = `lch(${lightness} 100% ${hue})`;
        ctx.beginPath();
        ctx.arc(a.x, a.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();
    }

    const geofence_cart = get_cartesians(geofence);

    ctx.strokeStyle = "lch(50% 80% 30deg)";
    ctx.beginPath();
    let path_end = viewer.scene.cartesianToCanvasCoordinates(geofence_cart.at(-1) as Cartesian3, scratchc3_a);
    ctx.moveTo(path_end.x, path_end.y);
    for (let i = 0; i < geofence_cart.length; i += 1) {
        let a = viewer.scene.cartesianToCanvasCoordinates(geofence_cart[i], scratchc3_a);
        ctx.lineTo(a.x, a.y);
    }

    ctx.stroke();
    ctx.restore();
}
