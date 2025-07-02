import { Cartesian2, Cartesian3, Ellipsoid, type Viewer } from "cesium";
import type { IMavData } from "./mav";
import type { MpccInterfaces } from "./rostypes/ros_msgs";
import { Arc, HANDLE_POINT_RADIUS, Line, Local2, make_line, type DubinsPath } from "./geometry";
import { Coord_Type, get_cartesians, type BMFA_Coords } from "./waypoints";



const scratchc3_a = new Cartesian3();
const scratchc3_b = new Cartesian3();

export function draw_point(point: Local2, ctx: CanvasRenderingContext2D, viewer: Viewer) {
    const a = viewer.scene.cartesianToCanvasCoordinates(point.toCartesian(), scratchc3_a);
    ctx.save();
    ctx.strokeStyle = "yellow";
    ctx.fillStyle = "#ffd040";
    ctx.beginPath();
    ctx.arc(a.x, a.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
    ctx.fill();
    ctx.restore();
}

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

export function draw_coordinates(coordinates: BMFA_Coords[], ctx: CanvasRenderingContext2D, viewer: Viewer, geofence: BMFA_Coords[]) {
    if (!viewer || !ctx || !coordinates || !geofence) return;
    const cartesians = get_cartesians(coordinates);
    ctx.save();
    for (let i = 0; i < coordinates.length; i += 1) {
        let a = viewer.scene.cartesianToCanvasCoordinates(cartesians[i], scratchc3_a);
        let mouse_inside = false;
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
        ctx.fillStyle = `lch(70% 100% ${hue})`;
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

export function draw_waypoint_distances(ctx: CanvasRenderingContext2D, viewer: Viewer, waypoints: BMFA_Coords[], mouseX: number, mouseY: number) {
    // note: distance in screen space used to draw line on canvas
    // distance in world space used to display distance in meters
    const mouse = new Cartesian2(mouseX, mouseY);
    const ellipsoid = viewer.scene.ellipsoid;
    const mouse_cart = viewer.camera.pickEllipsoid(mouse, ellipsoid);
    if (mouse_cart == undefined) return;
    let closest_point_cart: Cartesian3 | undefined;
    let closest_dist = 1e9; // TODO infinity
    const cartesians = Cartesian3.fromDegreesArray(waypoints.flatMap((w) => [w.longitude, w.latitude]));
    // calculate distance in real-world space
    for (const c of cartesians) {
        const dist = Cartesian3.distance(mouse_cart, c);
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_point_cart = c;
        }
    }
    if (!closest_point_cart || !mouse_cart) return;
    if (closest_dist > 50) return;

    ctx.strokeStyle = "lightblue";
    ctx.fillStyle = ctx.strokeStyle;

    draw_distance_dashed(ctx, viewer, closest_point_cart, mouse_cart);
}

export function draw_distance_dashed(ctx: CanvasRenderingContext2D, viewer: Viewer, point_1: Cartesian3, point_2: Cartesian3) {
    const dash_len = 5;
    const closest_dist = Cartesian3.distance(point_1, point_2);
    const p1_scr = viewer.scene.cartesianToCanvasCoordinates(point_1),
        p2_scr = viewer.scene.cartesianToCanvasCoordinates(point_2);
    const dist_scr = Cartesian2.distance(p1_scr, p2_scr);
    const dashes = Math.floor(dist_scr / dash_len / 2);
    const dir_scr = Cartesian2.subtract(p2_scr, p1_scr, scratchc3_a);

    if (Cartesian2.magnitude(dir_scr) < 1e-5) return;
    Cartesian2.normalize(dir_scr, dir_scr);

    const half_way = Cartesian2.multiplyByScalar(dir_scr, dist_scr / 2, scratchc3_b);
    const midpoint = Cartesian2.add(p1_scr, half_way, scratchc3_b);
    Cartesian2.multiplyByScalar(dir_scr, dash_len, dir_scr);
    ctx.beginPath();
    for (let i = 0; i < dashes; i++) {
        ctx.moveTo(p1_scr.x, p1_scr.y);
        Cartesian2.add(p1_scr, dir_scr, p1_scr);
        ctx.lineTo(p1_scr.x, p1_scr.y);
        Cartesian2.add(p1_scr, dir_scr, p1_scr);
    }
    ctx.stroke();

    ctx.beginPath();
    ctx.fillText(closest_dist.toFixed(1), midpoint.x, midpoint.y);
}