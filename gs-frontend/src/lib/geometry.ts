import { EcefToEnu, EnuToEcef } from "$lib/projection";
import { Cartesian3, Color, CornerType, Cartographic, Entity, Viewer, Cartesian2 } from "cesium";

// Some point at buckminster gliding club
export const ORIGIN = Cartographic.fromDegrees(-0.7097051097617251, 52.830542659049435, 146 + 60); // approx airfield elevation ????
export const HANDLE_POINT_RADIUS = 4;
const TRI_SIZE = 10;

// to avoid instantiating objects continuously
// may be premature optimisation but cesium does it so i will too
let scratchc3_a: Cartesian3 = new Cartesian3();
let scratchc3_b: Cartesian3 = new Cartesian3();

export class Local3 {
    _x: number;
    _y: number;
    _z: number;
    _cache_c3: Cartesian3 | undefined = undefined;
    constructor(x: number, y: number, z: number) {
        this._x = x;
        this._y = y;
        this._z = z;
    }
    toCartesian(): Cartesian3 {
        if (this._cache_c3 === undefined) {
            this._cache_c3 = EnuToEcef(this, ORIGIN);
        }
        return this._cache_c3;
    }
    static fromCartesian(c: Cartesian3): Local3 {
        return EcefToEnu(c, ORIGIN);
    }
    set x(x: number) {
        this.x = x;
        this._cache_c3 = undefined;
    }
    set y(y: number) {
        this.y = y;
        this._cache_c3 = undefined;
    }
    set z(z: number) {
        this.z = z;
        this._cache_c3 = undefined;
    }
    get x(): number {
        return this._x;
    }
    get y(): number {
        return this._y;
    }
    get z(): number {
        return this._z;
    }

    add(p: Local3): Local3 {
        return new Local3(p.x + this._x, p.y + this._y, p.z + this._z);
    }

    sub(p: Local3): Local3 {
        return new Local3(this._x - p.x, this._y - p.y, this._z - p.z);
    }

    dot(p: Local3): number {
        return this._x * p.x + this._y * p.y + this._z * p.z;
    }

    mul(s: number): Local3 {
        return new Local3(this._x * s, this._y * s, this._z * s);
    }

    static distance(a: Local3, b: Local3): number {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dz = a.z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}

export class Line {
    start: Local3;
    end: Local3;
    constructor(start: Local3, end: Local3) {
        this.start = start;
        this.end = end;
    }

    draw(ctx: CanvasRenderingContext2D, viewer: Viewer) {
        // TODO cache these?
        const a = viewer.scene.cartesianToCanvasCoordinates(this.start.toCartesian(), scratchc3_a);
        const b = viewer.scene.cartesianToCanvasCoordinates(this.end.toCartesian(), scratchc3_b);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(a.x, a.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.arc(b.x, b.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();

        let dir = new Cartesian2(b.x - a.x, b.y - a.y);
        if (Cartesian2.magnitudeSquared(dir) < 0.01) {
            return;
        }

        Cartesian2.normalize(dir, dir);
        const norm = new Cartesian2(-dir.y, dir.x);
        const midpoint_x = (a.x + b.x) / 2 - (TRI_SIZE / 2) * dir.x;
        const midpoint_y = (a.y + b.y) / 2 - (TRI_SIZE / 2) * dir.y;
        ctx.beginPath();
        ctx.moveTo(midpoint_x + (norm.x * TRI_SIZE) / 2, midpoint_y + (norm.y * TRI_SIZE) / 2);
        ctx.lineTo(midpoint_x - (norm.x * TRI_SIZE) / 2, midpoint_y - (norm.y * TRI_SIZE) / 2);
        ctx.lineTo(midpoint_x + dir.x * TRI_SIZE, midpoint_y + dir.y * TRI_SIZE);
        ctx.lineTo(midpoint_x + (norm.x * TRI_SIZE) / 2, midpoint_y + (norm.y * TRI_SIZE) / 2);
        ctx.closePath();
        ctx.fill();
    }

    serialise(): any {
        // swap x and y for NED
        return { type: "line", start: [this.start.y, this.start.x], end: [this.end.y, this.end.x] };
    }

    static deserialise(d: any): Line {
        return new Line(new Local3(d.start.y, d.start.x, 0.0), new Local3(d.end.y, d.end.x, 0.0));
    }
}

export class Arc {
    centre: Local3;
    radius: number;
    theta0: number; // radians
    dangle: number; // radians
    direction: 1 | -1;
    constructor(centre: Local3, radius: number, theta0: number, dangle: number, direction: 1 | -1) {
        this.centre = centre;
        this.radius = radius;
        this.theta0 = theta0;
        this.dangle = dangle;
        this.direction = direction;
    }

    serialise(): any {
        return {
            type: "arc",
            centre: [this.centre.y, this.centre.x], // swap x and y for NED
            heading: ang_mod(Math.PI / 2 - this.theta0),
            arclength: this.dangle * this.radius,
            direction: this.direction
        };
    }

    static deserialise(d: any): Arc {
        return new Arc(
            new Local3(d.centre.y, d.centre.x, 0),
            d.radius,
            -(d.heading - Math.PI / 2),
            d.arclength / d.radius,
            d.direction
        );
    }

    static from_centre_and_points(centre: Local3, a: Local3, b: Local3, direction: 1 | -1) {
        const r = Local3.distance(centre, a);
        let theta0 = -Math.atan2(a.y - centre.y, a.x - centre.x);
        let theta1 = -Math.atan2(b.y - centre.y, b.x - centre.x);
        let arc_length = ang_mod2(direction == -1 ? theta0 - theta1 : theta1 - theta0);
        return new Arc(centre, r, theta0, arc_length, direction);
    }

    draw(ctx: CanvasRenderingContext2D, viewer: Viewer, inhibit_endpoints?: boolean) {
        const centre = viewer.scene.cartesianToCanvasCoordinates(this.centre.toCartesian(), scratchc3_a);
        const rad_point_local = new Local3(this.centre.x + this.radius, this.centre.y, this.centre.z);
        const rad_point_screen = viewer.scene.cartesianToCanvasCoordinates(rad_point_local.toCartesian());
        const rad = Cartesian2.distance(centre, rad_point_screen);
        const x_axis = new Cartesian2();
        Cartesian2.subtract(rad_point_screen, centre, x_axis);
        const dtheta = Math.atan2(x_axis.y, x_axis.x);
    
        const theta0 = ang_mod(this.theta0 + dtheta);
        const arc_length = this.direction == 1 ? this.dangle : -this.dangle;
        const theta1 = ang_mod(theta0 + arc_length);
        const half_theta = ang_mod(theta0 + arc_length / 2);

        // hack to get the radius in screen space
        ctx.beginPath();
        ctx.arc(centre.x, centre.y, rad, theta0, theta1, this.direction == -1);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(centre.x, centre.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();
        if (!inhibit_endpoints) {
            ctx.beginPath();
            ctx.arc(
                centre.x + rad * Math.cos(theta0),
                centre.y + rad * Math.sin(theta0),
                HANDLE_POINT_RADIUS,
                0,
                2 * Math.PI
            );
            ctx.fill();
            ctx.beginPath();
            ctx.arc(
                centre.x + rad * Math.cos(theta1),
                centre.y + rad * Math.sin(theta1),
                HANDLE_POINT_RADIUS,
                0,
                2 * Math.PI
            );
            ctx.fill();
        }

        const half_theta_screen = new Cartesian2(
            centre.x + rad * Math.cos(half_theta),
            centre.y + rad * Math.sin(half_theta)
        );
        const norm = new Cartesian2();
        Cartesian2.subtract(half_theta_screen, centre, norm);
        if (Cartesian2.magnitudeSquared(norm) < 0.1) {
            return;
        }
        Cartesian2.normalize(norm, norm);
        const tangent = new Cartesian2(-norm.y, norm.x);
        Cartesian2.multiplyByScalar(tangent, -this.direction, tangent);
        const arrow_point = new Cartesian2(
            half_theta_screen.x - (tangent.x * TRI_SIZE) / 2,
            half_theta_screen.y - (tangent.y * TRI_SIZE) / 2
        );
        const arrow_base_inner = new Cartesian2(
            half_theta_screen.x + ((tangent.x + norm.x) * TRI_SIZE) / 2,
            half_theta_screen.y + ((tangent.y + norm.y) * TRI_SIZE) / 2
        );
        const arrow_base_outer = new Cartesian2(
            half_theta_screen.x + ((tangent.x - norm.x) * TRI_SIZE) / 2,
            half_theta_screen.y + ((tangent.y - norm.y) * TRI_SIZE) / 2
        );
        ctx.beginPath();
        ctx.moveTo(arrow_point.x, arrow_point.y);
        ctx.lineTo(arrow_base_inner.x, arrow_base_inner.y);
        ctx.lineTo(arrow_base_outer.x, arrow_base_outer.y);
        ctx.moveTo(arrow_point.x, arrow_point.y);
        ctx.closePath();
        ctx.fill();
    }
}
export function angle_delta(theta0: number, theta1: number): number {
    // this is awful and slow
    // why is it so hard to find the signed difference between two angles??
    return Math.atan2(Math.sin(theta0 - theta1), Math.cos(theta0 - theta1));
}
function ang_mod(a: number): number {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
}
function ang_mod2(a: number): number {
    while (a > 2 * Math.PI) a -= 2 * Math.PI;
    while (a < 0) a += 2 * Math.PI;
    return a;
}
