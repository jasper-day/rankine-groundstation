import { EcefToEnu, EnuToEcef } from "$lib/projection";
import { Cartesian3, Cartographic, Viewer, Cartesian2 } from "cesium";

// Some point at buckminster gliding club
// export const ORIGIN = Cartographic.fromDegrees(-0.7097051097617251, 52.830542659049435, 146 + 60); // approx airfield elevation ????

// Holyrood Park
export const ORIGIN = Cartographic.fromDegrees(-0.707980, 52.780515, 137); // BMFA runway center
export const HANDLE_POINT_RADIUS = 4;
const TRI_SIZE = 10;

// to avoid instantiating objects continuously
// may be premature optimisation but cesium does it so i will too
let scratchc3_a: Cartesian3 = new Cartesian3();
let scratchc3_b: Cartesian3 = new Cartesian3();
let scratchc2: Cartesian2 = new Cartesian2();

export function quaternion_to_RPY(q: { w: number; x: number; y: number; z: number; }) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // https://github.com/rawify/Quaternion.js/blob/main/src/quaternion.js (order YPR)

    const w = q.w, x = q.x, y = q.y, z = q.z;
    const wx = w * x, wy = w * y, wz = w * z;
    const xx = x * x, xy = x * y, xz = x * z;
    const yy = y * y, yz = y * z, zz = z * z;

    function asin(t: number) {
        return t >= 1 ? Math.PI / 2 : (t <= -1 ? -Math.PI / 2 : Math.asin(t));
    }

    return {
        yaw: Math.atan2(2 * (xy + wz), 1 - 2 * (yy + zz)), // Heading / Yaw
        pitch: -asin(2 * (xz - wy)), // Attitude / Pitch
        roll: Math.atan2(2 * (yz + wx), 1 - 2 * (xx + yy)), // Bank / Roll
    };
}


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
        this._x = x;
        this._cache_c3 = undefined;
    }
    set y(y: number) {
        this._y = y;
        this._cache_c3 = undefined;
    }
    set z(z: number) {
        this._z = z;
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
    mag(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }
    mag2(): number {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }

    static distance(a: Local3, b: Local3): number {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dz = a.z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    normalise() {
        const inv_mag = 1.0 / this.mag();
        this.x *= inv_mag;
        this.y *= inv_mag;
        this.z *= inv_mag;
    }
}

export class Line {
    start: Local3;
    end: Local3;
    width: [number, number, number, number]; // left - right, front - back
    constructor(start: Local3, end: Local3) {
        this.start = start;
        this.end = end;
        this.width = [20, 20, 20, 20];
    }

    draw(ctx: CanvasRenderingContext2D, viewer: Viewer) {
        // TODO cache these?
        const a = viewer.scene.cartesianToCanvasCoordinates(this.start.toCartesian(), scratchc3_a);
        const b = viewer.scene.cartesianToCanvasCoordinates(this.end.toCartesian(), scratchc3_b);
        ctx.strokeStyle = "yellow";
        ctx.fillStyle = "#ffd040";
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

        /*
        const area_points = this.allowed_area_handle_points(viewer);

        ctx.fillStyle = "#ffff0050";
        ctx.beginPath();
        ctx.moveTo(area_points[0].x, area_points[0].y);
        ctx.lineTo(area_points[1].x, area_points[1].y);
        ctx.lineTo(area_points[3].x, area_points[3].y);
        ctx.lineTo(area_points[2].x, area_points[2].y);
        ctx.lineTo(area_points[0].x, area_points[0].y);
        ctx.closePath();
        ctx.fill();

        ctx.fillStyle = "#ffd040";
        for (const pos of area_points) {
            ctx.beginPath();
            ctx.arc(pos.x, pos.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
            ctx.fill();
        }*/
    }

    allowed_area_handle_points(v: Viewer): [Cartesian2, Cartesian2, Cartesian2, Cartesian2] {
        const a = this.start;
        const b = this.end;
        const dir = new Local3(b.x - a.x, b.y - a.y, b.z - a.z);
        dir.normalise();

        const norm = new Local3(dir.y, -dir.x, 0);

        const w0_l = this.width[0];
        const w0_r = this.width[1];
        const w1_l = this.width[2];
        const w1_r = this.width[3];
        return [
            new Local3(a.x - norm.x * w0_l, a.y - norm.y * w0_l, a.z),
            new Local3(a.x + norm.x * w0_r, a.y + norm.y * w0_r, a.z),
            new Local3(b.x - norm.x * w1_l, b.y - norm.y * w1_l, b.z),
            new Local3(b.x + norm.x * w1_r, b.y + norm.y * w1_r, b.z)
        ].map((p) => v.scene.cartesianToCanvasCoordinates(p.toCartesian())) as [
                Cartesian2,
                Cartesian2,
                Cartesian2,
                Cartesian2
            ];
    }

    serialise(): any {
        // swap x and y for NED
        return { type: "line", start: [this.start.y, this.start.x], end: [this.end.y, this.end.x] };
    }

    static deserialise(d: any): Line {
        return new Line(new Local3(d.start[1], d.start[0], 0.0), new Local3(d.end[1], d.end[0], 0.0));
    }

    eval(arclength: number): Local3 {
        let direction = this.end.sub(this.start)
        let length = this.length()
        let t = Math.min(Math.max(arclength / length, 0), length)
        return this.start.add(direction.mul(t))
    }

    length(): number {
        let direction = this.end.sub(this.start);
        return Math.sqrt(Math.pow(direction.x, 2) + Math.pow(direction.y, 2))
    }
}

export class Arc {
    centre: Local3;
    radius: number;
    theta0: number; // radians
    dangle: number; // radians, signed
    width: [number, number, number, number];
    constructor(centre: Local3, radius: number, theta0: number, dangle: number) {
        this.centre = centre;
        this.radius = radius;
        this.theta0 = theta0;
        this.dangle = dangle;
        this.width = [20, 20, 20, 20];
    }

    theta1() {
        return ang_mod(this.theta0 + this.dangle);
    }

    half_theta() {
        return ang_mod(this.theta0 + this.dangle / 2);
    }

    serialise(): any {
        return {
            type: "arc",
            centre: [this.centre.y, this.centre.x], // swap x and y for NED
            heading: ang_mod(this.theta0),
            arclength: this.dangle * this.radius,
            radius: this.radius
        };
    }

    static deserialise(d: any): Arc {
        return new Arc(new Local3(d.centre[1], d.centre[0], 0), d.radius, d.heading, d.arclength / d.radius);
    }

    static from_centre_and_points(centre: Local3, a: Local3, b: Local3, direction: 1 | -1, rad?: number) {
        let r;
        if (rad === undefined) r = Local3.distance(centre, a);
        else r = rad;
        // start heading from North to East
        let theta0 = Math.atan2(a.x - centre.x, a.y - centre.y);
        // end heading, N-E
        let theta1 = Math.atan2(b.x - centre.x, b.y - centre.y);
        let arc_length = ang_mod2(direction == -1 ? theta0 - theta1 : theta1 - theta0);
        return new Arc(centre, r, theta0, direction * arc_length);
    }

    static from_tangent_and_points(tangent: Local3, p1: Local3, p2: Local3) {
        // https://math.stackexchange.com/a/2464407
        // let points A and B on the circle be (a, b) and (c, d) respectively
        // let m_t be the gradient of the tangent at point A
        // We know AB is a chord, therefore it is perpendicular to the radius
        // Even moreso, its perpendicular bisector passes through the centre.
        // Method: find the equation of AB's perpendicular bisector, and
        // the equation of a line through A perpendicular to the tangent. These
        // lines must intersect at the centre. Set equal and solve for x and y.
        //
        // Calculate the midpoint (j, k) of the line AB
        //     j = a + (c-a)/2
        //     k = b + (d-b)/2
        // Calculate the gradient m_ab of the like AB
        //     m_ab = (d-b)/(c-a)
        // calculate the gradient m_jk of the perpendicular bisector of AB (through jk)
        //     m_jk = -1/m_ab = (a-c)/(d-b)
        // The gneral equation of a line can be used
        //     y = m(x-a)+b
        // to find the equation of the perpendicular bisector of AB (through jk)
        //     y_1 = m_jk(x - j) + k
        // Now use same equation to find the equation of the line AC where C is the centre
        // The gradient of the line must be perpendicular to the tangent gradient, since AC is a radius
        //     y_2 = (-1/m_t)(x-a) + b
        // Find the intersection by setting equal
        //     m_jk(x-j) + k = (-1/m_t)(x-a)+b
        // Substitute in for m_jk, j, and k; solve for x (wolfram alpha moment)
        //     x = (-a²m_t + 2a(b-d) + m_t(b² - 2bd + c² + d²)/(2(m_t(c-a) + b - d))
        // Use either of the two previous equations to find the y coordinate given x
        //     y = (-1/m_t)(x-a) + b
        const m_t = tangent.y / tangent.x;
        const a = p1.x;
        const b = p1.y;
        const c = p2.x;
        const d = p2.y;
        // Given points (a, b) and (c, d) with tangent gradient m_t, calculate the centre (x, y)
        const x =
            (a * a * -m_t + 2 * a * (b - d) + m_t * (b * b - 2 * b * d + c * c + d * d)) /
            (2 * (m_t * (c - a) + b - d));
        const y = (-1 / m_t) * (x - a) + b;
        let centre = new Local3(x, y, p1.z);

        // project location of mouse pointer onto tangent line
        const lineCA = tangent,
            // b length in CA direction
            lineCB = p2.sub(p1),
            // distance along CA
            distCAtoB = lineCA.mul(lineCA.dot(lineCB) / lineCA.dot(lineCA)),
            // normal distance from CA to B
            distBtoCA = lineCB.sub(distCAtoB),
            // right hand rotation of CA
            lineCA_rot90_RH = new Local3(-lineCA.y, lineCA.x, lineCA.z),
            // which side are we on?
            side = lineCA_rot90_RH.dot(distBtoCA),
            // idk why the direction is opposite
            dir = -Math.sign(side);

        return Arc.from_centre_and_points(centre, p1, p2, dir == 1 ? 1 : -1);
    }

    get_endpoint_local(which?: "Start" | "End"): Local3 {
        let theta = which == "Start" ? this.theta0 : this.theta0 + this.dangle;
        const theta_XY = -Arc.NEtoXY(ang_mod(theta));
        return new Local3(
            this.radius * Math.cos(theta_XY) + this.centre.x,
            this.radius * Math.sin(theta_XY) + this.centre.y,
            this.centre.z
        );
    }

    tangent_at_endpoint(): Local3 {
        // there's a lot of multiplying by -1 here
        // I'm not sure why. but it's all necessary. I think.
        const theta = -Arc.NEtoXY(ang_mod(this.theta0 + this.dangle));
        return new Local3(Math.sin(theta), -Math.cos(theta), 0).mul(Math.sign(this.dangle));
    }

    static NEtoXY(angle: number): number {
        return angle - Math.PI / 2;
    }

    draw_handle_points(ctx: CanvasRenderingContext2D, v: Viewer) {
        // Draw centrepoint
        ctx.beginPath();
        const centre = v.scene.cartesianToCanvasCoordinates(this.centre.toCartesian());
        ctx.arc(centre.x, centre.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();
        // Draw points at the ends of the arc
        ctx.beginPath();
        const p1 = v.scene.cartesianToCanvasCoordinates(this.get_endpoint_local("Start").toCartesian());
        ctx.arc(p1.x, p1.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();
        ctx.beginPath();
        const p2 = v.scene.cartesianToCanvasCoordinates(this.get_endpoint_local("End").toCartesian());
        ctx.arc(p2.x, p2.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();
    }

    draw_direction_arrow(ctx: CanvasRenderingContext2D, v: Viewer) {
        const half_theta_pos = new Local3(
            this.centre.x + this.radius * Math.cos(Math.PI / 2 - this.half_theta()),
            this.centre.y + this.radius * Math.sin(Math.PI / 2 - this.half_theta()),
            this.centre.z
        );

        const half_theta_screen = v.scene.cartesianToCanvasCoordinates(half_theta_pos.toCartesian());
        const centre_screen = v.scene.cartesianToCanvasCoordinates(this.centre.toCartesian());

        const norm = new Cartesian2();
        Cartesian2.subtract(half_theta_screen, centre_screen, norm);
        if (Cartesian2.magnitudeSquared(norm) < 0.1) {
            return;
        }

        Cartesian2.normalize(norm, norm);

        const tangent = new Cartesian2(-norm.y, norm.x);
        Cartesian2.multiplyByScalar(tangent, Math.sign(this.dangle), tangent);
        const dir = tangent;
        const midpoint_x = half_theta_screen.x - (TRI_SIZE / 2) * dir.x;
        const midpoint_y = half_theta_screen.y - (TRI_SIZE / 2) * dir.y;
        ctx.beginPath();
        ctx.moveTo(midpoint_x + (norm.x * TRI_SIZE) / 2, midpoint_y + (norm.y * TRI_SIZE) / 2);
        ctx.lineTo(midpoint_x - (norm.x * TRI_SIZE) / 2, midpoint_y - (norm.y * TRI_SIZE) / 2);
        ctx.lineTo(midpoint_x + dir.x * TRI_SIZE, midpoint_y + dir.y * TRI_SIZE);
        ctx.lineTo(midpoint_x + (norm.x * TRI_SIZE) / 2, midpoint_y + (norm.y * TRI_SIZE) / 2);
        ctx.closePath();
        ctx.fill();
    }

    draw_allowed_region(ctx: CanvasRenderingContext2D, v: Viewer) {
        ctx.fillStyle = "#ffff0050";

        const vertex = (theta: number, side: "Inner" | "Outer", t: number) => {
            const [x, y] = this.point_on_region_boundary(theta, side, t, v);
            ctx.lineTo(x, y);
        };

        const abs_arc_length = Math.abs(this.dangle);
        const n_steps = 32;
        const s = abs_arc_length / n_steps;
        const step = this.dangle < 0 ? -s : s;
        const theta1 = ang_mod(this.theta0 + this.dangle);

        ctx.beginPath();
        const [x_start, y_start] = this.point_on_region_boundary(this.theta0, "Outer", 0, v);
        ctx.moveTo(x_start, y_start);

        let theta = this.theta0;
        for (let i = 0; i < n_steps; i++) {
            vertex(theta, "Outer", i / n_steps);
            theta += step;
        }
        vertex(theta1, "Outer", 1);
        for (let theta = theta1, i = n_steps - 1; i >= 0; theta -= step, i--) {
            vertex(theta, "Inner", i / n_steps);
        }
        vertex(this.theta0, "Inner", 0);

        ctx.closePath();
        ctx.fill();

        // Draw handle points to control width of allowed region
        ctx.fillStyle = "#ffd040";
        for (const pos of this.allowed_region_handle_points(v)) {
            ctx.beginPath();
            ctx.arc(pos.x, pos.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
            ctx.fill();
        }
    }

    draw_arc(ctx: CanvasRenderingContext2D, v: Viewer) {
        const vertex = (theta: number, first?: boolean) => {
            const local_x = this.radius * Math.cos(Math.PI / 2 - theta) + this.centre.x;
            const local_y = this.radius * Math.sin(Math.PI / 2 - theta) + this.centre.y;
            const pos = v.scene.cartesianToCanvasCoordinates(new Local3(local_x, local_y, this.centre.z).toCartesian());
            if (first) ctx.moveTo(pos.x, pos.y);
            else ctx.lineTo(pos.x, pos.y);
        };

        const abs_arc_length = Math.abs(this.dangle);
        const n_steps = 32;
        const s = abs_arc_length / n_steps;
        const step = this.dangle < 0 ? -s : s;

        ctx.beginPath();
        vertex(this.theta0, true);
        let theta = this.theta0;
        for (let i = 0; i < n_steps; i++) {
            vertex(theta);
            theta += step;
        }
        vertex(this.theta1());
        ctx.stroke();
    }

    point_on_region_boundary(theta: number, side: "Inner" | "Outer", t: number, viewer: Viewer): [number, number] {
        // which side we're on, left or right. 0 or 1
        const f = (side == "Inner" && this.dangle < 0) || (side == "Outer" && this.dangle > 0) ? 0 : 1;

        const rad_now =
            this.radius + (side == "Inner" ? -1 : 1) * (this.width[0 + f] * (1 - t) + this.width[2 + f] * t);
        const local_x = rad_now * Math.cos(Math.PI / 2 - theta) + this.centre.x;
        const local_y = rad_now * Math.sin(Math.PI / 2 - theta) + this.centre.y;
        const pos = viewer.scene.cartesianToCanvasCoordinates(
            new Local3(local_x, local_y, this.centre.z).toCartesian()
        );
        return [pos.x, pos.y];
    }

    allowed_region_handle_points(v: Viewer): [Cartesian2, Cartesian2, Cartesian2, Cartesian2] {
        const right = this.dangle > 0 ? "Inner" : "Outer";
        const left = this.dangle > 0 ? "Outer" : "Inner";
        return [
            Cartesian2.fromArray(this.point_on_region_boundary(this.theta0, left, 0, v)),
            Cartesian2.fromArray(this.point_on_region_boundary(this.theta0, right, 0, v)),
            Cartesian2.fromArray(this.point_on_region_boundary(this.theta1(), left, 1, v)),
            Cartesian2.fromArray(this.point_on_region_boundary(this.theta1(), right, 1, v))
        ];
    }

    draw(ctx: CanvasRenderingContext2D, viewer: Viewer) {
        ctx.strokeStyle = "yellow";
        ctx.fillStyle = "#ffd040";

        this.draw_arc(ctx, viewer);
        this.draw_handle_points(ctx, viewer);
        this.draw_direction_arrow(ctx, viewer);
        // this.draw_allowed_region(ctx, viewer);
    }

    eval(arclength: number) {
        let arclength_clipped = Math.min(Math.max(arclength, 0), this.length())
        let angle = this.theta0 + arclength_clipped / this.radius * this.dir();
        let cosines = new Local3(Math.cos(angle), Math.sin(angle), 0)
        return this.centre.add(cosines.mul(this.radius))
    }

    length() {
        return Math.abs(this.dangle) * this.radius
    }

    dir() {
        return this.dangle > 0 ? 1 : -1;
    }
}

export type DubinsPath = (Arc | Line)[];

export function path_eval(path: DubinsPath, arclength: number) {
    if (path.length == 0) return new Local3(0, 0, 0);
    let lengths = path.map((o) => o.length());
    let i = 0;
    while (i != path.length) {
        if (arclength < lengths[i]) {
            break;
        }
        arclength -= lengths[i];
        ++i;
    }
    if (i == path.length) {
        // final element is guaranteed to exist by entry guard
        return path.at(-1)?.eval(path.at(-1)?.length() as number) as Local3;
    }
    return path[i].eval(arclength);
}

export function deserialise_path(path: string) {
    return JSON.parse(path).map((ds: any) => {
        if (ds.type == "arc") {
            return Arc.deserialise(ds);
        } else if (ds.type == "line") {
            return Line.deserialise(ds);
        }
    });
}

export function serialise_path(path: DubinsPath) {
    return JSON.stringify(path.map(sh => sh.serialise()));
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

export function make_line(start: Local3, prev_shape: Line | Arc | undefined, mouse_local: Local3): Line {
    let end;
    if (prev_shape !== undefined) {
        const dist = Local3.distance(start, mouse_local);
        let dir;
        if (prev_shape instanceof Line) {
            dir = prev_shape.end.sub(prev_shape.start);
            dir.normalise();
        } else {
            dir = prev_shape.tangent_at_endpoint();
        }
        end = dir.mul(dist).add(start);
    } else {
        end = mouse_local;
    }
    return new Line(start, end);
}

export function make_arc(
    start: Local3,
    prev_shape: Line | Arc | undefined,
    mouse_local: Local3
): [Arc, Local3] | undefined {
    const p1 = start;
    const p2 = mouse_local;

    // Points are too close, can't make an arc
    if (Local3.distance(p1, p2) < 0.01) return undefined;

    let tangent;
    if (prev_shape === undefined) {
        // TODO ????
        return undefined;
    } else if (prev_shape instanceof Line) {
        tangent = prev_shape.end.sub(prev_shape.start);
    } else {
        // uhh find tangent of arc
        tangent = prev_shape.tangent_at_endpoint();
    }
    return [Arc.from_tangent_and_points(tangent, p1, p2), p2];
}


