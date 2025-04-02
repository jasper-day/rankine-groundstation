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
let scratchc2: Cartesian2 = new Cartesian2();

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
    width_start: number;
    width_end: number;
    constructor(start: Local3, end: Local3) {
        this.start = start;
        this.end = end;
        this.width_start = 10;
        this.width_end = 10;
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

        ctx.fillStyle = "#ffff0050";
        ctx.beginPath();
        const diff = new Cartesian2();
        Cartesian2.subtract(a, b, diff);
        const w0 = this.width_start * (Cartesian2.distance(a, b) / Local3.distance(this.start, this.end));
        const w1 = this.width_end * (Cartesian2.distance(a, b) / Local3.distance(this.start, this.end));
        ctx.moveTo(a.x + norm.x * w0, a.y + norm.y * w0);
        ctx.lineTo(b.x + norm.x * w1, b.y + norm.y * w1);
        ctx.lineTo(b.x - norm.x * w1, b.y - norm.y * w1);
        ctx.lineTo(a.x - norm.x * w0, a.y - norm.y * w0);
        ctx.lineTo(a.x + norm.x * w0, a.y + norm.y * w0);
        ctx.closePath()
        ctx.fill();
    }

    serialise(): any {
        // swap x and y for NED
        return { type: "line", start: [this.start.y, this.start.x], end: [this.end.y, this.end.x] };
    }

    static deserialise(d: any): Line {
        return new Line(new Local3(d.start[1], d.start[0], 0.0), new Local3(d.end[1], d.end[0], 0.0));
    }
}

export class Arc {
    centre: Local3;
    radius: number;
    theta0: number; // radians
    dangle: number; // radians, signed
    width_start: number;
    width_end: number;
    constructor(centre: Local3, radius: number, theta0: number, dangle: number) {
        this.centre = centre;
        this.radius = radius;
        this.theta0 = theta0;
        this.dangle = dangle;
        this.width_start = 10;
        this.width_end = 10;
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
        return new Arc(
            new Local3(d.centre[1], d.centre[0], 0),
            d.radius,
            d.heading,
            d.arclength / d.radius,
        );
    }

    static from_centre_and_points(centre: Local3, a: Local3, b: Local3, direction: 1 | -1) {
        const r = Local3.distance(centre, a);
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
``
        const m_t = tangent.y / tangent.x;
        const a = p1.x;
        const b = p1.y;
        const c = p2.x;
        const d = p2.y;
        // Given points (a, b) and (c, d) with tangent gradient m_t, calculate the centre (x, y)
        const x = (a*a*(-m_t)+2*a*(b-d)+m_t*(b*b-2*b*d+c*c+d*d))/(2*(m_t*(c-a)+b-d));
        const y = (-1/m_t)*(x-a) + b;
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
        	  lineCA_rot90_RH = new Local3(- lineCA.y, lineCA.x, lineCA.z),
        // which side are we on?
        	  side = lineCA_rot90_RH.dot(distBtoCA),
        // idk why the direction is opposite
        	  dir = -Math.sign(side);

        return Arc.from_centre_and_points(centre, p1, p2, dir == 1 ? 1 : -1);
    }

    get_screenspace_params(viewer: Viewer): { centre: Cartesian2; rad: number; theta0: number; theta1: number } {
        const centre = viewer.scene.cartesianToCanvasCoordinates(this.centre.toCartesian(), scratchc3_a);
        // hack to get the radius in screen space
        const rad_point_local = new Local3(this.centre.x + this.radius, this.centre.y, this.centre.z);
        const rad_point_screen = viewer.scene.cartesianToCanvasCoordinates(rad_point_local.toCartesian());
        const rad = Cartesian2.distance(centre, rad_point_screen);
        const x_axis = new Cartesian2();
        Cartesian2.subtract(rad_point_screen, centre, x_axis);
        // heading from North to East
        const dtheta = Math.atan2(x_axis.y, x_axis.x);

        const arc_length = this.dangle;
        const theta0 = ang_mod(this.theta0 + dtheta);
        const theta1 = ang_mod(theta0 + arc_length);
        return { centre: centre, rad: rad, theta0: theta0, theta1: theta1 };
    }

    get_endpoint(centre: Cartesian2, rad: number, theta: number): Cartesian2 {
        const x = centre.x + rad * Math.cos(theta);
        const y = centre.y + rad * Math.sin(theta);
        scratchc2.x = x;
        scratchc2.y = y;
        return scratchc2;
    }

    tangent_at_endpoint(): Local3 {
        const theta = this.theta0;
        return new Local3(-Math.sin(theta), Math.cos(theta), 0);
    }
    
    static NEtoXY(angle: number) {
        return angle - Math.PI / 2;
    }

    draw(ctx: CanvasRenderingContext2D, viewer: Viewer, inhibit_endpoints?: boolean) {
        const { centre, rad, theta0, theta1 } = this.get_screenspace_params(viewer);
        const arc_length = this.dangle;
        const half_theta = ang_mod(theta0 + arc_length / 2);
        const path_width_begin = this.width_start * (rad / this.radius);
        const path_width_end = this.width_end * (rad / this.radius);

        // Convert headings to XY (E-N angle)
        const theta0_XY = Arc.NEtoXY(theta0),
            theta1_XY = Arc.NEtoXY(theta1),
            half_theta_XY = Arc.NEtoXY(half_theta);

        ctx.strokeStyle = "yellow";
        ctx.fillStyle = "#ffd040";

        // Draw arc
        ctx.beginPath();
        ctx.arc(centre.x, centre.y, rad, theta0_XY, theta1_XY, arc_length < 0);
        ctx.stroke();

        // Draw centrepoint
        ctx.beginPath();
        ctx.arc(centre.x, centre.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
        ctx.fill();
        // Draw points at the ends of the arc
        if (!inhibit_endpoints) {
            ctx.beginPath();
            const p1 = this.get_endpoint(centre, rad, theta0_XY);
            ctx.arc(p1.x, p1.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
            ctx.fill();
            ctx.beginPath();
            const p2 = this.get_endpoint(centre, rad, theta1_XY);
            ctx.arc(p2.x, p2.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
            ctx.fill();
        }

        // Draw arrow in direction of travel
        const half_theta_screen = new Cartesian2(
            centre.x + rad * Math.cos(half_theta_XY),
            centre.y + rad * Math.sin(half_theta_XY)
        );
        const norm = new Cartesian2();
        Cartesian2.subtract(half_theta_screen, centre, norm);
        if (Cartesian2.magnitudeSquared(norm) < 0.1) {
            return;
        }
        Cartesian2.normalize(norm, norm);
        const tangent = new Cartesian2(-norm.y, norm.x);
        Cartesian2.multiplyByScalar(tangent, -Math.sign(arc_length), tangent);
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


        // Draw allowed region transparently around the arc
        // ctx.strokeStyle = "#ffff0050";
        // ctx.lineWidth = path_width;
        // ctx.beginPath();
        // ctx.arc(centre.x, centre.y, rad, theta0_XY, theta1_XY, arc_length < 0);
        // ctx.stroke();
        // ctx.lineWidth = 2;
        ctx.fillStyle = "#ffff0050";
        const vertex = (theta: number, side: "Inner" | "Outer", t: number) => {
            // const t = (theta - theta0_XY) / (theta1_XY - theta0_XY); // fraction of the way round
            const rad_now = rad + (side == "Inner" ? -1 : 1) * (path_width_begin * (1 - t) + path_width_end * t);
            const x = rad_now * Math.cos(theta) + centre.x;
            const y = rad_now * Math.sin(theta) + centre.y;
            ctx.lineTo(x, y);
        };
        const x_start = (rad + path_width_begin) * Math.cos(theta0_XY) + centre.x;
        const y_start = (rad + path_width_begin) * Math.sin(theta0_XY) + centre.y;
        const abs_arc_length = Math.abs(arc_length);
        const n_steps = 32;
        const s = abs_arc_length / n_steps;
        const step = arc_length < 0 ? -s : s;
        ctx.beginPath();
        ctx.moveTo(x_start, y_start);
        let theta = theta0_XY;
        for (let i = 0; i < n_steps; i++) {
            vertex(theta, "Outer", i / n_steps);
            theta += step;
        }
        vertex(theta1_XY, "Outer", 1);
        for (let theta = theta1_XY, i = n_steps - 1; i >= 0; theta -= step, i--) {
            vertex(theta, "Inner", i / n_steps);
        }
        vertex(theta0_XY, "Inner", 0);
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
