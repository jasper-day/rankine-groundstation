
import { EcefToEnu, EnuToEcef } from "$lib/projection";
import { Cartesian3, Color, CornerType, Cartographic, Entity, Viewer, Cartesian2 } from 'cesium';

// Some point at buckminster gliding club
export const ORIGIN = Cartographic.fromDegrees(-0.7097051097617251, 52.830542659049435, 146 + 60); // approx airfield elevation ????
export const HANDLE_POINT_RADIUS = 4;

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
    get x(): number { return this._x; }
    get y(): number { return this._y; }
    get z(): number { return this._z; }

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

    toEntity(): Entity {
        return new Entity({
            name: "",
            corridor: {
                positions: [this.start.toCartesian(), this.end.toCartesian()],
                height: ORIGIN.height,
                width: 2.0,
                cornerType: CornerType.MITERED,
                material: Color.RED,
                outline: false,
            },
        });
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

		let dir = new Cartesian2((b.x - a.x), (b.y - a.y));
		if (Cartesian2.magnitudeSquared(dir) < 0.01) {
		    return;
		};

		const tri_size = 10;
		Cartesian2.normalize(dir, dir);
		const norm = new Cartesian2(-dir.y, dir.x);
		const midpoint_x = (a.x + b.x) / 2 - tri_size / 2 * dir.x;
		const midpoint_y = (a.y + b.y) / 2 - tri_size / 2 * dir.y;
		ctx.beginPath();
		ctx.moveTo(midpoint_x + norm.x * tri_size / 2, midpoint_y + norm.y * tri_size / 2);
		ctx.lineTo(midpoint_x - norm.x * tri_size / 2, midpoint_y - norm.y * tri_size / 2);
		ctx.lineTo(midpoint_x + dir.x * tri_size, midpoint_y + dir.y * tri_size);
		ctx.lineTo(midpoint_x + norm.x * tri_size / 2, midpoint_y + norm.y * tri_size / 2);
		ctx.closePath();
		ctx.fill();
		
    }
}

export class Arc {
    centre: Local3;
    radius: number;
    theta0: number; // radians
    theta1: number; // >= theta0
    direction: 1 | -1;
    constructor(centre: Local3, radius: number, theta0: number, theta1: number, direction: 1 | -1) {
        this.centre = centre;
        this.radius = radius;
        this.theta0 = theta0;
        this.theta1 = theta1;
        // yes I did choose these names specifically to make them line up well
        this.direction = direction;
    }

    toEntities(): Entity[] {
        let vertices = [];
        let theta = this.theta0;

        while (theta < this.theta1) {
            const x = this.centre.x + this.radius * Math.cos(theta);
            const y = this.centre.y + this.radius * Math.sin(theta);
            vertices.push(new Local3(x, y, this.centre.z).toCartesian());
            theta += 0.05
        }
        return [new Entity({
            name: "",
            // polyline: {
            //   positions: vertices,
            //   width: 2.0,
            //   material: Color.RED,
            // },
            corridor: {
                positions: vertices,
                height: ORIGIN.height,
                width: 2.0,
                cornerType: CornerType.MITERED,
                material: Color.RED,
                outline: false,
            },
        }),
        new Entity({
            name: "",
            position: this.centre.toCartesian(),
            ellipse: {
                semiMinorAxis: 0.8,
                semiMajorAxis: 0.8,
                height: ORIGIN.height,
                material: Color.RED,
            }
        })];
    }

    static from_centre_and_points(centre: Local3, a: Local3, b: Local3, direction: 1 | -1) {
        const r = Local3.distance(centre, a);
        const theta0 = -Math.atan2(a.y - centre.y, a.x - centre.x);
        const theta1 = -Math.atan2(b.y - centre.y, b.x - centre.x);
        return new Arc(centre, r, theta0, theta1, direction);
    }

    draw(ctx: CanvasRenderingContext2D, viewer: Viewer, inhibit_endpoints?: boolean) {
		const centre = viewer.scene.cartesianToCanvasCoordinates(
			this.centre.toCartesian(),
			scratchc3_a
		);
		// hack to get the radius in screen space
		const centre_plus_rad = new Local3(this.centre.x + this.radius, this.centre.y, this.centre.z);
		const rad_point = viewer.scene.cartesianToCanvasCoordinates(
			centre_plus_rad.toCartesian()
		);
		const rad = Cartesian2.distance(centre, rad_point);
		ctx.beginPath();
		ctx.arc(centre.x, centre.y, rad, this.theta0, this.theta1, this.direction == -1);
		ctx.stroke();
		ctx.beginPath();
		ctx.arc(centre.x, centre.y, HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
		ctx.fill();
		if (!inhibit_endpoints) {
    		ctx.beginPath();
    		ctx.arc(centre.x + rad * Math.cos(this.theta0), centre.y + rad * Math.sin(this.theta0), HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
    		ctx.fill();
    		ctx.beginPath();
    		ctx.arc(centre.x + rad * Math.cos(this.theta1), centre.y + rad * Math.sin(this.theta1), HANDLE_POINT_RADIUS, 0, 2 * Math.PI);
    		ctx.fill();
		}
    }
}
