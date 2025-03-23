
import { EcefToEnu, EnuToEcef } from "$lib/projection";
import { Cartesian3, Color, CornerType, Cartographic, Entity } from 'cesium';

// Some point at buckminster gliding club
export const ORIGIN = Cartographic.fromDegrees(-0.7097051097617251, 52.830542659049435, 146 + 60); // approx airfield elevation ????

export class Local3 {
    x: number;
    y: number;
    z: number;
    constructor(x: number, y: number, z: number) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    toCartesian(): Cartesian3 {
        return EnuToEcef(this, ORIGIN);
    }
    static fromCartesian(c: Cartesian3): Local3 {
        return EcefToEnu(c, ORIGIN);
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
}

export class Arc {
    centre: Local3;
    radius: number;
    theta0: number; // radians
    theta1: number; // >= theta0
    constructor(centre: Local3, radius: number, theta0: number, theta1: number) {
        this.centre = centre;
        this.radius = radius;
        this.theta0 = theta0;
        this.theta1 = theta1;
        // yes I did choose these names specifically to make them line up well
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
}

