import { Cartesian3, Ellipsoid } from "cesium";

export enum Coord_Type {
    RUNWAY, GEOFENCE, WAYPOINT, PAYLOAD
};

export interface BMFA_Coords {
    name: string;
    latitude: number;
    longitude: number;
    type: Coord_Type;
}

export function get_cartesians(coordinates: BMFA_Coords[]) {
    const degrees_heights_array = coordinates
                .map((coord) => {
                    return [coord.longitude, coord.latitude, 170];
                })
                .flat();
    return Cartesian3.fromDegreesArrayHeights(degrees_heights_array, Ellipsoid.WGS84);
}

export function to_czml(coordinate: BMFA_Coords) {
    return {
        name: coordinate.name,
        label: {
            show: true,
            fillColor: {
                rgba: [255, 255, 255, 255],
            },
            font: "10px sans-serif",
            horizontalOrigin: "LEFT",
            pixelOffset: {
                cartesian2: [8, 0],
            },
            style: "FILL",
            text: coordinate.name,
            showBackground: true,
            backgroundColor: {
                rgba: [0, 133, 171, 100],
            },
        },
        position: {
            cartographicDegrees: [coordinate.longitude, coordinate.latitude, 200]
        }

    }
}
