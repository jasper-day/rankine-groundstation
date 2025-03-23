
import { Cartesian3, Cartographic } from 'cesium';
import { Local3 } from "$lib/geometry";

/// https://gist.github.com/govert/1b373696c9a27ff4c72a
// WGS-84 geodetic constants
const a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const b = 6356752.314245;     // Derived Earth semiminor axis (m)
const f = (a - b) / a;           // Ellipsoid Flatness
const e_sq = f * (2 - f);    // Square of Eccentricity

// Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to 
// East-North-Up coordinates in a Local Tangent Plane that is centered at the 
// (WGS-84) Geodetic point (lat0, lon0, h0).
export function EcefToEnu(ecef: Cartesian3, origin: Cartographic): Local3 {
    const lambda = origin.latitude;
    const phi = origin.longitude;
    const s = Math.sin(lambda);
    const N = a / Math.sqrt(1 - e_sq * s * s);

    const sin_lambda = Math.sin(lambda);
    const cos_lambda = Math.cos(lambda);
    const cos_phi = Math.cos(phi);
    const sin_phi = Math.sin(phi);

    const x0 = (origin.height + N) * cos_lambda * cos_phi;
    const y0 = (origin.height + N) * cos_lambda * sin_phi;
    const z0 = (origin.height + (1 - e_sq) * N) * sin_lambda;

    const xd = ecef.x - x0;
    const yd = ecef.y - y0;
    const zd = ecef.z - z0;

    // This is the matrix multiplication
    const xEast = -sin_phi * xd + cos_phi * yd;
    const yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    const zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    return new Local3(xEast, yNorth, zUp);
}

// Inverse of EcefToEnu. Converts East-North-Up coordinates (xEast, yNorth, zUp) in a
// Local Tangent Plane that is centered at the (WGS-84) Geodetic point (lat0, lon0, h0)
// to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
export function EnuToEcef(local: Local3, origin: Cartographic): Cartesian3 {
    var lambda = origin.latitude;
    var phi = origin.longitude;
    var s = Math.sin(lambda);
    var N = a / Math.sqrt(1 - e_sq * s * s);

    var sin_lambda = Math.sin(lambda);
    var cos_lambda = Math.cos(lambda);
    var cos_phi = Math.cos(phi);
    var sin_phi = Math.sin(phi);

    const x0 = (origin.height + N) * cos_lambda * cos_phi;
    const y0 = (origin.height + N) * cos_lambda * sin_phi;
    const z0 = (origin.height + (1 - e_sq) * N) * sin_lambda;

    const xd = -sin_phi * local.x - cos_phi * sin_lambda * local.y + cos_lambda * cos_phi * local.z;
    const yd = cos_phi * local.x - sin_lambda * sin_phi * local.y + cos_lambda * sin_phi * local.z;
    const zd = cos_lambda * local.y + sin_lambda * local.z;

    const x = xd + x0;
    const y = yd + y0;
    const z = zd + z0;
    return new Cartesian3(x, y, z);
}
