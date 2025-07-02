//
// https://mavlink.io/en/services/mission.html#command_message_type

import { Cartographic, Ellipsoid } from "cesium";
import { Arc, Line, Local2, path_eval, path_length, type DubinsPath } from "./geometry";
import { MavrosMsgs } from "./rostypes/ros_msgs";
import type { BMFA_Coords } from "./waypoints";

/// Turn a DubinsPath into a partial mission, missing takeoff and landing
/// Spacing determines the spacing between intermediate waypoints along the path
export function get_path_wps(path: DubinsPath, cruise_altitude: number, spacing: number): MavrosMsgs.Waypoint[] {
    let coords: Cartographic[] = [];
    for (let s = 0; s < path_length(path); s += spacing) {
        let local = path_eval(path, s),
            cartes = local.toCartesian(),
            cart = Cartographic.fromCartesian(cartes, Ellipsoid.WGS84);
        coords.push(cart);
    }

    return coords.map((coord) => {
            return {
                frame: MavrosMsgs.WaypointFrame.FRAME_GLOBAL_RELATIVE_ALT_INT,
                command: MavrosMsgs.CommandCodeConst.NAV_WAYPOINT,
                param2: spacing / 2, // accept radius
                param3: 0,
                x_lat: degrees(coord.latitude),
                y_long: degrees(coord.longitude),
                z_alt: cruise_altitude
            } as MavrosMsgs.Waypoint;
        });    
}

function get_dubins_wp(segment: (Arc | Line), altitude: number) {
    if (segment instanceof Arc) {
        let center_coords = Cartographic.fromCartesian(segment.centre.toCartesian(), Ellipsoid.WGS84);
        return {
            frame: MavrosMsgs.WaypointFrame.FRAME_GLOBAL_RELATIVE_ALT_INT,
            command: MavrosMsgs.CommandCodeConst.NAV_LOITER_TURNS,
            param1: Math.abs(segment.dangle) / 2 / Math.PI, // number of turns
            param2: 1, 
            param3: segment.radius * segment.dir(), // radius (negative for counterclockwise)
            param4: 1, // xtrack leave tangent
            x_lat: degrees(center_coords.latitude),
            y_long: degrees(center_coords.longitude),
            z_alt: altitude
        } as MavrosMsgs.Waypoint;
    } else {
        let endpoint_coords = Cartographic.fromCartesian(segment.end.toCartesian(), Ellipsoid.WGS84);
        return {
            frame: MavrosMsgs.WaypointFrame.FRAME_GLOBAL_RELATIVE_ALT_INT,
            command: MavrosMsgs.CommandCodeConst.NAV_WAYPOINT,
            param1: 0.0,
            param2: 30.0, // accept radius
            param3: 0.0,
            param4: 0.0,
            x_lat: degrees(endpoint_coords.latitude),
            y_long: degrees(endpoint_coords.longitude),
            z_alt: altitude
        } as MavrosMsgs.Waypoint
    }
}

function degrees(rad: number) {
    return rad * 180 / Math.PI;
}

export function get_dubins_wps(path: DubinsPath, cruise_altitude: number) {
    return path.map((segment) => get_dubins_wp(segment, cruise_altitude));
}

/// Settable geofence option
export function get_geofence_wps(geofence: BMFA_Coords[]) {
    return geofence.map((coords) => {
        return {
            frame: MavrosMsgs.WaypointFrame.FRAME_GLOBAL_RELATIVE_ALT_INT,
            command: MavrosMsgs.CommandCodeConst.NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            param1: geofence.length, // number of vertices
            x_lat: coords.latitude,
            y_long: coords.longitude,
            z_alt: 0.0
        } as MavrosMsgs.Waypoint;
    });

}

export function export_waypoints(waypoints: MavrosMsgs.Waypoint[]) {
    let text = waypoints.map((waypoint, i) => `${i}\t${waypoint.is_current ?? 0}\t${waypoint.frame}\t${waypoint.command}\t${waypoint.param1}\t${waypoint.param2}\t${waypoint.param3}\t${waypoint.param4}\t${waypoint.x_lat}\t${waypoint.y_long}\t${waypoint.z_alt}\t${1}`).join('\n')
    return "QGC WPL 110\n" + text;
}

export function download_string(text: string, file_type: string, file_name: string) {
  var blob = new Blob([text], { type: file_type });

  var a = document.createElement('a');
  a.download = file_name;
  a.href = URL.createObjectURL(blob);
  a.dataset.downloadurl = [file_type, a.download, a.href].join(':');
  a.style.display = "none";
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(function() { URL.revokeObjectURL(a.href); }, 1500);
}