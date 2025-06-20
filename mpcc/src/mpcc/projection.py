"Projection utilities. See projection.ts"

from dataclasses import dataclass
import math
from pymap3d.enu import geodetic2ecef, ecef2geodetic

# WGS-84 geodetic constants
a = 6378137.0  # WGS-84 Earth semimajor axis (m)
b = 6356752.314245  # Derived Earth semiminor axis (m)
f = (a - b) / a  # Ellipsoid Flatness
e_sq = f * (2 - f)  # Square of Eccentricity

@dataclass
class Cartesian3:
    "Units of meters"
    x: float
    y: float
    z: float

@dataclass
class Cartographic:
    "Units degrees, degrees, meters"
    latitude: float
    longitude: float
    altitude: float

@dataclass
class Enu:
    "Units meters"
    e: float
    n: float
    u: float
   

def ecef_to_enu(ecef: Cartesian3, origin: Cartographic):
    lambda_ = math.radians(origin.latitude)
    phi = math.radians(origin.longitude)
    s = math.sin(lambda_)
    N = a / math.sqrt(1 - e_sq * s * s)
    sin_lambda = math.sin(lambda_)
    cos_lambda = math.cos(lambda_)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)
    x0 = (origin.altitude + N) * cos_lambda * cos_phi
    y0 = (origin.altitude + N) * cos_lambda * sin_phi
    z0 = (origin.altitude + (1 - e_sq) * N) * sin_lambda
    xd = ecef.x - x0
    yd = ecef.y - y0
    zd = ecef.z - z0
    x_east = -sin_phi * xd + cos_phi * yd
    y_north = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd
    z_up = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
    return Enu(e=x_east, n=y_north, u=z_up)

def enu_to_ecef(local: Enu, origin: Cartographic):
   lambda_ = math.radians(origin.latitude)
   phi = math.radians(origin.longitude)
   s = math.sin(lambda_)
   N = a / math.sqrt(1 - e_sq * s * s)
   sin_lambda = math.sin(lambda_)
   cos_lambda = math.cos(lambda_)
   cos_phi = math.cos(phi)
   sin_phi = math.sin(phi)
   x0 = (origin.altitude + N) * cos_lambda * cos_phi
   y0 = (origin.altitude + N) * cos_lambda * sin_phi
   z0 = (origin.altitude + (1 - e_sq) * N) * sin_lambda
   xd = -sin_phi * local.e - cos_phi * sin_lambda * local.n + cos_lambda * cos_phi * local.u
   yd = cos_phi * local.e - sin_lambda * sin_phi * local.n + cos_lambda * sin_phi * local.u
   zd = cos_lambda * local.n + sin_lambda * local.u
   x = xd + x0
   y = yd + y0
   z = zd + z0
   return Cartesian3(x=x, y=y, z=z)

def geodetic_to_enu(geodetic: Cartographic, origin: Cartographic):
    x, y, z = geodetic2ecef(lat = geodetic.latitude, lon = geodetic.longitude, alt = geodetic.altitude)
    ecef = Cartesian3(x, y, z)
    return ecef_to_enu(ecef, origin)

def enu_to_geodetic(enu: Enu, origin: Cartographic):
    ecef = enu_to_ecef(enu, origin)
    lat, lon, alt = ecef2geodetic(ecef.x, ecef.y, ecef.z, deg=False)
    return Cartographic(math.degrees(lat), math.degrees(lon), alt)