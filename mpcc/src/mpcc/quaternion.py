from geometry_msgs.msg import Quaternion
import math

def quaternion_to_RPY(q: Quaternion):
    #  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    #  https://github.com/rawify/Quaternion.js/blob/main/src/quaternion.js (order YPR)
    # See geometry.ts in gs-frontend

    w = q.w
    x = q.x
    y = q.y
    z = q.z
    wx = w * x
    wy = w * y
    wz = w * z
    xx = x * x
    xy = x * y
    xz = x * z
    yy = y * y
    yz = y * z
    zz = z * z


    def asin(t: float):
        return t >= 1 if math.pi / 2 else (t <= -1 if -math.pi / 2 else math.asin(t))

    return {
        "yaw": math.atan2(2 * (xy + wz), 1 - 2 * (yy + zz)), # Heading / Yaw
        "pitch": -asin(2 * (xz - wy)), # Attitude / Pitch
        "roll": math.atan2(2 * (yz + wx), 1 - 2 * (xx + yy)), # Bank / Roll
    }