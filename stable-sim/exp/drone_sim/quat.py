# Small quaternion helpers. Frame is FLU: x forward, y left, z up.
# Quaternions are [w, x, y, z] and rotate body vectors into the world.
import math


def qnorm(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) or 1.0
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def qconj(q):
    return [q[0], -q[1], -q[2], -q[3]]


# Rotate vector v (body) into the world frame.
def qrot(q, v):
    w, x, y, z = q
    ix = w * v[0] + y * v[2] - z * v[1]
    iy = w * v[1] + z * v[0] - x * v[2]
    iz = w * v[2] + x * v[1] - y * v[0]
    iw = -x * v[0] - y * v[1] - z * v[2]
    return [ix * w - iw * x - iy * z + iz * y,
            iy * w - iw * y - iz * x + ix * z,
            iz * w - iw * z - ix * y + iy * x]


# Advance q by body angular rate w (rad/s) over dt.
def qintegrate(q, w, dt):
    h = 0.5 * dt
    return qnorm([q[0] - h * (q[1] * w[0] + q[2] * w[1] + q[3] * w[2]),
                  q[1] + h * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]),
                  q[2] + h * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0]),
                  q[3] + h * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0])])


# Roll, pitch, yaw in radians. +roll = right side down, +pitch = nose down, +yaw = nose left.
def euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def qyaw(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
