from __future__ import annotations

from .data_types import Quat, Vec3


def add_vec3(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub_vec3(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale_vec3(v: Vec3, scalar: float) -> Vec3:
    return (v[0] * scalar, v[1] * scalar, v[2] * scalar)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_quat(q: Quat) -> Quat:
    norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    if norm_sq <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    inv_norm = norm_sq ** -0.5
    return (q[0] * inv_norm, q[1] * inv_norm, q[2] * inv_norm, q[3] * inv_norm)
