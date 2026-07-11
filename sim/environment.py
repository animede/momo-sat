"""環境モデル: 太陽方向、食判定、地磁気ダイポール、外乱トルク（ICD §4 準拠）。"""

from __future__ import annotations

import numpy as np

from common.types import R_EARTH
from sim.dynamics import gravity_gradient_torque  # re-exported for sim.environment consumers

# 黄道傾斜角 [rad]
OBLIQUITY = np.radians(23.44)

# 地磁気ダイポール強度 [T]（赤道上、地表）
B0_DIPOLE = 3.12e-5

# 残留磁気モーメント [A m^2]（body 固定、ランダム方向、大きさ固定）
RESIDUAL_DIPOLE_MAGNITUDE = 5.0e-3

# 定常小外乱トルク（大気抵抗等相当）[N m]
CONSTANT_DISTURBANCE_TORQUE_MAG = 1.0e-8


def sun_direction_eci(t: float) -> np.ndarray:
    """時刻 t [s]（エポックからの経過秒）における太陽方向単位ベクトル（ECI）。

    平均黄経 lambda = 280.46 deg + 0.9856474 deg/day * (t/86400)
    黄道傾斜 23.44 deg。
    """
    days = t / 86400.0
    lam = np.radians(280.46 + 0.9856474 * days)
    eps = OBLIQUITY
    sx = np.cos(lam)
    sy = np.cos(eps) * np.sin(lam)
    sz = np.sin(eps) * np.sin(lam)
    v = np.array([sx, sy, sz])
    return v / np.linalg.norm(v)


def is_eclipse(pos_eci: np.ndarray, sun_dir_eci: np.ndarray) -> bool:
    """円筒影食判定。

    太陽と反対側で、太陽方向軸からの垂直距離が地球半径未満なら食。
    """
    r_dot_s = float(np.dot(pos_eci, sun_dir_eci))
    if r_dot_s >= 0.0:
        return False
    r_perp = pos_eci - r_dot_s * sun_dir_eci
    return float(np.linalg.norm(r_perp)) < R_EARTH


def magnetic_field_eci(pos_eci: np.ndarray) -> np.ndarray:
    """地軸整列ダイポール磁場（ECI 表示）[T]。

    B = (B0 R_E^3 / r^3) * (3 (m_hat . r_hat) r_hat - m_hat)
    m_hat = -z_hat_ECI
    """
    r = np.linalg.norm(pos_eci)
    r_hat = pos_eci / r
    m_hat = np.array([0.0, 0.0, -1.0])
    b = (B0_DIPOLE * R_EARTH ** 3 / r ** 3) * (3.0 * np.dot(m_hat, r_hat) * r_hat - m_hat)
    return b


def residual_dipole_torque(b_body: np.ndarray, residual_dipole_body: np.ndarray) -> np.ndarray:
    """残留磁気モーメントによるトルク（body 系）: tau = m x B。"""
    return np.cross(residual_dipole_body, b_body)


def constant_disturbance_torque(direction_body: np.ndarray) -> np.ndarray:
    """定常小外乱トルク（大気抵抗等相当、body 固定方向 x 大きさ）。"""
    return CONSTANT_DISTURBANCE_TORQUE_MAG * direction_body
