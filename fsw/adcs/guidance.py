"""目標クォータニオン生成（sun / nadir / target / zenith 指向）。

body 座標系（docs/02 §1.2）:
- +Z: カメラ視線方向
- +X: 進行方向基準面

ICD §3: q は ECI→body（v_body = R(q) @ v_eci）。
"""

from __future__ import annotations

import numpy as np

from common.types import OMEGA_EARTH, R_EARTH
from fsw.quat_utils import quat_from_rot_columns, quat_from_two_vectors, quat_normalize


def _orthonormalize(primary: np.ndarray, secondary_hint: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """primary を第 1 軸として、secondary_hint に近い第 2 軸との直交基底を作る。

    戻り値は (e1, e2, e3) の ECI 表現（e1 = primary 方向）。
    """
    e1 = primary / (np.linalg.norm(primary) + 1e-15)
    hint = secondary_hint / (np.linalg.norm(secondary_hint) + 1e-15)
    if abs(np.dot(e1, hint)) > 0.999:
        # ほぼ平行なら別の任意ベクトルを使う
        hint = np.array([0.0, 1.0, 0.0]) if abs(e1[0]) > 0.9 else np.array([1.0, 0.0, 0.0])
    e3 = np.cross(e1, hint)
    e3 = e3 / (np.linalg.norm(e3) + 1e-15)
    e2 = np.cross(e3, e1)
    e2 = e2 / (np.linalg.norm(e2) + 1e-15)
    return e1, e2, e3


def sunpoint_quat(sun_eci: np.ndarray) -> np.ndarray:
    """SUNPOINT: body -Z 軸を太陽方向へ。残り自由度は最短回転（自由）。"""
    if sun_eci is None or np.linalg.norm(sun_eci) < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0])
    sun_dir = sun_eci / np.linalg.norm(sun_eci)
    # R(q) @ sun_dir(ECI) = body -Z（太陽が body -Z 方向に見える姿勢）
    return quat_from_two_vectors(sun_dir, np.array([0.0, 0.0, -1.0]))


def nadir_quat(pos_eci: np.ndarray, vel_eci: np.ndarray) -> np.ndarray:
    """NADIR_IMAGING: body +Z を地心方向（-r_hat）へ、+X を速度方向寄りに拘束。"""
    r_norm = np.linalg.norm(pos_eci)
    if r_norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    nadir_dir = -pos_eci / r_norm  # body +Z が向くべき ECI 方向
    v_norm = np.linalg.norm(vel_eci)
    vel_hint = vel_eci / v_norm if v_norm > 1e-6 else np.array([1.0, 0.0, 0.0])

    # body +Z = e1(nadir_dir), body +X は vel_hint に近い方向、body +Y = Z x X
    e_z, _, _ = _orthonormalize(nadir_dir, vel_hint)
    # e_z が +Z の ECI 表現。+X を vel_hint 方向へ射影して直交化。
    x_raw = vel_hint - np.dot(vel_hint, e_z) * e_z
    x_norm = np.linalg.norm(x_raw)
    if x_norm < 1e-6:
        # 速度がほぼ nadir 方向 → 任意軸
        x_raw = np.array([1.0, 0.0, 0.0]) - np.dot(np.array([1.0, 0.0, 0.0]), e_z) * e_z
        x_norm = np.linalg.norm(x_raw)
        if x_norm < 1e-6:
            x_raw = np.array([0.0, 1.0, 0.0]) - np.dot(np.array([0.0, 1.0, 0.0]), e_z) * e_z
            x_norm = np.linalg.norm(x_raw)
    e_x = x_raw / x_norm
    e_y = np.cross(e_z, e_x)
    e_y = e_y / np.linalg.norm(e_y)
    # 再直交化（e_x = e_y x e_z で厳密な右手系に）
    e_x = np.cross(e_y, e_z)
    e_x = e_x / np.linalg.norm(e_x)

    # R(q) の各行 = body 軸(X,Y,Z) を ECI で見た方向
    return quat_from_rot_columns(e_x, e_y, e_z)


def target_quat(pos_eci: np.ndarray, target_ecef_pos: np.ndarray, t: float) -> np.ndarray:
    """TARGET_IMAGING: body +Z を衛星→ターゲット地表点方向へ。

    target_ecef_pos: ターゲット地表点の ECEF 位置 [m]（地心から）。
    ECEF→ECI は簡易恒星時 theta = OMEGA_EARTH * t で回転。
    """
    theta = OMEGA_EARTH * t
    c, s = np.cos(theta), np.sin(theta)
    rz = np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])
    target_eci = rz @ target_ecef_pos
    line_of_sight = target_eci - pos_eci
    los_norm = np.linalg.norm(line_of_sight)
    if los_norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    z_dir = line_of_sight / los_norm  # body +Z の ECI 方向

    # 残り自由度: nadir 方向に近い基準を用いて拘束（進行方向情報がなければ位置ベクトルを使う）
    r_norm = np.linalg.norm(pos_eci)
    hint = -pos_eci / r_norm if r_norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    e_z, e_x, e_y = _orthonormalize(z_dir, hint)
    return quat_from_rot_columns(e_x, e_y, e_z)


def target_ecef_from_latlon(lat_deg: float, lon_deg: float) -> np.ndarray:
    """緯度経度からターゲット ECEF 位置ベクトル（地球表面上、半径 R_EARTH）を計算。"""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    x = R_EARTH * np.cos(lat) * np.cos(lon)
    y = R_EARTH * np.cos(lat) * np.sin(lon)
    z = R_EARTH * np.sin(lat)
    return np.array([x, y, z])


def observe_quat(target_eci: np.ndarray) -> np.ndarray:
    """OBSERVE: body +Z 軸を慣性方向 target_eci へ。残り自由度は最短回転（自由）。

    docs/09 §2: R(q_t) @ u_eci = [0,0,1]（body +Z = カメラを慣性方向へ）。
    """
    if target_eci is None or np.linalg.norm(target_eci) < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0])
    target_dir = target_eci / np.linalg.norm(target_eci)
    return quat_from_two_vectors(target_dir, np.array([0.0, 0.0, 1.0]))


def downlink_quat(pos_eci: np.ndarray) -> np.ndarray:
    """DOWNLINK: body -Z を天頂（+r_hat）方向へ。"""
    r_norm = np.linalg.norm(pos_eci)
    if r_norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    zenith_dir = pos_eci / r_norm
    # R(q) @ zenith(ECI) = body -Z（S 帯パッチ面 -Z を天頂へ）
    return quat_from_two_vectors(zenith_dir, np.array([0.0, 0.0, -1.0]))
