"""クォータニオン・回転行列の共通ユーティリティ（ICD §3 準拠）。

規約:
- q = [x, y, z, w]（スカラ末尾）、ECI→body 回転。
- v_body = R(q) @ v_eci
  R(q) = (w^2 - |v|^2) I3 + 2 v v^T - 2 w [v]x
- 伝播（ω は body 系）: qdot = 1/2 Omega(omega) q
  Omega(omega) = [[-[w]x, w], [-w^T, 0]]  (4x4, スカラ末尾)
"""

from __future__ import annotations

import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    """3 次元ベクトルの歪対称行列 [v]x。"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """R(q): v_body = R(q) @ v_eci （ECI→body、Shuster 規約）。"""
    v = q[:3]
    w = q[3]
    return (
        (w * w - v.dot(v)) * np.eye(3)
        + 2.0 * np.outer(v, v)
        - 2.0 * w * skew(v)
    )


def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """クォータニオン積 q1 (x) q2（スカラ末尾、Shuster 規約）。

    合成回転の意味: R(q1 (x) q2) = R(q1) @ R(q2)
    （つまり q2 を先に適用してから q1 を適用、という合成規則）。
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 - y1 * z2 + z1 * y2
    y = w1 * y2 + x1 * z2 + y1 * w2 - z1 * x2
    z = w1 * z2 - x1 * y2 + y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w])


def quat_conj(q: np.ndarray) -> np.ndarray:
    """共役（単位クォータニオンでは逆元）。"""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_propagate(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """ジャイロ角速度 omega（body 系）で q を dt だけ伝播し正規化する。"""
    wx, wy, wz = omega
    big_omega = np.array([
        [0.0, wz, -wy, wx],
        [-wz, 0.0, wx, wy],
        [wy, -wx, 0.0, wz],
        [-wx, -wy, -wz, 0.0],
    ])
    qdot = 0.5 * big_omega @ q
    q_new = q + qdot * dt
    return quat_normalize(q_new)


def quat_from_two_vectors(v_from: np.ndarray, v_to: np.ndarray) -> np.ndarray:
    """v_from を v_to に最短回転で重ねるクォータニオンを返す（q = [x,y,z,w]）。

    R(q) @ v_from ≒ v_to となる q。
    """
    a = v_from / (np.linalg.norm(v_from) + 1e-15)
    b = v_to / (np.linalg.norm(v_to) + 1e-15)
    cos_theta = float(np.clip(a.dot(b), -1.0, 1.0))
    if cos_theta > 1.0 - 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    if cos_theta < -1.0 + 1e-12:
        # 180 度回転: a に直交する任意軸を選ぶ
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        axis = axis / np.linalg.norm(axis)
        return np.array([axis[0], axis[1], axis[2], 0.0])
    # 受動規約（R(q) = ... - 2w[v]x）では、v_from を v_to へ移す能動回転の
    # 軸は b×a 側に取る必要がある（a×b だと逆回転になる）
    axis = np.cross(b, a)
    w = 1.0 + cos_theta
    q = np.array([axis[0], axis[1], axis[2], w])
    return quat_normalize(q)


def quat_from_rot_columns(x_axis: np.ndarray, y_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    """body 座標軸を ECI 表示した 3 本の直交ベクトル（R(q) の行に相当）から
    q（ECI→body）を Shepperd 法で求める。

    R(q) の各"行"が ECI 系での body 軸方向を表す（v_body = R @ v_eci なので
    R の行 i は「body の i 軸を ECI で見た方向」）。
    """
    r = np.vstack([x_axis, y_axis, z_axis])  # R(q) そのもの
    # 注意: 以下の Shepperd 法は標準規約（R = ... + 2w[v]x）向けの公式のため、
    # ICD の受動規約（- 2w[v]x）ではベクトル部の符号が反転して求まる。
    # 末尾で v 部を反転して返す。
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return quat_normalize(np.array([-x, -y, -z, w]))


def quat_error(q_target: np.ndarray, q_est: np.ndarray) -> np.ndarray:
    """誤差クォータニオン q_err = q_target (x) q_est^-1。

    R(q_err) = R(q_target) @ R(q_est)^-1（現在の body 系から目標 body 系への
    回転、ベクトル部は body 系表示）。q_err が単位クォータニオンに近いほど
    整定。制御則は fsw/adcs/controller.py の quat_error_vec を参照。
    """
    return quat_mult(q_target, quat_conj(q_est))


def angle_between_quats_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """2 つのクォータニオン間の回転角 [deg]。"""
    qe = quat_mult(q1, quat_conj(q2))
    w = float(np.clip(abs(qe[3]), -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(w)))
