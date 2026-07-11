"""軌道・姿勢ダイナミクス（ICD §3 座標系規約に厳密準拠）。

軌道: 二体問題 + J2 摂動、RK4 積分（ECI, [m], [m/s]）。
姿勢: オイラー方程式（RW 角運動量込み）+ クォータニオン運動学、RK4 積分。

クォータニオン規約: q = [x, y, z, w]（スカラ末尾）、ECI→body 回転。
  v_body = R(q) @ v_eci
  R(q) = (w^2 - |v|^2) I3 + 2 v v^T - 2 w [v]x
運動学: qdot = 1/2 Omega(omega) q, Omega(omega) = [[-[w]x, w], [-w^T, 0]]
"""

from __future__ import annotations

import numpy as np

from common.types import INERTIA, J2, MU_EARTH, R_EARTH


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
    """R(q): v_body = R(q) @ v_eci （ECI→body、ICD §3 準拠）。"""
    v = q[:3]
    w = q[3]
    return (
        (w * w - v.dot(v)) * np.eye(3)
        + 2.0 * np.outer(v, v)
        - 2.0 * w * skew(v)
    )


def quat_kinematics(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """qdot = 1/2 Omega(omega) q （4x4, スカラ末尾）。"""
    wx, wy, wz = omega
    big_omega = np.array([
        [0.0, wz, -wy, wx],
        [-wz, 0.0, wx, wy],
        [wy, -wx, 0.0, wz],
        [-wx, -wy, -wz, 0.0],
    ])
    return 0.5 * big_omega @ q


# ---------------------------------------------------------------------------
# 軌道伝播: 二体 + J2、RK4
# ---------------------------------------------------------------------------

def orbit_accel(pos: np.ndarray, j2_enabled: bool = True) -> np.ndarray:
    """ECI 位置 pos [m] における重力加速度 [m/s^2]（二体 + J2）。"""
    r = np.linalg.norm(pos)
    a_two_body = -MU_EARTH * pos / r ** 3

    if not j2_enabled:
        return a_two_body

    x, y, z = pos
    factor = 1.5 * J2 * MU_EARTH * R_EARTH ** 2 / r ** 5
    z2_r2 = 5.0 * z ** 2 / r ** 2
    ax = factor * x * (z2_r2 - 1.0)
    ay = factor * y * (z2_r2 - 1.0)
    az = factor * z * (z2_r2 - 3.0)
    return a_two_body + np.array([ax, ay, az])


def orbit_derivative(state: np.ndarray, j2_enabled: bool = True,
                      extra_accel_eci: np.ndarray | None = None) -> np.ndarray:
    """state = [pos(3), vel(3)] -> dstate/dt。

    extra_accel_eci: 摂動加速度 [m/s^2]（ECI, 1 ステップ内一定として近似）。
    None のときは既存挙動（二体+J2 のみ）と完全に同一。
    """
    pos = state[:3]
    vel = state[3:]
    acc = orbit_accel(pos, j2_enabled=j2_enabled)
    if extra_accel_eci is not None:
        acc = acc + extra_accel_eci
    return np.concatenate([vel, acc])


def rk4_step(state: np.ndarray, dt: float, deriv_fn) -> np.ndarray:
    """汎用 RK4 ステップ（deriv_fn(state) -> dstate/dt）。"""
    k1 = deriv_fn(state)
    k2 = deriv_fn(state + 0.5 * dt * k1)
    k3 = deriv_fn(state + 0.5 * dt * k2)
    k4 = deriv_fn(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def propagate_orbit(pos: np.ndarray, vel: np.ndarray, dt: float,
                     j2_enabled: bool = True,
                     extra_accel_eci: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """RK4 で 1 ステップ軌道伝播。

    extra_accel_eci: 摂動加速度 [m/s^2]（ECI）。RK4 の各ステージの微分項に
    注入される（v への単純加算ではない）。デフォルト None で既存挙動不変。
    """
    state = np.concatenate([pos, vel])
    new_state = rk4_step(
        state, dt,
        lambda s: orbit_derivative(s, j2_enabled=j2_enabled, extra_accel_eci=extra_accel_eci),
    )
    return new_state[:3], new_state[3:]


def orbital_period(a: float) -> float:
    """半長軸 a [m] の円軌道周期 [s]。"""
    return 2.0 * np.pi * np.sqrt(a ** 3 / MU_EARTH)


# ---------------------------------------------------------------------------
# 姿勢伝播: オイラー方程式 + クォータニオン運動学、RK4
# ---------------------------------------------------------------------------

def attitude_derivative(state: np.ndarray, torque_ext: np.ndarray,
                         h_rw: np.ndarray, inertia: np.ndarray = INERTIA) -> np.ndarray:
    """state = [q(4), omega(3)] -> dstate/dt。

    I omega_dot = torque_ext - omega x (I omega + h_rw)
    qdot = 1/2 Omega(omega) q

    torque_ext, h_rw は積分ステップ内で一定として扱う（RK4 の各ステージで
    呼び出し元が最新値を渡す設計にはしていないため、Simulator 側で
    ステップ内一定と近似する）。
    """
    q = state[:4]
    omega = state[4:]

    h_total = inertia @ omega + h_rw
    omega_dot = np.linalg.solve(inertia, torque_ext - np.cross(omega, h_total))
    qdot = quat_kinematics(q, omega)
    return np.concatenate([qdot, omega_dot])


def propagate_attitude(q: np.ndarray, omega: np.ndarray, dt: float,
                        torque_ext: np.ndarray, h_rw: np.ndarray,
                        inertia: np.ndarray = INERTIA) -> tuple[np.ndarray, np.ndarray]:
    """RK4 で 1 ステップ姿勢伝播。torque_ext, h_rw はステップ内一定と近似。"""
    state = np.concatenate([q, omega])
    new_state = rk4_step(
        state, dt,
        lambda s: attitude_derivative(s, torque_ext, h_rw, inertia=inertia),
    )
    q_new = quat_normalize(new_state[:4])
    omega_new = new_state[4:]
    return q_new, omega_new


def gravity_gradient_torque(pos_eci: np.ndarray, q: np.ndarray,
                             inertia: np.ndarray = INERTIA) -> np.ndarray:
    """重力傾斜トルク（body 系）。

    tau_gg = 3 mu / r^5 * (r_body x (I r_body))
    """
    r = np.linalg.norm(pos_eci)
    r_body = quat_to_rot(q) @ pos_eci
    return (3.0 * MU_EARTH / r ** 5) * np.cross(r_body, inertia @ r_body)
