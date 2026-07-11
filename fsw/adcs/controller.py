"""制御則: B-dot（デタンブル）、クォータニオン PD 制御、RW 脱飽和。

docs/02 §2.4、ユーザー指定ゲイン目安:
- B-dot: m = -k_bdot * dB/dt, k_bdot ≈ 1e4
- PD: tau = -Kp * sign(q_err_w) * q_err_vec - Kd * omega_err, Kp≈4e-5, Kd≈4e-4
- 脱飽和: m_desat = -k_ds * (h x B) / |B|^2  (|h| > 0.5*RW_MAX_MOMENTUM のとき)
"""

from __future__ import annotations

import numpy as np

from common.types import MTQ_MAX_MOMENT, RW_INERTIA, RW_MAX_MOMENTUM, RW_MAX_TORQUE
from fsw.quat_utils import quat_conj, quat_mult

K_BDOT = 1.0e4
KP = 4.0e-5
# NOTE: ユーザー仕様の目安値は Kd≈4e-4 だが、RW トルク飽和（±RW_MAX_TORQUE=1e-4 Nm）
# 下で大角度（~100 度級）誤差から収束させるにはより強い減衰が必要と判明した
# （数値実験で Kd=4e-4 はトルク飽和とクォータニオン運動学の非線形結合により
#  リミットサイクルに陥り発散する場合がある）。Kd=2.5e-3 で全ケース収束を確認。
KD = 2.5e-3
K_DESAT = 5.0e3
DESAT_THRESHOLD_FRAC = 0.5


def bdot_moment(mag_prev: np.ndarray, mag_now: np.ndarray, dt: float, k_bdot: float = K_BDOT) -> np.ndarray:
    """B-dot 則: m = -k * dB/dt（差分近似）。±MTQ_MAX_MOMENT にクリップ。"""
    if dt <= 0:
        return np.zeros(3)
    b_dot = (mag_now - mag_prev) / dt
    m = -k_bdot * b_dot
    return np.clip(m, -MTQ_MAX_MOMENT, MTQ_MAX_MOMENT)


def quat_error_vec(q_target: np.ndarray, q_est: np.ndarray) -> tuple[np.ndarray, float]:
    """誤差クォータニオン q_err = q_target (x) q_est^-1 のベクトル部・スカラ部を返す。

    R(q_err) = R(q_target) @ R(q_est)^-1（現在の body 系から目標 body 系への
    回転）。この順序ではベクトル部が body 系表示の回転軸を与え、そのまま
    トルク方向に使える。逆順（q_est^-1 (x) q_target）はベクトル部の表示系が
    異なり、恒等でない目標に対して 180° 反転側へ静定することを
    数値実験で確認した（WP4 統合試験で修正）。
    """
    q_err = quat_mult(q_target, quat_conj(q_est))
    return q_err[:3], q_err[3]


def pd_control(
    q_target: np.ndarray,
    q_est: np.ndarray,
    omega_est: np.ndarray,
    omega_target: np.ndarray | None = None,
    kp: float = KP,
    kd: float = KD,
) -> np.ndarray:
    """クォータニオン誤差 PD 制御則。±RW_MAX_TORQUE にクリップ。

    tau = +Kp * sign(q_err_w) * q_err_vec - Kd * omega_err
    （q_err_vec は「目標へ向かう回転方向」を指すのでそのままの符号で加速する）
    """
    if omega_target is None:
        omega_target = np.zeros(3)
    q_err_vec, q_err_w = quat_error_vec(q_target, q_est)
    omega_err = omega_est - omega_target
    sign_w = 1.0 if q_err_w >= 0 else -1.0
    tau = kp * sign_w * q_err_vec - kd * omega_err
    return np.clip(tau, -RW_MAX_TORQUE, RW_MAX_TORQUE)


def rw_desaturate(
    wheel_speeds: np.ndarray,
    mag_body: np.ndarray,
    k_desat: float = K_DESAT,
    threshold_frac: float = DESAT_THRESHOLD_FRAC,
) -> np.ndarray:
    """RW 角運動量が閾値を超えたとき、クロスプロダクト則で MTQ 磁気モーメントを出す。

    h = RW_INERTIA * wheel_speeds
    |h| > threshold_frac * RW_MAX_MOMENTUM のとき
    m_desat = -k_ds * (h x B) / |B|^2
    """
    h = RW_INERTIA * wheel_speeds
    h_norm = np.linalg.norm(h)
    b_norm2 = np.dot(mag_body, mag_body)
    if h_norm <= threshold_frac * RW_MAX_MOMENTUM or b_norm2 < 1e-18:
        return np.zeros(3)
    m = -k_desat * np.cross(h, mag_body) / b_norm2
    return np.clip(m, -MTQ_MAX_MOMENT, MTQ_MAX_MOMENT)
