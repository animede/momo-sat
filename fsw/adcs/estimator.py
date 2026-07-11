"""姿勢決定: ジャイロ伝播 + TRIAD 補正。

ICD §3 座標系規約、docs/02 §2.4 準拠。

参照ベクトル（ECI 側）モデル:
- 太陽方向: 黄道傾斜 23.44°、平均黄経 λ = 280.46° + 0.9856474 deg/day * (t/86400)
- 地磁気: 地軸整列ダイポール B = (B0 R_E^3 / r^3) (3 (m_hat . r_hat) r_hat - m_hat)
  m_hat = -z_eci, B0 = 3.12e-5 T
"""

from __future__ import annotations

import numpy as np

from common.types import R_EARTH
from fsw.quat_utils import (
    quat_conj,
    quat_from_rot_columns,
    quat_mult,
    quat_normalize,
    quat_propagate,
    quat_to_rot,
)

OBLIQUITY_DEG = 23.44
B0 = 3.12e-5  # [T]


def sun_direction_eci(t: float) -> np.ndarray:
    """簡易太陽方向モデル（ECI, 単位ベクトル）。"""
    lam_deg = 280.46 + 0.9856474 * (t / 86400.0)
    lam = np.radians(lam_deg)
    eps = np.radians(OBLIQUITY_DEG)
    x = np.cos(lam)
    y = np.sin(lam) * np.cos(eps)
    z = np.sin(lam) * np.sin(eps)
    v = np.array([x, y, z])
    return v / np.linalg.norm(v)


def mag_field_eci(t: float, pos_eci: np.ndarray) -> np.ndarray:
    """地軸整列ダイポール近似による地磁気ベクトル（ECI）[T]。"""
    r = np.linalg.norm(pos_eci)
    if r < 1e-6:
        return np.zeros(3)
    r_hat = pos_eci / r
    m_hat = np.array([0.0, 0.0, -1.0])
    b = (B0 * R_EARTH ** 3 / r ** 3) * (3.0 * np.dot(m_hat, r_hat) * r_hat - m_hat)
    return b


class AttitudeEstimator:
    """ジャイロ伝播 + TRIAD 補正の姿勢決定器（ジャイロバイアス一次推定付き）。"""

    # バイアス推定: TRIAD 補正の姿勢イノベーション（body 系回転ベクトル）を
    # 積分してバイアスに帰着させる PI 型相補フィルタの I ゲイン
    KI_BIAS = 5.0e-4
    BIAS_LIMIT = np.radians(0.5)  # バイアス推定値の飽和 [rad/s]（発散防止）

    # スタートラッカ補正のブレンド率（docs/09 §2）: TRIAD（太陽+磁場、alpha=0.2）
    # より高重み。ST 計測は σ=10 秒角相当と TRIAD よりはるかに高精度なため。
    ST_BLEND_ALPHA = 0.5

    def __init__(self) -> None:
        self.q_est = np.array([0.0, 0.0, 0.0, 1.0])
        self.omega_est = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self._initialized = False

    def reset(self, q0: np.ndarray | None = None) -> None:
        self.q_est = quat_normalize(q0.copy()) if q0 is not None else np.array([0.0, 0.0, 0.0, 1.0])
        self.omega_est = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self._initialized = True

    def update(
        self,
        gyro: np.ndarray,
        mag_body: np.ndarray,
        sun_body: np.ndarray | None,
        t: float,
        pos_eci: np.ndarray,
        dt: float,
        st_quat: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """1 ステップぶんの姿勢決定を行い (q_est, omega_est) を返す。

        st_quat: スタートラッカ計測姿勢（有効なら TRIAD より高重みでブレンド、
        docs/09 §2）。None のときは既存どおり TRIAD/ジャイロ伝播のみを使う。
        """
        if not self._initialized:
            self.reset()

        self.omega_est = gyro - self.gyro_bias

        # ジャイロ伝播（バイアス補正後の角速度で）
        q_prop = quat_propagate(self.q_est, self.omega_est, dt)

        st_valid = st_quat is not None and np.all(np.isfinite(st_quat))

        # TRIAD 補正（太陽・磁場が両方有効なとき）
        if sun_body is not None and np.all(np.isfinite(sun_body)) and np.linalg.norm(mag_body) > 1e-9:
            sun_eci = sun_direction_eci(t)
            mag_eci_vec = mag_field_eci(t, pos_eci)
            mag_eci_norm = np.linalg.norm(mag_eci_vec)
            sun_body_norm = np.linalg.norm(sun_body)
            mag_body_norm = np.linalg.norm(mag_body)
            if mag_eci_norm > 1e-12 and sun_body_norm > 1e-9 and mag_body_norm > 1e-9:
                q_triad = self._triad(
                    sun_eci / np.linalg.norm(sun_eci),
                    mag_eci_vec / mag_eci_norm,
                    sun_body / sun_body_norm,
                    mag_body / mag_body_norm,
                )
                # ジャイロ伝播結果と TRIAD 結果を単純ブレンド（相補フィルタ的）
                self.q_est = self._slerp_blend(q_prop, q_triad, 0.2)
                # 姿勢イノベーション（伝播→TRIAD の body 系回転ベクトル）を
                # 積分してジャイロバイアスを推定。バイアスが正（読み過ぎ）だと
                # 伝播が進み過ぎ、イノベーション e は負方向に出るため -KI*e で更新
                q_innov = quat_mult(q_triad, quat_conj(q_prop))
                if q_innov[3] < 0:
                    q_innov = -q_innov
                e_innov = 2.0 * q_innov[:3]
                self.gyro_bias = np.clip(
                    self.gyro_bias - self.KI_BIAS * e_innov,
                    -self.BIAS_LIMIT, self.BIAS_LIMIT,
                )
            else:
                self.q_est = q_prop
        else:
            self.q_est = q_prop

        # スタートラッカ補正（有効なら TRIAD と同じ経路で高重みブレンド、
        # イノベーション積分則も流用してバイアス推定を続行する）。
        if st_valid:
            q_st = quat_normalize(st_quat.copy())
            q_before_st = self.q_est
            self.q_est = self._slerp_blend(q_before_st, q_st, self.ST_BLEND_ALPHA)
            q_innov = quat_mult(q_st, quat_conj(q_before_st))
            if q_innov[3] < 0:
                q_innov = -q_innov
            e_innov = 2.0 * q_innov[:3]
            self.gyro_bias = np.clip(
                self.gyro_bias - self.KI_BIAS * e_innov,
                -self.BIAS_LIMIT, self.BIAS_LIMIT,
            )

        return self.q_est, self.omega_est

    @staticmethod
    def _triad(v1_eci: np.ndarray, v2_eci: np.ndarray, v1_body: np.ndarray, v2_body: np.ndarray) -> np.ndarray:
        """TRIAD 法: 2 組のベクトル対から姿勢クォータニオンを求める。

        v1 を主基準（太陽方向）、v2 を副基準（磁場）として直交基底を作る。
        """
        # body 側基底
        t1b = v1_body
        t2b = np.cross(v1_body, v2_body)
        t2b_norm = np.linalg.norm(t2b)
        if t2b_norm < 1e-9:
            # ほぼ平行 → TRIAD 不能、恒等回転を返す
            return np.array([0.0, 0.0, 0.0, 1.0])
        t2b = t2b / t2b_norm
        t3b = np.cross(t1b, t2b)

        # ECI 側基底
        t1e = v1_eci
        t2e = np.cross(v1_eci, v2_eci)
        t2e_norm = np.linalg.norm(t2e)
        if t2e_norm < 1e-9:
            return np.array([0.0, 0.0, 0.0, 1.0])
        t2e = t2e / t2e_norm
        t3e = np.cross(t1e, t2e)

        # R_body_from_triad @ [t1e t2e t3e] = [t1b t2b t3b]
        # R(q) の行 i = body i 軸を ECI で見た方向、なので
        # 各 body 基底ベクトルを ECI 表現で書き下す必要がある。
        # M_body = [t1b t2b t3b] (列ベクトル), M_eci = [t1e t2e t3e]
        # R = M_body @ M_eci^T  (R(q) @ v_eci = v_body)
        m_body = np.column_stack([t1b, t2b, t3b])
        m_eci = np.column_stack([t1e, t2e, t3e])
        r = m_body @ m_eci.T
        return quat_from_rot_columns(r[0, :], r[1, :], r[2, :])

    @staticmethod
    def _slerp_blend(q_a: np.ndarray, q_b: np.ndarray, alpha: float) -> np.ndarray:
        """簡易 slerp（小角度なので線形ブレンド+正規化で近似）。"""
        dot = np.dot(q_a, q_b)
        if dot < 0:
            q_b = -q_b
        q = (1.0 - alpha) * q_a + alpha * q_b
        return quat_normalize(q)
