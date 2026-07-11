"""天体観測 OBSERVE モード管理（docs/09_astro_observation_extension.md §2）。

- GroundCommand("OBSERVE", {"ra_deg", "dec_deg", "duration_s", "exposure_s"}) を保持する。
- フレームシーケンス状態機械: 撮像条件成立を確認して露光(exposure_s)→
  ギャップ(FRAME_GAP_S) を観測窓 duration_s の間くり返す。
- 撮像条件（docs/09 §2）: 指向誤差 < 0.5deg、対目標レート誤差 < 0.05deg/s、
  目標非掩蔽（地平線角 + マージンより大きい離角）、RW 各軸 |h| < 0.8*上限。
- frames_captured / observe_time（累積露光 [s]）の管理、観測完了判定。
"""

from __future__ import annotations

import numpy as np

from common.types import (
    EARTH_LIMB_MARGIN_DEG,
    FRAME_GAP_S,
    R_EARTH,
    RW_INERTIA,
    RW_MAX_MOMENTUM,
)

# docs/09 §2 は真値 0.5deg。FSW は推定値でしか判定できないため、
# payload_manager.py（撮像条件 AD-2）と同じ考え方でマージンを取り 0.4 deg で
# 判定する（推定誤差ぶん、真の指向誤差が僅かに 0.5deg を超えて撮像条件が
# 成立してしまうケースを避ける）。
POINTING_ERROR_LIMIT_DEG = 0.4
RATE_ERROR_LIMIT_DEG_S = 0.05
RW_MOMENTUM_FRAC_LIMIT = 0.8


def target_direction_eci(ra_deg: float, dec_deg: float) -> np.ndarray:
    """RA/Dec [deg] から慣性系（ECI）単位ベクトルを計算する。

    u_eci = (cos(dec)cos(ra), cos(dec)sin(ra), sin(dec))
    """
    ra = np.radians(ra_deg)
    dec = np.radians(dec_deg)
    return np.array([
        np.cos(dec) * np.cos(ra),
        np.cos(dec) * np.sin(ra),
        np.sin(dec),
    ])


def is_occulted(target_eci: np.ndarray, pos_eci: np.ndarray) -> bool:
    """目標方向が地球に掩蔽されているか（FSW/sim 共通判定式、docs/09 §2, §3）。

    地平線角 rho = arcsin(R_EARTH/r)。目標方向 u と地心方向 (-r_hat) の
    離角 theta が theta < rho + EARTH_LIMB_MARGIN_DEG のとき掩蔽/迷光 NG。
    """
    r = np.linalg.norm(pos_eci)
    if r < 1e-6:
        return True
    r_hat = pos_eci / r
    geocenter_dir = -r_hat
    u = target_eci / (np.linalg.norm(target_eci) + 1e-15)
    cos_theta = float(np.clip(np.dot(u, geocenter_dir), -1.0, 1.0))
    theta_deg = float(np.degrees(np.arccos(cos_theta)))
    ratio = np.clip(R_EARTH / r, -1.0, 1.0)
    rho_deg = float(np.degrees(np.arcsin(ratio)))
    return theta_deg < (rho_deg + EARTH_LIMB_MARGIN_DEG)


class ObserveManager:
    """OBSERVE モードのフレームシーケンス状態機械・撮像条件判定を保持する。"""

    def __init__(self) -> None:
        self.ra_deg = 0.0
        self.dec_deg = 0.0
        self.duration_s = 0.0
        self.exposure_s = 0.0

        self.frames_captured = 0
        self.observe_time = 0.0  # 累積露光時間 [s]（FSW 側）

        self._elapsed_s = 0.0       # 観測窓に入ってからの経過時間
        self._in_exposure = False
        self._phase_timer = 0.0     # 現在フェーズ（露光 or ギャップ）の経過時間
        self._started = False
        self._completion_consumed = False

    def reset(self) -> None:
        self.frames_captured = 0
        self.observe_time = 0.0
        self._elapsed_s = 0.0
        self._in_exposure = False
        self._phase_timer = 0.0
        self._started = False
        self._completion_consumed = False

    def request(self, ra_deg: float, dec_deg: float, duration_s: float, exposure_s: float) -> None:
        """新規 OBSERVE コマンド受理。累積量はリセットして再開する。"""
        self.ra_deg = float(ra_deg)
        self.dec_deg = float(dec_deg)
        self.duration_s = max(float(duration_s), 0.0)
        self.exposure_s = max(float(exposure_s), 1e-6)
        self.reset()
        self._started = True

    def target_eci(self) -> np.ndarray:
        return target_direction_eci(self.ra_deg, self.dec_deg)

    def complete(self) -> bool:
        return self._started and self._elapsed_s >= self.duration_s and self.duration_s > 0.0

    def consume_completion_edge(self) -> bool:
        """観測完了を検出した最初の呼び出しでのみ True を返す（エッジトリガ）。

        以後は request() で再起動されるまで False を返し続ける
        （images_stored への二重加算を防ぐ）。
        """
        if self.complete() and not self._completion_consumed:
            self._completion_consumed = True
            return True
        return False

    def conditions_met(
        self,
        pointing_error_deg: float,
        rate_error_deg_s: float,
        pos_eci: np.ndarray,
        wheel_speeds: np.ndarray,
    ) -> bool:
        """撮像条件 4 項目（docs/09 §2）を判定する。"""
        if pointing_error_deg >= POINTING_ERROR_LIMIT_DEG:
            return False
        if rate_error_deg_s >= RATE_ERROR_LIMIT_DEG_S:
            return False
        if is_occulted(self.target_eci(), pos_eci):
            return False
        h = RW_INERTIA * np.asarray(wheel_speeds, dtype=float)
        if np.any(np.abs(h) >= RW_MOMENTUM_FRAC_LIMIT * RW_MAX_MOMENTUM):
            return False
        return True

    def step(
        self,
        dt: float,
        pointing_error_deg: float,
        rate_error_deg_s: float,
        pos_eci: np.ndarray,
        wheel_speeds: np.ndarray,
    ) -> bool:
        """1 制御ステップ実行し shutter_open を返す。

        フレームシーケンス（docs/09 §2）:
          1. 撮像条件成立を確認 -> shutter_open=True を exposure_s 継続（露光）
          2. shutter_open=False を FRAME_GAP_S 継続（読み出し・脱飽和許可窓）
        条件不成立時はギャップ相当として待機する（露光は開始しない）。
        """
        if not self._started or self.complete():
            return False

        self._elapsed_s += dt

        if self._in_exposure:
            self._phase_timer += dt
            self.observe_time += dt
            if self._phase_timer >= self.exposure_s - 1e-9:
                # 露光完了 -> 1 フレーム取得、ギャップへ
                self.frames_captured += 1
                self._in_exposure = False
                self._phase_timer = 0.0
            return True
        else:
            # ギャップ中、または撮像条件待ち
            if self._phase_timer < FRAME_GAP_S:
                self._phase_timer += dt
                return False
            # ギャップ終了。条件が成立していれば新しい露光を開始する。
            if self.conditions_met(pointing_error_deg, rate_error_deg_s, pos_eci, wheel_speeds):
                self._in_exposure = True
                self._phase_timer = dt
                self.observe_time += dt
                return True
            # 条件不成立: ギャップ待機を継続（タイマーは FRAME_GAP_S に留め置く）
            self._phase_timer = FRAME_GAP_S
            return False
