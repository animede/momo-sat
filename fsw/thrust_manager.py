"""電気推進 THRUST モード管理（docs/08_thrust_extension.md §3）。

- GroundCommand("THRUST", {"duration_s": T}) の目標秒数を保持する。
- 噴射許可 4 条件（指向誤差 / 日照 / SoC / RW 角運動量）を判定し thruster_on を出す。
- 累積噴射時間・FSW 推定推進剤消費量を積算する。
"""

from __future__ import annotations

import numpy as np

from common.types import (
    G0,
    RW_INERTIA,
    RW_MAX_MOMENTUM,
    THRUST_FORCE,
    THRUST_ISP,
)

POINTING_ERROR_LIMIT_DEG = 5.0
SOC_LIMIT = 0.5
RW_MOMENTUM_FRAC_LIMIT = 0.8

# 推進剤消費率 [kg/s] = F / (Isp * g0)
_MDOT_KG_S = THRUST_FORCE / (THRUST_ISP * G0)


class ThrustManager:
    """THRUST モードの噴射許可判定・累積量管理を保持する。"""

    def __init__(self) -> None:
        self.duration_s = 0.0
        self.thrust_time = 0.0        # 累積噴射時間 [s]（FSW 側、許可判定に基づく積算）
        self.propellant_used_g = 0.0  # 消費推進剤 [g]（FSW 推定値）

    def reset(self) -> None:
        self.duration_s = 0.0
        self.thrust_time = 0.0
        self.propellant_used_g = 0.0

    def request(self, duration_s: float) -> None:
        """新規 THRUST コマンド受理。累積量はリセットして再開する。"""
        self.duration_s = max(float(duration_s), 0.0)
        self.thrust_time = 0.0
        self.propellant_used_g = 0.0

    def complete(self) -> bool:
        return self.thrust_time >= self.duration_s and self.duration_s > 0.0

    def evaluate(
        self,
        pointing_error_deg: float,
        solar_power: float,
        battery_soc: float,
        wheel_speeds: np.ndarray,
        dt: float,
    ) -> bool:
        """噴射許可 4 条件を判定し、許可されていれば累積量を積算して True を返す。

        条件（docs/08 §3）:
          1. 指向誤差 < 5 deg
          2. 日照中（solar_power > 0）
          3. battery_soc > 0.5
          4. RW 各軸 |h| < 0.8*RW_MAX_MOMENTUM （h = RW_INERTIA * wheel_speeds）
        既に累積噴射時間が目標に達していれば噴射しない（duration_s=0 未設定時も同様）。
        """
        if self.duration_s <= 0.0 or self.complete():
            return False

        if pointing_error_deg >= POINTING_ERROR_LIMIT_DEG:
            return False
        if not (solar_power > 0.0):
            return False
        if not (battery_soc > SOC_LIMIT):
            return False

        h = RW_INERTIA * np.asarray(wheel_speeds, dtype=float)
        if np.any(np.abs(h) >= RW_MOMENTUM_FRAC_LIMIT * RW_MAX_MOMENTUM):
            return False

        self.thrust_time += dt
        self.propellant_used_g += _MDOT_KG_S * dt * 1000.0
        return True
