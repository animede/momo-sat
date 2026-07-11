"""ペイロード管理: 撮像シーケンス・画像カウント管理。

ICD §5:
- NADIR/TARGET モードで指向誤差 <0.5° かつ |ω|<0.05°/s のとき camera_trigger。
- 1 枚ごとに 5 s 以上間隔。payload_power は撮像モード中 True。
- images_stored は FSW 内部でカウント（トリガ成功を仮定して +1、上限 MAX_STORED_IMAGES）。
"""

from __future__ import annotations

from common.types import MAX_STORED_IMAGES

# 要求 AD-2 は真値 0.5 deg。FSW は推定値でしか判定できないため、
# 姿勢決定誤差 (AD-1: 0.1 deg) ぶんのマージンを取って 0.4 deg で判定する
POINTING_ERROR_LIMIT_DEG = 0.4
RATE_LIMIT_DEG_S = 0.05
MIN_TRIGGER_INTERVAL_S = 5.0


class PayloadManager:
    """撮像トリガ判定と画像蓄積カウントを管理する。"""

    def __init__(self) -> None:
        self.images_stored = 0
        self.images_requested = 0  # コマンドで要求された残り撮像枚数
        self._last_trigger_t: float | None = None
        self.target_latlon: tuple[float, float] | None = None

    def request_nadir(self, count: int) -> None:
        self.images_requested = max(int(count), 0)

    def request_target(self, lat_deg: float, lon_deg: float) -> None:
        self.target_latlon = (lat_deg, lon_deg)
        self.images_requested = 1

    def sequence_complete(self) -> bool:
        return self.images_requested <= 0

    def try_trigger(
        self,
        t: float,
        pointing_error_deg: float,
        omega_norm_deg_s: float,
    ) -> bool:
        """撮像条件を満たしていればトリガし、内部カウントを更新して True を返す。"""
        if self.images_requested <= 0:
            return False
        if pointing_error_deg >= POINTING_ERROR_LIMIT_DEG:
            return False
        if omega_norm_deg_s >= RATE_LIMIT_DEG_S:
            return False
        if self._last_trigger_t is not None and (t - self._last_trigger_t) < MIN_TRIGGER_INTERVAL_S:
            return False
        if self.images_stored >= MAX_STORED_IMAGES:
            return False

        self._last_trigger_t = t
        self.images_stored += 1
        self.images_requested -= 1
        return True

    def reset_target(self) -> None:
        self.target_latlon = None
