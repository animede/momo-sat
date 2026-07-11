"""通信管理: GroundCommand 解釈・Telemetry 生成・ダウンリンク管理。

ICD §5: DOWNLINK モードで gs_visible のとき tx_active=True。
伝送完了推定はレート DOWNLINK_RATE_MBPS x 送信時間の積算で
images_downlinked を更新（IMAGE_SIZE_MB 毎に 1 枚）。
"""

from __future__ import annotations

from common.types import DOWNLINK_RATE_MBPS, IMAGE_SIZE_MB

MBPS_TO_MBYTE_PER_S = 1.0 / 8.0  # Mbit/s -> MByte/s


class CommManager:
    """ダウンリンク進捗（送信済みデータ量）を積算管理する。"""

    def __init__(self) -> None:
        self.images_downlinked = 0
        self._accumulated_mb = 0.0
        self.last_cmd = ""

    def reset_progress(self) -> None:
        self._accumulated_mb = 0.0

    def downlink_pending(self, images_stored: int) -> int:
        """未送信の画像枚数。"""
        return max(images_stored - self.images_downlinked, 0)

    def update_tx(self, tx_active: bool, dt: float, images_stored: int) -> None:
        """送信中ならデータ量を積算し、1 枚分たまるごとに images_downlinked を増やす。"""
        if not tx_active:
            return
        if self.downlink_pending(images_stored) <= 0:
            return
        self._accumulated_mb += DOWNLINK_RATE_MBPS * MBPS_TO_MBYTE_PER_S * dt
        while self._accumulated_mb >= IMAGE_SIZE_MB and self.downlink_pending(images_stored) > 0:
            self._accumulated_mb -= IMAGE_SIZE_MB
            self.images_downlinked += 1

    def all_downlinked(self, images_stored: int) -> bool:
        return self.downlink_pending(images_stored) == 0
