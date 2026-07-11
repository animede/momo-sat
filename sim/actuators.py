"""アクチュエータモデル（ICD §4）: RW（一次遅れ+飽和）、MTQ、カメラ、送信機。"""

from __future__ import annotations

import numpy as np

from common.types import (
    IMAGE_SIZE_MB,
    MAX_STORED_IMAGES,
    MTQ_MAX_MOMENT,
    RW_MAX_MOMENTUM,
    RW_MAX_TORQUE,
)

RW_TIME_CONSTANT = 0.05  # [s] 一次遅れ時定数


class ReactionWheelAssembly:
    """RW 3 軸: 指令トルクへの一次遅れ + 角運動量飽和。

    ICD 規約: wheel_torque は「機体に働かせたいトルク」。
      機体へ: +wheel_torque（実トルク、一次遅れ後）
      ホイール角運動量 h_rw へ: -wheel_torque を積算（反作用）
    飽和判定は h_rw（各軸 ±RW_MAX_MOMENTUM）で行い、飽和中はそれ以上
    その方向へ増えないよう実トルクを制限する。
    """

    def __init__(self):
        self.actual_torque = np.zeros(3)   # 一次遅れ後の実トルク（機体側）[N m]
        self.h_rw = np.zeros(3)            # ホイール角運動量 [N m s]

    def step(self, commanded_torque: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """1 ステップ更新。

        戻り値: (body_torque, h_rw)
          body_torque: 機体に働くトルク（+wheel_torque 相当、一次遅れ後の実トルク）
          h_rw: 更新後のホイール角運動量
        """
        cmd = np.clip(commanded_torque, -RW_MAX_TORQUE, RW_MAX_TORQUE)

        # 一次遅れ: actual_torque_dot = (cmd - actual_torque) / tau
        alpha = dt / RW_TIME_CONSTANT
        alpha = min(alpha, 1.0)
        self.actual_torque = self.actual_torque + alpha * (cmd - self.actual_torque)

        # h_rw の変化は -actual_torque * dt（反作用）。飽和端で同方向にさらに
        # 増えようとする場合は実トルクを無効化する（=それ以上飽和しない）。
        new_h = self.h_rw - self.actual_torque * dt
        applied_torque = self.actual_torque.copy()
        for i in range(3):
            if abs(new_h[i]) > RW_MAX_MOMENTUM:
                if (self.h_rw[i] >= RW_MAX_MOMENTUM and self.actual_torque[i] < 0.0) or \
                   (self.h_rw[i] <= -RW_MAX_MOMENTUM and self.actual_torque[i] > 0.0):
                    # 既に飽和端にいて同方向に増やそうとしている -> トルク無効
                    applied_torque[i] = 0.0
                    new_h[i] = self.h_rw[i]
                else:
                    new_h[i] = np.clip(new_h[i], -RW_MAX_MOMENTUM, RW_MAX_MOMENTUM)

        self.h_rw = new_h
        # 機体側に及ぶトルクは +wheel_torque（実トルク）
        body_torque = applied_torque
        return body_torque, self.h_rw


def mtq_torque(m_command: np.ndarray, b_body: np.ndarray) -> np.ndarray:
    """MTQ トルク: tau = m x B（理想、遅れなし）。m は上限クリップ済みを想定。"""
    m = np.clip(m_command, -MTQ_MAX_MOMENT, MTQ_MAX_MOMENT)
    return np.cross(m, b_body)


class Camera:
    """撮像: payload_power=True かつトリガ立ち上がりで 1 枚撮像成功。"""

    def __init__(self):
        self.images_stored = 0
        self._prev_trigger = False

    def step(self, camera_trigger: bool, payload_power: bool) -> bool:
        """撮像成功したか（この step で+1 したか）を返す。"""
        rising_edge = camera_trigger and not self._prev_trigger
        self._prev_trigger = camera_trigger
        success = False
        if rising_edge and payload_power and self.images_stored < MAX_STORED_IMAGES:
            self.images_stored += 1
            success = True
        return success


class Transmitter:
    """S 帯送信機: gs_visible かつ tx_active の間だけ伝送が進む。"""

    def __init__(self):
        self.images_downlinked = 0
        self._mb_progress = 0.0

    def step(self, tx_active: bool, gs_visible: bool, dt: float,
             downlink_rate_mbps: float, images_available: int) -> None:
        if not (tx_active and gs_visible):
            return
        if self.images_downlinked >= images_available:
            return
        # rate は Mbit/s -> MB/s は /8
        mb_per_s = downlink_rate_mbps / 8.0
        self._mb_progress += mb_per_s * dt
        while self._mb_progress >= IMAGE_SIZE_MB and self.images_downlinked < images_available:
            self._mb_progress -= IMAGE_SIZE_MB
            self.images_downlinked += 1
