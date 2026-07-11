"""電力モデル（ICD §4）: 太陽電池 5 面 cos 則、負荷集計、バッテリ SoC 積分。"""

from __future__ import annotations

import numpy as np

from common.types import BATTERY_CAPACITY_WH, THRUST_POWER

# 太陽電池面法線（body 系、単位ベクトル）: +-X, +-Y, -Z の 5 面（+Z は開口面のため無し）
SOLAR_PANEL_NORMALS = [
    np.array([1.0, 0.0, 0.0]),
    np.array([-1.0, 0.0, 0.0]),
    np.array([0.0, 1.0, 0.0]),
    np.array([0.0, -1.0, 0.0]),
    np.array([0.0, 0.0, -1.0]),
]

# 1 面あたりの直射時最大発生電力 [W]（5 面合計ピーク 4.6W、docs/02 準拠のためスケール）
PANEL_PEAK_POWER_W = 4.6 / 5.0

# 負荷 [W]
LOAD_BUS_W = 0.6          # EPS + OBC + RX 常時
LOAD_ADCS_W = 0.5         # ADCS（SAFE 以外）
LOAD_HEATER_W = 1.0
LOAD_PAYLOAD_W = 1.5      # 撮像中
LOAD_TX_W = 2.0           # tx_active かつ gs_visible

CHARGE_EFFICIENCY = 0.9
BATTERY_V_MIN = 6.0
BATTERY_V_SLOPE = 2.4  # V per SoC=1.0


def solar_power_generated(r_body: np.ndarray, sun_dir_body: np.ndarray | None,
                           eclipse: bool) -> float:
    """5 面太陽電池の発生電力 [W]（cos 則、負値クリップ）。"""
    if eclipse or sun_dir_body is None:
        return 0.0
    total = 0.0
    for n_hat in SOLAR_PANEL_NORMALS:
        cos_theta = float(np.dot(n_hat, sun_dir_body))
        if cos_theta > 0.0:
            total += PANEL_PEAK_POWER_W * cos_theta
    return total


def total_load(mode_is_safe: bool, heater_on: bool, payload_power: bool,
                tx_active: bool, gs_visible: bool, thruster_on: bool = False) -> float:
    """負荷集計 [W]。

    thruster_on: 電気推進 ON（docs/08 §2: 噴射中のみ +THRUST_POWER）。
    デフォルト False で既存呼び出し元の挙動は不変。
    """
    load = LOAD_BUS_W
    if not mode_is_safe:
        load += LOAD_ADCS_W
    if heater_on:
        load += LOAD_HEATER_W
    if payload_power:
        load += LOAD_PAYLOAD_W
    if tx_active and gs_visible:
        load += LOAD_TX_W
    if thruster_on:
        load += THRUST_POWER
    return load


class Battery:
    """バッテリ SoC 積分（充放電効率込み）、電圧モデル。"""

    def __init__(self, soc_init: float = 1.0):
        self.soc = soc_init

    def step(self, generated_w: float, load_w: float, dt: float) -> None:
        net_w = generated_w - load_w
        if net_w >= 0.0:
            energy_wh = net_w * CHARGE_EFFICIENCY * (dt / 3600.0)
        else:
            energy_wh = net_w / CHARGE_EFFICIENCY * (dt / 3600.0)
        self.soc += energy_wh / BATTERY_CAPACITY_WH
        self.soc = float(np.clip(self.soc, 0.0, 1.0))

    @property
    def voltage(self) -> float:
        return BATTERY_V_MIN + BATTERY_V_SLOPE * self.soc
