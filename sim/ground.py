"""地上局モデル: 東京局（35.68N, 139.77E）可視判定、コマンドタイムライン注入。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from common.types import GroundCommand, OMEGA_EARTH, R_EARTH

GS_LAT_DEG = 35.68
GS_LON_DEG = 139.77
GS_MIN_ELEVATION_DEG = 10.0


def ground_station_pos_eci(t: float) -> np.ndarray:
    """東京局の ECI 位置 [m]（地球自転: theta = OMEGA_EARTH * t、t=0 で経度 0 が X 軸）。"""
    lat = np.radians(GS_LAT_DEG)
    lon = np.radians(GS_LON_DEG)
    theta = OMEGA_EARTH * t
    lon_eci = lon + theta
    x = R_EARTH * np.cos(lat) * np.cos(lon_eci)
    y = R_EARTH * np.cos(lat) * np.sin(lon_eci)
    z = R_EARTH * np.sin(lat)
    return np.array([x, y, z])


def elevation_deg(sat_pos_eci: np.ndarray, t: float) -> float:
    """地上局から見た衛星の仰角 [deg]。"""
    gs_pos = ground_station_pos_eci(t)
    gs_up = gs_pos / np.linalg.norm(gs_pos)
    los = sat_pos_eci - gs_pos
    los_norm = np.linalg.norm(los)
    if los_norm < 1e-6:
        return 90.0
    los_hat = los / los_norm
    sin_el = float(np.dot(los_hat, gs_up))
    return float(np.degrees(np.arcsin(np.clip(sin_el, -1.0, 1.0))))


def is_visible(sat_pos_eci: np.ndarray, t: float) -> bool:
    return elevation_deg(sat_pos_eci, t) >= GS_MIN_ELEVATION_DEG


@dataclass
class TimelineEntry:
    t: float
    command: GroundCommand
    force_uplink: bool = False


@dataclass
class GroundStation:
    """コマンドタイムラインを保持し、可視条件（または force_uplink）に応じて注入する。"""

    timeline: list[TimelineEntry] = field(default_factory=list)
    _injected: list[bool] = field(default_factory=list, repr=False)

    def __post_init__(self):
        self._injected = [False] * len(self.timeline)

    def poll(self, t: float, gs_visible: bool) -> list[GroundCommand]:
        """t 到達済みかつ（可視 or force_uplink）で未注入のコマンドをまとめて返す。"""
        ready: list[GroundCommand] = []
        for i, entry in enumerate(self.timeline):
            if self._injected[i]:
                continue
            if t < entry.t:
                continue
            if entry.force_uplink or gs_visible:
                ready.append(entry.command)
                self._injected[i] = True
        return ready
