"""SimBridge: Simulator + FlightSoftware を実時間駆動するブリッジ。

GSI サーバ（gsi/server.py）から呼び出される中核ロジック。SCC からの
time/link/cmd メッセージをシミュレータへ反映し、tlm dict を生成する。
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np

from common.types import FSW_DT, GroundCommand, Mode, Telemetry
from fsw.main import FlightSoftware
from gsi.mapping import telemetry_to_dict
from sim.scenarios._orbit import (
    SSO_A_M,
    SSO_E,
    SSO_I_DEG,
    SSO_NU0_DEG,
    SSO_RAAN_DEG,
)
from sim.simulator import ScenarioConfig, Simulator

# 統合試験は長時間の連続運用を想定するため、duration_s は名目上「無限相当」の
# 大きな値としておく（Simulator.run() は本ブリッジでは使わず、step_to() で
# 独自にループを回すため duration_s 自体は消費されない）。
_DURATION_NOMINAL_S = 1.0e12


def build_default_scenario() -> ScenarioConfig:
    """GSI 用の既定シナリオ（SSO 500km、タイムライン空、判定関数なし）を作る。"""
    return ScenarioConfig(
        name="GSI",
        duration_s=_DURATION_NOMINAL_S,
        a_m=SSO_A_M,
        e=SSO_E,
        i_deg=SSO_I_DEG,
        raan_deg=SSO_RAAN_DEG,
        nu0_deg=SSO_NU0_DEG,
        q0=np.array([0.0, 0.0, 0.0, 1.0]),
        omega0_rad_s=np.zeros(3),
        timeline=[],
        soc_init=1.0,
        battery_temp_init_c=10.0,
        seed=42,
        fsw_config=None,
        check_fn=None,
    )


class SimBridge:
    """Simulator + FlightSoftware を保持し、実時間ペーシングで歩進させる。

    スレッド安全性: step_to() はペーシングスレッドからのみ呼ばれる想定。
    set_link()/inject_command()/get_tlm_dict() は受信スレッドから呼ばれうる
    ため、内部状態へのアクセスは _lock で保護する。
    """

    def __init__(self, scenario: ScenarioConfig | None = None) -> None:
        self.cfg = scenario if scenario is not None else build_default_scenario()
        self.fsw = FlightSoftware(self.cfg.fsw_config) if self.cfg.fsw_config else FlightSoftware()
        self.sim = Simulator(self.cfg, fsw=self.fsw)
        self.dt = FSW_DT

        self._lock = threading.RLock()
        self._last_tlm: Telemetry = self.fsw.get_telemetry()
        self._last_solar_power = 0.0
        self._last_mode: Mode = self._last_tlm.mode

    # -- 歩進 -------------------------------------------------------------
    def step_once(self) -> None:
        """dt=FSW_DT 1 ステップぶん進める（measure→fsw.step→apply→propagate）。"""
        with self._lock:
            sensors = self.sim.measure()
            cmd = self.fsw.step(sensors, self.dt)
            self.sim.apply(cmd)
            self.sim.propagate(self.dt)
            self._last_solar_power = float(self.sim._cached_solar_w)
            self._last_tlm = self.fsw.get_telemetry()

    def step_to(self, sim_target_t: float, max_steps: int | None = None) -> int:
        """現在の sim.t が sim_target_t に達するまで dt 刻みで歩進する。

        max_steps が指定された場合はその回数までしか進めない（ペーシング
        ループが「1 実秒あたり最大 N ステップ」を守るため）。
        戻り値は実際に実行したステップ数。
        """
        steps_done = 0
        # 浮動小数点誤差の蓄積を避けるため半ステップ分のマージンで比較する。
        while self.sim.t < sim_target_t - self.dt * 0.5:
            if max_steps is not None and steps_done >= max_steps:
                break
            self.step_once()
            steps_done += 1
        return steps_done

    # -- 外部駆動 -----------------------------------------------------------
    def set_link(self, up: bool) -> None:
        """SCC からのリンク状態で gs_visible を外部駆動する（docs/06 §3-3）。"""
        with self._lock:
            self.sim.gs_visible_override = bool(up)

    def inject_command(self, cmd: GroundCommand) -> None:
        """EXEC 到達済みコマンドを FSW へ投入する（可視判定は SCC 側が済ませている）。"""
        with self._lock:
            self.fsw.handle_command(cmd)

    # -- テレメトリ取得 -------------------------------------------------------
    def get_tlm_dict(self) -> dict[str, Any]:
        with self._lock:
            # _cached_gs_visible は measure() 初回実行まで存在しないため、
            # 未ステップ時は _gs_visible()（override 反映済み）へフォールバック
            gs_visible = bool(getattr(self.sim, "_cached_gs_visible",
                                      self.sim._gs_visible()))
            return telemetry_to_dict(self._last_tlm, gs_visible, self._last_solar_power)

    def get_mode(self) -> Mode:
        with self._lock:
            return self._last_tlm.mode

    def get_sim_t(self) -> float:
        with self._lock:
            return self.sim.t
