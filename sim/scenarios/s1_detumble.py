"""S1: デタンブル検証シナリオ。

初期 omega = [6,5,6] deg/s（合計 ~10 deg/s）、t=0 で SET_MODE DETUMBLE を
force_uplink 注入、2 軌道（11400s）実行。

判定（AD-4「2 軌道以内に 0.5 deg/s 以下へデタンブリングできること」）:
|omega| <= 0.5 deg/s が 60 s 連続する時刻が 2 軌道以内に存在すること。
（デタンブル完了後は SUNPOINT へ自律遷移し、姿勢変更スルーで一時的に
レートが上がるのは正常動作のため、最終瞬時値では判定しない）
"""

from __future__ import annotations

import numpy as np

from common.types import GroundCommand, Mode
from sim.ground import TimelineEntry
from sim.simulator import ScenarioConfig
from sim.scenarios._orbit import SSO_A_M, SSO_E, SSO_I_DEG, SSO_NU0_DEG, SSO_RAAN_DEG

DURATION_S = 11400.0


def check(sim) -> dict:
    # 真値レート履歴（1 Hz ログ）から |omega|<=0.5 deg/s が 60 s 連続した
    # 最初の時刻を求める
    achieved_t = None
    consecutive = 0
    for row in sim.log_rows:
        rate_deg_s = np.degrees(np.linalg.norm(
            [row["wx_true"], row["wy_true"], row["wz_true"]]
        ))
        if rate_deg_s <= 0.5:
            consecutive += 1
            if consecutive >= 60 and achieved_t is None:
                achieved_t = row["t"]
        else:
            consecutive = 0
    passed = achieved_t is not None and achieved_t <= DURATION_S
    detail = (
        f"detumble achieved at t = {achieved_t:.0f} s (<= {DURATION_S:.0f} s = 2 orbits)"
        if achieved_t is not None
        else "never sustained |omega| <= 0.5 deg/s for 60 s"
    )
    return {
        "name": "AD-4 detumble within 2 orbits",
        "passed": passed,
        "detail": detail,
    }


def build_scenario() -> ScenarioConfig:
    omega0 = np.radians(np.array([6.0, 5.0, 6.0]))
    timeline = [
        TimelineEntry(
            t=0.0,
            command=GroundCommand(type="SET_MODE", params={"mode": Mode.DETUMBLE}),
            force_uplink=True,
        ),
    ]
    return ScenarioConfig(
        name="S1",
        duration_s=DURATION_S,
        a_m=SSO_A_M, e=SSO_E, i_deg=SSO_I_DEG, raan_deg=SSO_RAAN_DEG, nu0_deg=SSO_NU0_DEG,
        q0=np.array([0.0, 0.0, 0.0, 1.0]),
        omega0_rad_s=omega0,
        timeline=timeline,
        soc_init=1.0,
        battery_temp_init_c=10.0,
        seed=1,
        check_fn=check,
    )
