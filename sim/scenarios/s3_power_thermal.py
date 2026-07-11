"""S3: 電力・熱収支検証シナリオ。

SUNPOINT 維持で 5 軌道（28500s）実行。
判定: min SoC >= 0.8 * 初期 SoC（EP-3 相当）、バッテリ温度 0〜45 degC（TH-1）。
"""

from __future__ import annotations

import numpy as np

from common.types import GroundCommand, Mode
from sim.ground import TimelineEntry
from sim.simulator import ScenarioConfig
from sim.scenarios._orbit import SSO_A_M, SSO_E, SSO_I_DEG, SSO_NU0_DEG, SSO_RAAN_DEG

DURATION_S = 28500.0
SOC_INIT = 1.0


def check(sim) -> dict:
    results = []

    if sim.log_rows:
        min_soc = min(r["soc"] for r in sim.log_rows)
        min_temp = min(r["batt_temp"] for r in sim.log_rows)
        max_temp = max(r["batt_temp"] for r in sim.log_rows)
    else:
        min_soc = float("nan")
        min_temp = float("nan")
        max_temp = float("nan")

    results.append({
        "name": "EP-3 min SoC>=0.8*init",
        "passed": min_soc >= 0.8 * SOC_INIT,
        "detail": f"min SoC = {min_soc:.4f} (required >= {0.8 * SOC_INIT:.4f})",
    })
    results.append({
        "name": "TH-1 battery temp in [0,45]degC",
        "passed": (min_temp >= 0.0) and (max_temp <= 45.0),
        "detail": f"batt_temp range = [{min_temp:.2f}, {max_temp:.2f}] degC",
    })

    return results


def build_scenario() -> ScenarioConfig:
    omega0 = np.radians(np.array([0.1, 0.1, 0.1]))
    timeline = [
        TimelineEntry(
            t=10.0,
            command=GroundCommand(type="SET_MODE", params={"mode": Mode.SUNPOINT}),
            force_uplink=True,
        ),
    ]
    return ScenarioConfig(
        name="S3",
        duration_s=DURATION_S,
        a_m=SSO_A_M, e=SSO_E, i_deg=SSO_I_DEG, raan_deg=SSO_RAAN_DEG, nu0_deg=SSO_NU0_DEG,
        q0=np.array([0.0, 0.0, 0.0, 1.0]),
        omega0_rad_s=omega0,
        timeline=timeline,
        soc_init=SOC_INIT,
        battery_temp_init_c=10.0,
        seed=3,
        check_fn=check,
    )
