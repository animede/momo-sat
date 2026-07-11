"""S2: 撮像パス検証シナリオ。

SUNPOINT で開始（t=10 に SET_MODE SUNPOINT を注入）、t=3000 に IMAGE_NADIR
count=4 を注入、t=6000 に DOWNLINK（force_uplink）を注入。
東京局の最初の可視パス（t≈11690–12120s）を含む 13000 s 実行。

判定（AD-2/AD-3 は撮像イベント時点の真値で評価する。直下追尾では軌道レート
~0.064 deg/s の機体回転が正常のため、レートは対目標追尾誤差で判定）:
  - images_stored >= 4（MR-2）
  - 各撮像時点の真の直下指向誤差 < 0.5 deg（AD-2）
  - 各撮像時点の真の対目標レート誤差 < 0.05 deg/s（AD-3）
  - images_downlinked >= 1
"""

from __future__ import annotations

import numpy as np

from common.types import GroundCommand, Mode
from sim.ground import TimelineEntry
from sim.simulator import ScenarioConfig
from sim.scenarios._orbit import SSO_A_M, SSO_E, SSO_I_DEG, SSO_NU0_DEG, SSO_RAAN_DEG

DURATION_S = 13000.0  # 東京局の最初の可視パス（~11690-12120s）を含める


def check(sim) -> dict:
    results = []

    images_stored = sim.camera.images_stored
    results.append({
        "name": "MR-2 images_stored>=4",
        "passed": images_stored >= 4,
        "detail": f"images_stored = {images_stored}",
    })

    # AD-2/AD-3: 撮像イベント時点の真値で評価（sim.capture_events、10 Hz 記録）
    events = sim.capture_events
    if events:
        max_nadir_err = max(e["nadir_err_deg"] for e in events)
        max_rate_err = max(e["rate_err_deg_s"] for e in events)
        times = ", ".join(f"{e['t']:.0f}s" for e in events)
    else:
        max_nadir_err = float("nan")
        max_rate_err = float("nan")
        times = "none"

    results.append({
        "name": "AD-2 true nadir err<0.5deg at capture",
        "passed": bool(events) and max_nadir_err < 0.5,
        "detail": f"max true nadir err at capture = {max_nadir_err:.4f} deg "
                  f"({len(events)} captures at {times})",
    })
    results.append({
        "name": "AD-3 true tracking-rate err<0.05deg/s at capture",
        "passed": bool(events) and max_rate_err < 0.05,
        "detail": f"max true rate err at capture = {max_rate_err:.4f} deg/s",
    })

    images_downlinked = sim.transmitter.images_downlinked
    results.append({
        "name": "images_downlinked>=1",
        "passed": images_downlinked >= 1,
        "detail": f"images_downlinked = {images_downlinked}",
    })

    return results


def build_scenario() -> ScenarioConfig:
    omega0 = np.radians(np.array([0.12, 0.12, 0.12]))  # ||omega0|| ~= 0.2 deg/s

    timeline = [
        TimelineEntry(
            t=10.0,
            command=GroundCommand(type="SET_MODE", params={"mode": Mode.SUNPOINT}),
            force_uplink=True,
        ),
        TimelineEntry(
            t=3000.0,
            command=GroundCommand(type="IMAGE_NADIR", params={"count": 4}),
            force_uplink=True,
        ),
        TimelineEntry(
            t=6000.0,
            command=GroundCommand(type="DOWNLINK", params={}),
            force_uplink=True,
        ),
    ]
    return ScenarioConfig(
        name="S2",
        duration_s=DURATION_S,
        a_m=SSO_A_M, e=SSO_E, i_deg=SSO_I_DEG, raan_deg=SSO_RAAN_DEG, nu0_deg=SSO_NU0_DEG,
        q0=np.array([0.0, 0.0, 0.0, 1.0]),
        omega0_rad_s=omega0,
        timeline=timeline,
        soc_init=1.0,
        battery_temp_init_c=10.0,
        seed=2,
        check_fn=check,
    )
