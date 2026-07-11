#!/usr/bin/env python
"""MOMO-SAT シナリオランナー。

使い方:
    .venv/bin/python run_sim.py S1
    .venv/bin/python run_sim.py S2
    .venv/bin/python run_sim.py S3

FSW（fsw/main.py の FlightSoftware）はここで遅延 import する。
fsw/ が未実装／エラーの場合はメッセージを出して終了する
（sim パッケージ単体は tests/test_sim_dynamics.py で検証可能）。
"""

from __future__ import annotations

import sys
import time

from sim.scenarios import SCENARIOS
from sim.simulator import Simulator


def _flatten_checks(check_result):
    if isinstance(check_result, dict):
        return [check_result]
    return list(check_result)


def main(argv: list[str]) -> int:
    if len(argv) != 2 or argv[1] not in SCENARIOS:
        print(f"usage: run_sim.py {{{'|'.join(SCENARIOS.keys())}}}")
        return 2

    scenario_name = argv[1]
    cfg = SCENARIOS[scenario_name]()

    try:
        from fsw.main import FlightSoftware
    except Exception as exc:  # noqa: BLE001
        print(f"[run_sim] fsw.main.FlightSoftware の import に失敗しました: {exc}")
        print("[run_sim] fsw/ が未実装の可能性があります。sim 単体テストは "
              "tests/test_sim_dynamics.py を参照してください。")
        return 1

    fsw_config = cfg.fsw_config
    try:
        fsw = FlightSoftware(fsw_config) if fsw_config is not None else FlightSoftware()
    except Exception as exc:  # noqa: BLE001
        print(f"[run_sim] FlightSoftware() の初期化に失敗しました: {exc}")
        return 1

    sim = Simulator(cfg, fsw=fsw)

    print(f"[run_sim] シナリオ {scenario_name} 実行開始 "
          f"(duration={cfg.duration_s:.0f}s, dt={sim.dt}s, steps={int(cfg.duration_s / sim.dt)})")
    t0 = time.time()
    sim.run()
    elapsed = time.time() - t0
    print(f"[run_sim] 実行完了 ({elapsed:.1f}s wall)")

    out_path = f"out/{scenario_name}.csv"
    sim.write_csv(out_path)
    print(f"[run_sim] ログ出力: {out_path} ({len(sim.log_rows)} 行)")

    if cfg.check_fn is None:
        print("[run_sim] 判定関数未定義。PASS/FAIL 判定なし。")
        return 0

    checks = _flatten_checks(cfg.check_fn(sim))
    all_passed = True
    print(f"\n[run_sim] === {scenario_name} 判定結果 ===")
    for c in checks:
        status = "PASS" if c["passed"] else "FAIL"
        if not c["passed"]:
            all_passed = False
        print(f"  [{status}] {c['name']}: {c['detail']}")

    print(f"\n[run_sim] 総合判定: {'PASS' if all_passed else 'FAIL'}")
    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
