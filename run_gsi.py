#!/usr/bin/env python
"""MOMO-SAT GSI（Ground-Satellite Interface）サーバ起動スクリプト。

使い方:
    .venv/bin/python run_gsi.py [--port 9200] [--max-steps-per-sec 2000] [--verbose]

SCC（衛星管制システム）の extsat アダプタが TCP クライアントとして接続し、
hello/time/link/cmd を送ってくる。詳細プロトコル: docs/06_scc_integration.md
"""

from __future__ import annotations

import argparse
import sys

from gsi.bridge import SimBridge, build_default_scenario
from gsi.server import DEFAULT_MAX_STEPS_PER_SEC, DEFAULT_PORT, GsiServer
from sim.scenarios._orbit import (
    SSO_A_M,
    SSO_ALTITUDE_M,
    SSO_E,
    SSO_I_DEG,
    SSO_RAAN_DEG,
)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MOMO-SAT GSI サーバ")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                         help=f"待ち受け TCP ポート（既定 {DEFAULT_PORT}）")
    parser.add_argument("--max-steps-per-sec", type=int, default=DEFAULT_MAX_STEPS_PER_SEC,
                         help=f"実 1 秒あたりの最大シミュレーションステップ数（既定 {DEFAULT_MAX_STEPS_PER_SEC}）")
    parser.add_argument("--verbose", action="store_true", help="デバッグログを有効化")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    scenario = build_default_scenario()
    bridge = SimBridge(scenario)
    server = GsiServer(
        port=args.port,
        max_steps_per_sec=args.max_steps_per_sec,
        verbose=args.verbose,
        bridge=bridge,
    )

    print("[run_gsi] MOMO-SAT GSI サーバ起動")
    print(f"[run_gsi]   待受ポート          : {args.port}")
    print(f"[run_gsi]   max_steps_per_sec  : {args.max_steps_per_sec} "
          f"(dt={bridge.dt}s, 実時間換算 最大 x{args.max_steps_per_sec * bridge.dt:.0f})")
    print("[run_gsi]   軌道諸元 (SSO)")
    print(f"[run_gsi]     高度            : {SSO_ALTITUDE_M / 1000.0:.1f} km (a={SSO_A_M/1000.0:.1f} km)")
    print(f"[run_gsi]     離心率 e        : {SSO_E}")
    print(f"[run_gsi]     傾斜角 i        : {SSO_I_DEG} deg")
    print(f"[run_gsi]     RAAN            : {SSO_RAAN_DEG} deg")
    print("[run_gsi] time メッセージ受信まで待機します（Ctrl-C で終了）")

    server.start()
    try:
        server.wait_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[run_gsi] 終了処理中...")
        server.stop()
        print("[run_gsi] 終了しました")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
