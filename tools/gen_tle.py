#!/usr/bin/env python
"""MOMO-SAT の SSO 軌道（sim/scenarios/_orbit.py）に対応する 2 行 TLE を生成する。

使い方:
    .venv/bin/python tools/gen_tle.py [--epoch 2026-07-11T00:00:00Z] > momo.tle

出力（stdout）:
    MOMO-SAT
    1 99999U 26001A   ...（NORAD 番号・国際符号は架空値）
    2 99999  97.4000 196.0000 0000000   0.0000   0.0000 15.24308387    01

TLE のフィールド定義・チェックサムアルゴリズムは NORAD 標準の two-line
element set 形式に従う（列位置は 1-indexed、チェックサムは各行末尾 1 桁で
「数字の総和 + '-' を 1 として mod 10」）。
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from common.types import MU_EARTH
from sim.scenarios._orbit import SSO_A_M, SSO_E, SSO_I_DEG, SSO_RAAN_DEG

SAT_NAME = "MOMO-SAT"
NORAD_NUMBER = "99999"
CLASSIFICATION = "U"
INTL_DESIGNATOR = "26001A  "  # 8 文字（年2桁+打上げ番号3桁+片3桁、空白パディング）
ELEMENT_SET_NUMBER = 1
REV_NUMBER = 1


def mean_motion_rev_per_day(a_m: float) -> float:
    """平均運動 n = sqrt(mu/a^3) を rev/day に換算する。"""
    n_rad_s = np.sqrt(MU_EARTH / a_m ** 3)
    return n_rad_s * 86400.0 / (2.0 * np.pi)


def epoch_year_and_day(epoch: dt.datetime) -> tuple[int, float]:
    """epoch (UTC, aware) から TLE epoch 用の (2桁年, 通日+端数) を返す。"""
    if epoch.tzinfo is None:
        epoch = epoch.replace(tzinfo=dt.timezone.utc)
    epoch = epoch.astimezone(dt.timezone.utc)
    year2 = epoch.year % 100
    day_of_year = epoch.timetuple().tm_yday
    seconds_of_day = (
        epoch.hour * 3600.0 + epoch.minute * 60.0 + epoch.second
        + epoch.microsecond / 1.0e6
    )
    frac_day = seconds_of_day / 86400.0
    return year2, day_of_year + frac_day


def _checksum(line_without_checksum: str) -> int:
    """TLE チェックサム: 数字の総和（'-' は 1 とカウント、他の非数字は 0） mod 10。"""
    total = 0
    for ch in line_without_checksum:
        if ch.isdigit():
            total += int(ch)
        elif ch == "-":
            total += 1
    return total % 10


def _format_exp(value: float) -> str:
    """TLE の指数表記フィールド（仮定小数点、符号付き仮数4桁+符号付き指数1桁、計8文字）。

    例: 0.0 -> " 00000-0"、テ表現は NDOT2/BSTAR 用。
    """
    if value == 0.0:
        return " 00000-0"
    sign = "-" if value < 0 else " "
    value = abs(value)
    exponent = 0
    mantissa = value
    while mantissa >= 1.0:
        mantissa /= 10.0
        exponent += 1
    while mantissa < 0.1 and mantissa > 0.0:
        mantissa *= 10.0
        exponent -= 1
    mantissa_digits = int(round(mantissa * 1.0e5))
    if mantissa_digits >= 100000:
        mantissa_digits //= 10
        exponent += 1
    exp_sign = "-" if exponent <= 0 else "+"
    return f"{sign}{mantissa_digits:05d}{exp_sign}{abs(exponent)}"


def build_tle(epoch: dt.datetime, a_m: float, e: float, i_deg: float, raan_deg: float) -> tuple[str, str, str]:
    year2, day_frac = epoch_year_and_day(epoch)
    n_rev_day = mean_motion_rev_per_day(a_m)

    # --- Line 0: 衛星名 ---
    name_line = SAT_NAME

    # --- Line 1 ---
    # cols: 1 line# | 3 blank | 5 satnum | 1 class | 1 blank | 8 intl desig |
    #       1 blank | 14 epoch (2yr+12 day) | 1 blank | 10 ndot | 1 blank |
    #       8 nddot | 1 blank | 8 bstar | 1 blank | 1 ephem type | 1 blank |
    #       4 elset# | 1 checksum
    epoch_str = f"{year2:02d}{day_frac:012.8f}"
    ndot_str = " .00000000"  # 平均運動の1次微分 = 0（仮定小数点、符号1桁+小数8桁）
    nddot_str = _format_exp(0.0)
    bstar_str = _format_exp(0.0)
    ephem_type = "0"
    elset_str = f"{ELEMENT_SET_NUMBER:4d}"

    line1_body = (
        f"1 {NORAD_NUMBER}{CLASSIFICATION} {INTL_DESIGNATOR} {epoch_str} "
        f"{ndot_str} {nddot_str} {bstar_str} {ephem_type} {elset_str}"
    )
    line1 = line1_body + str(_checksum(line1_body))

    # --- Line 2 ---
    # cols: 1 line# | 1 blank | 5 satnum | 1 blank | 8 incl | 1 blank | 8 raan |
    #       1 blank | 7 ecc (no dot) | 1 blank | 8 argp | 1 blank | 8 M | 1 blank
    #       | 11 mean motion | 5 rev# | 1 checksum
    incl_str = f"{i_deg:8.4f}"
    raan_str = f"{raan_deg:8.4f}"
    ecc_str = f"{int(round(e * 1.0e7)):07d}"
    argp_str = f"{0.0:8.4f}"
    manom_str = f"{0.0:8.4f}"
    mmotion_str = f"{n_rev_day:11.8f}"
    revnum_str = f"{REV_NUMBER:5d}"

    line2_body = (
        f"2 {NORAD_NUMBER} {incl_str} {raan_str} {ecc_str} {argp_str} "
        f"{manom_str} {mmotion_str}{revnum_str}"
    )
    line2 = line2_body + str(_checksum(line2_body))

    return name_line, line1, line2


def verify_tle(line1: str, line2: str) -> None:
    """形式チェック: 各行 69 文字、チェックサム再計算が一致すること。"""
    for line in (line1, line2):
        assert len(line) == 69, f"TLE line length must be 69, got {len(line)}: {line!r}"
        body, cksum = line[:-1], line[-1]
        expected = _checksum(body)
        assert int(cksum) == expected, f"checksum mismatch: line={line!r} expected={expected} got={cksum}"
    assert line1[0] == "1" and line2[0] == "2"
    assert line1[2:7] == line2[2:7], "satellite number must match between lines"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MOMO-SAT SSO 軌道に対応する TLE を生成する")
    parser.add_argument("--epoch", type=str, default="2026-07-11T00:00:00Z",
                         help="TLE エポック (ISO8601 UTC、既定 2026-07-11T00:00:00Z)")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    epoch_str = args.epoch.replace("Z", "+00:00")
    epoch = dt.datetime.fromisoformat(epoch_str)
    if epoch.tzinfo is None:
        epoch = epoch.replace(tzinfo=dt.timezone.utc)

    name_line, line1, line2 = build_tle(epoch, SSO_A_M, SSO_E, SSO_I_DEG, SSO_RAAN_DEG)
    verify_tle(line1, line2)

    print(name_line)
    print(line1)
    print(line2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
