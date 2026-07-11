#!/usr/bin/env python3
"""MOMO-SAT 超小型イオンエンジン（電気推進）搭載成立性 数値検証。

このスクリプトは MOMO-SAT の既存モジュール
  - common/types.py (MU_EARTH, R_EARTH, BATTERY_CAPACITY_WH, RW_MAX_TORQUE,
                      RW_MAX_MOMENTUM, INERTIA)
を「参照のみ」で使用し、既存ファイルは一切変更しない。

想定スラスタは特定製品ではなく「パラメータ空間」（F, Isp, P の組合せ）で
評価する。物理的に整合しない組合せ（効率 eta = F*ve/(2P) > 70%）は除外する。

検証項目:
  検証1: ミッション Delta-v 需要（高度維持・軌道遷移・デオービット）
  検証2: 電力・デューティ成立性（1Uの核心制約）
  検証3: 質量・体積・推進剤収支
  検証4: 搭載環境適合（幾何配置・磁気・トルク外乱）

実行: .venv/bin/python tools/study_ion_engine_budget.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.types import (  # noqa: E402
    MU_EARTH,
    R_EARTH,
    BATTERY_CAPACITY_WH,
    RW_MAX_TORQUE,
    RW_MAX_MOMENTUM,
    INERTIA,
)

G0 = 9.80665  # 標準重力加速度 [m/s^2]

# ---------------------------------------------------------------------------
# 共通パラメータ（既存設計値、docs/02, docs/07, docs/subsystems/*.md 準拠）
# ---------------------------------------------------------------------------

MASS_NOW_KG = 1.250          # docs/02 現設計質量
MASS_MAX_KG = 1.330          # ME-4 上限
MASS_MARGIN_G = (MASS_MAX_KG - MASS_NOW_KG) * 1000.0  # 80 g

ALT_KEEP_M = 500.0e3         # 高度維持基準高度
ALT_TRANSFER_M = 550.0e3     # 軌道遷移先高度（例）
ALT_DEORBIT_M = 300.0e3      # デオービット目標高度

AREA_DRAG_M2 = 0.01          # 1U 断面積 [m^2]（study_mtq_orbit_force.py と同一）
CD_DRAG = 2.2                # 抗力係数（同上）

RHO_LOW = 1.0e-13            # kg/m^3 @500km 太陽活動低（mtq study 前提と整合）
RHO_HIGH = 1.0e-12           # kg/m^3 @500km 太陽活動高（10倍、太陽活動幅の代表値）

# 電力収支（docs/02, docs/subsystems/eps.md 準拠）
P_ORBIT_AVG_W = 2.0          # EP-1: 太陽電池発生電力 軌道平均
P_BUS_ALWAYS_W = 1.2         # 常時バス負荷（軌道平均消費 ≈1.86W から抜粋、
                              # 設問指定の "常時バス負荷 ~1.2W" を採用）
BATTERY_WH = BATTERY_CAPACITY_WH   # 25.0 Wh
SOC_SAFE_THRESHOLD = 0.30    # EP-4
SOC_FLOOR_FOR_EP = 0.50      # 検証2の制約: バッテリを SoC50%以上に保つ
SUNLIT_FRACTION = 0.622      # eps.md 3.3節 日照率

# 質量特性（mechanical_design.md 準拠）
CG_Z_OFFSET_M = 17.8e-3      # CG -Z オフセット 17.8mm（設問指定値）
RW_LEVER_ARM_M = 5.0e-3      # スラスタ推力ベクトル-CGレバーアーム（設問指定）

# パラメータ空間
F_LIST_N = [10e-6, 100e-6, 330e-6, 1.0e-3]        # 推力 [N]
ISP_LIST_S = [800.0, 1500.0, 2500.0, 5000.0]       # 比推力 [s]
P_LIST_W = [2.0, 5.0, 10.0, 30.0]                  # 入力電力 [W]

ETA_MAX = 0.70                # 効率上限（物理的整合性フィルタ）

F_DRAG_500KM_N = 6.4e-8       # 高度維持前提（study summary 記載値、rho=1e-13）


# ---------------------------------------------------------------------------
# ユーティリティ
# ---------------------------------------------------------------------------

def thruster_efficiency(f_n: float, isp_s: float, p_w: float) -> float:
    """eta = F * ve / (2P), ve = Isp * g0."""
    ve = isp_s * G0
    return f_n * ve / (2.0 * p_w)


def valid_combinations():
    """(F, Isp, P, eta) の物理的に整合するタプルを返す（eta<=70%）。"""
    combos = []
    for f_n in F_LIST_N:
        for isp_s in ISP_LIST_S:
            for p_w in P_LIST_W:
                eta = thruster_efficiency(f_n, isp_s, p_w)
                if 0.0 < eta <= ETA_MAX:
                    combos.append((f_n, isp_s, p_w, eta))
    return combos


def orbital_period(a_m: float) -> float:
    return 2.0 * np.pi * np.sqrt(a_m ** 3 / MU_EARTH)


def circular_velocity(a_m: float) -> float:
    return np.sqrt(MU_EARTH / a_m)


def atmospheric_drag_force(rho: float, v_mps: float) -> float:
    return 0.5 * rho * v_mps ** 2 * CD_DRAG * AREA_DRAG_M2


def hohmann_delta_v(r1_m: float, r2_m: float) -> float:
    """r1(円軌道) -> r2(円軌道) ホーマン遷移の合計 Delta-v [m/s]。"""
    v1 = np.sqrt(MU_EARTH / r1_m)
    v2 = np.sqrt(MU_EARTH / r2_m)
    a_t = 0.5 * (r1_m + r2_m)
    v_p = np.sqrt(MU_EARTH * (2.0 / r1_m - 1.0 / a_t))
    v_a = np.sqrt(MU_EARTH * (2.0 / r2_m - 1.0 / a_t))
    dv1 = abs(v_p - v1)
    dv2 = abs(v2 - v_a)
    return dv1 + dv2


def rocket_equation_prop_mass(m0_kg: float, dv_mps: float, isp_s: float) -> float:
    """ツィオルコフスキーの式から推進剤質量を計算 [kg]。"""
    ve = isp_s * G0
    m_final = m0_kg * np.exp(-dv_mps / ve)
    return m0_kg - m_final


# ---------------------------------------------------------------------------
# 検証1: ミッション Delta-v 需要
# ---------------------------------------------------------------------------

def verification1():
    print("\n" + "=" * 78)
    print("検証1: ミッション Delta-v 需要")
    print("=" * 78)

    r_keep = R_EARTH + ALT_KEEP_M
    v_keep = circular_velocity(r_keep)
    period_keep = orbital_period(r_keep)
    n_orbits_2y = 2.0 * 365.25 * 86400.0 / period_keep

    print(f"\n高度 500km: r={r_keep/1e3:.1f} km, v_circ={v_keep:.3f} m/s, "
          f"周期={period_keep/60:.2f} min, 2年間の周回数={n_orbits_2y:.1f}")

    # --- 1.1 高度維持: 大気抵抗を打ち消すために必要な Delta-v (2年分) ---
    print("\n[1.1 高度維持 Delta-v（2年分、太陽活動幅 rho=1e-13〜1e-12 kg/m^3）]")
    print(f"{'rho [kg/m^3]':>14s} {'F_drag [N]':>12s} {'a_drag [m/s^2]':>16s} "
          f"{'dv/day [m/s]':>13s} {'dv/2yr [m/s]':>13s}")
    keep_results = {}
    for rho in (RHO_LOW, RHO_HIGH):
        f_drag = atmospheric_drag_force(rho, v_keep)
        a_drag = f_drag / MASS_NOW_KG
        dv_per_day = a_drag * 86400.0
        dv_2yr = a_drag * (2.0 * 365.25 * 86400.0)
        keep_results[rho] = {"f_drag": f_drag, "a_drag": a_drag,
                              "dv_per_day": dv_per_day, "dv_2yr": dv_2yr}
        print(f"{rho:>14.2e} {f_drag:>12.4e} {a_drag:>16.4e} "
              f"{dv_per_day:>13.4e} {dv_2yr:>13.2f}")

    # 参照値との整合確認（mtq_orbit_raising_summary.md: rho=1e-13 で F=6.4e-8N）
    f_ref = atmospheric_drag_force(RHO_LOW, v_keep)
    print(f"\n[整合確認] rho=1e-13 での本計算 F_drag = {f_ref:.3e} N "
          f"(docs/studies 前提値 6.4e-8 N との比 = {f_ref/F_DRAG_500KM_N:.3f})")

    # --- 1.2 軌道遷移例: 500->550km ホーマン + 離心率補正 ---
    print("\n[1.2 軌道遷移: 500km -> 550km ホーマン相当 Delta-v]")
    r1 = R_EARTH + ALT_KEEP_M
    r2 = R_EARTH + ALT_TRANSFER_M
    dv_hohmann = hohmann_delta_v(r1, r2)
    print(f"  r1={r1/1e3:.1f}km, r2={r2/1e3:.1f}km, Hohmann dv = {dv_hohmann:.3f} m/s")

    # 離心率補正: 遷移完了後、軌道が円からわずかに外れる場合の補正 Delta-v
    # 保守的に遷移 dv の 10% を circularization/eccentricity trim 余裕として計上
    dv_ecc_trim = 0.10 * dv_hohmann
    dv_transfer_total = dv_hohmann + dv_ecc_trim
    print(f"  離心率補正マージン(遷移dvの10%) = {dv_ecc_trim:.3f} m/s")
    print(f"  遷移合計 Delta-v = {dv_transfer_total:.3f} m/s")

    # --- 1.3 デオービット: 500km -> 300km ---
    print("\n[1.3 能動的デオービット: 500km -> 300km（25年則対応の加速）]")
    r3 = R_EARTH + ALT_DEORBIT_M
    dv_deorbit_hohmann = hohmann_delta_v(r1, r3)
    print(f"  r1={r1/1e3:.1f}km, r3={r3/1e3:.1f}km, "
          f"Hohmann式デオービット遷移 dv = {dv_deorbit_hohmann:.3f} m/s")
    print("  (300km到達後は大気抵抗が急増し自然再突入が加速するため、"
          "実際の投入dvはこの近似値がほぼ上限)")

    # --- 1.4 合計需要ケース ---
    print("\n[1.4 合計 Delta-v 需要ケース（控えめ/標準/野心的）]")
    dv_keep_low = keep_results[RHO_LOW]["dv_2yr"]
    dv_keep_high = keep_results[RHO_HIGH]["dv_2yr"]

    case_conservative = dv_keep_low  # 高度維持のみ、太陽活動低（最小需要）
    case_standard = dv_keep_high + dv_transfer_total  # 高度維持(高rho) + 1回遷移
    case_ambitious = dv_keep_high + dv_transfer_total + dv_deorbit_hohmann  # +デオービット

    print(f"  控えめ（高度維持のみ, rho=1e-13）        : {case_conservative:8.2f} m/s")
    print(f"  標準　（高度維持(rho=1e-12) + 遷移1回）  : {case_standard:8.2f} m/s")
    print(f"  野心的（標準 + 能動デオービット）        : {case_ambitious:8.2f} m/s")

    return {
        "keep_results": keep_results,
        "dv_transfer_total": dv_transfer_total,
        "dv_deorbit": dv_deorbit_hohmann,
        "cases": {
            "conservative": case_conservative,
            "standard": case_standard,
            "ambitious": case_ambitious,
        },
        "v_keep": v_keep,
        "r_keep": r_keep,
    }


# ---------------------------------------------------------------------------
# 検証2: 電力・デューティ成立性
# ---------------------------------------------------------------------------

def verification2(dv_cases: dict):
    print("\n" + "=" * 78)
    print("検証2: 電力・デューティ成立性（1U の核心制約）")
    print("=" * 78)

    print(f"\n前提: 軌道平均発電 {P_ORBIT_AVG_W} W, 常時バス負荷 {P_BUS_ALWAYS_W} W, "
          f"バッテリ {BATTERY_WH} Wh, SoC50%維持制約, 日照率 {SUNLIT_FRACTION:.3f}")

    # 軌道平均で使える余剰電力（バス負荷を差し引いた後、スラスタに回せる電力の目安）
    p_surplus_avg = P_ORBIT_AVG_W - P_BUS_ALWAYS_W
    print(f"軌道平均余剰電力（発電 - 常時バス負荷）= {p_surplus_avg:.3f} W")

    combos = valid_combinations()
    print(f"\n物理的に整合する (F,Isp,P) 組合せ数（eta<=70%）: {len(combos)} / "
          f"{len(F_LIST_N)*len(ISP_LIST_S)*len(P_LIST_W)}")

    # --- 2.1 許容デューティ比 d の計算 ---
    # モデルA: 軌道平均ベース（バッテリ非依存、単純平均電力収支）
    #   d_avg = p_surplus_avg / P_thruster  (d<=1)
    # モデルB: 日照中のみ噴射・バッテリをSoC50%以上に保つ制約
    #   日照中に得られる余剰電力 P_sunlit_surplus = 発電(日照中実効) - バス負荷
    #   食中はスラスタ停止、バス負荷のみ消費（バッテリで賄う）。
    #   1軌道当たりのエネルギー収支がゼロ以上（SoC不変・50%を维持）となる
    #   日照中デューティ d を求める。
    period_keep = orbital_period(R_EARTH + ALT_KEEP_M)
    t_orbit = period_keep
    t_eclipse = t_orbit * (1.0 - SUNLIT_FRACTION)
    t_sunlit = t_orbit * SUNLIT_FRACTION

    # 日照中の実効発電（軌道平均2.0Wは食込み平均なので、日照中実効発電に換算）
    p_sunlit_gen = P_ORBIT_AVG_W / SUNLIT_FRACTION  # 日照中のみ発電と仮定した実効値
    print(f"\n軌道周期 = {t_orbit/60:.2f} min（日照 {t_sunlit/60:.2f} min / "
          f"食 {t_eclipse/60:.2f} min）")
    print(f"日照中実効発電 P_sunlit_gen = P_avg/日照率 = {p_sunlit_gen:.3f} W")

    print("\n[2.1 スラスタ電力ごとの許容デューティ比 d（SoC50%以上維持、日照中噴射のみ）]")
    print(f"{'P [W]':>6s} {'d_avg(常時可)':>13s} {'d_sunlit(日照限定)':>18s} "
          f"{'エネルギー収支':>14s}")
    duty_by_power = {}
    for p_w in P_LIST_W:
        # モデルA: 常時（日照食問わず）一定デューティで噴射する場合、
        # 軌道平均で発電-消費がプラスを保つ上限デューティ
        d_avg = min(1.0, max(0.0, p_surplus_avg / p_w)) if p_w > 0 else 0.0

        # モデルB: 日照中のみ、食中はバス負荷のみ（バッテリから）
        # 1軌道のエネルギー収支: E_gen_sunlit - E_bus_orbit - E_thruster_sunlit >= 0
        # E_gen_sunlit = P_orbit_avg * t_orbit  (軌道平均発電はそのままエネルギー)
        # E_bus_orbit = P_BUS_ALWAYS_W * t_orbit
        # E_thruster_sunlit = p_w * d * t_sunlit
        e_gen_orbit = P_ORBIT_AVG_W * t_orbit
        e_bus_orbit = P_BUS_ALWAYS_W * t_orbit
        e_surplus_orbit = e_gen_orbit - e_bus_orbit
        if p_w * t_sunlit > 0:
            d_sunlit = min(1.0, max(0.0, e_surplus_orbit / (p_w * t_sunlit)))
        else:
            d_sunlit = 0.0

        duty_by_power[p_w] = {"d_avg": d_avg, "d_sunlit": d_sunlit}
        balance = "収支均衡上限" if d_sunlit < 1.0 else "電力に余裕あり(d=1可)"
        print(f"{p_w:>6.1f} {d_avg:>13.3f} {d_sunlit:>18.3f} {balance:>14s}")

    # --- 2.2 平均推力 F*d と大気抵抗の比較 ---
    print(f"\n[2.2 平均推力 F*d と大気抵抗の比較（大気抵抗 F_drag={F_DRAG_500KM_N:.2e} N @rho=1e-13）]")
    print(f"{'F [uN]':>8s} {'Isp [s]':>8s} {'P [W]':>6s} {'eta':>6s} "
          f"{'d_sunlit':>9s} {'F*d [N]':>10s} {'F*d/F_drag':>11s} {'判定':>8s}")
    combo_results = []
    for f_n, isp_s, p_w, eta in combos:
        d_sunlit = duty_by_power[p_w]["d_sunlit"]
        f_avg = f_n * d_sunlit
        ratio = f_avg / F_DRAG_500KM_N
        verdict = "維持可" if ratio >= 1.0 else "不足"
        combo_results.append({
            "F": f_n, "Isp": isp_s, "P": p_w, "eta": eta,
            "d_sunlit": d_sunlit, "f_avg": f_avg, "ratio": ratio, "verdict": verdict,
        })
        print(f"{f_n*1e6:>8.0f} {isp_s:>8.0f} {p_w:>6.1f} {eta:>6.3f} "
              f"{d_sunlit:>9.3f} {f_avg:>10.3e} {ratio:>11.3f} {verdict:>8s}")

    # --- 2.3 各 Delta-v ケースの達成所要期間 ---
    print("\n[2.3 各 Delta-v ケースの達成所要期間（年）: dv_needed = a_avg * t]")
    print("(a_avg = F*d / m0, m0 = 1.25 kg として概算。d は上記 d_sunlit を使用)")
    header = f"{'F [uN]':>8s} {'Isp':>6s} {'P [W]':>6s} {'d':>6s}"
    for case_name in dv_cases["cases"]:
        header += f" {case_name+'[yr]':>16s}"
    print(header)
    duration_results = []
    for c in combo_results:
        a_avg = c["f_avg"] / MASS_NOW_KG
        row = f"{c['F']*1e6:>8.0f} {c['Isp']:>6.0f} {c['P']:>6.1f} {c['d_sunlit']:>6.3f}"
        durations = {}
        for case_name, dv_need in dv_cases["cases"].items():
            if a_avg > 0:
                t_s = dv_need / a_avg
                t_yr = t_s / (365.25 * 86400.0)
            else:
                t_yr = float("inf")
            durations[case_name] = t_yr
            row += f" {t_yr:>16.2f}"
        duration_results.append({**c, "durations": durations})
        print(row)

    # --- 2.4 1Uで成立する(F,Isp,P)領域のまとめ表 ---
    print("\n[2.4 電力2W平均の1Uで成立する(F,Isp,P)領域（F*d>=F_drag かつ dv標準ケース <= 2年）]")
    print(f"{'F [uN]':>8s} {'Isp [s]':>8s} {'P [W]':>6s} {'eta':>6s} "
          f"{'高度維持':>8s} {'標準dv<=2yr':>12s} {'総合判定':>8s}")
    feasible = []
    for d in duration_results:
        keep_ok = d["ratio"] >= 1.0
        dv_ok = d["durations"]["standard"] <= 2.0
        overall = "成立" if (keep_ok and dv_ok) else "不成立"
        if keep_ok and dv_ok:
            feasible.append(d)
        print(f"{d['F']*1e6:>8.0f} {d['Isp']:>8.0f} {d['P']:>6.1f} {d['eta']:>6.3f} "
              f"{'OK' if keep_ok else 'NG':>8s} {'OK' if dv_ok else 'NG':>12s} {overall:>8s}")

    print(f"\n成立する組合せ数: {len(feasible)} / {len(duration_results)}")
    if feasible:
        print("成立組合せ一覧:")
        for f in feasible:
            print(f"  F={f['F']*1e6:.0f}uN, Isp={f['Isp']:.0f}s, P={f['P']:.1f}W, "
                  f"eta={f['eta']:.3f}, d={f['d_sunlit']:.3f}, "
                  f"標準ケース所要={f['durations']['standard']:.2f}年")

    return {"combo_results": combo_results, "duration_results": duration_results,
            "duty_by_power": duty_by_power, "feasible": feasible}


# ---------------------------------------------------------------------------
# 検証3: 質量・体積・推進剤収支
# ---------------------------------------------------------------------------

def verification3(dv_cases: dict):
    print("\n" + "=" * 78)
    print("検証3: 質量・体積・推進剤収支")
    print("=" * 78)

    print(f"\n現設計質量 {MASS_NOW_KG*1000:.0f} g, 上限 {MASS_MAX_KG*1000:.0f} g, "
          f"余裕 {MASS_MARGIN_G:.0f} g")

    # --- 3.1 スラスタ構成質量の想定 ---
    print("\n[3.1 スラスタ+PPU+タンク 構成質量の想定レンジ（一般的な超小型EP文献値レンジ）]")
    # 質量内訳は特定製品に依らず一般的な超小型スラスタのオーダーで概算する
    mass_breakdown = {
        "スラスタヘッド": (5, 30),      # g
        "PPU（電源処理ユニット）": (30, 150),  # g
        "推進剤タンク+バルブ+配管": (10, 60),  # g
    }
    total_low = sum(v[0] for v in mass_breakdown.values())
    total_high = sum(v[1] for v in mass_breakdown.values())
    for name, (lo, hi) in mass_breakdown.items():
        print(f"  {name:24s}: {lo:4d} - {hi:4d} g")
    print(f"  {'システム合計（推進剤別）':24s}: {total_low:4d} - {total_high:4d} g")
    print(f"\n80g 制約との比較: 下限ケース({total_low}g) "
          f"{'収まる' if total_low <= MASS_MARGIN_G else '収まらない'}, "
          f"上限ケース({total_high}g) "
          f"{'収まる' if total_high <= MASS_MARGIN_G else '収まらない'}")
    deficit_low = total_low - MASS_MARGIN_G
    deficit_high = total_high - MASS_MARGIN_G
    print(f"  下限ケースの超過/余裕: {deficit_low:+.0f} g")
    print(f"  上限ケースの超過/余裕: {deficit_high:+.0f} g")
    if deficit_high > 0:
        print(f"  -> 上限ケースでは既存機器を約 {deficit_high:.0f} g 削減、"
              f"または 1.5U 化（利用可能質量を増やす設計変更）が必要")

    # --- 3.2 推進剤質量: 各 Delta-v ケース x Isp でロケット方程式 ---
    print("\n[3.2 推進剤質量（ロケット方程式, m0=1.25kg および m0=1.33kg で比較）]")
    print(f"{'dvケース':>12s} {'dv[m/s]':>9s} {'Isp[s]':>7s} "
          f"{'m_prop(m0=1.25kg)[g]':>20s} {'m_prop(m0=1.33kg)[g]':>20s}")
    prop_mass_results = {}
    for case_name, dv in dv_cases["cases"].items():
        for isp_s in ISP_LIST_S:
            m_prop_125 = rocket_equation_prop_mass(1.25, dv, isp_s) * 1000.0
            m_prop_133 = rocket_equation_prop_mass(1.33, dv, isp_s) * 1000.0
            prop_mass_results[(case_name, isp_s)] = {"m0_125": m_prop_125, "m0_133": m_prop_133}
            print(f"{case_name:>12s} {dv:>9.1f} {isp_s:>7.0f} "
                  f"{m_prop_125:>20.4f} {m_prop_133:>20.4f}")

    # --- 3.3 体積: 0.2U (200 cm^3) レイアウト影響 ---
    print("\n[3.3 体積 0.2U（200 cm^3）を割ける場合のレイアウト定性確認]")
    print("  mechanical_design.md のスタック配置（-Z半分: PC/104スタック+バッテリ"
          "+RWアセンブリ+MTQ+GPS+UHF収納機構+S帯パッチ）は既に -Z 面が高密度実装。")
    print("  -Z 面 S帯パッチアンテナ本体(No.16, 40x40x6mm)、RWアセンブリ張出し"
          "(No.11, 25x25x34mm)、バッテリスパイン(No.9, 20x37x65mm)が既存主要因子。")
    print("  0.2U(200cm^3、例えば 100x100x20mm 相当の層)を新規に割り当てる場合、"
          "-Z面パネル外側への外部搭載（スラスタヘッドのみ突出）が現実的だが、"
          "ME-3（突起<=6.5mm、CDS規定）を満たすには推進剤タンク・PPUは内部固定必須。")
    print("  -Z半分は既に-23.25mm(センサ基板下端)から-56.75mm(パネル面)までほぼ"
          "占有されており、200cm^3の連続空間は現状レイアウトに存在しない"
          "（定性確認: 内部再配置または1.5U化が必要）。")

    return {"mass_breakdown": mass_breakdown, "total_low": total_low, "total_high": total_high,
            "prop_mass_results": prop_mass_results}


# ---------------------------------------------------------------------------
# 検証4: 搭載環境適合
# ---------------------------------------------------------------------------

def verification4():
    print("\n" + "=" * 78)
    print("検証4: 搭載環境適合")
    print("=" * 78)

    # --- 4.1 光学汚染: 幾何配置要求 ---
    print("\n[4.1 光学汚染回避の幾何配置要求]")
    print("  +Z 開口（光学系クリアアパーチャ>=32mm, payload_optics.md）に対し、")
    print("  スラスタは -Z 面（S帯パッチアンテナ搭載面、mechanical_design.md No.16）")
    print("  近傍への配置を要求する。プルームの後方拡散は幾何学的に開口から")
    print("  最も遠い配置となり、視線（+Z方向）への直接入射を排除できる。")
    print("  汚染性（金属推進剤FEEP/インジウム vs ヨウ素 vs イオン液体の比較）は")
    print("  調査担当（docs/studies/cubesat_ion_engine_survey.md）に委ねる。")
    print("  幾何要求のみ定義: (1) 噴射方向は -Z 軸から円錐半角30度以上を確保し")
    print("  +Z開口方向への直接プルームを避ける。(2) スラスタ開口と光学開口の")
    print("  視線交差（line-of-sight）がない配置とする。(3) 可能であれば")
    print("  推力方向を軌道進行方向(+X系)に近づけ、-Z面法線から傾斜させる。")

    # --- 4.2 磁気クリーンネス ---
    print("\n[4.2 磁気クリーンネス（MT-3: 残留磁気モーメント<=5e-3 A m^2）]")
    print("  PPU（電源処理ユニット、DC-DCコンバータ・高圧発生回路を含む）は")
    print("  トランス・インダクタ等の磁性コアと大電流配線を伴うため、")
    print("  既存の残留磁気バジェット（adcs.md 3.2節: m_res=5.0e-3 A m^2 前提、")
    print("  外乱トルク 2.49e-7 N m、外乱トルク合計の97%を占め支配的要因）に")
    print("  対し追加の磁気モーメントを持ち込むリスクが高い。")
    print("  注意点: (1) PPU配線はツイストペア/シールドで往復電流のループ面積を")
    print("  最小化する。(2) 高圧発生部の磁性コア選定は低残留磁化材とする。")
    print("  (3) 既存 MT-3 バジェット5.0e-3 A m^2 は PPU追加分を含んでいないため、")
    print("  スラスタ搭載時は磁気モーメント再配分（他機器分含めた再検証）が必要。")

    # --- 4.3 推力ベクトルとCGオフセットによる外乱トルク ---
    print(f"\n[4.3 推力ベクトルとCGオフセットによる外乱トルク]")
    print(f"  前提: F=330uN, レバーアーム(スラスタ搭載位置からCGまでの推力軸直交成分)"
          f"={RW_LEVER_ARM_M*1e3:.1f}mm")
    f_thrust = 330e-6
    torque_thrust = f_thrust * RW_LEVER_ARM_M
    print(f"  トルク T = F x r = {f_thrust:.3e} N x {RW_LEVER_ARM_M:.3e} m "
          f"= {torque_thrust:.4e} N m")

    print(f"\n  RW容量との比較:")
    print(f"    RW最大トルク RW_MAX_TORQUE = {RW_MAX_TORQUE:.2e} N m")
    print(f"    RW最大角運動量 RW_MAX_MOMENTUM = {RW_MAX_MOMENTUM:.2e} N m s")
    ratio_torque = torque_thrust / RW_MAX_TORQUE
    print(f"    トルク比 T_thrust/RW_MAX_TORQUE = {ratio_torque:.4f} "
          f"({ratio_torque*100:.2f}%)")

    # 連続噴射時の角運動量蓄積（RW容量に対する飽和までの時間）
    t_sat = RW_MAX_MOMENTUM / torque_thrust
    print(f"\n  連続噴射時、RW角運動量が飽和(={RW_MAX_MOMENTUM:.2e} N m s)するまでの時間:")
    print(f"    t_sat = RW_MAX_MOMENTUM / T_thrust = {t_sat:.1f} s "
          f"= {t_sat/60:.2f} min")
    period_keep = orbital_period(R_EARTH + ALT_KEEP_M)
    print(f"    軌道周期 {period_keep/60:.1f} min に対する比 = {t_sat/period_keep:.3f}")

    # 既存外乱トルクバジェット（adcs.md 2.57e-7 N m）との比較
    existing_disturbance_torque = 2.57e-7
    ratio_vs_existing = torque_thrust / existing_disturbance_torque
    print(f"\n  既存外乱トルク合計（adcs.md §3.5）= {existing_disturbance_torque:.2e} N m "
          f"に対する比 = {ratio_vs_existing:.1f} 倍")

    # CG_Z_OFFSET_M を使った別ケース（推力軸がCGオフセットそのものを介する場合）
    torque_cg_offset = f_thrust * CG_Z_OFFSET_M
    print(f"\n  参考: レバーアームを実測CGオフセット(-Z 17.8mm)とした場合:")
    print(f"    T = {f_thrust:.3e} N x {CG_Z_OFFSET_M:.3e} m = {torque_cg_offset:.4e} N m "
          f"({torque_cg_offset/RW_MAX_TORQUE*100:.1f}% of RW_MAX_TORQUE)")

    verdict_5mm = "RW容量内で吸収可能" if ratio_torque < 1.0 else "RW容量を超過"
    print(f"\n  判定（レバーアーム5mm前提）: {verdict_5mm}")
    print(f"    トルクはRW最大トルクの{ratio_torque*100:.1f}%であり単発では吸収できるが、")
    print(f"    既存外乱トルク({existing_disturbance_torque:.2e} N m)の"
          f"{ratio_vs_existing:.0f}倍と圧倒的に大きいため、")
    print(f"    連続噴射中は約{t_sat:.0f}秒（軌道周期の{t_sat/period_keep*100:.1f}%）"
          f"でRW角運動量が飽和し、MTQによる頻繁な脱飽和運用が必須となる。")

    return {
        "torque_thrust_5mm": torque_thrust,
        "torque_thrust_cgoffset": torque_cg_offset,
        "ratio_torque": ratio_torque,
        "t_sat": t_sat,
        "existing_disturbance_torque": existing_disturbance_torque,
        "ratio_vs_existing": ratio_vs_existing,
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    t_start = time.time()

    print("#" * 78)
    print("# MOMO-SAT 超小型イオンエンジン搭載成立性 数値検証")
    print("# common/types.py 定数を参照。既存ファイル変更なし。")
    print("#" * 78)

    dv_cases = verification1()
    v2 = verification2(dv_cases)
    v3 = verification3(dv_cases)
    v4 = verification4()

    print("\n" + "=" * 78)
    print("総合サマリ")
    print("=" * 78)
    print(f"Delta-v需要: 控えめ={dv_cases['cases']['conservative']:.1f} m/s, "
          f"標準={dv_cases['cases']['standard']:.1f} m/s, "
          f"野心的={dv_cases['cases']['ambitious']:.1f} m/s")
    print(f"1Uで成立する(F,Isp,P)組合せ数: {len(v2['feasible'])} / "
          f"{len(v2['duration_results'])}")
    print(f"質量: スラスタ系合計 {v3['total_low']}-{v3['total_high']} g "
          f"vs 余裕 {MASS_MARGIN_G:.0f} g")
    print(f"推力トルク外乱（F=330uN, r=5mm）: {v4['torque_thrust_5mm']:.3e} N m "
          f"({v4['ratio_torque']*100:.1f}% of RW_MAX_TORQUE), "
          f"RW飽和まで {v4['t_sat']:.0f} s")

    elapsed = time.time() - t_start
    print(f"\n総実行時間: {elapsed:.2f} s")
    assert elapsed < 60.0, "60秒以内完走の要求を超過"


if __name__ == "__main__":
    main()
