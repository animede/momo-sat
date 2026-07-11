#!/usr/bin/env python3
"""数値実験: 地磁気ダイポールと衛星搭載電磁石（MTQ）の相互作用で
軌道高度を維持（進行方向へ増速）できるかを検証する。

このスクリプトは MOMO-SAT の既存モジュール
  - common/types.py       (MU_EARTH, R_EARTH, MTQ_MAX_MOMENT)
  - sim/environment.py    (magnetic_field_eci: 地軸整列ダイポール, B0=3.12e-5 T)
  - sim/dynamics.py       (orbit_accel, rk4_step 等の RK4 軌道伝播)
を「参照のみ」で使用し、既存ファイルは一切変更しない。

実験構成:
  実験1: 双極子勾配力 F = grad(m . B) の直接計算。
         軌道1周にわたり、|m|=MTQ_MAX_MOMENT 固定で進行方向成分 F_t を
         最大化する m の向きを探索し、F_t の軌道平均・最大を求める。
  実験2: 実験1の最適操舵磁気力を摂動として二体問題を RK4 積分し、
         半長軸 a の変化 Δa を m=0.2, m=200, 大気抵抗のみの3ケースで比較。
         エネルギー整合と無摂動ドリフトによる数値精度検証も行う。
  実験3: 大気抵抗を相殺するのに必要な |m| を逆算し、1U コイルでの
         巻数×電流・導体質量・抵抗損失電力を概算して 1U リソースと比較。

実行: .venv/bin/python tools/study_mtq_orbit_force.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.types import MU_EARTH, R_EARTH, MTQ_MAX_MOMENT  # noqa: E402
from sim.environment import magnetic_field_eci, B0_DIPOLE  # noqa: E402
from sim.dynamics import orbit_accel, rk4_step  # noqa: E402
from sim.scenarios._orbit import SSO_A_M, SSO_I_DEG, SSO_ALTITUDE_M  # noqa: E402

# ---------------------------------------------------------------------------
# 共通パラメータ
# ---------------------------------------------------------------------------

SAT_MASS_KG = 1.25          # docs/02_system_architecture.md 合計質量
A0 = SSO_A_M                # 軌道半長軸 [m] (円軌道, 500km高度)
INCL_DEG = SSO_I_DEG         # 97.4 deg (sim/scenarios/_orbit.py と同じ)
ALT_M = SSO_ALTITUDE_M

# 大気抵抗パラメータ（実験2/3で使用、ユーザ指定値）
RHO_ATM = 1.0e-13           # kg/m^3 (500km 目安)
CD = 2.2
AREA_DRAG = 0.01            # m^2 (1U 断面)

G0 = 9.80665


def circular_orbit_state(a: float, incl_deg: float, raan_deg: float = 0.0,
                          nu0_deg: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """円軌道の ECI 位置・速度を返す（近点引数=0 として RAAN 基準面から構成）。"""
    incl = np.radians(incl_deg)
    raan = np.radians(raan_deg)
    nu = np.radians(nu0_deg)

    # 軌道面内座標 (perifocal に相当、円軌道なので nu がそのまま角度)
    r_pf = np.array([a * np.cos(nu), a * np.sin(nu), 0.0])
    v_circ = np.sqrt(MU_EARTH / a)
    v_pf = np.array([-v_circ * np.sin(nu), v_circ * np.cos(nu), 0.0])

    # 回転: R3(-raan) R1(-incl)  (近点引数=0 と仮定)
    cO, sO = np.cos(raan), np.sin(raan)
    ci, si = np.cos(incl), np.sin(incl)

    r3 = np.array([
        [cO, -sO, 0.0],
        [sO, cO, 0.0],
        [0.0, 0.0, 1.0],
    ])
    r1 = np.array([
        [1.0, 0.0, 0.0],
        [0.0, ci, -si],
        [0.0, si, ci],
    ])
    rot = r3 @ r1
    return rot @ r_pf, rot @ v_pf


def orbital_period(a: float) -> float:
    return 2.0 * np.pi * np.sqrt(a ** 3 / MU_EARTH)


# ---------------------------------------------------------------------------
# 実験1: 双極子勾配力
# ---------------------------------------------------------------------------

def dipole_force_grad(pos_eci: np.ndarray, m_vec: np.ndarray, delta: float = 1.0) -> np.ndarray:
    """F = grad(m . B(r)) を中心差分で数値的に計算する [N]。

    m_vec: 磁気モーメントベクトル [A m^2]（ECI 表示、body=ECI 整列と仮定し
           純粋に「実現可能な最良の力」を評価する理想化）
    delta: 中心差分刻み [m]
    """
    force = np.zeros(3)
    for axis in range(3):
        dpos = np.zeros(3)
        dpos[axis] = delta
        b_plus = magnetic_field_eci(pos_eci + dpos)
        b_minus = magnetic_field_eci(pos_eci - dpos)
        force[axis] = np.dot(m_vec, (b_plus - b_minus)) / (2.0 * delta)
    return force


def optimal_m_direction(pos_eci: np.ndarray, vel_eci: np.ndarray, m_mag: float,
                         delta: float = 1.0, n_sample: int = 2000,
                         refine: bool = True) -> tuple[np.ndarray, float]:
    """進行方向成分 F_t = F . v_hat を最大化する m の向きを探索する。

    grad(m.B) は m に関して線形なので、F_t(m_hat) = m_hat . G^T v_hat の形
    (G_ij = dB_j/dx_i、つまり grad(m.B) = G @ m) になり、これは
    m_hat = argmax の解析解 = (G^T v_hat) / |G^T v_hat| が厳密最適である。
    これを解析的に求めたうえで、独立に球面サンプリングでも検証する
    （数値最適化の妥当性チェック）。
    """
    v_hat = vel_eci / np.linalg.norm(vel_eci)

    # G_ij = dB_j/dx_i を中心差分で構成（3x3 ヤコビアン）
    g = np.zeros((3, 3))
    for axis in range(3):
        dpos = np.zeros(3)
        dpos[axis] = delta
        b_plus = magnetic_field_eci(pos_eci + dpos)
        b_minus = magnetic_field_eci(pos_eci - dpos)
        g[axis, :] = (b_plus - b_minus) / (2.0 * delta)

    # F = G @ m  (F_i = sum_j dB_j/dx_i * m_j ... 検算: F = grad(m.B), F_i = d/dx_i (m.B) = m . dB/dx_i = g[i,:].m)
    # F_t = v_hat . F = v_hat . (G @ m) = (G^T v_hat) . m
    coeff = g.T @ v_hat
    coeff_norm = np.linalg.norm(coeff)
    if coeff_norm < 1e-30:
        m_hat_analytic = np.zeros(3)
        ft_analytic = 0.0
    else:
        m_hat_analytic = coeff / coeff_norm
        ft_analytic = coeff_norm * m_mag

    if not refine:
        return m_mag * m_hat_analytic, ft_analytic

    # 球面サンプリングによる独立検証（Fibonacci sphere）
    best_ft = -np.inf
    best_dir = m_hat_analytic
    golden = np.pi * (3.0 - np.sqrt(5.0))
    for i in range(n_sample):
        z = 1.0 - 2.0 * i / (n_sample - 1)
        r_xy = np.sqrt(max(0.0, 1.0 - z * z))
        theta = golden * i
        d = np.array([r_xy * np.cos(theta), r_xy * np.sin(theta), z])
        f = dipole_force_grad(pos_eci, m_mag * d, delta=delta)
        ft = np.dot(f, v_hat)
        if ft > best_ft:
            best_ft = ft
            best_dir = d

    return m_mag * best_dir, best_ft


def experiment1(delta_list=(0.1, 1.0, 10.0), n_points: int = 200):
    print("\n" + "=" * 78)
    print("実験1: 双極子勾配力 F = grad(m.B) の直接計算 と 進行方向成分の最適化")
    print("=" * 78)

    pos0, vel0 = circular_orbit_state(A0, INCL_DEG)
    period = orbital_period(A0)

    results_by_delta = {}
    for delta in delta_list:
        ft_values = []
        f_mag_values = []
        for k in range(n_points):
            nu_deg = 360.0 * k / n_points
            pos, vel = circular_orbit_state(A0, INCL_DEG, nu0_deg=nu_deg)
            m_opt, ft_analytic = optimal_m_direction(
                pos, vel, MTQ_MAX_MOMENT, delta=delta, n_sample=0, refine=False)
            ft_values.append(ft_analytic)
            f_mag_values.append(np.linalg.norm(dipole_force_grad(pos, m_opt, delta=delta)))
        ft_values = np.array(ft_values)
        results_by_delta[delta] = {
            "ft_max": ft_values.max(),
            "ft_avg": ft_values.mean(),
            "ft_min": ft_values.min(),
        }

    print(f"\n軌道: a={A0/1e3:.1f} km (高度 {ALT_M/1e3:.0f} km), i={INCL_DEG} deg, "
          f"周期={period/60:.1f} min, |m|={MTQ_MAX_MOMENT} A m^2")
    print(f"\n{'delta [m]':>10s}  {'F_t_max [N]':>14s}  {'F_t_avg [N]':>14s}  {'F_t_min [N]':>14s}")
    for delta, r in results_by_delta.items():
        print(f"{delta:>10.2f}  {r['ft_max']:>14.6e}  {r['ft_avg']:>14.6e}  {r['ft_min']:>14.6e}")

    # 数値微分刻みの妥当性: delta=1m の結果を基準に、他の刻みとの相対差
    ref = results_by_delta[1.0]
    print("\n[数値微分刻み感度] delta=1m を基準とした F_t_avg の相対差:")
    for delta, r in results_by_delta.items():
        rel = abs(r["ft_avg"] - ref["ft_avg"]) / abs(ref["ft_avg"]) if ref["ft_avg"] != 0 else 0.0
        print(f"  delta={delta:6.2f} m: F_t_avg={r['ft_avg']:.6e} N, "
              f"相対差={rel:.3e}")

    # サンプリングによる独立検証（delta=1m, 代表点 nu=0,90,180,270）
    print("\n[解析解 vs 球面サンプリング(2000点) の独立検証] (delta=1m)")
    max_rel_err = 0.0
    for nu_deg in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0):
        pos, vel = circular_orbit_state(A0, INCL_DEG, nu0_deg=nu_deg)
        m_analytic, ft_analytic = optimal_m_direction(pos, vel, MTQ_MAX_MOMENT, delta=1.0,
                                                        n_sample=0, refine=False)
        _, ft_sampled = optimal_m_direction(pos, vel, MTQ_MAX_MOMENT, delta=1.0,
                                             n_sample=2000, refine=True)
        rel_err = abs(ft_analytic - ft_sampled) / max(abs(ft_analytic), 1e-30)
        max_rel_err = max(max_rel_err, rel_err)
        print(f"  nu={nu_deg:6.1f} deg: F_t(解析)={ft_analytic:.6e} N, "
              f"F_t(サンプリング)={ft_sampled:.6e} N, 相対誤差={rel_err:.3e}")
    print(f"  -> 最大相対誤差 = {max_rel_err:.3e} (解析解と独立サンプリングの一致度)")

    return results_by_delta[1.0], A0, INCL_DEG


# ---------------------------------------------------------------------------
# 実験2: 軌道積分での長期効果
# ---------------------------------------------------------------------------

def steered_magnetic_accel(pos: np.ndarray, vel: np.ndarray, m_mag: float,
                            delta: float = 1.0) -> np.ndarray:
    """最適操舵磁気力による加速度 [m/s^2]（F_t 最大化方向, |m|=m_mag 固定）。"""
    if m_mag <= 0.0:
        return np.zeros(3)
    m_opt, _ = optimal_m_direction(pos, vel, m_mag, delta=delta, n_sample=0, refine=False)
    f = dipole_force_grad(pos, m_opt, delta=delta)
    return f / SAT_MASS_KG


def drag_accel(vel: np.ndarray) -> np.ndarray:
    """大気抵抗による加速度 [m/s^2]（反進行方向）。"""
    v = np.linalg.norm(vel)
    if v < 1e-9:
        return np.zeros(3)
    f_mag = 0.5 * RHO_ATM * v ** 2 * CD * AREA_DRAG
    return -(f_mag / SAT_MASS_KG) * (vel / v)


def semi_major_axis(pos: np.ndarray, vel: np.ndarray) -> float:
    """vis-viva から半長軸 a を算出。"""
    r = np.linalg.norm(pos)
    v2 = float(np.dot(vel, vel))
    inv_a = 2.0 / r - v2 / MU_EARTH
    return 1.0 / inv_a


def specific_energy(pos: np.ndarray, vel: np.ndarray) -> float:
    r = np.linalg.norm(pos)
    v2 = float(np.dot(vel, vel))
    return 0.5 * v2 - MU_EARTH / r


def run_orbit_integration(mode: str, n_orbits: float, dt: float, m_mag: float = 0.0,
                           delta: float = 1.0, record_every: int = 50):
    """二体問題 + 摂動力 (mode: 'none', 'mtq', 'drag') を RK4 積分。

    戻り値: dict(times, a_hist, energy_hist, work_hist(=累積 F_t*v dt の理論値))
    """
    pos, vel = circular_orbit_state(A0, INCL_DEG)
    period = orbital_period(A0)
    t_end = n_orbits * period
    n_steps = int(np.ceil(t_end / dt))

    def perturb_accel(p, v):
        if mode == "mtq":
            return steered_magnetic_accel(p, v, m_mag, delta=delta)
        elif mode == "drag":
            return drag_accel(v)
        else:
            return np.zeros(3)

    def deriv(state):
        p = state[:3]
        v = state[3:]
        acc = orbit_accel(p, j2_enabled=False) + perturb_accel(p, v)
        return np.concatenate([v, acc])

    a_hist = []
    t_hist = []
    energy_hist = []
    work_cumulative = 0.0
    work_hist = []

    t = 0.0
    for step in range(n_steps + 1):
        if step % record_every == 0 or step == n_steps:
            a_hist.append(semi_major_axis(pos, vel))
            t_hist.append(t)
            energy_hist.append(specific_energy(pos, vel))
            work_hist.append(work_cumulative)
        if step == n_steps:
            break
        # RK4 (perturb_accel は中間ステージでも都度評価: rk4_step の deriv を使い回す)
        state = np.concatenate([pos, vel])
        new_state = rk4_step(state, dt, deriv)

        # 仕事 F_t . v の台形積分による理論エネルギー変化(摂動力の specific power)
        acc1 = perturb_accel(pos, vel)
        p_power1 = float(np.dot(acc1, vel))
        pos, vel = new_state[:3], new_state[3:]
        acc2 = perturb_accel(pos, vel)
        p_power2 = float(np.dot(acc2, vel))
        work_cumulative += 0.5 * (p_power1 + p_power2) * dt

        t += dt

    return {
        "t": np.array(t_hist),
        "a": np.array(a_hist),
        "energy": np.array(energy_hist),
        "work": np.array(work_hist),
        "period": period,
    }


def experiment2(ft_avg_ref: float):
    print("\n" + "=" * 78)
    print("実験2: 軌道積分での長期効果 (二体問題 + 摂動力, RK4)")
    print("=" * 78)

    n_orbits = 10.0
    dt_main = 1.0

    print(f"\n積分条件: n_orbits={n_orbits}, dt={dt_main} s, "
          f"周期={orbital_period(A0)/60:.2f} min")

    cases = []

    t0 = time.time()
    res_none = run_orbit_integration("none", n_orbits, dt_main)
    cases.append(("(0) 無摂動(基準, 数値誤差評価用)", res_none, 0.0))
    t1 = time.time()

    res_a = run_orbit_integration("mtq", n_orbits, dt_main, m_mag=MTQ_MAX_MOMENT)
    cases.append(("(a) MTQ m=0.2 A m^2 (実機上限, 最適操舵)", res_a, MTQ_MAX_MOMENT))
    t2 = time.time()

    res_b = run_orbit_integration("mtq", n_orbits, dt_main, m_mag=200.0)
    cases.append(("(b) MTQ m=200 A m^2 (1000倍, 感度確認)", res_b, 200.0))
    t3 = time.time()

    res_c = run_orbit_integration("drag", n_orbits, dt_main)
    cases.append(("(c) 大気抵抗のみ (rho=1e-13, Cd=2.2, A=0.01 m^2)", res_c, None))
    t4 = time.time()

    print(f"\n積分所要時間: none={t1-t0:.2f}s, m=0.2:{t2-t1:.2f}s, "
          f"m=200:{t3-t2:.2f}s, drag:{t4-t3:.2f}s")

    print(f"\n{'ケース':<48s} {'Δa/軌道 [m]':>14s} {'Δa/日 [m]':>14s} {'a_final-a0 [m]':>16s}")
    day_s = 86400.0
    period = orbital_period(A0)
    summary = {}
    for name, res, m_mag in cases:
        da_total = res["a"][-1] - res["a"][0]
        da_per_orbit = da_total / n_orbits
        da_per_day = da_total / (n_orbits * period) * day_s
        print(f"{name:<48s} {da_per_orbit:>14.6e} {da_per_day:>14.6e} {da_total:>16.6e}")
        summary[name] = {"da_total": da_total, "da_per_orbit": da_per_orbit,
                          "da_per_day": da_per_day}

    # --- 無摂動ドリフトが (a) の Δa より十分小さいことの確認 ---
    da_none = abs(res_none["a"][-1] - res_none["a"][0])
    da_a = abs(res_a["a"][-1] - res_a["a"][0])
    ratio = da_none / da_a if da_a > 0 else float("inf")
    print(f"\n[数値誤差チェック] 無摂動ケースの |Δa| = {da_none:.3e} m")
    print(f"                  MTQ(m=0.2) ケースの |Δa| = {da_a:.3e} m")
    print(f"                  比率 (無摂動ドリフト / (a)のΔa) = {ratio:.3e}")
    drift_ok = ratio < 0.05
    print(f"                  -> {'OK: 無摂動ドリフトは(a)の5%未満' if drift_ok else '要注意: ドリフトが無視できない大きさ'}")

    if not drift_ok:
        print("\n  dt を 0.2s に下げて再検証...")
        dt_fine = 0.2
        res_none_fine = run_orbit_integration("none", n_orbits, dt_fine)
        res_a_fine = run_orbit_integration("mtq", n_orbits, dt_fine, m_mag=MTQ_MAX_MOMENT)
        da_none_fine = abs(res_none_fine["a"][-1] - res_none_fine["a"][0])
        da_a_fine = abs(res_a_fine["a"][-1] - res_a_fine["a"][0])
        ratio_fine = da_none_fine / da_a_fine if da_a_fine > 0 else float("inf")
        print(f"  dt={dt_fine}s: 無摂動|Δa|={da_none_fine:.3e} m, "
              f"(a)|Δa|={da_a_fine:.3e} m, 比率={ratio_fine:.3e}")

    # --- エネルギー整合チェック: dE/dt(数値) vs F_t.v(理論仕事) ---
    print("\n[エネルギー整合チェック] ケース(a) m=0.2 A m^2")
    de_numeric = res_a["energy"][-1] - res_a["energy"][0]
    de_work = res_a["work"][-1] - res_a["work"][0]
    rel_err_energy = abs(de_numeric - de_work) / max(abs(de_work), 1e-30)
    print(f"  比エネルギー変化(数値積分から) dE = {de_numeric:.6e} J/kg")
    print(f"  F_t.v の時間積分(理論仕事)     dW = {de_work:.6e} J/kg")
    print(f"  相対誤差 |dE - dW| / |dW| = {rel_err_energy:.3e}")
    energy_ok = rel_err_energy < 0.01
    print(f"  -> {'OK: 数値積分とエネルギー収支が整合' if energy_ok else '要確認: 乖離が大きい、dt低減を検討'}")

    return summary, res_a, res_c, energy_ok, drift_ok


# ---------------------------------------------------------------------------
# 実験3: 大気抵抗相殺に必要な磁気モーメントの逆算
# ---------------------------------------------------------------------------

def experiment3(ft_avg_at_02: float):
    print("\n" + "=" * 78)
    print("実験3: 大気抵抗相殺に必要な磁気モーメントの逆算 と 1U コイル成立性")
    print("=" * 78)

    # 円軌道での代表的な大気抵抗力（軌道速度一定と近似）
    v_circ = np.sqrt(MU_EARTH / A0)
    f_drag = 0.5 * RHO_ATM * v_circ ** 2 * CD * AREA_DRAG
    print(f"\n軌道速度 v_circ = {v_circ:.3f} m/s")
    print(f"大気抵抗力 F_drag = 0.5*rho*v^2*Cd*A = {f_drag:.6e} N "
          f"(rho={RHO_ATM:.1e}, Cd={CD}, A={AREA_DRAG} m^2)")

    # F_t は m に比例 (実験1: F_t_avg(m=0.2) が既知)
    # 必要な m = 0.2 * (f_drag / F_t_avg(0.2))
    if ft_avg_at_02 <= 0:
        print("F_t_avg <= 0 のため逆算不可（進行方向への正味の力が得られない）")
        return

    m_req = MTQ_MAX_MOMENT * (f_drag / ft_avg_at_02)
    print(f"\n実験1での F_t_avg(m=0.2 A m^2) = {ft_avg_at_02:.6e} N (線形比例関係を利用)")
    print(f"大気抵抗を軌道平均で相殺するのに必要な |m| = {m_req:.6e} A m^2")
    print(f"  (実機上限 MTQ_MAX_MOMENT=0.2 A m^2 の {m_req / MTQ_MAX_MOMENT:.3e} 倍)")

    # 1U コイルでの実現性概算
    # コイル: 一辺 0.1m 正方形、面積 A_coil ~ 0.01 m^2, 銅線
    print("\n--- 1U コイルでの実現性概算 ---")
    a_coil = 0.01  # m^2 (1U 側面)
    side = np.sqrt(a_coil)  # 0.1 m
    print(f"コイル形状: 一辺 {side*100:.1f} cm 正方形, 面積 A_coil={a_coil} m^2")

    # m = N * I * A_coil  ->  N*I = m_req / A_coil
    ni_req = m_req / a_coil
    print(f"必要 N*I (巻数×電流) = m_req / A_coil = {ni_req:.6e} A-turns")

    # 銅線: AWG30 相当, 断面積 ~0.0507 mm^2, 抵抗率 1.68e-8 Ohm*m, 密度 8960 kg/m^3
    rho_cu = 1.68e-8   # Ohm*m
    density_cu = 8960.0  # kg/m^3
    wire_area = 0.0507e-6  # m^2 (AWG30)
    awg = "AWG30"

    for i_test in (0.05, 0.1, 0.2, 0.5, 1.0, 2.0):
        n_turns = ni_req / i_test
        wire_len = n_turns * 4.0 * side  # 正方形一巻き = 4*side
        wire_mass = wire_len * wire_area * density_cu
        wire_resistance = rho_cu * wire_len / wire_area
        power_loss = i_test ** 2 * wire_resistance
        print(f"  I={i_test:5.2f} A -> N={n_turns:10.1f} turns, "
              f"導体長={wire_len:8.1f} m, 導体質量={wire_mass:7.4f} kg, "
              f"R={wire_resistance:8.2f} Ohm, 損失電力={power_loss:8.2f} W  ({awg})")

    print("\n1U リソース上限: 電力 2.0 W (軌道平均), 質量 1.33 kg (docs/02, MTQ×3 合計 60g)")
    print("実機 MTQ 定格: |m|<=0.2 A m^2 (common/types.py MTQ_MAX_MOMENT), 消費電力 0.2 W (docs/02)")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    t_start = time.time()
    exp1_ref, a0, incl = experiment1()
    ft_avg = exp1_ref["ft_avg"]
    ft_max = exp1_ref["ft_max"]

    summary2, res_a, res_c, energy_ok, drift_ok = experiment2(ft_avg)

    experiment3(ft_avg)

    print("\n" + "=" * 78)
    print("総合サマリ")
    print("=" * 78)
    print(f"F_t_avg (m=0.2 A m^2, 最適操舵) = {ft_avg:.6e} N")
    print(f"F_t_max (m=0.2 A m^2, 最適操舵) = {ft_max:.6e} N")
    for name, s in summary2.items():
        print(f"  {name}: Δa/日 = {s['da_per_day']:.6e} m/day")
    print(f"数値精度: エネルギー整合 {'OK' if energy_ok else 'NG'}, "
          f"無摂動ドリフト十分小 {'OK' if drift_ok else 'NG'}")
    print(f"\n総実行時間: {time.time() - t_start:.2f} s")


if __name__ == "__main__":
    main()
