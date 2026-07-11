# MOMO-SAT 電気推進拡張仕様（THRUST モード）

文書番号: MOMO-EXT-001 / Rev A / 2026-07-11
背景: docs/studies/cubesat_ion_engine_summary.md（1U 搭載は nanoFEEP 級で条件付き成立、
「パルス噴射+RW 脱飽和の交互運用」の SILS 検証が残課題）
型定義の正: common/types.py（Mode.THRUST、ActuatorCommand.thruster_on、THRUST_* 定数）

## 1. 目的

nanoFEEP 級電気推進（20 µN / Isp 3000 s / 1 W）を FSW・シミュレータに実装し、
以下を閉ループ SILS で検証する:

- V1: 噴射トルク外乱（CG オフセット 5 mm → 1×10⁻⁷ N·m）下で姿勢維持と
  RW 運動量管理（脱飽和と並行）が成立すること
- V2: 軌道上昇効果がガウス方程式の予測 da/dt = 2(F/m)/n と整合すること
- V3: 電力制約（日照中のみ噴射、SoC ガード）下でデューティ運用が回ること
- V4: 推進剤消費の FSW 推定がシミュレータ真値と整合すること

## 2. シミュレータ側仕様

- スラスタは −X 面搭載、**body +X 方向**に THRUST_FORCE の推力。
  `thruster_on=True` の間、軌道運動へ加速度 a = R(q)ᵀ[F/m_sat,0,0]（ECI）を付加
  （軌道 RK4 の摂動項に追加。m_sat=1.25 kg 固定で可、推進剤消費 <1 g は無視）
- 姿勢へは外乱トルク τ = [0, THRUST_FORCE×THRUST_LEVER_ARM, 0]（body、1×10⁻⁷ N·m）を付加
- 電力負荷 +THRUST_POWER（噴射中のみ）
- 真値記録: 累積噴射時間、消費推進剤 mdot=F/(Isp·G0)、軌道長半径 a
  （vis-viva: a = 1/(2/r − v²/μ)）を CSV へ追加

## 3. FSW 側仕様

- `GroundCommand("THRUST", {"duration_s": T})` 受理で Mode.THRUST へ
- ガイダンス: NADIR と同一（+Z→地心、+X→速度方向 = プログレード噴射）、
  軌道レートフィードフォワード付き PD + MTQ 脱飽和は既存則を使用
- **噴射許可条件**（全成立時のみ thruster_on=True）:
  1. 指向誤差 < 5°（推力損失 cos5°≈0.4% と外乱方向の管理）
  2. 日照中（solar_power > 0）
  3. battery_soc > 0.5
  4. RW 角運動量 各軸 |h| < 0.8×RW_MAX_MOMENTUM（超過時は噴射一時停止、
     脱飽和継続、回復で再開 — これが「パルス噴射+脱飽和の交互運用」）
- 累積噴射時間 ≥ duration_s で SUNPOINT へ自律復帰
- TLM: thrust_time（累積 [s]）、propellant_used_g = thrust_time×F/(Isp·G0)×1000
- SoC<0.30 の SAFE 遷移等、既存の自律遷移は THRUST 中も有効（SAFE で噴射即停止）

## 4. 検証シナリオ S4（sim/scenarios/s4_thrust.py）

- 軌道・初期条件は S2 と同様（SSO 500 km、初期レート小）
- t=10 SET_MODE SUNPOINT、t=600 THRUST duration_s=3600（force_uplink）、2 軌道（11400 s）実行
- PASS 判定:
  1. **V2**: Δa（真値）が予測 2(F/m_sat)/n × t_on（t_on=シミュレータ真値の累積噴射時間）
     の ±20% 以内、かつ Δa > 50 m（効果が数値誤差でないこと）
  2. **V1**: RW 角運動量 各軸 max|h| < 0.95×RW_MAX_MOMENTUM（飽和なし）
  3. **V1**: 噴射中の真の推力軸方向誤差（body +X vs 速度方向）< 5°
  4. **V3**: min SoC ≥ 0.45、かつ食中の噴射ゼロ
  5. **V4**: FSW の propellant_used_g がシミュレータ真値と相対 5% 以内
  6. 累積噴射 3600 s 達成後 SUNPOINT へ自律復帰
- 参考数値: a_t=1.6×10⁻⁵ m/s²、n=1.1085×10⁻³ rad/s → da/dt≈0.0289 m/s（噴射中）、
  3600 s で Δa≈104 m。無摂動ドリフト（S1 実績 ~1.4×10⁻⁷ m/日）より 9 桁大きく判定可能

## 5. 検証結果（2026-07-11 実施、`run_sim.py S4`）

**総合判定: PASS（6/6 項目）** — 独立再実行で確認済み

| 項目 | 実測 | 判定 |
|---|---|---|
| V2 軌道上昇 | Δa=117.6 m vs 予測 103.9 m（誤差 13.1%、噴射 3600.1 s） | PASS |
| V1 RW 運動量 | max\|h\| = 3.69×10⁻⁴ N·m·s（限度の 39%、飽和なし） | PASS |
| V1 推力軸誤差 | 最大 0.75°（噴射中 36,001 サンプル、限度 5°） | PASS |
| V3 電力 | min SoC = 0.893、食中噴射 0 サンプル | PASS |
| V4 推進剤 | FSW 推定 = 真値 = 0.00245 g（誤差 0.00%） | PASS |
| 自律復帰 | 累積 3600 s 達成 → SUNPOINT 復帰 | PASS |

実装上の知見:
- osculating 軌道長半径は J2 短周期変動（±十 km 級）を持つため、Δa 評価は
  噴射前後各 1 周期の**同一窓長**平均で行う（窓長不一致だと残差 ~340 m）。
  V2 の残差 13% は主にこの平均化残差によるもので、判定許容 ±20% 内。
- THRUST 投入は SUNPOINT 収束後とする運用制約が必要（大角度スルー過渡中の
  投入は RW 運動量蓄積を招く）→ ConOps に反映すべき事項。
- RW 運動量は既存の脱飽和則のみで噴射トルク外乱（1×10⁻⁷ N·m）を管理でき、
  「噴射一時停止」ガード（0.8 閾値）は本シナリオでは発動しなかった
  （ガードは太陽活動高時・長時間噴射用の保険として妥当）。

## 6. 回帰条件

- 既存シナリオ S1/S2/S3、単体テスト（FSW/sim/GSI）がすべて PASS のままであること
- gsi/mapping.py が Mode.THRUST で例外を出さないこと（tlm mode はそのまま "THRUST" を送出、
  SCC 側表示は未対応で可）
