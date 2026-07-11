# MOMO-SAT ソフトウェア ICD（FSW ⇔ シミュレータ インターフェース制御文書）

文書番号: MOMO-ICD-001 / Rev A / 2026-07-11

本文書は Python プロトタイプにおける FSW とシミュレータの結合仕様を定める。
**型定義の正は `common/types.py` であり、サブエージェントによる変更を禁止する。**

## 1. 実行モデル

シミュレータがマスタークロックを持ち、dt = `FSW_DT` = 0.1 s（10 Hz）で
FSW を呼び出す共シミュレーション方式（SILS）。

```python
from common.types import SensorData, ActuatorCommand, GroundCommand, Telemetry
from fsw.main import FlightSoftware

fsw = FlightSoftware()                      # config 省略可（dict で上書き可）
for k in range(n_steps):
    sensors: SensorData = sim.measure()      # センサモデル出力
    cmd: ActuatorCommand = fsw.step(sensors, dt)
    sim.apply(cmd)                           # アクチュエータモデルへ
    sim.propagate(dt)                        # RK4 で 1 ステップ積分
    if ground.has_command() and sensors.gs_visible:
        fsw.handle_command(ground.pop())     # GroundCommand
    tlm: Telemetry = fsw.get_telemetry()     # 1 Hz でログ（毎ステップ呼んでも良い）
```

## 2. FSW 公開 API（fsw/main.py）

```python
class FlightSoftware:
    def __init__(self, config: dict | None = None): ...
    def step(self, sensors: SensorData, dt: float) -> ActuatorCommand: ...
    def handle_command(self, cmd: GroundCommand) -> None: ...
    def get_telemetry(self) -> Telemetry: ...
```

制約:
- `step()` は例外を外へ漏らさない（内部でフォルト記録し SAFE 遷移）。
- `wheel_torque` は ±`RW_MAX_TORQUE`、`mtq_moment` は ±`MTQ_MAX_MOMENT` に
  FSW 側でクリップして出力する（シミュレータ側でも防御的にクリップする）。
- 起動直後のモードは SAFE。`SET_MODE DETUMBLE` コマンドまたは
  起動後 60 s の自律遷移で DETUMBLE へ。

## 3. 座標系・クォータニオン規約

- ECI: J2000 簡易（歳差章動無視）。地上局・ターゲットの ECEF→ECI 変換は
  グリニッジ恒星時 θ = OMEGA_EARTH × t（t=0 で経度 0° が X 軸）と簡易化する。
- クォータニオン q = [x, y, z, w]（スカラ末尾）、**ECI→body** 回転。
  ベクトル変換: `v_body = R(q) @ v_eci`。R(q) の定義は両者とも
  標準的な回転行列（Shuster 規約、右手系）を使用すること。
- 角速度はすべて body 系表示の対慣性角速度。

## 4. 挙動仕様（シミュレータ側）

| 項目 | 仕様 |
|---|---|
| camera_trigger | payload_power=True かつトリガ立ち上がりで 1 枚撮像成功。蓄積枚数+1（上限 MAX_STORED_IMAGES）。消費 1.5 W×2 s |
| tx_active | gs_visible=True の間のみ実際に伝送。レート DOWNLINK_RATE_MBPS で IMAGE_SIZE_MB を送り切ると images_downlinked+1。消費 2 W |
| heater_on | バッテリへ 1 W 加熱、消費 1 W |
| センサノイズ | gyro: σ=0.01°/s + バイアス 0.05°/s、mag: σ=100 nT、sun: σ=0.1°、GPS: σ=10 m / 0.1 m/s |
| RW モデル | 指令トルクへの一次遅れ（τ=0.05 s）、角運動量 ±RW_MAX_MOMENTUM で飽和 |
| MTQ | τ = m × B（理想、遅れなし） |
| 外乱 | 重力傾斜トルク + 残留磁気 (5e-3 A m²) + 大気トルク相当の定常小外乱 |
| 電力 | 各面太陽電池の cos 則発生電力、負荷集計、SoC 積分。食判定は円筒影 |
| 熱 | バッテリ 1 節点: 太陽入熱+内部発熱+ヒータ − 放射冷却 |
| 地上局 | 1 局（東京: 35.68°N, 139.77°E）、仰角 10° 以上で可視 |

## 5. FSW 動作仕様（モード別出力）

| モード | wheel_torque | mtq_moment | その他 |
|---|---|---|---|
| SAFE | 0 | B-dot | 全ペイロード OFF、ヒータ制御のみ有効 |
| DETUMBLE | 0 | B-dot | \|ω\|<0.5°/s が 60 s 継続で SUNPOINT へ自律遷移 |
| SUNPOINT | PD（−Z→太陽） | 脱飽和 | 食中は姿勢保持（バイアス補正付きジャイロ伝播） |
| NADIR_IMAGING | PD（+Z→地心、軌道レート FF 付き） | 脱飽和 | 推定指向誤差<0.4°（真値 0.5° に対する姿勢決定マージン）かつ対目標レート誤差<0.05°/s で撮像、指定枚数完了で SUNPOINT へ |
| TARGET_IMAGING | PD（+Z→ターゲット） | 脱飽和 | 直下距離<100 km で撮像 |
| DOWNLINK | PD（−Z→地心逆＝天頂） | 脱飽和 | gs_visible 中 tx_active=True、全画像送信完了で SUNPOINT へ |

共通自律遷移: `battery_soc < 0.30` → SAFE（ヒステリシス: 0.5 まで回復で解除可）。
バッテリ温度 < 0 ℃ で heater_on=True、> 5 ℃ で OFF（全モード共通）。

## 6. 検証インターフェース

シミュレータは真値（q_true, omega_true, pos, SoC, 温度…）を CSV
（`out/<scenario>.csv`）に 1 Hz で記録し、シナリオ終了時に要求適合
（AD-2, AD-3, AD-4, EP-3, MR-2 相当）を PASS/FAIL 表示する。
真値と TLM の突き合わせで姿勢決定誤差（AD-1）も評価する。
