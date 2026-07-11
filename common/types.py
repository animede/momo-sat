"""MOMO-SAT 共有型定義（ICD: docs/03_software_icd.md）

FSW とシミュレータ間の唯一のインターフェース。
このファイルはメイン設計者が管理する。サブエージェントは変更禁止。

座標系・単位の規約:
- ECI: J2000 簡易慣性座標系 [m], [m/s]
- body: 機体座標系（+Z がカメラ視線方向）
- クォータニオン q = [x, y, z, w]（スカラ末尾）、ECI→body の回転を表す
- 角速度 omega: body 座標系で表した対慣性角速度 [rad/s]
- 磁場 [T]、磁気モーメント [A m^2]、トルク [N m]
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np


class Mode(str, Enum):
    SAFE = "SAFE"
    DETUMBLE = "DETUMBLE"
    SUNPOINT = "SUNPOINT"
    NADIR_IMAGING = "NADIR_IMAGING"
    TARGET_IMAGING = "TARGET_IMAGING"
    DOWNLINK = "DOWNLINK"
    THRUST = "THRUST"  # 電気推進による軌道上昇（docs/08 拡張仕様）
    OBSERVE = "OBSERVE"  # 天体観測: 慣性方向指向 + 連続フレーム撮像（docs/09 拡張仕様）


@dataclass
class SensorData:
    """シミュレータ → FSW（毎ステップ、10 Hz）"""

    t: float                      # シミュレーション時刻 [s]
    gyro: np.ndarray              # (3,) body 角速度計測値 [rad/s]（ノイズ+バイアス込み）
    mag: np.ndarray               # (3,) body 磁場計測値 [T]
    sun: np.ndarray | None        # (3,) body 太陽方向単位ベクトル。食中・視野外は None
    gps_pos_eci: np.ndarray       # (3,) ECI 位置 [m]
    gps_vel_eci: np.ndarray       # (3,) ECI 速度 [m/s]
    wheel_speeds: np.ndarray      # (3,) RW 回転角速度 [rad/s]（各機体軸）
    battery_voltage: float        # [V]
    battery_soc: float            # 0.0–1.0
    solar_power: float            # 太陽電池発生電力 [W]
    battery_temp: float           # バッテリ温度 [degC]
    gs_visible: bool              # 地上局可視フラグ（通信機の AOS 検出相当）
    st_quat: np.ndarray | None = None  # スタートラッカ姿勢計測 [x,y,z,w]（非搭載/無効時 None、docs/09）


@dataclass
class ActuatorCommand:
    """FSW → シミュレータ（毎ステップ）"""

    wheel_torque: np.ndarray = field(
        default_factory=lambda: np.zeros(3))   # (3,) RW トルク指令 [N m]（機体軸、上限 ±1e-4）
    mtq_moment: np.ndarray = field(
        default_factory=lambda: np.zeros(3))   # (3,) MTQ 磁気モーメント [A m^2]（上限 ±0.2）
    camera_trigger: bool = False               # True の 1 ステップで 1 枚撮像
    tx_active: bool = False                    # S 帯送信 ON（画像ダウンリンク）
    heater_on: bool = False                    # バッテリヒータ ON
    payload_power: bool = False                # ペイロード電源 ON（撮像準備）
    thruster_on: bool = False                  # 電気推進 ON（body +X 方向に THRUST_FORCE）
    shutter_open: bool = False                 # 天体観測の露光中フラグ（レベル信号、docs/09）


@dataclass
class Telemetry:
    """FSW が 1 Hz で更新する状態テレメトリ（get_telemetry() で取得）"""

    t: float = 0.0
    mode: Mode = Mode.SAFE
    q_est: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))  # 推定姿勢（ECI→body）
    omega_est: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s]
    pointing_error_deg: float = 0.0   # 目標姿勢との誤差角 [deg]（目標未設定時 0）
    battery_soc: float = 0.0
    battery_voltage: float = 0.0
    battery_temp: float = 0.0
    wheel_speeds: np.ndarray = field(default_factory=lambda: np.zeros(3))
    images_stored: int = 0            # 蓄積画像枚数
    images_downlinked: int = 0        # 伝送完了画像枚数
    uptime: float = 0.0               # FSW 起動からの経過秒
    last_cmd: str = ""                # 最後に受理したコマンド種別
    fault_flags: int = 0              # ビットフラグ（0=正常）
    thrust_time: float = 0.0          # 累積噴射時間 [s]（THRUST 拡張）
    propellant_used_g: float = 0.0    # 消費推進剤 [g]（FSW 推定値）
    frames_captured: int = 0          # 天体観測フレーム数（OBSERVE 拡張）
    observe_time: float = 0.0         # 累積露光時間 [s]（OBSERVE 拡張）


@dataclass
class GroundCommand:
    """地上局 → FSW（可視時のみシミュレータ経由で投入）

    type と params の組:
      "SET_MODE"       {"mode": Mode}            # モード遷移要求
      "IMAGE_NADIR"    {"count": int}            # 直下撮像 n 枚（NADIR_IMAGING へ）
      "IMAGE_TARGET"   {"lat_deg": float, "lon_deg": float}  # 指定地点撮像
      "DOWNLINK"       {}                        # 蓄積画像のダウンリンク開始
      "THRUST"         {"duration_s": float}     # 累積噴射 duration_s 秒の軌道上昇（THRUST へ）
      "OBSERVE"        {"ra_deg": float, "dec_deg": float,
                        "duration_s": float, "exposure_s": float}
                                                 # 慣性方向(RA/Dec)を指向し露光 exposure_s の
                                                 # フレームを duration_s の間 連続取得（OBSERVE へ）
      "RESET"          {}                        # SAFE へ戻す
    """

    type: str
    params: dict[str, Any] = field(default_factory=dict)


# 物理定数・共通パラメータ（両実装で共用）
MU_EARTH = 3.986004418e14      # 地球重力定数 [m^3/s^2]
R_EARTH = 6371.0e3             # 地球平均半径 [m]
J2 = 1.08263e-3
OMEGA_EARTH = 7.2921159e-5     # 地球自転角速度 [rad/s]

# 衛星パラメータ
INERTIA = np.diag([2.1e-3, 2.1e-3, 1.9e-3])  # 慣性テンソル [kg m^2]
RW_MAX_TORQUE = 1.0e-4         # RW 最大トルク [N m]
RW_MAX_MOMENTUM = 1.0e-3       # RW 最大角運動量 [N m s]
RW_INERTIA = 1.0e-6            # RW ロータ慣性 [kg m^2]
MTQ_MAX_MOMENT = 0.2           # MTQ 最大磁気モーメント [A m^2]
BATTERY_CAPACITY_WH = 25.0     # バッテリ容量 [Wh]
IMAGE_SIZE_MB = 1.0            # 圧縮後画像サイズ [MB]
DOWNLINK_RATE_MBPS = 0.1       # S 帯実効レート [Mbit/s]
MAX_STORED_IMAGES = 32         # 画像ストレージ容量 [枚]
FSW_DT = 0.1                   # FSW 制御周期 [s]（10 Hz）

# 天体観測拡張（docs/09）
PIXEL_IFOV_RAD = 2.74e-5       # 1 画素の視野角 [rad]（2.74 µm / 100 mm ≈ 5.65 秒角）
FRAME_GAP_S = 1.0              # フレーム間ギャップ [s]（読み出し + 脱飽和許可窓）
EARTH_LIMB_MARGIN_DEG = 10.0   # 地球縁からの回避マージン [deg]（迷光・掩蔽判定）
ST_QUAT_SIGMA_ARCSEC = 10.0    # スタートラッカ計測ノイズ (1σ, 各軸) [arcsec]
ST_UPDATE_HZ = 5.0             # スタートラッカ更新レート [Hz]
ST_MAX_RATE_DEG_S = 1.0        # スタートラッカが追尾可能な最大レート [deg/s]（超過で無効）

# 電気推進（nanoFEEP 級、docs/studies/cubesat_ion_engine_summary.md §5）
THRUST_FORCE = 20.0e-6         # 推力 [N]（body +X 方向、スラスタは -X 面搭載）
THRUST_ISP = 3000.0            # 比推力 [s]
THRUST_POWER = 1.0             # 噴射中の消費電力 [W]
THRUST_LEVER_ARM = 0.005       # 推力線と CG のオフセット [m]（外乱トルク = F×アーム、body Y 軸回り）
G0 = 9.80665                   # 標準重力加速度 [m/s^2]（推進剤消費 mdot = F/(Isp*g0)）
