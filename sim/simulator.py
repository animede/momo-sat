"""Simulator クラス: メインループ（ICD §1 のループ、dt=FSW_DT で FSW と結合）。"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from common.types import (
    ActuatorCommand,
    EARTH_LIMB_MARGIN_DEG,
    FSW_DT,
    G0,
    INERTIA,
    MTQ_MAX_MOMENT,
    MU_EARTH,
    Mode,
    PIXEL_IFOV_RAD,
    R_EARTH,
    RW_MAX_TORQUE,
    Telemetry,
    THRUST_FORCE,
    THRUST_ISP,
    THRUST_LEVER_ARM,
)
from sim.actuators import Camera, ReactionWheelAssembly, Transmitter, mtq_torque
from sim.dynamics import propagate_attitude, propagate_orbit, quat_to_rot
from sim.environment import (
    RESIDUAL_DIPOLE_MAGNITUDE,
    constant_disturbance_torque,
    gravity_gradient_torque,
    is_eclipse,
    magnetic_field_eci,
    residual_dipole_torque,
    sun_direction_eci,
)
from sim.ground import GroundStation, is_visible
from sim.power import Battery, solar_power_generated, total_load
from sim.sensors import SensorSuite, wheel_speeds_from_momentum
from sim.thermal import ThermalModel
from common.types import DOWNLINK_RATE_MBPS
from fsw.observe_manager import is_occulted

LOAD_PAYLOAD_OBSERVE_W = 1.5  # 露光中の payload 電力負荷 [W]（docs/09 §3）


@dataclass
class ScenarioConfig:
    """シナリオ定義（sim/scenarios/ から生成）。"""

    name: str
    duration_s: float
    a_m: float
    e: float
    i_deg: float
    raan_deg: float
    nu0_deg: float = 0.0
    q0: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    omega0_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3))
    timeline: list = field(default_factory=list)  # list[TimelineEntry]
    soc_init: float = 1.0
    battery_temp_init_c: float = 10.0
    seed: int = 42
    fsw_config: dict | None = None
    check_fn: Callable[["Simulator"], dict] | None = None
    star_tracker: bool = False  # スタートラッカモデルの有効化（docs/09 §3、既定無効）


def orbital_elements_to_state(a: float, e: float, i_deg: float, raan_deg: float,
                               nu_deg: float, mu: float) -> tuple[np.ndarray, np.ndarray]:
    """古典軌道要素（近点引数=0 と仮定）から ECI 位置・速度を計算する。"""
    i = np.radians(i_deg)
    raan = np.radians(raan_deg)
    nu = np.radians(nu_deg)
    p = a * (1.0 - e ** 2)
    r = p / (1.0 + e * np.cos(nu))

    # perifocal 座標系（近点引数=0 なので周回面基準 = 昇交点方向が x）
    r_pf = np.array([r * np.cos(nu), r * np.sin(nu), 0.0])
    v_pf = np.sqrt(mu / p) * np.array([-np.sin(nu), e + np.cos(nu), 0.0])

    cos_raan, sin_raan = np.cos(raan), np.sin(raan)
    cos_i, sin_i = np.cos(i), np.sin(i)

    # R3(-raan) R1(-i) （近点引数=0 のため R3(-argp) は省略）
    r3 = np.array([
        [cos_raan, -sin_raan, 0.0],
        [sin_raan, cos_raan, 0.0],
        [0.0, 0.0, 1.0],
    ])
    r1 = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cos_i, -sin_i],
        [0.0, sin_i, cos_i],
    ])
    rot = r3 @ r1
    pos_eci = rot @ r_pf
    vel_eci = rot @ v_pf
    return pos_eci, vel_eci


class Simulator:
    """6DoF シミュレータ本体。FSW との結合ループを ICD §1 に従い実行する。"""

    def __init__(self, scenario: ScenarioConfig, fsw: Any | None = None):
        self.cfg = scenario
        self.dt = FSW_DT
        self.t = 0.0

        pos, vel = orbital_elements_to_state(
            scenario.a_m, scenario.e, scenario.i_deg, scenario.raan_deg,
            scenario.nu0_deg, MU_EARTH,
        )
        self.pos = pos
        self.vel = vel
        self.q = scenario.q0.copy()
        self.omega = scenario.omega0_rad_s.copy()

        self.rw = ReactionWheelAssembly()
        self.camera = Camera()
        self.transmitter = Transmitter()
        self.battery = Battery(soc_init=scenario.soc_init)
        self.thermal = ThermalModel(temp_init_c=scenario.battery_temp_init_c)
        self.sensors = SensorSuite(
            rng=np.random.default_rng(scenario.seed), star_tracker=scenario.star_tracker,
        )
        self.ground = GroundStation(timeline=list(scenario.timeline))

        self.fsw = fsw

        # 外部（GSI 等）から可視フラグを駆動するためのオーバーライド。
        # None のときは既存の内蔵地上局モデル（is_visible）を使う。
        self.gs_visible_override: bool | None = None

        self._last_cmd = ActuatorCommand()
        self._mode_is_safe = True  # SAFE 起動時想定（FSW 未接続時のデフォルト）

        self.log_rows: list[dict] = []
        self._last_log_t = -1.0

        # 撮像イベント記録: 撮像成功ステップの真値評価
        # (t, 真の直下指向誤差 [deg], 真の対目標レート誤差 [deg/s])
        # AD-2/AD-3 は「撮像時点」で判定するため 10 Hz のイベントで記録する
        self.capture_events: list[dict] = []

        # OBSERVE 拡張（docs/09 §3）: shutter_open のレベル期間を 1 フレームと
        # した像流れ（真値）記録。frame_events は
        # (t_start, exposure_s, smear_px, occulted, valid) を保持する。
        self.frame_events: list[dict] = []
        self._frame_in_progress = False
        self._frame_t_start = 0.0
        self._frame_smear_rad = 0.0
        self._frame_occulted = False
        self._frame_elapsed_s = 0.0
        self._last_shutter_open = False
        self.smear_px_last = 0.0
        self.mtq_nonzero_during_exposure = 0  # V4 判定用カウンタ（露光中 mtq!=0 のステップ数）
        self._cached_shutter_open = False
        # 露光中の真の指向誤差（+Z vs 目標方向）記録（V1 判定用、S2 の
        # capture_events と同じ考え方: 撮像＝露光の実施区間で判定する）。
        self.observe_pointing_errors_deg: list[float] = []
        self._frame_max_pointing_err_deg = 0.0

        # THRUST 拡張（docs/08 §2）: 真値の累積噴射時間・推進剤消費、
        # 噴射中の真の推力軸方向誤差（body +X vs 速度方向）の記録。
        self.thrust_time_true = 0.0
        self.propellant_g_true = 0.0
        self.thrust_pointing_errors_deg: list[float] = []
        self._sat_mass_kg = 1.25
        self._mdot_kg_s = THRUST_FORCE / (THRUST_ISP * G0)
        self._cached_thruster_on = False

    # -- 補助: 環境量 -----------------------------------------------------
    def _sun_dir_eci(self) -> np.ndarray:
        return sun_direction_eci(self.t)

    def _eclipse(self) -> bool:
        return is_eclipse(self.pos, self._sun_dir_eci())

    def _b_field_eci(self) -> np.ndarray:
        return magnetic_field_eci(self.pos)

    def _gs_visible(self) -> bool:
        if self.gs_visible_override is not None:
            return self.gs_visible_override
        return is_visible(self.pos, self.t)

    # -- センサ計測 ---------------------------------------------------------
    def measure(self):
        r_body = quat_to_rot(self.q)
        b_eci = self._b_field_eci()
        sun_eci = self._sun_dir_eci()
        eclipse = self._eclipse()
        gs_vis = self._gs_visible()

        sun_body_for_power = None
        if not eclipse:
            sun_body_for_power = r_body @ sun_eci

        solar_w = solar_power_generated(r_body, sun_body_for_power, eclipse)

        wheel_speeds_true = wheel_speeds_from_momentum(self.rw.h_rw)

        sensor_data = self.sensors.measure(
            t=self.t,
            q=self.q,
            omega_true=self.omega,
            pos_eci=self.pos,
            vel_eci=self.vel,
            b_eci=b_eci,
            sun_dir_eci=sun_eci,
            wheel_speeds_true=wheel_speeds_true,
            battery_voltage=self.battery.voltage,
            battery_soc=self.battery.soc,
            solar_power=solar_w,
            battery_temp=self.thermal.temp_c,
            gs_visible=gs_vis,
        )
        self._cached_solar_w = solar_w
        self._cached_eclipse = eclipse
        self._cached_gs_visible = gs_vis
        return sensor_data

    # -- アクチュエータ適用 --------------------------------------------------
    def apply(self, cmd: ActuatorCommand) -> None:
        # 防御的クリップ（ICD 3.: シミュレータ側でも防御的にクリップ）
        wheel_torque_cmd = np.clip(cmd.wheel_torque, -RW_MAX_TORQUE, RW_MAX_TORQUE)
        mtq_cmd = np.clip(cmd.mtq_moment, -MTQ_MAX_MOMENT, MTQ_MAX_MOMENT)

        self._last_cmd = ActuatorCommand(
            wheel_torque=wheel_torque_cmd,
            mtq_moment=mtq_cmd,
            camera_trigger=cmd.camera_trigger,
            tx_active=cmd.tx_active,
            heater_on=cmd.heater_on,
            payload_power=cmd.payload_power,
            thruster_on=cmd.thruster_on,
            shutter_open=cmd.shutter_open,
        )

        # モード判定は FSW テレメトリから（SAFE で ADCS 負荷 OFF）
        if self.fsw is not None:
            tlm = self.fsw.get_telemetry()
            self._mode_is_safe = (tlm.mode == Mode.SAFE)

        # RW 更新
        body_torque_rw, h_rw = self.rw.step(wheel_torque_cmd, self.dt)
        self._cached_rw_body_torque = body_torque_rw

        # MTQ トルク（body 磁場が必要 -> measure 時にキャッシュしておく）
        r_body = quat_to_rot(self.q)
        b_body = r_body @ self._b_field_eci()
        self._cached_mtq_torque = mtq_torque(mtq_cmd, b_body)

        # カメラ
        img_success = self.camera.step(cmd.camera_trigger, cmd.payload_power)
        self._cached_img_success = img_success
        if img_success:
            # 撮像時点の真値評価を記録（AD-2/AD-3 検証用）
            # body +Z 軸の ECI 方向 = R(q) の第 3 行
            z_body_eci = r_body[2, :]
            nadir_dir = -self.pos / np.linalg.norm(self.pos)
            nadir_err_deg = float(np.degrees(
                np.arccos(np.clip(np.dot(z_body_eci, nadir_dir), -1.0, 1.0))
            ))
            # 対目標レート誤差: 軌道レート追尾からのずれ（真値）
            omega_ref_body = r_body @ (
                np.cross(self.pos, self.vel) / float(np.dot(self.pos, self.pos))
            )
            rate_err_deg_s = float(np.degrees(
                np.linalg.norm(self.omega - omega_ref_body)
            ))
            self.capture_events.append({
                "t": self.t,
                "nadir_err_deg": nadir_err_deg,
                "rate_err_deg_s": rate_err_deg_s,
            })

        # 送信機
        self.transmitter.step(
            tx_active=cmd.tx_active,
            gs_visible=self._cached_gs_visible,
            dt=self.dt,
            downlink_rate_mbps=DOWNLINK_RATE_MBPS,
            images_available=self.camera.images_stored,
        )

        # THRUST 拡張: 噴射中の真値記録（累積時間・推進剤消費・推力軸方向誤差）
        self._cached_thruster_on = bool(cmd.thruster_on)
        if cmd.thruster_on:
            self.thrust_time_true += self.dt
            self.propellant_g_true += self._mdot_kg_s * self.dt * 1000.0
            # body +X 方向の ECI 表現 = R(q)^T @ [1,0,0] = R(q) の第 1 行の転置
            # （R(q) は v_body = R(q) @ v_eci なので、R(q)^T が body->ECI）
            x_body_eci = r_body.T @ np.array([1.0, 0.0, 0.0])
            v_norm = np.linalg.norm(self.vel)
            if v_norm > 1e-6:
                vel_hat = self.vel / v_norm
                cos_err = float(np.clip(np.dot(x_body_eci, vel_hat), -1.0, 1.0))
                thrust_err_deg = float(np.degrees(np.arccos(cos_err)))
                self.thrust_pointing_errors_deg.append(thrust_err_deg)

        # OBSERVE 拡張（docs/09 §3）: shutter_open のレベル期間を 1 フレームとした
        # 像流れ（真値）記録。ω_perp は body 系角速度 ω から +Z 軸まわり成分を
        # 除いた直交成分（ロール成分は視野中心の流れに寄与しないため除外）。
        self._cached_shutter_open = bool(cmd.shutter_open)
        if cmd.shutter_open:
            if not self._frame_in_progress:
                self._frame_in_progress = True
                self._frame_t_start = self.t
                self._frame_smear_rad = 0.0
                self._frame_occulted = False
                self._frame_elapsed_s = 0.0

            omega_perp = self.omega.copy()
            omega_perp[2] = 0.0
            self._frame_smear_rad += float(np.linalg.norm(omega_perp)) * self.dt
            self._frame_elapsed_s += self.dt

            observe_mgr = getattr(self.fsw, "observe", None)
            if observe_mgr is not None:
                target_dir = observe_mgr.target_eci()
                if is_occulted(target_dir, self.pos):
                    self._frame_occulted = True
                # 真の指向誤差（+Z vs 目標方向、V1 判定用）
                z_body_eci = r_body[2, :]
                cos_err = float(np.clip(np.dot(z_body_eci, target_dir), -1.0, 1.0))
                err_deg = float(np.degrees(np.arccos(cos_err)))
                self.observe_pointing_errors_deg.append(err_deg)
                self._frame_max_pointing_err_deg = max(self._frame_max_pointing_err_deg, err_deg)

            if np.any(mtq_cmd != 0.0):
                self.mtq_nonzero_during_exposure += 1
        elif self._frame_in_progress:
            self._finalize_frame_event()

        # 電力
        load_w = total_load(
            mode_is_safe=self._mode_is_safe,
            heater_on=cmd.heater_on,
            payload_power=cmd.payload_power,
            tx_active=cmd.tx_active,
            gs_visible=self._cached_gs_visible,
            thruster_on=cmd.thruster_on,
        )
        self.battery.step(self._cached_solar_w, load_w, self.dt)
        self._cached_load_w = load_w

        # 熱
        self.thermal.step(
            illuminated=not self._cached_eclipse,
            load_w=load_w,
            heater_on=cmd.heater_on,
            dt=self.dt,
        )

    def _finalize_frame_event(self) -> None:
        """進行中のフレーム（shutter_open レベル期間）を frame_events へ確定する。

        smear_px = smear_rad / PIXEL_IFOV_RAD、valid = smear_px<1.0 and not occulted
        （docs/09 §3）。
        """
        smear_px = self._frame_smear_rad / PIXEL_IFOV_RAD
        occulted = self._frame_occulted
        valid = bool(smear_px < 1.0 and not occulted)
        self.frame_events.append({
            "t_start": self._frame_t_start,
            "exposure_s": self._frame_elapsed_s,
            "smear_px": smear_px,
            "occulted": occulted,
            "valid": valid,
            "max_pointing_err_deg": self._frame_max_pointing_err_deg,
        })
        self.smear_px_last = smear_px
        self._frame_in_progress = False
        self._frame_smear_rad = 0.0
        self._frame_elapsed_s = 0.0
        self._frame_occulted = False
        self._frame_max_pointing_err_deg = 0.0

    # -- 伝播 -----------------------------------------------------------
    def propagate(self, dt: float) -> None:
        # THRUST 拡張（docs/08 §2）: thruster_on の間、軌道運動へ摂動加速度
        # a_eci = R(q)^T @ [F/m_sat, 0, 0] を付加する（RK4 の微分項に注入、
        # ステップ内 ECI 一定近似）。R(q) は v_body=R(q)@v_eci（ECI->body）
        # なので R(q)^T が body->ECI。
        extra_accel_eci = None
        if getattr(self, "_cached_thruster_on", False):
            r_body_pre = quat_to_rot(self.q)
            accel_body = np.array([THRUST_FORCE / self._sat_mass_kg, 0.0, 0.0])
            extra_accel_eci = r_body_pre.T @ accel_body

        self.pos, self.vel = propagate_orbit(
            self.pos, self.vel, dt, j2_enabled=True, extra_accel_eci=extra_accel_eci,
        )

        # 外乱トルク
        tau_gg = gravity_gradient_torque(self.pos, self.q)
        b_body = quat_to_rot(self.q) @ self._b_field_eci()
        tau_residual = residual_dipole_torque(
            b_body, self.sensors.residual_dipole_direction * RESIDUAL_DIPOLE_MAGNITUDE,
        )
        tau_const = constant_disturbance_torque(self.sensors.disturbance_direction)
        tau_ext = (
            tau_gg + tau_residual + tau_const
            + self._cached_rw_body_torque + self._cached_mtq_torque
        )
        # THRUST 外乱トルク（docs/08 §2）: tau = [0, F*lever_arm, 0]（body Y 軸回り）
        if getattr(self, "_cached_thruster_on", False):
            tau_ext = tau_ext + np.array([0.0, THRUST_FORCE * THRUST_LEVER_ARM, 0.0])

        self.q, self.omega = propagate_attitude(
            self.q, self.omega, dt, tau_ext, self.rw.h_rw, inertia=INERTIA,
        )
        self.t += dt

    # -- ログ -------------------------------------------------------------
    def log(self, tlm: Telemetry | None) -> None:
        if self.t - self._last_log_t < 1.0 - 1e-9:
            return
        self._last_log_t = self.t

        pointing_err = tlm.pointing_error_deg if tlm is not None else 0.0
        mode = tlm.mode.value if tlm is not None else "N/A"
        q_est = tlm.q_est if tlm is not None else np.array([0.0, 0.0, 0.0, 1.0])

        # vis-viva: a = 1/(2/r - v^2/mu)（THRUST 拡張、docs/08 §2 真値記録）
        r_norm = np.linalg.norm(self.pos)
        v_norm2 = float(np.dot(self.vel, self.vel))
        sma_m = 1.0 / (2.0 / r_norm - v_norm2 / MU_EARTH)

        row = {
            "t": self.t,
            "mode": mode,
            "qx_true": self.q[0], "qy_true": self.q[1], "qz_true": self.q[2], "qw_true": self.q[3],
            "wx_true": self.omega[0], "wy_true": self.omega[1], "wz_true": self.omega[2],
            "qx_est": q_est[0], "qy_est": q_est[1], "qz_est": q_est[2], "qw_est": q_est[3],
            "pointing_err_deg": pointing_err,
            "soc": self.battery.soc,
            "batt_temp": self.thermal.temp_c,
            "batt_V": self.battery.voltage,
            "solar_W": self._cached_solar_w,
            "wx_rw": wheel_speeds_from_momentum(self.rw.h_rw)[0],
            "wy_rw": wheel_speeds_from_momentum(self.rw.h_rw)[1],
            "wz_rw": wheel_speeds_from_momentum(self.rw.h_rw)[2],
            "images_stored": self.camera.images_stored,
            "images_downlinked": self.transmitter.images_downlinked,
            "gs_visible": self._cached_gs_visible,
            "eclipse": self._cached_eclipse,
            "thrust_on": self._cached_thruster_on,
            "thrust_time_true": self.thrust_time_true,
            "propellant_g_true": self.propellant_g_true,
            "sma_m": sma_m,
            "shutter_open": getattr(self, "_cached_shutter_open", False),
            "frames_total": len(self.frame_events),
            "frames_valid": sum(1 for f in self.frame_events if f["valid"]),
            "smear_px_last": self.smear_px_last,
        }
        self.log_rows.append(row)

    def write_csv(self, path: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if not self.log_rows:
            return
        fieldnames = list(self.log_rows[0].keys())
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.log_rows:
                writer.writerow(row)

    # -- メインループ ---------------------------------------------------------
    def run(self) -> list[dict]:
        n_steps = int(round(self.cfg.duration_s / self.dt))
        for _ in range(n_steps):
            sensors = self.measure()

            if self.fsw is not None:
                cmd = self.fsw.step(sensors, self.dt)
            else:
                cmd = ActuatorCommand()

            self.apply(cmd)
            self.propagate(self.dt)

            gs_vis_after = self._cached_gs_visible
            ready_cmds = self.ground.poll(self.t, gs_vis_after)
            if self.fsw is not None:
                for gc in ready_cmds:
                    self.fsw.handle_command(gc)

            tlm = self.fsw.get_telemetry() if self.fsw is not None else None
            self.log(tlm)

        # シミュレーション終了時に露光継続中のフレームがあれば確定させる
        # （露光途中で観測窓が打ち切られた場合でも真値を記録に残す）。
        if self._frame_in_progress:
            self._finalize_frame_event()

        return self.log_rows
