"""FlightSoftware: FSW 公開 API エントリポイント（ICD §2 準拠）。

class FlightSoftware:
    def __init__(self, config: dict | None = None): ...
    def step(self, sensors: SensorData, dt: float) -> ActuatorCommand: ...
    def handle_command(self, cmd: GroundCommand) -> None: ...
    def get_telemetry(self) -> Telemetry: ...
"""

from __future__ import annotations

import numpy as np

from common.types import (
    MTQ_MAX_MOMENT,
    RW_MAX_TORQUE,
    ActuatorCommand,
    GroundCommand,
    Mode,
    SensorData,
    Telemetry,
)
from fsw.adcs import controller, guidance
from fsw.adcs.estimator import AttitudeEstimator, sun_direction_eci
from fsw.comm_manager import CommManager
from fsw.datastore import DataStore
from fsw.eps_manager import EpsManager
from fsw.mode_manager import ModeManager
from fsw.observe_manager import ObserveManager
from fsw.payload_manager import PayloadManager
from fsw.quat_utils import angle_between_quats_deg, quat_to_rot
from fsw.thrust_manager import ThrustManager

# フォルトフラグビット
FAULT_STEP_EXCEPTION = 1 << 0
FAULT_SENSOR_NAN = 1 << 1


class FlightSoftware:
    """MOMO-SAT フライトソフトウェアのトップレベルクラス。"""

    def __init__(self, config: dict | None = None) -> None:
        cfg = config or {}

        self.mode_mgr = ModeManager()
        self.estimator = AttitudeEstimator()
        self.eps = EpsManager()
        self.payload = PayloadManager()
        self.comm = CommManager()
        self.datastore = DataStore()
        self.thrust = ThrustManager()
        self.observe = ObserveManager()

        # 制御ゲイン（config で上書き可能）
        self.k_bdot = float(cfg.get("k_bdot", controller.K_BDOT))
        self.kp = float(cfg.get("kp", controller.KP))
        self.kd = float(cfg.get("kd", controller.KD))
        self.k_desat = float(cfg.get("k_desat", controller.K_DESAT))

        self.uptime = 0.0
        self.fault_flags = 0
        self.last_cmd = ""

        self._prev_mag_body: np.ndarray | None = None
        self._q_target = np.array([0.0, 0.0, 0.0, 1.0])
        self._pointing_error_deg = 0.0

        self._tlm = Telemetry()

    # ------------------------------------------------------------------
    # 公開 API
    # ------------------------------------------------------------------

    def step(self, sensors: SensorData, dt: float) -> ActuatorCommand:
        """1 制御ステップ実行。例外は内部で捕捉し SAFE 遷移して安全側の指令を返す。"""
        try:
            return self._step_impl(sensors, dt)
        except Exception:
            self.fault_flags |= FAULT_STEP_EXCEPTION
            self.mode_mgr.force_mode(Mode.SAFE)
            self._tlm.fault_flags = self.fault_flags
            return ActuatorCommand()

    def handle_command(self, cmd: GroundCommand) -> None:
        """地上コマンドを解釈しモード・パラメータへ反映する。"""
        try:
            self._handle_command_impl(cmd)
        except Exception:
            self.fault_flags |= FAULT_STEP_EXCEPTION

    def get_telemetry(self) -> Telemetry:
        return self._tlm

    # ------------------------------------------------------------------
    # 内部実装
    # ------------------------------------------------------------------

    def _step_impl(self, sensors: SensorData, dt: float) -> ActuatorCommand:
        self.uptime += dt

        sensor_fault = self._check_sensor_nan(sensors)
        if sensor_fault:
            self.fault_flags |= FAULT_SENSOR_NAN

        # --- 姿勢決定（フォルトでもジャイロ以外を使わないようベストエフォート） ---
        gyro = sensors.gyro if not sensor_fault else np.zeros(3)
        mag_body = sensors.mag if np.all(np.isfinite(sensors.mag)) else np.zeros(3)
        sun_body = sensors.sun if (sensors.sun is not None and np.all(np.isfinite(sensors.sun))) else None
        pos_eci = sensors.gps_pos_eci if np.all(np.isfinite(sensors.gps_pos_eci)) else np.zeros(3)
        vel_eci = sensors.gps_vel_eci if np.all(np.isfinite(sensors.gps_vel_eci)) else np.zeros(3)

        st_quat = sensors.st_quat if (sensors.st_quat is not None and np.all(np.isfinite(sensors.st_quat))) else None

        q_est, omega_est = self.estimator.update(
            gyro, mag_body, sun_body, sensors.t, pos_eci, dt, st_quat=st_quat
        )
        omega_norm_deg_s = float(np.degrees(np.linalg.norm(omega_est)))

        # --- EPS 判定 ---
        low_soc_fault = self.eps.update_low_soc(sensors.battery_soc)
        heater_on = self.eps.update_heater(sensors.battery_temp)

        # --- モード自律遷移 ---
        sequence_complete = self.payload.sequence_complete()
        all_downlinked = self.comm.all_downlinked(self.payload.images_stored)
        thrust_complete = self.thrust.complete()
        observe_complete = self.observe.complete()
        mode = self.mode_mgr.step_autonomy(
            dt,
            omega_norm_deg_s,
            low_soc_fault,
            sensor_fault,
            sequence_complete,
            all_downlinked,
            thrust_complete,
            observe_complete,
        )

        # --- 誘導: モードに応じた目標クォータニオン + 目標角速度 ---
        self._q_target = self._compute_guidance(mode, sensors.t, pos_eci, vel_eci)
        self._pointing_error_deg = angle_between_quats_deg(self._q_target, q_est)
        omega_target = self._compute_omega_target(mode, pos_eci, vel_eci, q_est)

        # --- 制御則: モードごとの wheel_torque / mtq_moment ---
        wheel_torque, mtq_moment = self._compute_control(
            mode, mag_body, sensors.wheel_speeds, q_est, omega_est, omega_target, dt
        )

        # --- ペイロード / 通信 / ヒータ 出力生成 ---
        camera_trigger = False
        payload_power = False
        if mode in (Mode.NADIR_IMAGING, Mode.TARGET_IMAGING):
            payload_power = True
            # 撮像レート条件は「目標追尾角速度に対する誤差」で判定する
            # （直下追尾では軌道レート ~0.064 deg/s の機体回転が正常状態のため）
            rate_error_deg_s = float(
                np.degrees(np.linalg.norm(omega_est - omega_target))
            )
            camera_trigger = self.payload.try_trigger(
                sensors.t, self._pointing_error_deg, rate_error_deg_s
            )

        tx_active = False
        if mode == Mode.DOWNLINK:
            tx_active = bool(sensors.gs_visible)
            self.comm.update_tx(tx_active, dt, self.payload.images_stored)

        thruster_on = False
        if mode == Mode.THRUST:
            thruster_on = self.thrust.evaluate(
                self._pointing_error_deg,
                sensors.solar_power,
                sensors.battery_soc,
                sensors.wheel_speeds,
                dt,
            )

        shutter_open = False
        if mode == Mode.OBSERVE:
            rate_error_deg_s = float(
                np.degrees(np.linalg.norm(omega_est - omega_target))
            )
            shutter_open = self.observe.step(
                dt, self._pointing_error_deg, rate_error_deg_s, pos_eci, sensors.wheel_speeds,
            )
            # 露光中の payload 電力負荷（docs/09 §3: 露光中 1.5W、既存の
            # payload_power 負荷と同扱い）。
            payload_power = shutter_open
            # 露光中の静穏化（docs/09 §2）: shutter_open 中は mtq_moment=0
            # （脱飽和はフレームギャップ・条件不成立中のみ許可）。
            if shutter_open:
                mtq_moment = np.zeros(3)

        # 観測窓完了時、機上スタック済み成果物 1 枚として images_stored+1。
        # mode_mgr.step_autonomy() が同一ステップ内で OBSERVE→SUNPOINT へ
        # 遷移させるため、この判定は mode==Mode.OBSERVE のブロック外（かつ
        # エッジトリガ）で行う必要がある（THRUST の thrust_complete と対称）。
        if self.observe.consume_completion_edge():
            self.payload.images_stored += 1

        cmd = ActuatorCommand(
            wheel_torque=wheel_torque,
            mtq_moment=mtq_moment,
            camera_trigger=camera_trigger,
            tx_active=tx_active,
            heater_on=heater_on,
            payload_power=payload_power,
            thruster_on=thruster_on,
            shutter_open=shutter_open,
        )

        self._prev_mag_body = mag_body.copy()

        self._update_telemetry(sensors, q_est, omega_est)
        return cmd

    def _compute_guidance(
        self, mode: Mode, t: float, pos_eci: np.ndarray, vel_eci: np.ndarray
    ) -> np.ndarray:
        if mode == Mode.SUNPOINT:
            sun_eci = sun_direction_eci(t)
            return guidance.sunpoint_quat(sun_eci)
        if mode in (Mode.NADIR_IMAGING, Mode.THRUST):
            # THRUST: body +Z→地心、+X→速度方向（プログレード噴射）。
            # nadir_quat をそのまま再利用する（docs/08 §3）。
            return guidance.nadir_quat(pos_eci, vel_eci)
        if mode == Mode.TARGET_IMAGING:
            if self.payload.target_latlon is not None:
                lat, lon = self.payload.target_latlon
                target_ecef = guidance.target_ecef_from_latlon(lat, lon)
                return guidance.target_quat(pos_eci, target_ecef, t)
            return guidance.nadir_quat(pos_eci, vel_eci)
        if mode == Mode.DOWNLINK:
            return guidance.downlink_quat(pos_eci)
        if mode == Mode.OBSERVE:
            return guidance.observe_quat(self.observe.target_eci())
        # SAFE / DETUMBLE: 目標姿勢なし（恒等）
        return np.array([0.0, 0.0, 0.0, 1.0])

    @staticmethod
    def _compute_omega_target(
        mode: Mode, pos_eci: np.ndarray, vel_eci: np.ndarray, q_est: np.ndarray
    ) -> np.ndarray:
        """目標角速度（body 系）を返す。

        地心系追尾モード（NADIR/TARGET/DOWNLINK）では目標姿勢が軌道レートで
        回転するため、軌道角速度 ω_orb = (r×v)/|r|^2 を body 系に変換して
        フィードフォワードする。SUNPOINT の目標は準慣性固定なのでゼロ。
        """
        if mode not in (Mode.NADIR_IMAGING, Mode.TARGET_IMAGING, Mode.DOWNLINK, Mode.THRUST):
            return np.zeros(3)
        r2 = float(np.dot(pos_eci, pos_eci))
        if r2 < 1.0:
            return np.zeros(3)
        omega_orb_eci = np.cross(pos_eci, vel_eci) / r2
        return quat_to_rot(q_est) @ omega_orb_eci

    def _compute_control(
        self,
        mode: Mode,
        mag_body: np.ndarray,
        wheel_speeds: np.ndarray,
        q_est: np.ndarray,
        omega_est: np.ndarray,
        omega_target: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        wheel_torque = np.zeros(3)
        mtq_moment = np.zeros(3)

        if mode in (Mode.SAFE, Mode.DETUMBLE):
            prev = self._prev_mag_body if self._prev_mag_body is not None else mag_body
            mtq_moment = controller.bdot_moment(prev, mag_body, dt, self.k_bdot)
        else:
            wheel_torque = controller.pd_control(
                self._q_target, q_est, omega_est,
                omega_target=omega_target, kp=self.kp, kd=self.kd,
            )
            mtq_moment = controller.rw_desaturate(wheel_speeds, mag_body, self.k_desat)

        wheel_torque = np.clip(wheel_torque, -RW_MAX_TORQUE, RW_MAX_TORQUE)
        mtq_moment = np.clip(mtq_moment, -MTQ_MAX_MOMENT, MTQ_MAX_MOMENT)
        return wheel_torque, mtq_moment

    def _handle_command_impl(self, cmd: GroundCommand) -> None:
        self.last_cmd = cmd.type

        if cmd.type == "SET_MODE":
            mode = cmd.params.get("mode")
            if isinstance(mode, str):
                mode = Mode(mode)
            if isinstance(mode, Mode):
                self.mode_mgr.force_mode(mode)

        elif cmd.type == "IMAGE_NADIR":
            count = int(cmd.params.get("count", 1))
            self.payload.request_nadir(count)
            self.mode_mgr.force_mode(Mode.NADIR_IMAGING)

        elif cmd.type == "IMAGE_TARGET":
            lat = float(cmd.params.get("lat_deg", 0.0))
            lon = float(cmd.params.get("lon_deg", 0.0))
            self.payload.request_target(lat, lon)
            self.mode_mgr.force_mode(Mode.TARGET_IMAGING)

        elif cmd.type == "DOWNLINK":
            self.comm.reset_progress()
            self.mode_mgr.force_mode(Mode.DOWNLINK)

        elif cmd.type == "THRUST":
            duration_s = float(cmd.params.get("duration_s", 0.0))
            self.thrust.request(duration_s)
            self.mode_mgr.force_mode(Mode.THRUST)

        elif cmd.type == "OBSERVE":
            ra_deg = float(cmd.params.get("ra_deg", 0.0))
            dec_deg = float(cmd.params.get("dec_deg", 0.0))
            duration_s = float(cmd.params.get("duration_s", 0.0))
            exposure_s = float(cmd.params.get("exposure_s", 1.0))
            self.observe.request(ra_deg, dec_deg, duration_s, exposure_s)
            self.mode_mgr.force_mode(Mode.OBSERVE)

        elif cmd.type == "RESET":
            self.mode_mgr.force_mode(Mode.SAFE)
            self.fault_flags = 0

    def _check_sensor_nan(self, sensors: SensorData) -> bool:
        arrays = [sensors.gyro, sensors.mag, sensors.gps_pos_eci, sensors.gps_vel_eci, sensors.wheel_speeds]
        for arr in arrays:
            if arr is None or not np.all(np.isfinite(arr)):
                return True
        if sensors.sun is not None and not np.all(np.isfinite(sensors.sun)):
            return True
        scalars = [sensors.battery_voltage, sensors.battery_soc, sensors.solar_power, sensors.battery_temp, sensors.t]
        for s in scalars:
            if not np.isfinite(s):
                return True
        return False

    def _update_telemetry(self, sensors: SensorData, q_est: np.ndarray, omega_est: np.ndarray) -> None:
        self._tlm = Telemetry(
            t=sensors.t,
            mode=self.mode_mgr.mode,
            q_est=q_est.copy(),
            omega_est=omega_est.copy(),
            pointing_error_deg=self._pointing_error_deg,
            battery_soc=sensors.battery_soc,
            battery_voltage=sensors.battery_voltage,
            battery_temp=sensors.battery_temp,
            wheel_speeds=sensors.wheel_speeds.copy(),
            images_stored=self.payload.images_stored,
            images_downlinked=self.comm.images_downlinked,
            uptime=self.uptime,
            last_cmd=self.last_cmd,
            fault_flags=self.fault_flags,
            thrust_time=self.thrust.thrust_time,
            propellant_used_g=self.thrust.propellant_used_g,
            frames_captured=self.observe.frames_captured,
            observe_time=self.observe.observe_time,
        )
        self.datastore.record(self._tlm)
