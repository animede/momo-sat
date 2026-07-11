"""センサ計測モデル（ICD §4）: ジャイロ/磁気/太陽/GPS/ホイール速度。"""

from __future__ import annotations

import numpy as np

from common.types import (
    RW_INERTIA,
    ST_MAX_RATE_DEG_S,
    ST_QUAT_SIGMA_ARCSEC,
    ST_UPDATE_HZ,
    SensorData,
)
from fsw.quat_utils import quat_mult, quat_normalize
from sim.dynamics import quat_to_rot
from sim.environment import is_eclipse

GYRO_BIAS_STD_DEG_S = 0.05   # 起動時ランダムバイアスの標準偏差 [deg/s]
GYRO_NOISE_STD_DEG_S = 0.01  # 白色ノイズ標準偏差 [deg/s]
MAG_NOISE_STD_T = 100e-9     # [T]
SUN_NOISE_STD_DEG = 0.1      # [deg]
GPS_POS_NOISE_STD_M = 10.0   # [m]
GPS_VEL_NOISE_STD_MS = 0.1   # [m/s]
ST_UPDATE_PERIOD_S = 1.0 / ST_UPDATE_HZ


class SensorSuite:
    """真値からノイズ・バイアス込みの計測値を生成する。"""

    def __init__(self, rng: np.random.Generator | None = None, star_tracker: bool = False):
        self.rng = rng if rng is not None else np.random.default_rng()
        self.star_tracker = star_tracker
        # 起動時に固定するジャイロバイアス（body 各軸、rad/s）
        self.gyro_bias = np.radians(
            self.rng.normal(0.0, GYRO_BIAS_STD_DEG_S, size=3)
        )
        # 残留磁気モーメント方向（body 固定、ランダム方向、単位ベクトル）
        direction = self.rng.normal(size=3)
        self.residual_dipole_direction = direction / np.linalg.norm(direction)

        # 定常小外乱トルクの body 固定方向（残留磁気とは独立な乱数方向）
        disturbance_dir = self.rng.normal(size=3)
        self.disturbance_direction = disturbance_dir / np.linalg.norm(disturbance_dir)

        self._last_st_t: float | None = None

    def measure(self, t: float, q: np.ndarray, omega_true: np.ndarray,
                pos_eci: np.ndarray, vel_eci: np.ndarray,
                b_eci: np.ndarray, sun_dir_eci: np.ndarray,
                wheel_speeds_true: np.ndarray,
                battery_voltage: float, battery_soc: float,
                solar_power: float, battery_temp: float,
                gs_visible: bool) -> SensorData:
        r_body = quat_to_rot(q)

        # ジャイロ: 真値 + 固定バイアス + 白色ノイズ
        gyro_noise = np.radians(self.rng.normal(0.0, GYRO_NOISE_STD_DEG_S, size=3))
        gyro_meas = omega_true + self.gyro_bias + gyro_noise

        # 磁気センサ: body 系磁場 + ノイズ
        b_body = r_body @ b_eci
        mag_meas = b_body + self.rng.normal(0.0, MAG_NOISE_STD_T, size=3)

        # 太陽センサ: 食中は None。太陽方向を body 系へ変換しノイズ付加。
        eclipse = is_eclipse(pos_eci, sun_dir_eci)
        if eclipse:
            sun_meas = None
        else:
            sun_body = r_body @ sun_dir_eci
            sun_body = sun_body / np.linalg.norm(sun_body)
            # 角度ノイズ: 接平面上に小変位を加えて再正規化する近似
            perturb = self.rng.normal(0.0, np.radians(SUN_NOISE_STD_DEG), size=3)
            perturb -= np.dot(perturb, sun_body) * sun_body
            sun_meas = sun_body + perturb
            sun_meas = sun_meas / np.linalg.norm(sun_meas)

        # GPS
        gps_pos = pos_eci + self.rng.normal(0.0, GPS_POS_NOISE_STD_M, size=3)
        gps_vel = vel_eci + self.rng.normal(0.0, GPS_VEL_NOISE_STD_MS, size=3)

        st_quat = self._measure_star_tracker(t, q, omega_true)

        return SensorData(
            t=t,
            gyro=gyro_meas,
            mag=mag_meas,
            sun=sun_meas,
            gps_pos_eci=gps_pos,
            gps_vel_eci=gps_vel,
            wheel_speeds=wheel_speeds_true.copy(),
            battery_voltage=battery_voltage,
            battery_soc=battery_soc,
            solar_power=solar_power,
            battery_temp=battery_temp,
            gs_visible=gs_visible,
            st_quat=st_quat,
        )

    def _measure_star_tracker(
        self, t: float, q: np.ndarray, omega_true: np.ndarray
    ) -> np.ndarray | None:
        """スタートラッカ計測モデル（docs/09 §3、ScenarioConfig.star_tracker 有効時のみ）。

        ST_UPDATE_HZ 間隔で st_quat = q_true (x) dq を出す（dq: 各軸
        ST_QUAT_SIGMA_ARCSEC の小角ノイズ）。|omega_true| > ST_MAX_RATE_DEG_S
        では追尾不能として None を返す（更新間隔外のステップも None）。
        """
        if not self.star_tracker:
            return None

        due = (
            self._last_st_t is None
            or (t - self._last_st_t) >= ST_UPDATE_PERIOD_S - 1e-9
        )
        if not due:
            return None
        self._last_st_t = t

        rate_deg_s = float(np.degrees(np.linalg.norm(omega_true)))
        if rate_deg_s > ST_MAX_RATE_DEG_S:
            return None

        sigma_rad = np.radians(ST_QUAT_SIGMA_ARCSEC / 3600.0)
        eps = self.rng.normal(0.0, sigma_rad, size=3)
        dq = quat_normalize(np.array([eps[0] / 2.0, eps[1] / 2.0, eps[2] / 2.0, 1.0]))
        st_quat = quat_mult(q, dq)
        return quat_normalize(st_quat)


def wheel_speeds_from_momentum(h_rw: np.ndarray) -> np.ndarray:
    """RW 角運動量 [N m s] からホイール角速度 [rad/s]（軸ごと RW_INERTIA で除算）。"""
    return h_rw / RW_INERTIA
