"""熱モデル（ICD §4）: バッテリ 1 節点熱モデル。

C dT/dt = Q_solar + Q_internal + Q_heater - sigma * eps * A * (T^4 - T_space^4)

eps*A は初期温度 10 degC 近傍で、定常負荷相当の熱入力と平衡するよう較正する
（「初期温度 20℃ 付近で平衡」との仕様指示に基づき、定常照射+平均負荷条件で
 平衡温度が概ね 20 degC 付近になるように eps*A を選定）。
"""

from __future__ import annotations

SIGMA_SB = 5.670374419e-8  # Stefan-Boltzmann [W/m^2/K^4]
T_SPACE_K = 3.0            # 宇宙背景温度 [K]

C_THERMAL = 200.0          # 熱容量 [J/K]
Q_SOLAR_ILLUMINATED_W = 3.0
INTERNAL_HEAT_FRACTION = 0.5  # 負荷の何割が内部発熱として寄与するか

# 放射結合定数 eps*A [m^2]。平均照射(duty~0.6)+平均負荷~1.5W(bus+adcs)+heater無しの
# 条件（内部発熱 0.5*1.5=0.75W、太陽入熱 0.6*3.0=1.8W、合計 2.55W）で
# 平衡温度が約 20 degC (293.15K) 付近になるよう較正。
EPS_AREA = 6.09e-3


def celsius_to_kelvin(t_c: float) -> float:
    return t_c + 273.15


def kelvin_to_celsius(t_k: float) -> float:
    return t_k - 273.15


class ThermalModel:
    """バッテリ 1 節点熱モデル。"""

    def __init__(self, temp_init_c: float = 10.0):
        self.temp_c = temp_init_c

    def step(self, illuminated: bool, load_w: float, heater_on: bool, dt: float) -> None:
        t_k = celsius_to_kelvin(self.temp_c)

        q_solar = Q_SOLAR_ILLUMINATED_W if illuminated else 0.0
        q_internal = INTERNAL_HEAT_FRACTION * load_w
        q_heater = 1.0 if heater_on else 0.0
        q_radiated = SIGMA_SB * EPS_AREA * (t_k ** 4 - T_SPACE_K ** 4)

        dt_dt = (q_solar + q_internal + q_heater - q_radiated) / C_THERMAL
        t_k_new = t_k + dt_dt * dt
        self.temp_c = kelvin_to_celsius(t_k_new)
