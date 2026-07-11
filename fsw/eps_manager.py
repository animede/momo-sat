"""EPS 管理: SoC 監視・負荷制御・ヒータ制御。

ICD §5: battery_soc < 0.30 → SAFE（ヒステリシス、0.5 で復帰可）。
バッテリ温度 < 0 degC で heater_on=True、> 5 degC で OFF。
"""

from __future__ import annotations

SOC_SAFE_ENTER = 0.30
SOC_SAFE_EXIT = 0.50
HEATER_ON_TEMP = 0.0
HEATER_OFF_TEMP = 5.0


class EpsManager:
    """SoC ヒステリシス判定とヒータ制御ロジックを保持する。"""

    def __init__(self) -> None:
        self._low_soc_latched = False
        self._heater_on = False

    def update_low_soc(self, battery_soc: float) -> bool:
        """SoC ヒステリシスを更新し、低 SoC フォルト状態かどうかを返す。"""
        if not self._low_soc_latched:
            if battery_soc < SOC_SAFE_ENTER:
                self._low_soc_latched = True
        else:
            if battery_soc >= SOC_SAFE_EXIT:
                self._low_soc_latched = False
        return self._low_soc_latched

    @property
    def low_soc(self) -> bool:
        return self._low_soc_latched

    def update_heater(self, battery_temp: float) -> bool:
        """バッテリ温度に応じてヒータ ON/OFF をヒステリシス制御。"""
        if battery_temp < HEATER_ON_TEMP:
            self._heater_on = True
        elif battery_temp > HEATER_OFF_TEMP:
            self._heater_on = False
        return self._heater_on

    @property
    def heater_on(self) -> bool:
        return self._heater_on
