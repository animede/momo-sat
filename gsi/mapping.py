"""SCC ⇔ MOMO-SAT のコマンド/テレメトリ変換（docs/06_scc_integration.md §5-6）。

- SCC コマンド → common.types.GroundCommand（またはローカル処理のみで転送しない）
- FSW Telemetry → SCC 向け tlm dict（tx_active は FSW テレメトリに無いため
  mode==DOWNLINK かつ gs_visible から導出する）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from common.types import GroundCommand, MAX_STORED_IMAGES, Mode, Telemetry

# SCC の adcs.mode 表示名へのモード変換（docs/06 §6）
_MODE_TO_SCC = {
    Mode.SAFE: "SAFE",
    Mode.DETUMBLE: "DETUMBLE",
    Mode.SUNPOINT: "SUNPOINT",
    Mode.NADIR_IMAGING: "NADIR",
    Mode.TARGET_IMAGING: "TARGET",
    Mode.DOWNLINK: "ZENITH",
}


@dataclass
class MappingResult:
    """SCC コマンド 1 件のマッピング結果。

    ground_command が None の場合は FSW へ転送しない（非対応コマンド）。
    accepted / detail は SCC への ack にそのまま使う。
    """

    ground_command: GroundCommand | None
    accepted: bool
    mapped: str
    detail: str = ""


def map_scc_command(name: str, args: Any = "") -> MappingResult:
    """SCC コマンド名（+ args）を GroundCommand へ変換する（docs/06 §5 の表）。

    args は SCC からは通常文字列（例: "" や "3"）で届く想定。PLD_CAPTURE の
    枚数指定は "n" のような単純な文字列表現、または {"count": n} 形式の
    どちらでも解釈できるよう寛容にパースする。
    """
    name = (name or "").strip()

    if name == "ADCS_DETUMBLE":
        return MappingResult(
            GroundCommand(type="SET_MODE", params={"mode": Mode.DETUMBLE}),
            True, "SET_MODE DETUMBLE",
        )

    if name == "ADCS_MODE_SUNPOINT":
        return MappingResult(
            GroundCommand(type="SET_MODE", params={"mode": Mode.SUNPOINT}),
            True, "SET_MODE SUNPOINT",
        )

    if name == "ADCS_MODE_NADIR":
        return MappingResult(
            GroundCommand(type="SET_MODE", params={"mode": Mode.NADIR_IMAGING}),
            True, "SET_MODE NADIR_IMAGING",
        )

    if name == "PLD_CAPTURE":
        count = _parse_count(args, default=1)
        return MappingResult(
            GroundCommand(type="IMAGE_NADIR", params={"count": count}),
            True, f"IMAGE_NADIR count={count}",
        )

    if name == "PLD_STANDBY":
        return MappingResult(
            GroundCommand(type="SET_MODE", params={"mode": Mode.SUNPOINT}),
            True, "SET_MODE SUNPOINT",
        )

    if name == "COM_TX_ON":
        return MappingResult(
            GroundCommand(type="DOWNLINK", params={}),
            True, "DOWNLINK",
        )

    if name == "COM_TX_OFF":
        return MappingResult(
            GroundCommand(type="SET_MODE", params={"mode": Mode.SUNPOINT}),
            True, "SET_MODE SUNPOINT",
        )

    if name == "OBC_REBOOT":
        return MappingResult(
            GroundCommand(type="RESET", params={}),
            True, "RESET",
        )

    if name == "EPS_LOAD_SHED":
        return MappingResult(
            GroundCommand(type="SET_MODE", params={"mode": Mode.SAFE}),
            True, "SET_MODE SAFE",
        )

    if name in ("EPS_HEATER_ON", "EPS_HEATER_OFF"):
        return MappingResult(
            None, False, "",
            detail="MOMO ヒータは自律制御のため地上コマンドは無視されます",
        )

    if name in ("OBC_TIME_SYNC", "OBC_HEALTH_DUMP"):
        return MappingResult(
            None, True, "",
            detail=f"{name}: momo-sat 側では即時 tlm 反映のみ（転送なし）",
        )

    if name == "PROP_BURN":
        return MappingResult(
            None, False, "",
            detail="MOMO-SAT は推進系非搭載のためコマンドを拒否しました",
        )

    # 未知コマンド: 安全側で拒否
    return MappingResult(
        None, False, "",
        detail=f"unknown SCC command: {name!r}",
    )


def _parse_count(args: Any, default: int = 1) -> int:
    """PLD_CAPTURE の args から枚数を取り出す（寛容パース）。"""
    if args is None or args == "":
        return default
    if isinstance(args, dict):
        try:
            return int(args.get("count", default))
        except (TypeError, ValueError):
            return default
    if isinstance(args, (int, float)):
        return int(args)
    if isinstance(args, str):
        s = args.strip()
        if not s:
            return default
        try:
            return int(float(s))
        except ValueError:
            return default
    return default


def telemetry_to_dict(tlm: Telemetry, gs_visible: bool, solar_power: float = 0.0) -> dict[str, Any]:
    """FSW Telemetry を GSI の tlm.data 形式へ変換する（docs/06 §4.2, §6）。

    tx_active は FSW Telemetry に存在しないため、mode==DOWNLINK かつ
    gs_visible（GSI 側で駆動される可視フラグ）から導出する。DOWNLINK 中でも
    可視でなければ実際には送信されない（fsw/main.py 参照: tx_active =
    gs_visible の AND）ため、この導出はシミュレータの実挙動と一致する。

    solar_power は common.types.Telemetry に存在しない量（SensorData /
    Simulator._cached_solar_w 由来）のため、呼び出し側（SimBridge）から
    明示的に渡す。
    """
    q = tlm.q_est
    tx_active = bool(tlm.mode == Mode.DOWNLINK and gs_visible)

    return {
        "mode": tlm.mode.value,
        "q_est": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        "omega_est_deg_s": [
            float(x) for x in _rad_s_to_deg_s(tlm.omega_est)
        ],
        "pointing_error_deg": float(tlm.pointing_error_deg),
        "battery_soc": float(tlm.battery_soc),
        "battery_voltage": float(tlm.battery_voltage),
        "battery_temp": float(tlm.battery_temp),
        "solar_power": float(solar_power),
        "wheel_speeds": [float(x) for x in tlm.wheel_speeds],
        "images_stored": int(tlm.images_stored),
        "images_downlinked": int(tlm.images_downlinked),
        "uptime": float(tlm.uptime),
        "last_cmd": tlm.last_cmd,
        "fault_flags": int(tlm.fault_flags),
        "tx_active": tx_active,
    }


def _rad_s_to_deg_s(omega_rad_s):
    import numpy as np
    return np.degrees(np.asarray(omega_rad_s, dtype=float))


def mode_to_scc(mode: Mode) -> str:
    """SCC adcs.mode 表示名（docs/06 §6 の変換表）。"""
    return _MODE_TO_SCC.get(mode, mode.value)


def images_stored_pct(images_stored: int) -> float:
    """cdh.storagePct = images_stored / MAX_STORED_IMAGES * 100。"""
    return float(images_stored) / float(MAX_STORED_IMAGES) * 100.0
