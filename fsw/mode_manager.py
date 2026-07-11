"""6 モード状態機械 + 自律遷移。

docs/02 §2.3, ICD §5:
起動 SAFE → 60s で自律 DETUMBLE。
|omega|<0.5 deg/s が 60 s 継続 → SUNPOINT。
撮像完了/送信完了 → SUNPOINT。
SoC<0.30 → SAFE（0.5 で復帰可）。フォルトでも SAFE。
"""

from __future__ import annotations

from common.types import Mode

SAFE_TO_DETUMBLE_TIMEOUT_S = 60.0
DETUMBLE_RATE_THRESHOLD_DEG_S = 0.5
DETUMBLE_SETTLE_TIME_S = 60.0


class ModeManager:
    """モード状態と自律遷移条件（タイマー類）を保持する。"""

    def __init__(self) -> None:
        self.mode = Mode.SAFE
        self._time_in_mode = 0.0
        self._low_rate_timer = 0.0

    def reset(self) -> None:
        self.mode = Mode.SAFE
        self._time_in_mode = 0.0
        self._low_rate_timer = 0.0

    def force_mode(self, mode: Mode) -> None:
        if mode != self.mode:
            self.mode = mode
            self._time_in_mode = 0.0
        self._low_rate_timer = 0.0

    def step_autonomy(
        self,
        dt: float,
        omega_norm_deg_s: float,
        low_soc_fault: bool,
        fault: bool,
        sequence_complete: bool,
        all_downlinked: bool,
        thrust_complete: bool = False,
        observe_complete: bool = False,
    ) -> Mode:
        """自律遷移条件を評価してモードを更新し、現在モードを返す。

        優先順位: フォルト/低 SoC > 完了イベント > タイマー遷移。
        thrust_complete: THRUST モードで累積噴射時間が目標に達したとき True
        （docs/08 §3: 累積噴射時間 >= duration_s で SUNPOINT へ自律復帰）。
        observe_complete: OBSERVE モードで観測窓（duration_s）が経過したとき True
        （docs/09 §2: 観測窓終了で SUNPOINT へ自律復帰）。
        """
        self._time_in_mode += dt

        if fault or low_soc_fault:
            self._set_mode(Mode.SAFE)
            return self.mode

        if self.mode == Mode.SAFE:
            if self._time_in_mode >= SAFE_TO_DETUMBLE_TIMEOUT_S:
                self._set_mode(Mode.DETUMBLE)

        elif self.mode == Mode.DETUMBLE:
            if omega_norm_deg_s < DETUMBLE_RATE_THRESHOLD_DEG_S:
                self._low_rate_timer += dt
            else:
                self._low_rate_timer = 0.0
            if self._low_rate_timer >= DETUMBLE_SETTLE_TIME_S:
                self._set_mode(Mode.SUNPOINT)

        elif self.mode in (Mode.NADIR_IMAGING, Mode.TARGET_IMAGING):
            if sequence_complete:
                self._set_mode(Mode.SUNPOINT)

        elif self.mode == Mode.DOWNLINK:
            if all_downlinked:
                self._set_mode(Mode.SUNPOINT)

        elif self.mode == Mode.THRUST:
            if thrust_complete:
                self._set_mode(Mode.SUNPOINT)

        elif self.mode == Mode.OBSERVE:
            if observe_complete:
                self._set_mode(Mode.SUNPOINT)

        return self.mode

    def _set_mode(self, mode: Mode) -> None:
        if mode != self.mode:
            self.mode = mode
            self._time_in_mode = 0.0
            self._low_rate_timer = 0.0

    @property
    def time_in_mode(self) -> float:
        return self._time_in_mode
