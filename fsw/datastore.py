"""TLM リングバッファ（直近 6000 サンプル保持）。

docs/02 §2.2, DH-4（テレメトリ蓄積: 最新 1 軌道分以上）対応。
1 軌道 ≈ 5700 s、1 Hz 記録想定なら 6000 サンプルで 1 軌道以上をカバーする。
"""

from __future__ import annotations

from collections import deque

from common.types import Telemetry

TLM_BUFFER_SIZE = 6000


class DataStore:
    """Telemetry スナップショットをリングバッファで保持する。"""

    def __init__(self, maxlen: int = TLM_BUFFER_SIZE) -> None:
        self._buf: deque[Telemetry] = deque(maxlen=maxlen)

    def record(self, tlm: Telemetry) -> None:
        self._buf.append(tlm)

    def latest(self) -> Telemetry | None:
        if not self._buf:
            return None
        return self._buf[-1]

    def history(self) -> list[Telemetry]:
        return list(self._buf)

    def __len__(self) -> int:
        return len(self._buf)
