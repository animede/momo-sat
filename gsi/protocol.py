"""GSI プロトコル: 改行区切り JSON（UTF-8）の encode/decode とメッセージ検証。

仕様: docs/06_scc_integration.md §4

フレーミング規約:
    各メッセージは 1 行の JSON オブジェクト + "\n"。
    受信側はソケットバッファを "\n" で分割してデコードする（StreamDecoder）。
"""

from __future__ import annotations

import json
from typing import Any

# SCC → momo-sat で許容するメッセージ type
VALID_INBOUND_TYPES = {"hello", "time", "link", "cmd"}
# momo-sat → SCC で送出する type
VALID_OUTBOUND_TYPES = {"hello", "ack", "tlm", "event"}


class ProtocolError(ValueError):
    """メッセージが GSI プロトコルとして不正な場合の例外。"""


def encode(msg: dict[str, Any]) -> bytes:
    """1 メッセージを改行終端 JSON バイト列にエンコードする。"""
    return (json.dumps(msg, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")


def decode_line(line: str) -> dict[str, Any]:
    """1 行（改行なし、または付いていても可）を JSON デコードして検証する。"""
    line = line.strip()
    if not line:
        raise ProtocolError("empty line")
    try:
        msg = json.loads(line)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"invalid JSON: {exc}") from exc
    if not isinstance(msg, dict):
        raise ProtocolError("message must be a JSON object")
    if "type" not in msg:
        raise ProtocolError("message missing 'type'")
    return msg


def validate_inbound(msg: dict[str, Any]) -> None:
    """SCC → momo-sat メッセージの最小フィールド検証（型不一致は寛容に扱う）。"""
    t = msg.get("type")
    if t not in VALID_INBOUND_TYPES:
        raise ProtocolError(f"unknown inbound type: {t!r}")
    if t == "hello":
        pass  # who/version は無くても許容
    elif t == "time":
        if "sim_epoch_ms" not in msg:
            raise ProtocolError("'time' message missing 'sim_epoch_ms'")
    elif t == "link":
        if "up" not in msg:
            raise ProtocolError("'link' message missing 'up'")
    elif t == "cmd":
        if "name" not in msg:
            raise ProtocolError("'cmd' message missing 'name'")


class StreamDecoder:
    """ソケットから受信したバイト断片を蓄積し、完成した行ごとにメッセージを取り出す。"""

    def __init__(self) -> None:
        self._buf = ""

    def feed(self, data: bytes) -> list[dict[str, Any]]:
        """受信バイト列を追加し、デコードできた完全な行のメッセージ一覧を返す。

        不正な行は ProtocolError として個別に送出せず、呼び出し側が
        line ごとに decode_line を呼べるよう、ここでは例外を握りつぶさない
        方針とする（bridge/server 側で try/except して継続する）。
        """
        self._buf += data.decode("utf-8", errors="replace")
        messages: list[dict[str, Any]] = []
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            msg = decode_line(line)
            validate_inbound(msg)
            messages.append(msg)
        return messages
