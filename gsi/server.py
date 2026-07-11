"""GsiServer: TCP サーバ（既定ポート 9200）。

SCC（管制システム）を単一クライアントとして受け付け、hello/time/link/cmd を
処理し、1 Hz で tlm・主要イベントを push する。詳細プロトコル: docs/06 §4。

設計方針:
- 受信・送信・シミュレーションペーシングをそれぞれ専用スレッドで駆動する
  スレッドベース実装（依存追加不可のため asyncio でなく threading を採用）。
- 例外でサーバプロセスが落ちないよう、クライアント処理・JSON パースの
  失敗はすべて warning ログのみで継続する。
- 切断時はシミュレーションを継続し、新規接続を受け付け続ける
  （docs/06 §4.2「切断時: momo-sat はシミュレーションを継続」）。
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time as time_mod
from typing import Any

from common.types import GroundCommand
from gsi.bridge import SimBridge
from gsi.mapping import map_scc_command
from gsi.protocol import ProtocolError, StreamDecoder, encode

logger = logging.getLogger("gsi.server")

PROTOCOL_VERSION = 1
DEFAULT_PORT = 9200
DEFAULT_MAX_STEPS_PER_SEC = 2000
TLM_PERIOD_SIM_S = 1.0  # tlm push 周期（シミュレーション時間基準）


class _ClientConn:
    """接続 1 件ぶんの送受信状態。送信はロックで直列化する。"""

    def __init__(self, sock: socket.socket, addr: Any) -> None:
        self.sock = sock
        self.addr = addr
        self.send_lock = threading.Lock()
        self.alive = True

    def send(self, msg: dict[str, Any]) -> bool:
        if not self.alive:
            return False
        try:
            data = encode(msg)
            with self.send_lock:
                self.sock.sendall(data)
            return True
        except OSError:
            self.alive = False
            return False

    def close(self) -> None:
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass


class GsiServer:
    """momo-sat 側 GSI TCP サーバ本体。"""

    def __init__(
        self,
        port: int = DEFAULT_PORT,
        max_steps_per_sec: int = DEFAULT_MAX_STEPS_PER_SEC,
        verbose: bool = False,
        bridge: SimBridge | None = None,
    ) -> None:
        self.port = port
        self.max_steps_per_sec = max_steps_per_sec
        self.verbose = verbose
        self.bridge = bridge if bridge is not None else SimBridge()

        self._server_sock: socket.socket | None = None
        self._stop_event = threading.Event()
        self._accept_thread: threading.Thread | None = None
        self._pacing_thread: threading.Thread | None = None

        self._client_lock = threading.Lock()
        self._client: _ClientConn | None = None

        # 時刻同期状態（docs/06 §4.3）
        self._time_lock = threading.Lock()
        self._have_time = False
        self._t0_sim_epoch_ms: int | None = None   # 最初の time で t=0 に対応付け
        self._last_scc_epoch_ms: int = 0
        self._last_scc_recv_wall: float = 0.0
        self._speed: float = 1.0

        self._last_mode = None
        self._last_tlm_push_sim_t = -1.0

        if self.verbose:
            logging.basicConfig(level=logging.DEBUG)
        else:
            logging.basicConfig(level=logging.INFO)

    # -- ライフサイクル -------------------------------------------------------
    def start(self) -> None:
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("0.0.0.0", self.port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(0.5)

        self._stop_event.clear()
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._pacing_thread = threading.Thread(target=self._pacing_loop, daemon=True)
        self._accept_thread.start()
        self._pacing_thread.start()
        logger.info("GSI server listening on :%d", self.port)

    def stop(self) -> None:
        self._stop_event.set()
        with self._client_lock:
            if self._client is not None:
                self._client.close()
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=2.0)
        if self._pacing_thread is not None:
            self._pacing_thread.join(timeout=2.0)
        logger.info("GSI server stopped")

    def wait_forever(self) -> None:
        try:
            while not self._stop_event.is_set():
                time_mod.sleep(0.2)
        except KeyboardInterrupt:
            pass

    # -- 接続受付 -----------------------------------------------------------
    def _accept_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                sock, addr = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with self._client_lock:
                if self._client is not None:
                    # 単一接続前提。既存接続があれば古い方を切って新規を受ける。
                    self._client.close()
                conn = _ClientConn(sock, addr)
                self._client = conn

            logger.info("client connected: %s", addr)
            t = threading.Thread(target=self._recv_loop, args=(conn,), daemon=True)
            t.start()

    def _recv_loop(self, conn: _ClientConn) -> None:
        decoder = StreamDecoder()
        conn.sock.settimeout(1.0)
        try:
            while not self._stop_event.is_set() and conn.alive:
                try:
                    data = conn.sock.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                try:
                    messages = decoder.feed(data)
                except ProtocolError as exc:
                    logger.warning("protocol error from %s: %s", conn.addr, exc)
                    continue
                for msg in messages:
                    try:
                        self._handle_message(conn, msg)
                    except Exception:  # noqa: BLE001
                        logger.exception("error handling message: %r", msg)
        finally:
            conn.alive = False
            with self._client_lock:
                if self._client is conn:
                    self._client = None
            logger.info("client disconnected: %s", conn.addr)

    # -- メッセージ処理 -------------------------------------------------------
    def _handle_message(self, conn: _ClientConn, msg: dict[str, Any]) -> None:
        mtype = msg.get("type")

        if mtype == "hello":
            with self._time_lock:
                t0 = self._t0_sim_epoch_ms if self._t0_sim_epoch_ms is not None else 0
            conn.send({
                "type": "hello",
                "who": "momo-sat",
                "version": PROTOCOL_VERSION,
                "t0_sim_epoch_ms": t0,
            })

        elif mtype == "time":
            self._handle_time(msg)

        elif mtype == "link":
            up = bool(msg.get("up", False))
            self.bridge.set_link(up)
            if self.verbose:
                logger.debug("link up=%s station=%s snr=%s", up, msg.get("station"), msg.get("snr"))

        elif mtype == "cmd":
            self._handle_cmd(conn, msg)

        else:
            logger.warning("unknown message type: %r", mtype)

    def _handle_time(self, msg: dict[str, Any]) -> None:
        epoch_ms = int(msg.get("sim_epoch_ms", 0))
        speed = float(msg.get("speed", 1.0))
        with self._time_lock:
            if self._t0_sim_epoch_ms is None:
                self._t0_sim_epoch_ms = epoch_ms
                self._have_time = True
            self._last_scc_epoch_ms = epoch_ms
            self._last_scc_recv_wall = time_mod.monotonic()
            self._speed = speed if speed > 0 else 1.0

    def _handle_cmd(self, conn: _ClientConn, msg: dict[str, Any]) -> None:
        cmd_id = msg.get("id")
        name = str(msg.get("name", ""))
        args = msg.get("args", "")

        result = map_scc_command(name, args)

        if result.ground_command is not None:
            self.bridge.inject_command(result.ground_command)

        conn.send({
            "type": "ack",
            "id": cmd_id,
            "accepted": result.accepted,
            "mapped": result.mapped,
            "detail": result.detail,
        })

        if result.ground_command is None:
            # 非転送コマンドも event で通知する（docs/06 §5 の注記）
            self._push_event(conn, "info" if result.accepted else "warn",
                              f"{name}: {result.detail}" if result.detail else name)

        if name == "OBC_HEALTH_DUMP":
            self._push_tlm(conn)

    # -- ペーシングループ（時刻追従 + tlm/event push） -----------------------------
    def _pacing_loop(self) -> None:
        last_wall = time_mod.monotonic()
        while not self._stop_event.is_set():
            time_mod.sleep(0.05)
            now = time_mod.monotonic()
            wall_dt = now - last_wall
            last_wall = now

            with self._time_lock:
                have_time = self._have_time
                t0 = self._t0_sim_epoch_ms
                last_epoch_ms = self._last_scc_epoch_ms
                last_recv_wall = self._last_scc_recv_wall
                speed = self._speed

            if not have_time or t0 is None:
                # time 未受信の間は歩進しない（仕様どおり）。
                continue

            # SCC の目標時刻を実時間経過分だけ外挿する
            # （time は 1Hz 送信のため、次の time までは speed で線形補外）。
            elapsed_since_recv = max(0.0, now - last_recv_wall)
            target_epoch_ms = last_epoch_ms + elapsed_since_recv * speed * 1000.0
            target_sim_t = (target_epoch_ms - t0) / 1000.0

            try:
                max_steps = max(1, int(self.max_steps_per_sec * wall_dt))
                self.bridge.step_to(target_sim_t, max_steps=max_steps)

                self._maybe_push_periodic()
            except Exception:
                # ペーシングスレッドは衛星シミュレーションの心臓部。
                # 一過性の例外（接続断直後の push 失敗等）で止めず、記録して継続する
                logger.exception("pacing loop error (continuing)")

    def _maybe_push_periodic(self) -> None:
        sim_t = self.bridge.get_sim_t()
        if sim_t - self._last_tlm_push_sim_t + 1e-9 < TLM_PERIOD_SIM_S:
            return
        self._last_tlm_push_sim_t = sim_t

        conn = self._current_conn()
        mode = self.bridge.get_mode()
        if self._last_mode is not None and mode != self._last_mode:
            self._push_event(conn, "info", f"mode transition: {self._last_mode.value} -> {mode.value}")
        self._last_mode = mode

        if conn is not None:
            self._push_tlm(conn)

    def _current_conn(self) -> _ClientConn | None:
        with self._client_lock:
            return self._client

    def _push_tlm(self, conn: _ClientConn | None) -> None:
        if conn is None:
            conn = self._current_conn()
        if conn is None:
            return
        with self._time_lock:
            t0 = self._t0_sim_epoch_ms

        sim_t = self.bridge.get_sim_t()
        sim_epoch_ms = int(t0 + sim_t * 1000.0) if t0 is not None else 0

        with self._time_lock:
            last_epoch_ms = self._last_scc_epoch_ms
            last_recv_wall = self._last_scc_recv_wall
            speed = self._speed
        now = time_mod.monotonic()
        elapsed_since_recv = max(0.0, now - last_recv_wall)
        target_epoch_ms = last_epoch_ms + elapsed_since_recv * speed * 1000.0
        lag_s = max(0.0, (target_epoch_ms - sim_epoch_ms) / 1000.0) if t0 is not None else 0.0

        data = self.bridge.get_tlm_dict()
        conn.send({
            "type": "tlm",
            "sim_epoch_ms": sim_epoch_ms,
            "lag_s": lag_s,
            "data": data,
        })

    def _push_event(self, conn: _ClientConn | None, level: str, text: str) -> None:
        if conn is None:
            conn = self._current_conn()
        if conn is None:
            return
        conn.send({
            "type": "event",
            "level": level,
            "source": "FSW",
            "text": text,
        })
