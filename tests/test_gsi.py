"""GSI 統合テスト（assert ベース、pytest 不使用）。

.venv/bin/python tests/test_gsi.py で実行。

実際に GsiServer を TCP で起動し、偽 SCC クライアント（テスト内で生ソケット
接続）から hello/time/link/cmd を送って ack/tlm/event の往復を検証する。
実時間を短縮するため time の speed は大きめ（60）を使う。
"""

from __future__ import annotations

import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gsi.bridge import SimBridge, build_default_scenario
from gsi.server import GsiServer

TEST_PORT = 19200
SPEED = 60.0


class FakeSccClient:
    """テスト用の偽 SCC クライアント（改行区切り JSON）。"""

    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
        self.sock.settimeout(0.5)
        self._buf = ""

    def send(self, msg: dict) -> None:
        data = (json.dumps(msg) + "\n").encode("utf-8")
        self.sock.sendall(data)

    def _drain(self) -> list[dict]:
        """届いている分だけ 1 回 recv して既知メッセージを取り出す。

        tlm は sim 時間 1Hz だが speed>1 では実時間では高頻度に届くため、
        「タイムアウトするまで読み続ける」実装だと打ち切れず呼び出し元の
        ポーリングループが実質ブロックしてしまう。1 回の recv 分だけ処理して
        すぐ返すことで、predicate 判定を挟みながら継続的に受信できるようにする。
        """
        msgs = []
        try:
            chunk = self.sock.recv(65536)
            if chunk:
                self._buf += chunk.decode("utf-8")
        except socket.timeout:
            pass
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.strip()
            if line:
                msgs.append(json.loads(line))
        return msgs

    def recv_until(self, predicate, timeout_s: float = 10.0) -> dict:
        """predicate(msg) が True になるメッセージが届くまで待つ。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for msg in self._drain():
                if predicate(msg):
                    return msg
            time.sleep(0.02)
        raise TimeoutError(f"timed out waiting for message matching predicate (timeout={timeout_s}s)")

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def _send_time(client: FakeSccClient, sim_epoch_ms: int, speed: float = SPEED) -> None:
    client.send({"type": "time", "sim_epoch_ms": sim_epoch_ms, "speed": speed})


def start_server() -> GsiServer:
    bridge = SimBridge(build_default_scenario())
    server = GsiServer(port=TEST_PORT, max_steps_per_sec=2000, verbose=False, bridge=bridge)
    server.start()
    return server


# ---------------------------------------------------------------------
# 1-3: hello 交換、time→tlm 開始（mode=SAFE）、link=false のまま cmd 受理
# ---------------------------------------------------------------------
def test_hello_time_tlm_and_cmd_without_link():
    server = start_server()
    try:
        client = FakeSccClient(TEST_PORT)
        try:
            t0_ms = int(time.time() * 1000)

            # 1. hello 交換
            client.send({"type": "hello", "who": "scc", "version": 1})
            hello_resp = client.recv_until(lambda m: m.get("type") == "hello", timeout_s=5.0)
            assert hello_resp["who"] == "momo-sat"
            assert hello_resp["version"] == 1
            assert "t0_sim_epoch_ms" in hello_resp

            # 2. time 送信 → tlm が届き始める（mode=SAFE から開始）
            _send_time(client, t0_ms)
            tlm_msg = client.recv_until(lambda m: m.get("type") == "tlm", timeout_s=10.0)
            assert tlm_msg["data"]["mode"] == "SAFE", tlm_msg["data"]["mode"]
            assert "sim_epoch_ms" in tlm_msg
            assert "lag_s" in tlm_msg

            # 3. link up=false のまま cmd ADCS_DETUMBLE → ack accepted=true
            client.send({"type": "cmd", "id": 1, "name": "ADCS_DETUMBLE", "args": ""})
            ack = client.recv_until(lambda m: m.get("type") == "ack" and m.get("id") == 1, timeout_s=5.0)
            assert ack["accepted"] is True, ack
            assert "DETUMBLE" in ack["mapped"], ack

            # FSW が DETUMBLE (または以降のモード) に入ったことを tlm で確認
            def _mode_is_detumble_or_later(m):
                if m.get("type") != "tlm":
                    return False
                return m["data"]["mode"] in ("DETUMBLE", "SUNPOINT")

            tlm2 = client.recv_until(_mode_is_detumble_or_later, timeout_s=15.0)
            assert tlm2["data"]["mode"] in ("DETUMBLE", "SUNPOINT")

            print("test_hello_time_tlm_and_cmd_without_link: PASS")
        finally:
            client.close()
    finally:
        server.stop()


# ---------------------------------------------------------------------
# 4-5: link up=true → tx 系導出が変わりうる、PLD_CAPTURE → mapped に IMAGE_NADIR
# ---------------------------------------------------------------------
def test_link_up_and_capture_command():
    server = start_server()
    try:
        client = FakeSccClient(TEST_PORT)
        try:
            t0_ms = int(time.time() * 1000)
            client.send({"type": "hello", "who": "scc", "version": 1})
            client.recv_until(lambda m: m.get("type") == "hello", timeout_s=5.0)

            _send_time(client, t0_ms)
            client.recv_until(lambda m: m.get("type") == "tlm", timeout_s=10.0)

            # link up=true
            client.send({"type": "link", "up": True, "station": "GS-TKY", "snr": 18.2})

            # cmd PLD_CAPTURE → ack に mapped="IMAGE_NADIR..." が入る
            client.send({"type": "cmd", "id": 2, "name": "PLD_CAPTURE", "args": ""})
            ack = client.recv_until(lambda m: m.get("type") == "ack" and m.get("id") == 2, timeout_s=5.0)
            assert ack["accepted"] is True, ack
            assert ack["mapped"].startswith("IMAGE_NADIR"), ack["mapped"]

            # cmd PROP_BURN → ack accepted=false
            client.send({"type": "cmd", "id": 3, "name": "PROP_BURN", "args": ""})
            ack2 = client.recv_until(lambda m: m.get("type") == "ack" and m.get("id") == 3, timeout_s=5.0)
            assert ack2["accepted"] is False, ack2

            # tlm が届き続けること（link 状態が tlm.data に反映されうる: tx_active は
            # DOWNLINK モードでない限り False のままなので、ここでは tlm 到達のみ確認）
            tlm_msg = client.recv_until(lambda m: m.get("type") == "tlm", timeout_s=10.0)
            assert "tx_active" in tlm_msg["data"]

            print("test_link_up_and_capture_command: PASS")
        finally:
            client.close()
    finally:
        server.stop()


# ---------------------------------------------------------------------
# 7: 時刻追従 — time を 2 回送って sim 時刻が前進していること
# ---------------------------------------------------------------------
def test_time_advances():
    server = start_server()
    try:
        client = FakeSccClient(TEST_PORT)
        try:
            t0_ms = int(time.time() * 1000)
            client.send({"type": "hello", "who": "scc", "version": 1})
            client.recv_until(lambda m: m.get("type") == "hello", timeout_s=5.0)

            _send_time(client, t0_ms)
            tlm1 = client.recv_until(lambda m: m.get("type") == "tlm", timeout_s=10.0)
            epoch1 = tlm1["sim_epoch_ms"]

            time.sleep(1.0)
            _send_time(client, t0_ms + int(1.0 * SPEED * 1000))

            def _epoch_advanced(m):
                return m.get("type") == "tlm" and m["sim_epoch_ms"] > epoch1

            tlm2 = client.recv_until(_epoch_advanced, timeout_s=10.0)
            assert tlm2["sim_epoch_ms"] > epoch1, (tlm2["sim_epoch_ms"], epoch1)

            print("test_time_advances: PASS")
        finally:
            client.close()
    finally:
        server.stop()


# ---------------------------------------------------------------------
# 8: 切断→再接続しても tlm が再開すること
# ---------------------------------------------------------------------
def test_reconnect_resumes_tlm():
    server = start_server()
    try:
        client1 = FakeSccClient(TEST_PORT)
        t0_ms = int(time.time() * 1000)
        client1.send({"type": "hello", "who": "scc", "version": 1})
        client1.recv_until(lambda m: m.get("type") == "hello", timeout_s=5.0)
        _send_time(client1, t0_ms)
        client1.recv_until(lambda m: m.get("type") == "tlm", timeout_s=10.0)
        client1.close()

        # サーバがシミュレーションを継続しつつ再接続を受け付けることを確認
        time.sleep(0.5)

        client2 = FakeSccClient(TEST_PORT)
        try:
            client2.send({"type": "hello", "who": "scc", "version": 1})
            hello2 = client2.recv_until(lambda m: m.get("type") == "hello", timeout_s=5.0)
            assert hello2["who"] == "momo-sat"

            _send_time(client2, t0_ms + int(2.0 * SPEED * 1000))
            tlm = client2.recv_until(lambda m: m.get("type") == "tlm", timeout_s=10.0)
            assert tlm["data"]["mode"] in ("SAFE", "DETUMBLE", "SUNPOINT")

            print("test_reconnect_resumes_tlm: PASS")
        finally:
            client2.close()
    finally:
        server.stop()


def main() -> None:
    test_hello_time_tlm_and_cmd_without_link()
    test_link_up_and_capture_command()
    test_time_advances()
    test_reconnect_resumes_tlm()
    print("\nALL TESTS PASSED")


if __name__ == "__main__":
    main()
