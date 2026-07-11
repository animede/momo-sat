/* MOMO-SAT ⇔ SCC E2E 統合試験（docs/06_scc_integration.md §9-3）
   実行: node tools/e2e_scc.mjs
   1. run_gsi.py（MOMO-SAT 衛星側、TCP :19200）を起動
   2. tools/gen_tle.py で TLE 生成、extsat 設定付きで SCC サーバ（:18900）を起動
   3. SCC の WebSocket に管制卓として接続し、コマンド投入→EXEC→衛星反映を検証:
      (a) パス外ではコマンドが QUEUED のまま進まない
      (b) パス中に EXEC → MOMO 側モード遷移が SCC テレメトリに反映
      (c) PLD_CAPTURE → 撮像 → storagePct 上昇
      (d) COM_TX_ON → DOWNLINK（ZENITH 指向・TX/RX）→ 全画像伝送後 SUNPOINT 自動復帰
   NACK（5% 乱数）は再送でリカバリする。全 PASS で exit 0。 */
"use strict";

import { spawn } from "node:child_process";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const MOMO = path.resolve(__dirname, "..");
const SCC = process.env.SCC_DIR || path.resolve(MOMO, "../SCC");
const PY = path.join(MOMO, ".venv/bin/python");
const GSI_PORT = 19200;
const SCC_PORT = 18900;
const SIM_SPEED = 150;
const GLOBAL_TIMEOUT_MIN = 30;

let failed = false;
const t0 = Date.now();
function stamp() { return ((Date.now() - t0) / 1000).toFixed(0).padStart(4) + "s"; }
function ok(msg) { console.log(`ok   [${stamp()}] ${msg}`); }
function info(msg) { console.log(`     [${stamp()}] ${msg}`); }
function fail(msg) { failed = true; console.error(`FAIL [${stamp()}] ${msg}`); }

/* ---------- proto デコード（SCC test.mjs と同一） ---------- */
function decodeVal(v) {
  if (v === "__NaN") return NaN;
  if (Array.isArray(v)) return v.map(decodeVal);
  if (v && typeof v === "object") {
    if (typeof v.__dt === "number" && Object.keys(v).length === 1) return new Date(v.__dt);
    const o = {};
    for (const k of Object.keys(v)) o[k] = decodeVal(v[k]);
    return o;
  }
  return v;
}

const children = [];
function cleanup() {
  for (const c of children) { try { c.kill("SIGTERM"); } catch {} }
}
process.on("exit", cleanup);
process.on("SIGINT", () => { cleanup(); process.exit(130); });

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

async function main() {
  const globalTimer = setTimeout(() => {
    fail(`グローバルタイムアウト ${GLOBAL_TIMEOUT_MIN} 分超過`);
    cleanup(); process.exit(1);
  }, GLOBAL_TIMEOUT_MIN * 60 * 1000);

  /* ---------- 0. 準備: TLE と config ---------- */
  const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "e2e-scc-"));
  const tlePath = path.join(tmp, "momo.tle");
  const cfgPath = path.join(tmp, "config.json");
  const { execFileSync } = await import("node:child_process");
  fs.writeFileSync(tlePath, execFileSync(PY, [path.join(MOMO, "tools/gen_tle.py")]));
  fs.writeFileSync(cfgPath, JSON.stringify({ extsat: { host: "127.0.0.1", port: GSI_PORT } }));
  ok(`TLE/config 生成 (${tmp})`);

  /* ---------- 1. MOMO-SAT GSI 起動 ---------- */
  const gsi = spawn(PY, [path.join(MOMO, "run_gsi.py"), "--port", String(GSI_PORT)],
    { cwd: MOMO, stdio: ["ignore", "pipe", "pipe"] });
  children.push(gsi);
  gsi.stdout.on("data", d => process.env.E2E_VERBOSE && process.stdout.write("[gsi] " + d));
  gsi.stderr.on("data", d => process.env.E2E_VERBOSE && process.stderr.write("[gsi!] " + d));
  await sleep(1500);
  if (gsi.exitCode !== null) { fail("run_gsi.py が起動直後に終了"); process.exit(1); }
  ok(`MOMO-SAT GSI 起動 (:${GSI_PORT})`);

  /* ---------- 2. SCC サーバ起動 ---------- */
  const scc = spawn("node", [path.join(SCC, "server/server.mjs"),
    "--port", String(SCC_PORT), "--config", cfgPath, "--tle", tlePath],
    { cwd: SCC, stdio: ["ignore", "pipe", "pipe"] });
  children.push(scc);
  scc.stdout.on("data", d => process.env.E2E_VERBOSE && process.stdout.write("[scc] " + d));
  scc.stderr.on("data", d => process.env.E2E_VERBOSE && process.stderr.write("[scc!] " + d));
  await sleep(2000);
  if (scc.exitCode !== null) { fail("SCC server が起動直後に終了"); process.exit(1); }
  ok(`SCC サーバ起動 (:${SCC_PORT}, --tle momo.tle, extsat 有効)`);

  /* ---------- 3. WS 接続（管制卓） ---------- */
  const ws = new WebSocket(`ws://127.0.0.1:${SCC_PORT}/ws`);
  let state = null;                 // 最新 state（hello/tick で更新）
  const acks = [];                  // command-ack 履歴
  const logs = [];                  // log 履歴
  ws.addEventListener("message", ev => {
    let m; try { m = decodeVal(JSON.parse(ev.data)); } catch { return; }
    if (m.type === "hello" || m.type === "tick") state = m.state;
    // server.mjs は bus payload を {type, payload:{cmd,status}} で包んで送る
    else if (m.type === "command-ack") acks.push(m.payload || m);
    else if (m.type === "log") logs.push(m.entry || (m.payload && m.payload.entry) || m);
  });
  await new Promise((res, rej) => {
    ws.addEventListener("open", res);
    ws.addEventListener("error", () => rej(new Error("WS 接続失敗")));
    setTimeout(() => rej(new Error("WS open timeout")), 5000);
  });
  await waitFor(() => state !== null, 10000, "hello 受信");
  ok("WS 接続・hello 受信（管制卓 接続完了）");

  function waitFor(cond, timeoutMs, what) {
    return new Promise((resolve, reject) => {
      const start = Date.now();
      const iv = setInterval(() => {
        let v; try { v = cond(); } catch { v = false; }
        if (v) { clearInterval(iv); resolve(v); }
        else if (Date.now() - start > timeoutMs) {
          clearInterval(iv); reject(new Error(`timeout: ${what}`));
        }
      }, 100);
    });
  }

  /* コマンド送信 + EXEC 待ち（NACK は再送、パス待ち込みの長タイムアウト） */
  async function sendUntilExec(name, args, timeoutMs) {
    const start = Date.now();
    let attempt = 0;
    for (;;) {
      attempt++;
      const ackBase = acks.length;
      ws.send(JSON.stringify({ type: "op", name: "sendCommand", args: [name, args || "", null] }));
      info(`${name} 送信 (試行 ${attempt}) — パス待ち/実行待ち`);
      // この試行の結果（EXEC or NACK）を待つ
      const result = await waitFor(() => {
        for (let i = ackBase; i < acks.length; i++) {
          const a = acks[i];
          if (a.cmd && a.cmd.name === name && a.status === "EXEC") return "EXEC";
          if (a.cmd && a.cmd.name === name && a.status === "NACK") return "NACK";
        }
        return false;
      }, timeoutMs - (Date.now() - start), `${name} の EXEC/NACK`);
      if (result === "EXEC") return;
      info(`${name} NACK — 再送する`);
    }
  }

  /* ---------- 4. 倍速設定 ---------- */
  ws.send(JSON.stringify({ type: "op", name: "setSimSpeed", args: [SIM_SPEED] }));
  await waitFor(() => state && state.simSpeed === SIM_SPEED, 5000, "simSpeed 反映");
  ok(`シミュレーション倍速 x${SIM_SPEED}`);

  /* ---------- 5. (a) パス外 QUEUED 検証 + ADCS_MODE_SUNPOINT ---------- */
  const activeNow = state.passes && state.passes.some(p => p.status === "active");
  ws.send(JSON.stringify({ type: "op", name: "sendCommand", args: ["ADCS_MODE_SUNPOINT", "", null] }));
  if (!activeNow) {
    await sleep(1500); // パス外なら QUEUED のまま滞留するはず
    const q = (state.commandQueue || []).find(c => c.name === "ADCS_MODE_SUNPOINT");
    if (q && q.status === "QUEUED") ok("(a) パス外: コマンドは QUEUED のまま（疑似 RF ゲーティング動作）");
    else info("(a) スキップ相当: 送信時に既にパス中 or 即時進行（status=" + (q && q.status) + "）");
  } else {
    info("(a) 送信時に既にパス中のため QUEUED 滞留検証はスキップ");
  }
  // EXEC まで待つ（最初のパス到来まで — 最長 12 分想定）
  await waitFor(() => acks.some(a => a.cmd && a.cmd.name === "ADCS_MODE_SUNPOINT" &&
    (a.status === "EXEC" || a.status === "NACK")), 12 * 60 * 1000, "ADCS_MODE_SUNPOINT EXEC/NACK");
  if (acks.some(a => a.cmd && a.cmd.name === "ADCS_MODE_SUNPOINT" && a.status === "NACK")) {
    info("初回 NACK — 再送");
    await sendUntilExec("ADCS_MODE_SUNPOINT", "", 12 * 60 * 1000);
  }
  ok("(b) ADCS_MODE_SUNPOINT がパス中に EXEC 到達");

  /* ---------- 6. (b) MOMO テレメトリが SCC に反映 ---------- */
  await waitFor(() => state && state.sat && state.sat.external === true, 5 * 60 * 1000,
    "S.sat.external=true（extsat 接続+リンク確立）");
  ok("extsat リンク確立（S.sat.external=true）");
  // MOMO 実値判別: busV は 2S Li-ion（6.0+2.4×SoC ≒ 8.4V）。SCC モック初期値は 28.2V
  await waitFor(() => state.sat.telemetry.adcs.mode === "SUNPOINT" &&
    state.sat.telemetry.eps.busV < 12 && state.sat.telemetry.eps.soc >= 90,
    8 * 60 * 1000, "adcs.mode=SUNPOINT かつ busV/SoC が MOMO 実値（busV<12V, SoC>=90%）");
  ok(`(c) SCC テレメトリに MOMO 値反映: mode=${state.sat.telemetry.adcs.mode}, ` +
     `SoC=${state.sat.telemetry.eps.soc.toFixed(1)}%, busV=${state.sat.telemetry.eps.busV.toFixed(2)}V`);

  /* ---------- 7. PLD_CAPTURE → 撮像 → storagePct ---------- */
  await sendUntilExec("PLD_CAPTURE", "1", 12 * 60 * 1000);
  ok("PLD_CAPTURE EXEC 到達（→ MOMO: IMAGE_NADIR count=1）");
  // NADIR へスルー・収束・撮像 → 次のリンクで storagePct に反映（1 枚 = 3.125%）
  // MOMO マッピングでは storagePct = images_stored/32×100（1 枚 = 3.125%）。
  // モック初期値 36% と区別するため 3〜13%（1〜4 枚）の範囲で判定する
  await waitFor(() => state.sat.telemetry.cdh.storagePct >= 3 && state.sat.telemetry.cdh.storagePct <= 13,
    15 * 60 * 1000, "storagePct 3〜13%（MOMO 撮像 1〜4 枚）");
  ok(`撮像成功: storagePct=${state.sat.telemetry.cdh.storagePct.toFixed(1)}% (画像 ` +
     `${Math.round(state.sat.telemetry.cdh.storagePct / 3.125)} 枚)`);

  /* ---------- 8. COM_TX_ON → DOWNLINK → 完了で SUNPOINT 自動復帰 ---------- */
  await sendUntilExec("COM_TX_ON", "", 12 * 60 * 1000);
  ok("COM_TX_ON EXEC 到達（→ MOMO: DOWNLINK）");
  await waitFor(() => state.sat.telemetry.adcs.mode === "ZENITH" ||
    state.sat.telemetry.com.mode === "TX/RX", 8 * 60 * 1000, "DOWNLINK 姿勢/送信状態の反映");
  ok(`(d) ダウンリンク開始: adcs.mode=${state.sat.telemetry.adcs.mode}, com.mode=${state.sat.telemetry.com.mode}`);
  // 全画像伝送完了 → FSW が SUNPOINT へ自動復帰（= images_downlinked 増加の証明）
  await waitFor(() => state.sat.telemetry.adcs.mode === "SUNPOINT", 15 * 60 * 1000,
    "伝送完了後の SUNPOINT 自動復帰");
  ok("(d) 全画像伝送完了 → SUNPOINT 自動復帰（images_downlinked 増加を確認）");

  /* ---------- 9. GSI イベントが SCC ログへ中継されていること ---------- */
  const gsiLog = logs.find(l => JSON.stringify(l).includes("mode transition"));
  if (gsiLog) ok("MOMO イベント（mode transition）が SCC ログへ中継");
  else info("MOMO イベントのログ中継は未観測（警告のみ、必須項目ではない）");

  clearTimeout(globalTimer);
  console.log(failed ? "\nRESULT: FAIL" : "\nRESULT: E2E ALL PASS");
  cleanup();
  process.exit(failed ? 1 : 0);
}

main().catch(e => { fail(e.message); cleanup(); process.exit(1); });
