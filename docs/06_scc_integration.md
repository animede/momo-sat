# MOMO-SAT ⇔ SCC 統合仕様書（GSI: Ground-Satellite Interface）

文書番号: MOMO-INT-001 / Rev A / 2026-07-11

## 1. 目的

別プロジェクトの衛星管制システム SCC（HIKARI-1 管制シミュレータ）と
MOMO-SAT の FSW+力学シミュレータを**疑似 RF リンク**で接続し、
「管制卓からのコマンド送信 → 衛星実行 → テレメトリ監視」の統合システム動作試験を
可能にする。SCC 側の模擬衛星（mockdata.js の内蔵テレメトリ生成）を、
本物の閉ループシミュレーション（MOMO-SAT）に置き換える。

## 2. アーキテクチャ

```
┌────────── SCC ──────────┐          ┌──────── momo-sat ────────┐
│ ブラウザ UI (管制卓)      │          │  run_gsi.py               │
│   ↕ WebSocket (既存)     │          │  ┌─ gsi/bridge.py ─────┐ │
│ server/server.mjs        │  TCP     │  │ Simulator(sim/)      │ │
│   mockdata.js(軌道/RF/   │  JSON    │  │  + FlightSoftware    │ │
│   パス/コマンドキュー)     │  lines  │  │  (fsw/) 実時間駆動    │ │
│   + server/extsat.mjs ───┼─────────┼→ │ gsi/server.py        │ │
│     (新規アダプタ)        │  :9200   │  └──────────────────────┘ │
└──────────────────────────┘          └───────────────────────────┘
```

- **責務分界**: SCC が「地上系＋RF リンク環境」の権威（パス予測・アンテナ・
  ロック判定・コマンドキュー）。momo-sat が「衛星」の権威（姿勢・電力・熱・
  撮像・モード遷移）。
- momo-sat 側 GSI が **TCP サーバ**（既定 :9200）、SCC の extsat アダプタが
  **クライアント**（rotctld アダプタと同じ 3 秒間隔再接続パターン）。
- SCC の内蔵衛星テレメトリ生成は `S.sat.external = true` のとき停止し（最小限の
  ガード追加）、MOMO テレメトリで上書きされる。extsat 未設定時は従来どおり
  （ブラウザ単独 mock モードも無変更）。

## 3. 疑似 RF リンクのゲーティング（通信の現実性）

1. **上り（コマンド）**: SCC 既存のコマンドライフサイクル（QUEUED→SENT→ACK→EXEC、
   パス可視時のみ進行、5% NACK）をそのまま使う。アダプタは bus の
   `command-ack (status=EXEC)` を購読し、EXEC 到達コマンドのみ GSI へ転送する。
2. **下り（テレメトリ）**: GSI は毎秒テレメトリを push するが、アダプタは
   **SCC 側リンク状態（いずれかの局が lock）** のときのみ `S.sat.telemetry` へ
   反映する。圏外では SCC 画面のテレメトリが更新停止（＝実運用と同じ「最終値保持」）。
3. **衛星側可視フラグ**: アダプタが 1 Hz で `link` メッセージを送り、momo-sat の
   `gs_visible` を SCC のリンク状態で**外部駆動**する（momo-sat 内蔵の東京局
   モデルは統合モードでは使わない）。これにより FSW の DOWNLINK 動作と SCC の
   パスが定義上一致する。

## 4. GSI プロトコル（TCP、改行区切り JSON、UTF-8）

### 4.1 SCC → momo-sat

```jsonc
{"type":"hello","who":"scc","version":1}
{"type":"time","sim_epoch_ms":1770000000000,"speed":60}   // 1 Hz + speed変更時
{"type":"link","up":true,"station":"GS-TKY","snr":18.2}   // 状態変化時 + 1 Hz
{"type":"cmd","id":123,"name":"ADCS_DETUMBLE","args":""}  // EXEC 到達時
```

### 4.2 momo-sat → SCC

```jsonc
{"type":"hello","who":"momo-sat","version":1,"t0_sim_epoch_ms":1770000000000}
{"type":"ack","id":123,"accepted":true,"mapped":"SET_MODE DETUMBLE","detail":""}
{"type":"tlm","sim_epoch_ms":...,"lag_s":0.0,"data":{
   "mode":"SUNPOINT","q_est":[x,y,z,w],"omega_est_deg_s":[...],
   "pointing_error_deg":0.1,"battery_soc":0.92,"battery_voltage":8.1,
   "battery_temp":12.3,"solar_power":2.1,"wheel_speeds":[...],
   "images_stored":2,"images_downlinked":1,"uptime":123.4,
   "last_cmd":"IMAGE_NADIR","fault_flags":0,"tx_active":true}}
{"type":"event","level":"info","source":"FSW","text":"NADIR_IMAGING: 撮像 1/4"}
```

- `hello` 交換後に運用開始。momo-sat は最初の `time` の `sim_epoch_ms` を
  シミュレーション t=0 に対応付ける。
- 切断時: momo-sat はシミュレーションを**継続**（衛星は地上と無関係に飛び続ける）。
  再接続時は現在状態からテレメトリ再開。SCC 側は切断中リンク欠損扱い。

### 4.3 時刻同期

- momo-sat は SCC の `time`（sim_epoch_ms, speed）を目標クロックとし、
  実時間ペーシングで `sim_t` を追従させる（dt=0.1 s 固定、1 実秒あたり
  最大 `--max-steps-per-sec`（既定 2000 ≒ ×200）まで加速）。
- SCC 倍速が追従上限を超える場合は遅れて追従し、遅延を `tlm.lag_s` で報告する
  （制限事項として明記。統合試験は ×60〜×120 を推奨）。

## 5. コマンドマッピング（SCC → MOMO GroundCommand）

| SCC コマンド | MOMO GroundCommand | 備考 |
|---|---|---|
| ADCS_DETUMBLE | SET_MODE DETUMBLE | |
| ADCS_MODE_SUNPOINT | SET_MODE SUNPOINT | |
| ADCS_MODE_NADIR | SET_MODE NADIR_IMAGING | |
| PLD_CAPTURE [n] | IMAGE_NADIR {count:n(既定1)} | 撮像条件成立時に FSW が自律実行 |
| PLD_STANDBY | SET_MODE SUNPOINT | |
| COM_TX_ON | DOWNLINK | 蓄積画像のダウンリンク開始 |
| COM_TX_OFF | SET_MODE SUNPOINT | |
| OBC_REBOOT | RESET | SAFE 起動相当 |
| EPS_LOAD_SHED | SET_MODE SAFE | 負荷遮断＝セーフモード |
| EPS_HEATER_ON/OFF | （転送しない） | MOMO ヒータは自律制御。ack=false + event で通知 |
| OBC_TIME_SYNC / OBC_HEALTH_DUMP | （転送しない） | ack=true + event（HEALTH_DUMP は tlm 即時 push） |
| PROP_BURN | （転送しない） | MOMO-SAT は推進系非搭載。ack=false + event |

非転送コマンドも SCC 内蔵の演出効果（ログ等）は従来どおり動作してよい。

## 6. テレメトリマッピング（MOMO tlm → SCC S.sat.telemetry）

| SCC ch | MOMO ソース | 変換 |
|---|---|---|
| eps.busV | battery_voltage | そのまま [V] |
| eps.soc | battery_soc | ×100 [%] |
| eps.solarW | solar_power | そのまま [W] |
| eps.battT / tcs.battT | battery_temp | そのまま [℃] |
| adcs.mode | mode | SAFE/DETUMBLE/SUNPOINT はそのまま、NADIR_IMAGING/TARGET_IMAGING→"NADIR"/"TARGET"、DOWNLINK→"ZENITH" |
| adcs.rateX/Y/Z | omega_est_deg_s | そのまま [°/s] |
| adcs.quat | q_est | **[x,y,z,w]→[w,x,y,z]**（SCC はスカラ先頭） |
| adcs.sunAngle | pointing_error_deg | 指向誤差を流用 |
| cdh.uptime | uptime | そのまま [s] |
| cdh.storagePct | images_stored | /MAX_STORED_IMAGES(32)×100 [%] |
| com.mode | tx_active | true→"TX/RX"、false→"RX" |
| （alarms/ログ） | fault_flags, event | fault_flags≠0 で serious ログ、event はそのまま SCC ログへ |

rssi/snr/bitrate 等の RF 品質は従来どおり **SCC 側モデルが権威**（衛星は関与しない）。
未マッピングの ch（cdh.cpu 等）は SCC 内蔵値を凍結（external 時は乱数歩行停止）で可。

## 7. 軌道の整合

- momo-sat の SSO（500 km, i=97.4°）に対応する TLE を `tools/gen_tle.py` で生成し、
  SCC を `--tle momo.tle` で起動する（SCC のパス予測・RF リンクはこの軌道で計算）。
- 両者の伝播器（SGP4 vs 二体+J2）・座標系原点は異なるため長時間で位相差が生じるが、
  衛星側可視は §3-3 のとおり SCC から外部駆動されるため通信整合性は保たれる。
  制限事項として momo-sat 内部の地上関連計算（TARGET_IMAGING の地表座標）は
  SCC の世界とはずれる（統合試験では NADIR 撮像を使用）。

## 8. 作業分担（WP）

| WP | 内容 | 担当 |
|---|---|---|
| INT-0 | 本仕様書・TLE 方式決定 | メイン（Fable） |
| INT-A | momo-sat 側: gsi/ パッケージ + run_gsi.py + tools/gen_tle.py + 単体テスト | Sonnet エージェント A |
| INT-B | SCC 側: server/extsat.mjs + server.mjs 組込 + mockdata.js 最小ガード + セルフテスト拡張 | Sonnet エージェント B |
| INT-C | E2E 統合試験（両プロセス起動・コマンド往復・判定）+ 検証報告改訂 | メイン（Fable） |

## 9. 受け入れ基準と検証結果（2026-07-11 INT-C 実施）

**結果: 全項目 PASS**（E2E: `node tools/e2e_scc.mjs`、45 秒 @×150 倍速）

| # | 基準 | 結果 |
|---|---|---|
| 1 | SCC セルフテスト ALL PASS（偽 extsat 6 項目込み） | PASS |
| 2 | momo-sat tests/test_gsi.py PASS | PASS（fsw/sim 回帰含む） |
| 3 | E2E: コマンド往復・テレメトリ反映・ダウンリンク | PASS（下表） |
| 4 | extsat 未設定時の SCC 完全互換 | PASS（起動確認・セルフテスト回帰） |

E2E 実測（×150 倍速、TLE 同期軌道）:
ADCS_MODE_SUNPOINT EXEC → SCC 表示 mode=SUNPOINT / SoC=100% / busV=8.40V（MOMO 実値）、
PLD_CAPTURE → NADIR 撮像 1 枚（storagePct=3.1%）、COM_TX_ON → ZENITH 指向・伝送完了後
SUNPOINT 自動復帰、GSI イベントの SCC ログ中継まで確認。

統合試験で検出・修正した不具合:
1. GSI ペーシングスレッドが初回 tlm push 時の未初期化属性参照で死亡し、衛星シミュレーションが
   停止（単体テストではメッセージ到着順の偶然で未検出）→ フォールバック追加 + ループ防御化
2. E2E ドライバの command-ack メッセージ形状の読み違い、および SCC モック初期値を
   MOMO 実値と誤認しうる緩い判定 → busV（8.4V 系 vs モック 28.2V）等で厳格化

## 10. 受け入れ基準（原文）

1. `node server/test.mjs`（SCC セルフテスト）が引き続き ALL PASS（偽 extsat テスト追加込み）。
2. momo-sat `tests/test_gsi.py` が PASS（偽 SCC クライアントでコマンド/tlm/ack/link 駆動を検証）。
3. E2E: SCC サーバ + run_gsi.py を起動し WS 経由で
   ADCS_MODE_SUNPOINT → PLD_CAPTURE → COM_TX_ON を投入すると、
   (a) パス外ではコマンドが QUEUED のまま、(b) パス中に EXEC → MOMO 側モード遷移、
   (c) SCC テレメトリ表示（soc/adcs.mode/storagePct）が MOMO 値に一致、
   (d) ダウンリンクで images_downlinked が増加、まで自動検証できること。
4. extsat 未設定時の SCC は従来と完全に同一動作（ブラウザ mock モード含む）。
