# ももsat1号（開発コード: MOMO-SAT）— 1U CubeSat 望遠撮像ミッション

10cm 角（1U）キューブサットに口径 30mm / F3.3（f=100mm）のテレフォト型望遠カメラを搭載し、
高度 500km SSO から GSD 13.7m の地表撮像を行う技術実証衛星の
設計・フライトソフトウェア（Python プロトタイプ）・数値シミュレータ一式。

衛星の正式名称は「ももsat1号」。リポジトリ内の文書番号・型定義等では開発コード
MOMO-SAT を用いる。インターステラテクノロジズ社の観測ロケット「MOMO」とは無関係である。

## 構成

```
docs/            設計文書（要求仕様・アーキテクチャ・ICD・実装計画・検証報告）
docs/subsystems/ サブシステム詳細設計書（7 分冊）
common/          FSW⇔シミュレータ共有型定義（ICD の正）
fsw/             フライトソフトウェア（OBC 制御ソフト、10Hz）
sim/             6DoF シミュレータ（軌道・姿勢・電力・熱・通信）
sim/scenarios/   検証シナリオ S1〜S3
tests/           単体テスト
out/             シミュレーション結果 CSV
```

## 実行方法

```bash
python3 -m venv .venv && .venv/bin/pip install numpy
.venv/bin/python run_sim.py S1   # デタンブル検証
.venv/bin/python run_sim.py S2   # 撮像→ダウンリンク検証
.venv/bin/python run_sim.py S3   # 電力・熱収支検証
.venv/bin/python run_sim.py S4   # 電気推進による軌道上昇検証（THRUST 拡張、docs/08）
.venv/bin/python run_sim.py S5   # 天体観測モード検証: 露光×ST 8ケーススイープ（docs/09）
```

## SCC（衛星管制システム）との統合運用

別プロジェクトの衛星管制システム SCC（HIKARI-1 管制シミュレータ）の管制卓から、
本衛星を疑似 RF リンク越しに運用できる（仕様: docs/06_scc_integration.md）。
以下では SCC のチェックアウト先を `$SCC_DIR`、本リポジトリを `$MOMO_SAT_DIR` とする。

```bash
# 1. 衛星側（GSI: TCP :9200 で SCC の接続を待つ）
.venv/bin/python run_gsi.py

# 2. TLE 生成と SCC サーバ起動（別端末で）
.venv/bin/python tools/gen_tle.py > momo.tle
cd "$SCC_DIR"
echo '{"extsat":{"host":"127.0.0.1","port":9200}}' > server/config.json
node server/server.mjs --config server/config.json --tle "$MOMO_SAT_DIR/momo.tle"

# 3. ブラウザで http://localhost:8000 を開き管制卓から運用
#    （ADCS_MODE_SUNPOINT / PLD_CAPTURE / COM_TX_ON など。パス中のみ実行される）

# E2E 自動試験（両プロセスを自動起動して往復検証）
node tools/e2e_scc.mjs
```
