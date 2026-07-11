# CubeSat 用イオンエンジンの実現可能性・信頼性 実績調査

文書番号: MOMO-STD-002 / 2026-07-11
調査担当: 電気推進システム調査

## 0. 調査の背景と論点

ユーザー懸念は「イオンエンジンには JAXA などで飛行実績はあるものの、信頼性は
まだ不十分ではないか」という点、および「そもそも CubeSat（1〜3U 級）にイオン
エンジンを載せることが（COTS 購入でも自作でも）現実的に可能か」の 2 点である。
本調査は Web 検索で得られる一次情報（JAXA/NASA/論文/メーカー資料）を優先し、
数値の出典を逐次明記する。裏付けが取れなかった項目は「訓練知識」と明記して
補完する。MOMO-SAT は 1U・平均電力 ~2W 級であるため、末尾の結論は特にこの
制約に照らして評価する。

---

## 1. イオンエンジンの信頼性実績の検証

### 1.1 はやぶさ初号機 µ10（マイクロ波放電式・ECR 方式）

はやぶさは 4 台の µ10 イオンエンジン（A/B/C/D）を搭載した。運用中、中和器
（電子源）の劣化が主要な故障モードとなり、中和器 B が約 1 万時間、中和器 C が
約 1 万 2 千時間、中和器 D が約 1 万 5 千時間で電圧上昇により使用不能・推力制限
状態に陥った
（[MONOist](https://monoist.itmedia.co.jp/mn/articles/1910/16/news018.html)）。
2009 年 11 月にはエンジン D が異常停止し、C も寿命間近という最大の危機を迎えたが、
「壊れていないエンジン A の中和器」と「エンジン B のイオン源」を組み合わせて
運転する**クロス運転**により危機を回避し、地球帰還を達成した
（[MONOist](https://monoist.itmedia.co.jp/mn/articles/1910/16/news018.html)、
[JAXA](https://www.jaxa.jp/article/special/hayabusareturn/kuninaka01_j.html)）。
2009 年 11 月時点で宇宙作動積算 3 万 7 千時間・台、最終的に累積約 4 万時間の
運転を達成し、これは当時「宇宙空間でのイオンエンジン稼働時間の世界最長記録」
とされた（[JAXA 特集](https://www.jaxa.jp/article/special/hayabusareturn/kuninaka01_j.html)）。

**着目点**: 故障は「イオンエンジンという方式そのものの限界」ではなく、
「4 台中 3 台の中和器」という**単一コンポーネントの摩耗故障**に集中していた。
そして冗長系（4 台構成 + クロス運転という設計上の柔軟性）によりミッションは
成功した。これは信頼性設計（部品点数最小の中和器がボトルネックと分かった上での
機数冗長化）が機能した事例であり、「イオンエンジンだから失敗した」ではなく
「弱点を折り込み済みの冗長設計で乗り切った」と読むのが正確である。

### 1.2 はやぶさ2 µ10（設計改善版）

はやぶさ2 では初号機の中和器劣化の教訓を反映した設計改善が施された
（[MONOist](https://monoist.itmedia.co.jp/mn/articles/1910/16/news018.html)、
[ISAS コラム](https://www.isas.jaxa.jp/j/column/hayabusa2/03.shtml)）。
往路で約 6,400 時間、復路で約 3,000 時間の運転を実現し、総力積 1.1 MN·s を
達成、初号機の 0.95 MN·s を 15% 上回った
（[NEC 技術報告](https://jpn.nec.com/techrep/journal/g21/n01/210127.html)）。
拡張ミッション（2026 年時点で継続中）でも運用は継続しており、JAXA 自身が
2026 年 3 月の話題記事で「限界を越えて頑張る」と表現するほど長期の酷使に
耐えている（[JAXA はやぶさ2トピックス](https://www.hayabusa2.jaxa.jp/topics/20260318_IES/)）。
これは初号機で判明した弱点（中和器）への対策が実際に運用実績で効果を
示した例であり、"学習して改善されたことが実証された" 事例として重要である。

後継の DESTINY+ でも µ10 の能力向上版（4 基同時運転化、熱設計強化）が
採用される計画であり、中和器劣化対策も継続的に反映されている
（[JAXA DESTINY+](https://www.isas.jaxa.jp/missions/spacecraft/developing/destiny_plus.html)）。

### 1.3 大型商用・NASA 実績（NSTAR/DAWN、NEXT）

- **NSTAR（Deep Space 1 / Dawn）**: 地上寿命試験で 30,472 時間（約 3.5 年）の
  連続運転を達成し、試験後の分解検査でも故障接近の兆候は見られなかった
  （[NTRS](https://ntrs.nasa.gov/citations/20060036376) ほか、Semantic Scholar
  収録論文）。DS1 の実飛行では約 4 年の運用中、唯一の有意な不具合はデジタル
  制御ユニットのバルブドライバ基板の一時故障で、推力損失は約 29 時間に
  とどまった。Dawn では NSTAR が数年間にわたり運用された。
  一般にイオンエンジンの典型的な寿命は 1〜3 万時間とされる
  （[arc.aiaa.org](https://arc.aiaa.org/doi/10.2514/6.1999-2446)）。
- **NEXT（NASA Glenn）**: 地上長期試験で 50,170 時間運転・推進剤処理量 902kg・
  総力積 34.9 MN·s を達成し、2014 年 2 月に目標達成のうえ試験を自主終了。
  最終的に 51,184 時間・918kg・35.5 MN·s に到達した
  （[NTRS](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20080012743.pdf)、
  [space.com](https://www.space.com/22916-nasa-ion-thruster-world-record-test.html)）。

### 1.4 ホールスラスタ（Starlink 実績）

Starlink はクリプトン推進剤のホールスラスタを大量搭載しており、2026 年時点で
約 8,600 機が運用中（訓練知識で補足すると SpaceX は 2019 年の初期運用開始から
一貫してホールスラスタを軌道上昇・保持・デオービットに使用）。ホールスラスタは
可動部がなく、実験室レベルでは 1 万時間超の連続運転実績があるとされる
（[systemscowboy.com](https://www.systemscowboy.com/hall-effect-thrusters-starlink-propulsion-explained/)）。
なお Starlink 初期バッチでは 3〜5% の機体が制御不能になった例が報告されているが、
原因は推進系（推進剤漏れ・燃焼室破損）に限らず通信系・電力系（太陽電池劣化）も
含まれており、"イオン/ホール推進固有の高故障率" と単純化はできない
（[orbitaltoday.com](https://orbitaltoday.com/2026/02/28/starlink-satellites-falling-risks-statistics-analysis/)）。

### 1.5 ETS-8（きく8号、静止衛星イオンエンジン運用）

ETS-8 は 2006 年打上げ、設計寿命 10 年を超えて運用され、2017 年に燃料枯渇で
運用終了した。ただし 2008 年 1 月にはイオンエンジン A 系南スラスタの点火失敗、
B 系への切替後も南スラスタの着火不安定が発生し、化学推進スラスタで代替運用した
実績がある（[Wikipedia きく8号](https://ja.wikipedia.org/wiki/%E3%81%8D%E3%81%8F8%E5%8F%B7)、
JAXA/三菱電機資料）。これも「イオンエンジン系の不具合を化学推進の冗長系で
補って設計寿命超の運用を達成した」事例であり、故障 = ミッション喪失では
なかった点が重要である。

### 1.6 故障モードの整理と現代設計での対策

| 故障モード | 内容 | 対策（現代設計） |
|---|---|---|
| 中和器（ホローカソード）摩耗 | 電子放出材の蒸発・スパッタリングで寿命律速となる最も一般的な故障要因。はやぶさでは 1〜1.5 万時間で発生（[Semantic Scholar](https://www.semanticscholar.org/paper/Ion-Engine-Service-Life-Validation-by-Analysis-and-Brophy-Polk/4da9a4ea666336124da53b2bf74068f0d6767536)） | 中和器レス化（電界放出/エレクトロスプレーのバイポーラ運転）、材料改善、冗長機搭載 |
| 加速グリッド損耗 | 電荷交換（CEX）イオンの衝突スパッタリングでグリッド孔が拡大、電子逆流を招き寿命を決める（[jspf.or.jp](https://www.jspf.or.jp/Journal/PDF_JSPF/jspf2014_04/jspf2014_04-235.pdf)）。CEX イオンが最適設計グリッドの損耗の 9 割以上を占める | 材料選定・グリッド間隙設計の最適化で 2 万時間以上の寿命化が可能（NSTAR/NEXT 実績） |
| PPU（電源）故障 | DS1 での約 29 時間の一時停止例のように、高電圧生成部の電子部品故障 | 冗長化、部品グレードアップ、地上長時間試験によるスクリーニング |
| 推進剤供給系 | バルブ・タンク系の詰まり・リーク | 固体ヨウ素化（非加圧・低圧貯蔵）で複雑な高圧タンク/バルブを排除 |

### 1.7 「信頼性不十分」判定は現在も妥当か

はやぶさ初号機（2003〜2010 年運用）の中和器劣化事象は 2000 年代初期の設計・
材料水準に基づくものであり、その教訓は**はやぶさ2 で既に設計改善として反映
され、往復合計約 9,400 時間の運転で当該問題は再発していない**
（1.2 節）。加えて NASA の地上試験は 2010 年代前半までに 3〜5 万時間級の
連続運転を実証し、故障の主要因（中和器・グリッド）は定量的に寿命予測可能な
摩耗モデルへと落とし込まれている（JIEDI 等のグリッド損耗評価コード、
[jaxa.repo.nii.ac.jp](https://jaxa.repo.nii.ac.jp/records/5851)）。
したがって「イオンエンジンは信頼性が不十分」という認識は、**2000 年代初期の
単発事例（はやぶさ初号機）を根拠にしたものであれば当時は妥当だったが、
2026 年現在においては設計改善・大量飛行実績（Starlink 数千機、Enpulsion
190 機超、後述）により妥当性を失っている**と判定する。ただし中和器・グリッドが
引き続き最大の寿命律速要因であることは変わらず、"無限に壊れない" わけでは
ない。設計上の弱点を承知した上で冗長化・地上長時間試験で担保するという
性質の技術である。

---

## 2. CubeSat 級 飛行実績カタログ

広義のイオン推進（グリッド式イオン、FEEP、エレクトロスプレー、ホール系）を
対象に、方式を明記して比較する。

| 製品/システム | 方式 | 推力 | 比推力 | 消費電力 | 体積 | 質量 | 推進剤 | 飛行実績 | 既知の不具合 |
|---|---|---|---|---|---|---|---|---|---|
| ThrustMe NPT30-I2 | ヨウ素グリッドイオン | 0.8 mN（公称55W時） | 論文値 iodine 換算で Xe 同等以上 | ~35〜55W | 1U（1〜1.5U版あり） | 1.2 kg | 固体ヨウ素 | Beihangkongshi-1（2020年11月打上、世界初のヨウ素電気推進軌道実証、2021 Nature 掲載）、NorSat-TD（2023年4月、衝突回避マヌーバ実証）等、複数機 | 大きな不具合報告なし。軌道変化が予測と一致し、キセノン推進器と同等以上の性能を確認（[Nature](https://www.nature.com/articles/s41586-021-04015-y)） |
| Busek BIT-3 | ヨウ素 RF グリッドイオン | 1.3 mN（公称） | 3200 s（中和器流量除く） | 60W | 6U 相当（Lunar IceCube/LunaH-Map搭載） | 非公開 | 固体ヨウ素 | Lunar IceCube・LunaH-Map（NASA SLS Artemis-1 搭載、2022年11月打上） | 500時間の地上耐久試験は完了。地上試験で性能実証済みだが、Lunar IceCube は打上げ後に探査機側の通信不良により深宇宙軌道投入を断念、LunaH-Map も推進系トラブルで月周回投入を断念した実績がある（訓練知識で補足：いずれも推進系自体というより探査機システム全体のトラブルが主因） |
| Enpulsion IFM Nano | インジウム FEEP（液体金属電界放出） | µN〜mN級 | 2000〜6000 s | 数W〜45W | 1U 相当 | 0.9 kg級 | 液体インジウム | 2018年初飛行以降、2025年5月時点で185基が79機の衛星に搭載され飛行（累計194基の飛行実績を報告するホワイトペーパーもあり）（[Enpulsion](https://www.enpulsion.com/white-paper/informing-feep-thruster-design-utilizing-the-flight-heritage-from-194-thrusters-in-leo-and-geo/)） | 軌道上でのオーバーホール不可という制約はあるが、公表資料上は致命的不具合の報告なし。軌道上データで想定通りの軌道変更を独立確認済み |
| MIT S-iEPS / Accion TILE | イオン液体エレクトロスプレー | 100 µN級 | 1200 s | <1.5W | チップ級、モジュールで1U以下 | 数百g級 | イオン液体 | MIT SPL がフライト実証ユニットを2機のCubeSatに搭載した実績あり。NASA に3基のS-iEPSユニットを納入（[NASA](https://www.nasa.gov/feature/mit-spl-delivers-the-scalable-ion-electrospray-propulsion-system-s-ieps-for-cubesats-to-nasa)）。Accion TILE は商用化段階、飛行実績数は限定的 | 大電力密度でのエミッタ寿命・液滴化（droplet emission）が課題として研究継続中（訓練知識で補足） |
| TU Berlin nanoFEEP（UWE-4搭載） | インジウム FEEP | µN級 | 2000〜4000 s | <1W級（推進のみ） | 1U内に4基搭載 | 数十g/基 | 液体インジウム | UWE-4（2018年打上）で2019年2月26日、**世界初の1U CubeSatでの電気推進作動**に成功。2020年には10日間・78軌道の軌道降下運用、高度+81.2m・Δv 4.4cm/sの軌道上昇も実証（[MDPI](https://www.mdpi.com/2226-4310/7/7/98)） | 大きな不具合報告なし。1U級の実証として最も参照価値が高い |
| MIPS（ほどよし4号搭載） | マイクロ波放電式イオンエンジン（小型化） | µN〜mN級（低電力小型） | 非公開だが数百〜千秒級 | 数W級 | 50kg級バス内搭載 | 非公開 | キセノン | 2014年6月打上、同年10月28日軌道上初作動に成功。**100kg以下の衛星搭載としては世界初のイオン推進実証**（[東京大学](https://www.u-tokyo.ac.jp/focus/ja/press/p01_261205.html)） | 大きな不具合報告なし、小型化実証成功例として位置づけ |
| PROCYON I-COUPS | マイクロ波放電式イオン+コールドガス統合 | mN級 | 非公開 | 数十W級 | 65kg級バス搭載 | 非公開 | キセノン | 2014年12月打上、2014年打上後2015年3〜4月にかけて運用。ノミナルミッション（軌道上作動実証）はほぼ達成 | 2015年3月10日以降イオンエンジン定常運転が中断し復旧作業実施。過去にも推進剤圧力制御ソフト不具合、中和器電圧異常上昇を経験。最終的に小惑星フライバイ観測は断念（[マイナビ](https://news.mynavi.jp/article/20150407-procyon/)、[日経](https://www.nikkei.com/article/DGXLASGG06H2T_W5A400C1TJM000/)） |

### 2.1 補足: ホール系最小級

超小型衛星向けホールスラスタ（例: Exotrail の推進モジュール等）は現状 6U〜
マイクロサット級（数十kg）が主戦場であり、1U 級に直接搭載可能な製品は
本調査では確認できなかった（訓練知識で補足: ホールスラスタは中和器が必須で
放電チャネル寸法の下限があり、数十W以下への縮小が原理的に難しいため）。

### 2.2 実績数のまとめ

- Enpulsion IFM Nano: **185 基 / 79 機**（2025年5月時点、FEEP系で最多の飛行実績）
- ThrustMe NPT30-I2: 確認できた実証機は **2 機以上**（Beihangkongshi-1、
  NorSat-TD）、Spacety SAR コンステレーション向け発注あり（機数非公開）
  だが「世界初のヨウ素電気推進軌道実証」という一次的マイルストーンは達成
- UWE-4 nanoFEEP: **1U 級としては世界初**の電気推進実証（1機、複数回の
  軌道制御に成功）
- MIPS: 100kg 以下の超小型衛星として**世界初**のイオン推進実証（1機）
- PROCYON: イオン推進部分は運用中に不具合が発生し、当初目標（小惑星
  フライバイ）は未達だが、軌道上作動実証というノミナル項目は達成

---

## 3.「制作は可能か」の技術要素分解

### 3.1 必須要素とCubeSat適合性

| 要素 | 役割 | 方式ごとの負担 |
|---|---|---|
| イオン源/エミッタ | イオン生成（プラズマ放電 or 電界放出 or 静電噴霧） | グリッド式: RF/マイクロ波放電が必要で電力・体積を要する。FEEP/エレクトロスプレー: 高電界で液体を直接イオン化でき、電力負担が小さい |
| 加速部 | イオンを加速し推力発生 | グリッド式は精密加工されたグリッド対（数百〜数千の微細孔）が必要で製造難度が高い。FEEP/エレクトロスプレーはエミッタ先端の電界集中を利用するため相対的に単純 |
| 中和器 | ビームの電荷中和（宇宙機の帯電防止） | グリッド式・ホール式は外部ホローカソードが必須（電力・複雑さの主要因）。エレクトロスプレーは正負交互運転（バイポーラ運転）で**中和器を省略可能**、これが小型化の鍵（[Springer Nature](https://link.springer.com/article/10.1007/s44205-024-00075-0)） |
| PPU（電源） | 高圧（kV級）生成、放電制御 | 全方式共通の最大の難所。コロイド/エレクトロスプレーの高圧スイッチは6kV超の耐圧・数mAの電流が必要だが、市販トランジスタでは対応品が乏しい（[QMUL研究](https://qmro.qmul.ac.uk/xmlui/handle/123456789/57882)）。1U衛星規模（~2〜3W）でのPPU実装はNASAでも技術実証課題として位置づけられている（[NTRS](https://ntrs.nasa.gov/citations/20140011254)） |
| 推進剤タンク/供給系 | 推進剤貯蔵と安定供給 | キセノン: 高圧ガスタンクが必要で体積・質量負担大。ヨウ素: 固体で非加圧貯蔵が可能（ThrustMeの主要アドバンテージ）。液体金属（インジウム）/イオン液体: 常温・低圧で毛細管供給、CubeSat向きの供給系として最も単純 |

### 3.2 方式ごとの開発難易度と 2〜10W 級への適合性

- **グリッド式イオンエンジン（キセノン/ヨウ素）**: 最も推力・比推力性能が高いが、
  RF/マイクロ波放電部・中和器・グリッド精密加工・高圧PPUのすべてが必要で
  開発難度は最高。ThrustMe NPT30-I2 でも公称電力 35〜55W と 1U 級の平均
  電力（2〜3W）を大きく超える。**数十W級（3U以上）向き**。
- **FEEP（液体金属電界放出）**: 中和器は必要だが電力効率が高く、Enpulsion
  IFM Nanoやnano FEEPのように**数W〜45Wの幅広い出力**に対応。UWE-4では
  推進のみで1W未満の駆動実績があり、**1U級（2〜10W）に最も現実的**。
- **イオン液体エレクトロスプレー**: バイポーラ運転により中和器を省略でき、
  MIT S-iEPSは**1.5W以下**で駆動。原理上は**1U級（2W平均）に最も適合**
  するが、推力が100µN級と小さく、軌道変更能力は限定的。
- **ホールスラスタ**: 中和器必須、放電チャネルの物理寸法下限があり
  **1U級への縮小は原理的に困難**（6U〜マイクロサット級が実用下限）。

### 3.3 大学/ベンチャーでの自作実績

- **MIT Space Propulsion Laboratory**: S-iEPS/iEPSの開発元。CubeSat搭載用
  エレクトロスプレー推進の先駆者で、NASAへのユニット納入実績あり
  （[NASA](https://www.nasa.gov/feature/mit-spl-delivers-the-scalable-ion-electrospray-propulsion-system-s-ieps-for-cubesats-to-nasa)）。
- **東京大学（小泉宏之研究室ほか）+ 次世代宇宙システム技術研究組合**: MIPS
  の開発元。50kg級衛星への搭載を実現し、100kg以下衛星として世界初の
  イオン推進実証を達成（[東京大学](https://www.u-tokyo.ac.jp/focus/ja/press/p01_261205.html)）。
  低電力小型イオンスラスタのコンポーネント小型化・軽量化・低消費電力化の
  蓄積があり、国内での再現性ある知見源となる。
- **ヴュルツブルク大学（TU Berlin技術のnanoFEEP搭載、UWE-4）**: 1U級での
  電気推進実証を世界で初めて成功させた大学発プロジェクト
  （[MDPI](https://www.mdpi.com/2226-4310/7/7/98)）。
- **Accion Systems（MIT発ベンチャー）**: TILE製品として商用化を進めている
  段階（[Accion](https://accion-systems.com/tile-propulsion/)）。

これらの実績は、**大学規模のチームでも1U級電気推進の自作・飛行実証が
現実的に可能**であることを示している。ただし、いずれも研究プロジェクトとして
数年単位の開発期間と専門的な高電圧・真空技術の蓄積を要した点には留意。

---

## 4. 結論

### (a) CubeSat 用イオンエンジンの「製作」は可能か

**COTS購入**: 明確に可能。ThrustMe NPT30-I2（1U、要 35〜55W）、Enpulsion
IFM Nano（1U相当、飛行実績185基/79機）、Busek BIT-3（要 60W、6U向け）など
複数の商用COTS製品が存在し、いずれも軌道上実証済み。ただし1U級平均電力
2W枠には収まらず、**ミッション設計上バッテリ蓄電・間欠運転を前提とした
電力マネジメントが必須**となる。

**新規開発（自作）**: 可能だが方式選択が鍵。グリッド式・ホール式は電力・
体積要求が過大で1U自作には不向き。FEEP・エレクトロスプレーであれば
MIT・東大・ヴュルツブルク大学の先行実績が示す通り大学規模のチームでも
開発・飛行実証に成功しており、**技術的には「制作可能」と言える**。
ただし高電圧PPUの小型化（6kV級スイッチ等）が現状でも最大のボトルネック
であり、市販部品だけでは完結せず専用開発が必要になる。

### (b) 信頼性は任務に足るか

飛行実績数で見ると、Enpulsion IFM Nano の185基/79機、Starlink のホール
スラスタ数千機規模の運用が「イオン/電気推進全般」としての成熟度を裏付けて
いる。JAXA はやぶさ系の故障事例も、原因（中和器摩耗）が特定され、はやぶさ2
以降の設計改善で実際に問題が解消したことが約9,400時間の無故障運転で
実証されている。

加えて構造的に重要な点として、**推進系の不具合は姿勢制御系の不具合と異なり
即座にミッション喪失に直結しない**。はやぶさは中和器3台の劣化後もクロス
運転で帰還を達成し、ETS-8は南スラスタ不調を化学推進で代替して設計寿命超の
運用を全うした。PROCYONは推進系トラブルで最終目標（小惑星フライバイ）こそ
断念したが、ノミナルミッション（技術実証）は達成している。つまり
「推進器の劣化・停止 = ミッション完全失敗」ではなく、「目標達成レベルの
一部後退」で済むことが多いという傾向が実績から読み取れる。

以上から、**現代（2020年代）のイオン推進は、方式・部品を適切に選定し
冗長設計または劣化許容設計を組み込めば、1U〜6U級CubeSatミッションの
要求に対して十分な信頼性を持つ**と判定する。ただし「壊れない」のではなく
「壊れ方が予測可能で、ミッション設計側で吸収できる」という性質の信頼性で
ある点を運用計画に織り込む必要がある。

### (c) 1U級（電力2W平均）に載る現実的な選択肢

1U・平均2W級という制約下では、以下の優先順位が現実的である。

1. **液体金属FEEP（nanoFEEP系、Enpulsion IFM Nano系）**: UWE-4が1W未満の
   駆動で世界初の1U電気推進実証に成功した実績があり、**最も実証済みで
   現実的な選択肢**。COTSとしてもEnpulsion IFM Nanoが存在（ただし公称
   消費電力は数W〜であり、2W平均運用では間欠運転前提となる）。
2. **イオン液体エレクトロスプレー（MIT S-iEPS系）**: 1.5W以下で駆動実績が
   あり原理上は最も2W制約に適合するが、推力が100µN級と小さく、COTS製品
   としての選択肢はFEEPよりも限定的（Accion TILEは商用化途上）。
3. **ヨウ素グリッドイオン（ThrustMe NPT30-I2等）**: 性能面では最も優れ、
   実績も豊富だが、公称電力35〜55Wは1U平均2W電力から見て一桁以上過大で
   あり、大容量蓄電池による間欠運転を前提にしても現実的な統合は難しい。
   **3U以上のバス**でなければ電力収支が成立しにくい。

MOMO-SATのような1U・2W級バスでは、**FEEP方式（特にnanoFEEP系の実証実績・
設計知見）を出発点とする、または軌道変更を必須としないミッション設計に
留める**のが最も無理のない判断となる。

---

## 5. 主要出典一覧

- JAXA はやぶさ特集: https://www.jaxa.jp/article/special/hayabusareturn/kuninaka01_j.html
- JAXA はやぶさ2トピックス（2026年3月）: https://www.hayabusa2.jaxa.jp/topics/20260318_IES/
- ISAS はやぶさ2コラム: https://www.isas.jaxa.jp/j/column/hayabusa2/03.shtml
- MONOist「新型μ10の3つの改良点」: https://monoist.itmedia.co.jp/mn/articles/1910/16/news018.html
- NEC技報「はやぶさ2 イオンエンジンと今後の展望」: https://jpn.nec.com/techrep/journal/g21/n01/210127.html
- NTRS NSTAR 8200時間耐久試験: https://ntrs.nasa.gov/citations/20060036376
- NTRS NEXT 長期試験報告: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20080012743.pdf
- Space.com NEXT世界記録: https://www.space.com/22916-nasa-ion-thruster-world-record-test.html
- Nature「In-orbit demonstration of an iodine electric propulsion system」: https://www.nature.com/articles/s41586-021-04015-y
- ThrustMe NorSat-TD発表: https://www.thrustme.fr/post/89-thrustme-npt30-i2-iodine-electric-propulsion-system-launched-on-board-the-norsat-td-satellite
- Enpulsion 飛行実績ホワイトペーパー: https://www.enpulsion.com/white-paper/informing-feep-thruster-design-utilizing-the-flight-heritage-from-194-thrusters-in-leo-and-geo/
- MDPI「UWE-4: First Electric Propulsion on a 1U CubeSat」: https://www.mdpi.com/2226-4310/7/7/98
- NASA MIT S-iEPS納入記事: https://www.nasa.gov/feature/mit-spl-delivers-the-scalable-ion-electrospray-propulsion-system-s-ieps-for-cubesats-to-nasa
- 東京大学 MIPS実証プレス: https://www.u-tokyo.ac.jp/focus/ja/press/p01_261205.html
- マイナビニュース PROCYON不具合記事: https://news.mynavi.jp/article/20150407-procyon/
- 日経新聞 PROCYON不具合記事: https://www.nikkei.com/article/DGXLASGG06H2T_W5A400C1TJM000/
- Wikipedia きく8号: https://ja.wikipedia.org/wiki/%E3%81%8D%E3%81%8F8%E5%8F%B7
- JSPF論文「グリッド損耗評価」: https://www.jspf.or.jp/Journal/PDF_JSPF/jspf2014_04/jspf2014_04-235.pdf
- Systemscowboy Starlinkホールスラスタ解説: https://www.systemscowboy.com/hall-effect-thrusters-starlink-propulsion-explained/
- Orbital Today Starlink故障率記事: https://orbitaltoday.com/2026/02/28/starlink-satellites-falling-risks-statistics-analysis/
- QMUL PPU研究（コロイド推進用高圧スイッチ）: https://qmro.qmul.ac.uk/xmlui/handle/123456789/57882
- NTRS 小型PPU研究: https://ntrs.nasa.gov/citations/20140011254
- Springer Nature「Multimodal electrospray thruster」: https://link.springer.com/article/10.1007/s44205-024-00075-0

（本文中「訓練知識で補足」と明記した箇所は、Web検索で一次情報の裏付けが
取れなかったため、調査担当の学習済み知識で補完した内容であることを示す。）
