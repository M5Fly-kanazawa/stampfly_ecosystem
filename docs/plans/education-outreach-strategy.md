# StampFly Ecosystem 教育普及戦略 / Education Outreach Strategy

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

StampFly Ecosystem を「広い教育階層」——小中高・高専・大学学部・大学院/研究者・社会人——に浸透させるための普及戦略。2026年7月に実施した内部資産棚卸し（教材在庫・導入体験・教育思想・無機体学習環境）と外部環境調査（競合分析・日本の教育制度と規制・市場現状・意思決定プロセス・教員支援・安全/保険・海外展開）の結果に基づく。

### ステータス

| 項目 | 状態 |
|------|------|
| 調査（内部棚卸し＋外部環境、計12テーマ） | ✅ 完了 (2026-07-08) |
| 戦略策定 | ✅ 本文書 (2026-07-08) |
| Horizon 1 実行 | ▶ 実行中（〜4週前半完了: v2026.07.1+フラッシャ2系統。§7 を 2026-07-09 改訂） |

### 結論（要旨）

| # | 結論 |
|---|------|
| 1 | **市場に空白がある。** DJIがTello/Tello EDU/RoboMaster TT教育事業を2023年末に終了し、2026年時点で公式後継はない。教育用ドローン市場の最大手が消えた空白を、CoDrone EDU（クローズド・小中高止まり）とHula-JP（カリキュラム薄い）が埋めつつある段階。 |
| 2 | **StampFly Ecosystemの構造的な独自性は「学年を超えて成長できる唯一の白箱」。** 小学生（micro:bit）から研究者（ESKF・システム同定・SILS）まで、同一機体・同一リポジトリ・4階層API（L0〜L3）で連続的に登れるプラットフォームは調査した限り世界に存在しない。 |
| 3 | **最大の課題は供給側（教材の質）ではなく需要側（採用のしやすさ）。** 学校導入向け機体比較記事にStampFlyは候補として載っていない。ESP-IDFビルド必須・教員向け完成パッケージ不在・クラスセット調達導線不在・安全/保険の制度整備不在が導入を阻んでいる。 |
| 4 | **橋頭堡（ビーチヘッド）は高専・大学学部。** 意思決定者は教員個人で、国立大学の目安では30〜50万円未満なら教員裁量で即決できる。1万円強のStampFlyは10台クラスセットでも裁量枠に収まる。教材も9割完成しており、最小の追加投資で最大の成果が出る層。 |
| 5 | **小中高はパートナーモデルで臨む（Horizon 2）。** 非情報系教員にリポジトリとAI対話を渡すモデルは成立しない。StampFly Edu（micro:bit + MakeCode）の製品化と、指導案・出前授業・教員研修を担うパートナー網が前提条件。 |

## 2. 現状分析

### 2.1 市場機会：Tello撤退が生んだ空白

| 事実 | 出典・時期 |
|------|-----------|
| DJIが教育製品（Tello / Tello EDU / RoboMaster EP・TT）の販売終了を告知 | 2023-12-29告知、サポートは2024-12まで |
| 国内代理店の2026年版学校導入向け比較記事は「Tello EDUは廃番のため除外」と明記 | HDL合同会社、2026-02 |
| 2026年時点でDJIの教育事業再開・後継機の公式発表なし | 2026-07-08調査時点 |

プログラミング教育用ドローンのデファクトが消滅し、教育現場は代替を探している。この空白は時間とともに他社（CoDrone EDU・Hula-JP等）が埋めるため、**参入タイミングとして今が最良**。

### 2.2 競合ポジション（2026年7月時点）

| 製品 | 単価 | 対象層 | ソース公開 | 教育浸透の仕組み | 弱点 |
|------|------|--------|-----------|----------------|------|
| **StampFly** | $49.95 / ¥10,439 | （潜在的に）小中〜研究者 | 完全公開（MIT） | 本文書で構築 | 導入摩擦・可視性・パッケージ不在 |
| CoDrone EDU | $249 / ¥59,800 | 小5〜高校 | 非公開（バイナリのみ） | クラスセット+PD研修+指導案+競技会（ドロカツ）+代理店網。米9,000校・日本140校以上 | 高価格・クローズド・制御則を学べない・上位層で頭打ち |
| Crazyflie 2.1+ | $240〜（バンドル$270〜$5,000） | 大学〜研究者 | 完全公開（GPL3） | 研究プラットフォームが大学講義へ降りる（Princeton・UC Berkeley・Chalmers・CMU） | 高価格・英語圏中心・K-12に降りられない |
| Hula-JP | ¥49,500 | 小中 | 不明 | 代理店（レッドクリフ）経由。産技高専等に導入 | 「体験重視・年間カリキュラム弱い」（代理店比較記事の評） |
| ESP-Drone | — | — | 公開（GPL3） | なし | 2022年末以降ほぼ休眠・教育実績なし |
| PX4 / ArduPilot | 機体次第 | 大学院〜研究 | 公開 | 学会・Developer Summit・トレーニング拠点 | 初学者に不向き・機体高価 |

**含意:** Bitcrazeは自社ブログ（2026-01）で「Students don't outgrow the platform. They grow with it.（学生はプラットフォームを卒業しない。共に成長する）」を差別化軸に掲げた。しかしCrazyflieの梯子は大学から始まり、CoDroneの梯子は中高で終わる。**StampFlyだけが梯子の全段を持てる**——ただし現状、その梯子は外から見えていない。

### 2.3 規制環境：100g未満の制度的優位

| 項目 | 内容 |
|------|------|
| 航空法（無人航空機規制） | 100g未満（StampFlyは約37g）は機体登録・飛行許可承認の対象外（「模型航空機」区分、2022年改正） |
| 屋内飛行 | 体育館・教室等の屋内は重量によらず航空法の適用対象外 |
| 残る制約 | 施設管理者の許可（自治体により飛行計画書・保険加入確認・監視員配置を要求する例：前橋市、足立区。条例で体育館飛行を原則禁止する自治体もあり）、小型無人機等飛行禁止法（重要施設周辺） |

「登録も許可申請も不要で、体育館でそのまま授業ができる」は競合の100g超機（CoDrone EDU 54.8gは同等、Hula-JPも100g未満）に対しては差ではないが、**産業用・研究用ドローン教育全般に対しては圧倒的な導入の軽さ**であり、営業資料の第一の訴求点になる。

### 2.4 内部資産の棚卸し

#### 教材資産（供給側はほぼ揃っている）

| 資産 | 内容 | 完成度 | 対象層 |
|------|------|--------|--------|
| ワークショップ（`docs/events/stampfly_workshop/`） | 4+1日間・12〜13レッスン（環境構築→モータ制御→P制御→システム同定→PID→姿勢推定→Python SDK）+ Day5競技会。Beamerスライド+TikZ図25点+講師ガイド+競技ルール+アンケート設計（21問） | ◎（アンケートは未実施） | 高専・大学 |
| 大学15回カリキュラム（`docs/university/` + `analysis/notebooks/education/`） | 半期15回（90分）シラバス+評価ルーブリック+**Jupyterノートブック01〜15は実装済み**（理論→シミュレーション予習→同梱サンプルデータ解析→実機実験（任意）の4段構成。実機なしでも各回の大半が完結） | ◎（ただしシラバスからノートブックへのリンクがなく「在るのに見えない」） | 学部3〜4年 |
| Examples（`examples/education/`） | Python実装例8本（hello_flight〜waypoint_mission・Allan分散）。全例が `connect_or_simulate()` 経由のため**実機なしでもそのまま動く**（接続失敗時は純Pythonシミュレータへ自動フォールバック） | ◎ | 入門〜中級 |
| 無機体学習環境 | `sf sils gui`（ブラウザでシナリオ作成・パラメータ54個編集・3Dリプレイ・合否ゲート表示、依存ゼロ）+ SILSシナリオ39本 + VPythonシミュレータ + 同梱サンプルデータ5本（`analysis/datasets/education/`）+ 実機ログ150本以上（`logs/`） | ◎（購入前に体験できる導線として未宣伝） | 全層 |
| ガイド（`docs/guides/`） | 安全・用語集・トラブルシューティング等6本、日英併記 | ◎ | 全層 |
| 4階層API | L0 `ws::*`（2関数で完結）→ L1 `sf::api`（制御則・推定器の差し替え）→ L2 HALラッパー（組込み学習）→ L3 `sf::internal`（ファーム実装） | ◎（設計思想として文書化済み） | 小中高〜実装者 |
| SILS + Code Identity | 実機と同一ソースを参照コンパイルするシミュレータ。SILSでPIDチューニング→実機書き込みが同一パラメータで通る | ◎ | 学部〜研究者 |
| Tello SDK互換API | djitellopy（Tello用Pythonライブラリ）が無改変で動く互換層。UDP:8889/8890は実機ファームのみが開くため**実機専用の入口**（SILSは文字列注入方式で検証） | ○（SILS検証済・実機未検証） | 中高〜Python入門層 |
| コード＝教材の規約 | 1関数50行以内・バイリンガルコメント・マジックナンバー禁止・@designタグ（設計文書への参照+判定） | ◎ | 全層 |

#### 実績資産（信用と接点）

| 実績 | 時期 |
|------|------|
| JUIDA理事長賞受賞「StampFly Ecosystem―あなた専用のDX/制御教育をAIと作る基盤」（Japan Drone 2026ポスターセッション） | 2026-06 |
| 金沢市「デジタル科」（市立小中の新設科目）での中学校StampFly体験授業 | 2025-10〜 |
| 兵庫県立西脇工業高校ドローン講習会（教職員向け+生徒向け） | 2024-12 |
| StampFly Edu共同研究開発（FAP factory・金沢工業大学・金沢エンジニアリングシステムズ、micro:bit+MakeCode+Tello-Bridge、小中学生向け） | 2024-10開始・プロトタイプ段階 |
| connpassハンズオン（石川、参加22/30名） | 2025-06 |
| 第67回自動制御連合講演会発表（M5Stack・スイッチサイエンス等と共同） | 2024-11 |
| ZEPマガジン連載・個人ブログ「ドローンの世界への扉」全30回連載・docswell公開スライド多数 | 2024〜 |
| GitHub: 本体Star 40/Fork 50、Workshop Star 5/**Fork 55** | 2026-07 |

**注目点:** WorkshopリポジトリのFork数（55）がStar数（5）を大きく上回る。これはハンズオン参加者が各自フォークして演習した実態を示す——**ワークショップ自体が最も機能している獲得チャネル**である証拠。

**「在るのに見えない」問題:** 教育ノートブック01〜15は実装済みなのに `docs/university/` のシラバスから接続されておらず、`analysis/notebooks/README.md`・`analysis/datasets/README.md` は「計画中」という古い記述のまま実体と乖離している。本調査でも当初「ノートブック未実装」と誤認されたほどで、**外部の教員が資産の全貌を発見するのはほぼ不可能**。供給側の実力と需要側から見えている姿のギャップが本エコシステム最大の構造問題であり、§2.5 欠落6（可視性）はリポジトリ内部の導線にも及ぶ。

### 2.5 弱点：需要側から見た6つの欠落

| # | 欠落 | 事実 | 影響する層 |
|---|------|------|-----------|
| 1 | **ビルド必須の導入障壁** | vehicleファームはESP-IDF環境構築+自前ビルドが唯一の導入経路。ビルド済みバイナリ配布なし・GitHub Releasesなし・所要時間の記載なし（controllerのみM5Burner用バイナリあり） | 全層（特に小中高教員には致命的） |
| 2 | **教員向け完成パッケージ不在** | K-12向け指導案・教員研修・認定制度・出前授業がない。競合CoDroneはPD研修2時間・5E形式指導案・出前授業（「学校側の負担は軽微」）を完備 | 小中高 |
| 3 | **調達導線不在** | クラスセット・数量割引・見積もりページがない。競合は12台$3,999（研修込）・PO払い・月額RaaSまで用意 | 全層 |
| 4 | **安全・保険の制度整備不在** | `docs/guides/safety.md` は運用手順のみ。保険・責任分担・施設許可書式・教員向け安全講習はカバー外。国レベルの統一基準も存在しない（＝先取りの機会） | 小中高・自治体 |
| 5 | **英語化の遅れ** | 教材スライド・演習資料は日本語のみ。海外フォーラムで英訳の明示的要望あり。StampFlyに関する英語論文ゼロ。GitHub Discussions無効 | 海外 |
| 6 | **可視性ゼロ** | 学校導入向け機体比較記事（2026年版）にStampFlyは不掲載。Qiita記事2件・Zenn記事0件。mkdocsサイト未公開。landingページは公開済み（2026-06-07、https://m5fly-kanazawa.github.io/stampfly_ecosystem/ ）だが**README・docsから一切リンクされておらず発見不能**（「在るのに見えない」の実例） | 全層 |

### 2.6 意思決定構造（誰が採用を決めるか）

| 層 | 意思決定者 | 決裁の壁 | 含意 |
|----|-----------|---------|------|
| 小中高（公立） | **校長**（補助教材は学校教育法34条2項に基づき校長が選定。教育委員会は届出制/承認制を自ら規則で定める） | 保護者負担への配慮・教委の届出/承認 | 教委への一括営業より、校長・教員への直接アプローチ+制度的受け皿（金沢デジタル科型）の二正面 |
| 高専・大学 | **教員個人** | 国立大の目安：50万円未満は教員発注可、30万円超で相見積り | **10台セット約12万円は教員裁量で即決可能**。スイッチサイエンスは高専機構と口座開設済みで請求書払い対応済み |
| 自治体（大量導入） | 教育委員会・共同調達協議会 | 入札・年度予算 | GIGA型共同調達は端末向けでドローン補助教材には及ばない。長期戦 |
| 社会人・ホビイスト | 本人 | なし | 価格1万円は衝動買い圏。コンテンツ量が決める |

### 2.7 K-12のクライアント端末環境（GIGAスクール構想）

児童生徒が使うのは学校管理下の1人1台端末であり、そのOS構成が小中向け提供物の技術方式を規定する。

| 学校段階 | 端末環境（2025〜26年時点） | 出典 |
|---------|--------------------------|------|
| 小中学校 | **ChromeOS 60%・iPadOS 31%・Windows 10%**（NEXT GIGA更新後。第1期の40/29/30%からChromeOSが+18pt、Windowsが−19pt） | MM総研 2025-07調査 |
| 高等学校 | 公費50.8%／BYOD49.2%（2024-05文科省）。OSは**Windows優勢**（2021年時点・都道府県数ベースで46%、ChromeOS 30%・iPad 8%） | 文科省・MM総研 |

**管理下端末の技術的制約（裏取り済み）:**

| 項目 | 事実 |
|------|------|
| ChromebookのWeb Serial（ブラウザからUSBシリアル書き込み） | Chrome 89+で技術的に可。ただし教育委員会の管理コンソール（`DefaultSerialGuardSetting`等）でブロックされ得る。MakeCodeのWebUSB書き込みは「USBストレージ禁止ポリシー下でも動作した」実践報告があり、教委への前例として提示可能 |
| ChromebookのLinux環境（Crostini）・アプリ | 学校管理端末では一般に無効化・ホワイトリスト運用。**ローカルにPythonや開発環境を入れる前提は成立しない** |
| iPad（31%） | SafariにWeb Serial/WebUSBは**存在しない**（WebKitが公式に実装拒否）。ブラウザからの書き込みは構造的に不可能。micro:bitは専用アプリ+Bluetooth経由のみ |
| ブラウザとUDP | ブラウザは生のUDPを話せないため、**Tello互換API（UDP:8889）はChromebook/iPadのブラウザから使えない** |

**戦略への含意:**

1. **小中向け提供物は「ブラウザ完結」が絶対条件。** 端末の91%がChromeOS/iPadOSであり、インストール型（ESP-IDF・Python・sf CLI）は論外。この制約下で成立している唯一の自社資産がStampFly Edu（micro:bit + MakeCode = ブラウザ+WebUSB）であり、**アーキテクチャ選択の正しさが端末シェアで裏付けられた**
2. **現状、小中の91%端末から機体へつながる経路が存在しない。** ブラウザはUDP不可・Python不可のため、現行vehicleファーム（ネットワーク面はUDPのみ）はChromebookから直接触れない。ただし旧`vehicle_old`にはWebSocketサーバ（ポート80、400HzテレメトリでROSブリッジが接続）の実装実績があり、**現行vehicleへの再移植で「ブラウザ・コックピット」（機体SoftAPにChromebookを接続→ブラウザだけで操縦・テレメトリ・Blockly実行）が開ける**。技術は社内実証済み（→H2-7）。なおROSブリッジは現在vehicle_oldのWebSocket前提のまま現行vehicleと非互換であり、再移植はこの整合性問題も同時に解消する
3. **Webフラッシャ（P0-1）はChromebookで動くがiPadでは動かない。** iPad自治体（31%）では「書き込み済み機体を配る」運用（教員・パートナー・販売時書き込み）が必須。P0-1の導入手引きには教委向けの管理コンソール許可設定（Web Serial）の記載を含める
4. **高校はWindows優勢+BYODのため、Python/Tello SDK互換の従来路線がそのまま通る。** 小中と高校で技術方式を分ける根拠が明確になった

## 3. 戦略の核

### 3.1 ポジショニング：「卒業しない教材」

> **StampFly Ecosystem は、小学生から研究者まで同じ機体で登り続けられる、世界で唯一のオープンなドローン制御教育プラットフォーム。**

3本の柱：

| 柱 | 内容 | 対抗軸 |
|----|------|--------|
| **白箱（ホワイトボックス）** | PID・ESKF・プロトコル・ミキサーまで全ソースが読める（MIT）。「ブラックボックスのAPIを呼ぶ」ではなく「中身を理解し差し替える」教育ができる | CoDrone EDU（クローズド）・Hula-JP |
| **梯子（ラダー）** | micro:bit/MakeCode（小中）→ Python/Tello SDK互換（中高）→ C++/4階層API（高専・学部）→ ESKF・システム同定・SILS（院・研究）。買い替え不要・環境乗り換え最小 | CoDrone（上で頭打ち）・Crazyflie（下に降りない） |
| **軽さ** | 約37g・100g未満で航空法の登録/許可不要、屋内授業がそのまま成立。価格は競合の1/5（$49.95） | 産業機ベースの教育全般 |

補助メッセージ（上位層向け）：**AIと作る自分専用カリキュラム**（JUIDA理事長賞受賞ビジョン）。リポジトリ全体が設計文書・要件・実装・シミュレータまで一貫して機械可読なため、AIに文脈を与えて自分の授業・研究に合わせた教材を生成できる。ただしこれは高専・大学教員・社会人など「自分でカリキュラムを設計する自由と能力がある層」への訴求であり、**K-12教員には完成品パッケージで臨む**（AI自作を求めるのは負荷の転嫁になる）。

### 3.2 階層別戦略マトリクス

| 層 | 入口 | 提供物（既存→要整備） | チャネル | 時期 |
|----|------|---------------------|---------|------|
| 小中 | StampFly Edu（micro:bit + MakeCode）——**ブラウザ完結必須**（端末はChromeOS 60%+iPad 31%、§2.7） | プロトタイプ→**製品化・指導案・出前授業・保険/安全パック**。将来はブラウザ・コックピット（H2-7） | FAP factory等パートナー・金沢モデル横展開・校長直接ルート | Horizon 2 |
| 高校（工業・SSH・探究） | Python（Tello SDK互換）+ 完成ファーム——**Windows優勢+BYODのためインストール型が通る**（§2.7） | 西脇工業の実績→**探究テーマ集・教員向け半日研修** | 工業高校ネットワーク・SSH課題研究・村田財団等の助成枠 | Horizon 1後半〜2 |
| **高専・大学学部（橋頭堡）** | ワークショップ+15回カリキュラム | 教材9割完成→**ノートブック完成・クラスセット導線・導入事例パック** | 学会（SICE等）・教員個人裁量予算・高専機構口座（開設済） | **Horizon 1** |
| 大学院・研究者 | SILS・システム同定・ESKF・ソース全体 | 完成→**英語論文・比較ベンチマーク公開** | 国際会議・研究室間の口コミ（Crazyflie型） | Horizon 1〜2 |
| 社会人・ホビイスト | 完成ファーム+ハンズオン+書籍 | ブログ/連載→**書籍化・M5Burner配布・定例ハンズオン** | M5Stackコミュニティ・connpass・技術書店/CQ系 | Horizon 1 |

### 3.3 橋頭堡の論理

高専・大学学部を最初に取る理由：

1. **教材の完成度が最も高い**（ワークショップ・シラバス・Examples）——追加投資が最小
2. **意思決定が最速**——教員個人の裁量予算で即決でき、決済インフラ（スイッチサイエンス×高専機構）も既にある
3. **本人の信用資産が最大**——大学教員・学会発表・JUIDA受賞は同業教員への説得力に直結
4. **上下への波及点**——高専・大学の教員は出前授業・オープンキャンパス・ジュニアドクター育成塾等を通じてK-12への配信主体にもなり、卒業生は社会人・研究者になる。梯子の中段を押さえると上下に伸ばせる
5. **競合不在**——この層で「日本語・安価・白箱・カリキュラム完備」を満たす選択肢は他にない（CrazyflieはL1相当のみ・英語・2.4倍の価格）

### 3.4 データフライホイール（証拠の蓄積）

競合CoDroneのカリキュラムすら「教育効果は正式に評価されていない」（第三者レビュー、2026）。**教育効果のエビデンスを学術的に出せるのは現役教員である本人の固有優位**であり、クローズド企業には模倣困難な堀になる。

```
ワークショップ/授業実施 → アンケート・学習ログ収集 → 教育効果の分析・論文化
        ↑                                                    ↓
   導入校の増加 ←── 財団助成・学会発表・導入事例パックの説得力向上
```

設計済みで未実施の21問アンケート（`docs/events/stampfly_workshop/survey/`）を**全イベントで必ず実施**することから始める。

## 4. 実行計画（3 Horizons）

### Horizon 1（〜2027-03）：摩擦の除去と橋頭堡の確立

#### P0（最優先・先行依存タスク）

| # | 施策 | 内容 | 解消する欠落 |
|---|------|------|-------------|
| P0-1 | **ビルド不要化** | GitHub Releasesでvehicle/controllerのビルド済みバイナリ配布+ブラウザ書き込み（ESP Web Tools等のWebSerial方式）+M5Burner登録。「箱を開けて15分で飛ぶ」を実現し、所要時間をドキュメントに明記。WebSerialはChromebookでも動くが教委ポリシーで要許可設定（手引きに記載）、**iPadでは不可**のため書き込み済み機体の配布運用も用意（§2.7） | 欠落1 |
| P0-2 | **オンライン可視化** | mkdocsサイトの公開（PagesはlandingのみのためlandingとPagesアーティファクトを統合し `/docs/` 配下に併載するのが最短）・公開済みlanding（2026-06-07〜）へのREADME/SNS/検索導線の整備・トップに「あなたはどの階層？」入口ルータ（小中/高校/高専大学/研究/ホビー別の最短経路） | 欠落6 |
| P0-3 | **既存教材の接続と動作保証** | ノートブック01〜15は実装済み——残作業は (a) シラバス⇔ノートブックのリンク接続、(b) 古い「計画中」READMEの現状化、(c) 無機体経路の動作保証（`sf sim run` のコントローラ無し起動バグは2026-07-08修正済み。`sf sils gui`・全ノートブックの通し確認）、(d) 「機体を買う前にブラウザで試す」導線の明文化 | 欠落6・橋頭堡の弾薬 |
| P0-4 | **クラスセット調達導線** | 「教育機関向け導入ページ」：10台/20台構成例・見積もりテンプレート・スイッチサイエンスB2B窓口への導線・教員裁量予算に収まる価格表 | 欠落3 |
| P0-5 | **安全・運用パック** | 保険の考え方（任意・施設側要求例）・施設許可申請書式（前橋市/足立区の様式を参考に雛形化）・責任分担の整理・教員向け安全チェックリスト。国の統一基準が不在の今、**デファクトを先取り** | 欠落4 |

#### P1（Horizon 1内で並行）

| # | 施策 | 内容 |
|---|------|------|
| P1-1 | ワークショップの定期開催+アンケート必須化 | 年3回以上（学会併設・オープンキャンパス・connpass）。毎回n=20〜30の効果データを蓄積 |
| P1-2 | 高専・大学パイロット5校 | 評価キット貸出（5台×5校）+導入教員への個別サポート。導入事例パック（写真・シラバス・学生の声）を作る |
| P1-3 | 学会チュートリアル | SICE等に「小型ドローンで学ぶ制御工学」チュートリアル/講習会を提案（調査の限りドローン特化チュートリアルは空白） |
| P1-4 | 書籍企画 | ブログ連載30回+ZEP連載を母体に商業出版1冊（CQ出版Interface特集の打診と並行） |
| P1-5 | Tello SDK互換の実機検証 | 中高・Python層の入口として重要。djitellopy無改変動作を実機で確認し「Telloの教材がそのまま動く」と言い切れる状態に |

### Horizon 2（2027年度）：K-12パートナーモデルと英語化

| # | 施策 | 内容 |
|---|------|------|
| H2-1 | StampFly Edu製品化 | FAP factory・KESとの共同開発を製品化まで推進。価格・保護カバー・クラスセット・指導案（45分×N コマの完成品）をセットで |
| H2-2 | 出前授業・研修パートナー網 | HDLモデル（出前授業で教員負担ゼロ）を参考に、地域パートナー（高専・大学研究室・企業）が出前授業を担える研修+教材キットを整備。認定講師制度（軽量版）を開始 |
| H2-3 | 金沢モデルの横展開 | 金沢市デジタル科（制度的受け皿+大学連携）の枠組みを文書化し、2自治体へ提案。ジャパンドローンプログラミングチャレンジ（金沢が全国大会地）との連携 |
| H2-4 | 英語化第一弾 | ワークショップスライドEN版（構成は既にバイリンガル規約準拠）・GitHub Discussions有効化・英語論文1本（IEEE EDUCON / IFAC教育シンポジウム等）・Elektor誌へ再レビュー働きかけ（2025-03レビューの指摘＝ホバリング/POS_HOLD未実装は解消済み） |
| H2-5 | 競技会の確立 | 既存受け皿の活用（全日本学生室内飛行ロボコン：StampFlyは全部門の重量制限内）+自前の「StampFly Cup」（`competition_rules.md`のホバリング競技を学会・オープンキャンパス併設で定例化） |
| H2-6 | 助成金の獲得 | 村田学術振興・教育財団（高校・高専STEAM）・パナソニック教育財団・ちゅうでん教育振興財団（高専）・経産省STEAMライブラリー・JSTジュニアドクター育成塾（高専経由の小中プログラム） |
| H2-7 | **ブラウザ・コックピット** | `vehicle_old`実装実績のあるWebSocketサーバ（ポート80）を現行vehicleへ再移植し、機体SoftAP+ブラウザだけで操縦・テレメトリ表示・Blockly実行を可能に。ChromeOS/iPad 91%の小中端末から**インストールゼロ**で機体に触れる唯一の経路（§2.7）。ROSブリッジの現行vehicle非互換も同時に解消 |

### Horizon 3（2028年度〜）：スケールと制度化

| # | 施策 | 内容 |
|---|------|------|
| H3-1 | 教材流通ルート | 教材卸（サクラクレパスeduce-web等）・教育専門代理店との提携。学校の既存伝票処理に乗せる |
| H3-2 | 団体レベルの認定 | JUIDA等との連携を個人表彰から団体提携・認定教材へ。高専機構との包括的な取り組み |
| H3-3 | 海外パイロット | 英語教材を武器に海外2大学で試行（Crazyflieの価格帯に届かない層＝新興国・コミュニティカレッジが狙い目。GitHubスター分析ではインド・スリランカ・ベトナム・米州立大の個人関心を確認済み） |
| H3-4 | コミュニティの自走化 | コントリビューションガイド・メンテナ育成・定例オンラインミートアップ。bus factor=1 の解消 |

## 5. リスクと対策

| リスク | 内容 | 対策 |
|--------|------|------|
| **供給継続** | M5StackがStampFlyを終売する可能性（Telloの教訓そのもの） | protocol/spec SSOTとHAL分離により機体非依存の教材構造を維持。BOM/回路図が公開されている強みを活かし、後継機・互換機に教材ごと移植可能な状態を保つ。JUIDA受賞理由の「製品継続性の問題の解決」を実装で裏付ける |
| **属人性** | 開発・教材・講師・営業が事実上1人に集中 | train-the-trainer（H2-2）・認定講師・コントリビューションガイド（H3-4）で分散。書籍・動画は「本人がいなくても学べる」资産 |
| **事故** | 学校での事故1件が市場全体を閉ざす | P0-5安全パックを導入拡大に**先行**させる。プロペラガード必須・飛行エリア規程・保険案内を教材に組み込み済みであることを営業資料の冒頭に |
| **AI幻覚** | 「AIとカリキュラム自作」訴求で低品質教材が生成され信頼を毀損 | 教員レビューループの明記（既存EVIDENCE.mdの方針を維持）・検証済みカリキュラムを正典として提供し、AIは「正典の改変・拡張」に位置づける |
| **バージョン漂流** | 教材と進化し続けるファームの不整合 | カリキュラム⇔ファームウェアreleaseの対応表を維持。教育機関には「学期中はバージョン固定」を推奨 |
| **競合の低価格化** | CoDrone等が価格を下げて白箱の弱みを突かれる前に | 価格ではなく「白箱+梯子+エビデンス」の堀を先に築く（データフライホイール） |

## 6. KPI

| KPI | 現状 (2026-07) | Horizon 1 (2027-03) | Horizon 2 (2028-03) | Horizon 3 (2029-03) |
|-----|---------------|--------------------|--------------------|--------------------|
| ビルド不要導入（Webフラッシャ） | なし | 公開 | — | — |
| ドキュメントサイト | 未公開 | 公開 | 英語版 | — |
| 高専・大学の正課/準正課導入 | 1（自校） | 5校 | 12校 | 25校 |
| 小中高の実施校（出前授業含む） | 2〜3（金沢・西脇） | 5校 | 20校 | 50校 |
| ワークショップ/ハンズオン開催 | 単発 | 年3回+アンケート100% | 年6回（パートナー開催含む） | パートナー自走 |
| 効果測定データ（アンケートn） | 0 | 60 | 300 | 1,000 |
| 教育系論文・解説記事 | 国内学会2件 | +国内2件 | +英語1件 | +英語2件 |
| 書籍 | 0 | 企画成立 | 出版 | 増刷/2冊目 |
| GitHub Star（ecosystem） | 40 | 150 | 400 | 1,000 |
| 助成金採択 | 0 | 1件 | 2件 | 3件 |

## 7. 直近アクション（90日）

### 改訂履歴

| 日付 | 内容 |
|------|------|
| 2026-07-08 | 初版（週割りは本節末尾の「完了済み」参照） |
| 2026-07-09 | 全面改訂: v2026.07.1 完了実績を反映し、(1) DXH高校教員向け体験講座（7/18-19）、(2) 「フラッシュだけユーザー」向け段階的ドキュメント体系の整備、の2軸で書き直し |

### 方針転換の要点

| # | 転換 | 理由 |
|---|------|------|
| 1 | 「フラッシュだけユーザー」（開発環境を持たない教員・生徒）を一級のユーザー像として位置づけ、ドキュメント体系を再構成する | v2026.07.1 でビルド不要の書き込み手段（Web/GUIフラッシャ）が完成し、導入の壁はツールからドキュメントへ移った。棚卸し（2026-07-09）の結論: パラメータ調整・ログ取得は開発環境なしで既に可能（`sf log wifi`・USBシリアル `param`）だが、その事実と手順を伝える非開発者向けドキュメントが存在しない |
| 2 | 高校（DX・情報教育）向けカリキュラムを高専・大学と並ぶ柱に引き上げる | DXH（高等学校DX加速化推進事業）の高校教員向け体験講座（7/18-19）が確定し、教員直接支援の形で高校に接地する機会が前倒しで到来。当初プランでは K-12 は Horizon 2（パートナー前提）だったが、教員研修型はこの限りではない |
| 3 | Windows実機検証は「貸出PCでのリハーサル+ベータテスタ」の2本立てに変更 | 手元に Windows 実機はないが、DXH 講座の貸出 Win11 PC でリハーサル検証が可能。ベータテスタには一般ユーザー環境の多様性（管理者権限なし・SmartScreen 等）の検証として協力を仰ぐ |

### 週割り（2026-07-09 起点）

| 期限 | アクション |
|------|-----------|
| 〜7/17（DXH準備・最優先） | ① 2時間カリキュラム確定（操縦体験→ビルド不要書き込み体験→開発環境の構築→プログラム書き換え・モータ制御。大学HP告知文に準拠。**PC(Win11/Core i5)・StampFly・Controller・USB-Cケーブルは全て大学からの貸し出し**のため環境の事前セットアップ・統一が可能=当日のインストールトラブルリスクを排除できる。「開発環境のインストール」パートは事前構築済み環境+手順のデモ/資料で構成）。② 当日教材の作成: 手順書（スクリーンショット付き書き込みガイドの初版を兼ねる）+ 進行スライド。③ 貸出 Win11 PC でのリハーサル（GUIフラッシャ .exe と Webフラッシャの実機検証を兼ねる。ベータテスタへの協力依頼は一般ユーザー環境の検証として並行実施）。④ 機材・会場準備（貸出PCの事前セットアップ〔フラッシャ・USBドライバ・開発環境〕、機材は**1人1セットずつ貸出で確定**=1人1台の演習形式〔定員10名分+予備機・予備バッテリー〕、飛行スペースの安全確保）。⑤ 所要時間実測（リハーサルで install→flash→初飛行を計測し、ドキュメントへ還流） |
| 7/18-19 | DXH 高校教員向け体験講座 本番（両日）。終了時アンケート実施（教員フィードバック第1号=データフライホイールの起点）。改善点を即日メモ化 |
| 〜8/7（4週） | 「フラッシュだけユーザー」ドキュメント体系スプリント（次セッションから実装）: ① 入口ページ（読者ルーティング: 飛ばしたい/教えたい/開発したい） ② 初回体験ガイド（入手・組み立て初期チェック→書き込み→初飛行） ③ 操縦・UIリファレンス（operation_manual を非開発者向けに再構成） ④ パラメータ調整入門（何を・なぜ・どう変えるか） ⑤ ログ取得入門（`sf log wifi` は開発環境不要であることを明記） ⑥ 「離着陸ができたら次へ」ガイド（Tello互換API→教育ノートブック→ソースビルドの段階導線） ⑦ トラブルシューティング拡充（LED/ブザーの状態別・症状別）。DXH の実測値・フィードバックを反映する。教育機関向け導入ページ（クラスセット・見積もり導線）の起草 |
| 〜9/3（8週） | SCI/SICE チュートリアル講座（9/10 大阪+Zoom）準備: 参加者事前準備手順書（DXH教材の発展版）+ 丸一日カリキュラム。教材接続スプリント（シラバス⇔ノートブック01〜15リンク、docsサイトのリンク切れ65件解消）。Windows ベータテスト結果の反映 |
| 〜10/8（12週） | 高校向け授業パッケージ公開（DXH実施結果を反映した指導案+教材の一式化）。安全・運用パック起草（前橋市/足立区様式の雛形化）。Tello SDK互換の実機検証。評価キット貸出プログラム設計（5校分）。書籍企画書 |

### 完了済み（初版の〜2週+〜4週前半、2026-07-08 時点）

| 項目 | 実績 |
|------|------|
| GitHub Releases 整備 | ✅ v2026.07.0 / v2026.07.1 発行。CalVer 規約（docs/contributing/versioning.md）、vehicle/controller × full/app + SHA256SUMS |
| ビルド不要の書き込み手段 | ✅ Webフラッシャ公開 LIVE（/flash/）+ デスクトップGUI「StampFly Flasher」3 OS 版をリリース添付（v2026.07.1）。リリース→サイト自動再デプロイの構造欠陥（GITHUB_TOKEN イベント抑止）も修正済み |
| docsサイト・導線 | ✅ mkdocs サイト公開、README/landing からのリンク整備、/flash/ ページの動的ダウンロードボタン |

## 8. 参考：調査の出典

本戦略の根拠となった調査結果（2026-07-08実施）の主要出典：

| 領域 | 主要出典 |
|------|---------|
| Tello撤退 | dronedj.com (2023-12-29)、HDL比較記事 (2026-02) |
| CoDrone EDU | robolink.com（価格・PD研修・クラスパック）、hdl-edu.com（日本展開・出前授業）、robotlab.com（RaaS） |
| Crazyflie | store.bitcraze.io、bitcraze.io ブログ（2025-07 大学採用、2026-01 教育市場分析） |
| 規制 | mlit.go.jp（100g基準）、前橋市・足立区の施設ガイドライン、npa.go.jp（小型無人機等飛行禁止法） |
| 意思決定構造 | mext.go.jp（補助教材の取扱い）、群馬大学会計ハンドブック（教員裁量枠）、switch-science.com（高専機構口座・B2B） |
| 市場現状 | shop.m5stack.com・switch-science.com（価格/在庫）、GitHub API（Star/Fork）、connpass、kanazawa-it.ac.jp（JUIDA受賞） |
| 海外 | elektormagazine.com（2025-03レビュー）、community.m5stack.com（英訳要望）、researchmap（英語発表歴） |
| GIGA端末環境 | MM総研（2025-07 NEXT GIGA調査: ChromeOS 60%/iPadOS 31%/Windows 10%）、文科省（高校端末整備 2024-05: 公費50.8%/BYOD49.2%）、chromeenterprise.google（`DefaultSerialGuardSetting`）、WebKit standards-positions（Web Serial/WebUSB実装拒否）、micro:bit公式（ChromebookでのWebUSB動作要件） |

---

<a id="english"></a>

## 1. Overview

### About This Document

An outreach strategy for making StampFly Ecosystem penetrate the full educational range — elementary/junior-high/high school, KOSEN (technical colleges), undergraduate, graduate/researchers, and working adults. Based on internal asset inventory and external environment research (competitors, Japanese education system and regulations, market status, adoption decision processes, teacher support, safety/insurance, overseas expansion) conducted in July 2026.

### Status

| Item | State |
|------|-------|
| Research (internal + external, 12 themes) | ✅ Done (2026-07-08) |
| Strategy formulation | ✅ This document (2026-07-08) |
| Horizon 1 execution | ⏳ Not started |

### Key Conclusions

| # | Conclusion |
|---|-----------|
| 1 | **There is a market vacuum.** DJI discontinued its education line (Tello / Tello EDU / RoboMaster TT) at the end of 2023 with no official successor as of 2026. |
| 2 | **StampFly Ecosystem's structural uniqueness is being "the only white-box platform students never outgrow."** No other platform spans elementary school (micro:bit) to research (ESKF, system identification, SILS) on one airframe, one repository, and a 4-tier API. |
| 3 | **The biggest problem is demand-side (ease of adoption), not supply-side (material quality).** StampFly does not even appear in the 2026 school-adoption drone comparison articles; ESP-IDF build requirement, missing teacher packages, missing procurement paths, and missing safety/insurance guidance block adoption. |
| 4 | **The beachhead is KOSEN and undergraduate programs.** Adoption there is decided by individual instructors within discretionary budgets (roughly under 300–500k JPY at national universities); a 10-unit StampFly class set (~120k JPY) fits easily, and the materials for this tier are ~90% complete. |
| 5 | **K-12 requires a partner model (Horizon 2).** Handing a repository and an AI assistant to non-IT teachers does not work; productizing StampFly Edu (micro:bit + MakeCode) plus lesson plans, visiting lectures, and teacher training via partners is the precondition. |

## 2. Situation Analysis (Summary)

**Market opportunity:** DJI's education exit (announced 2023-12) left the programmable-education-drone market to CoDrone EDU ($249, closed-source, caps out at high school) and Hula-JP (weak curriculum). Crazyflie ($240+, GPL3) owns the university-and-above tier but does not reach K-12 and is priced 5x higher than StampFly ($49.95).

**Regulatory advantage:** Under Japan's amended Aviation Act (2022), sub-100g aircraft (StampFly ≈ 37g) are exempt from registration and flight permits; indoor flight is outside the Act entirely. Facility-level rules (flight plans, insurance confirmation, spotters) still apply and should be templated (see P0-5).

**Internal assets:** a 4+1-day workshop (12–13 lessons, Beamer slides, competition rules, survey design), a 15-week university curriculum (notebooks 01–15 already implemented with bundled sample datasets — most sessions run hardware-free via a simulator fallback, but the syllabus does not link to them: assets exist yet are invisible), 8 Python examples, bilingual guides, the 4-tier API (L0 `ws::*` → L3 `sf::internal`), a Code-Identity SILS simulator with a browser GUI, and a Tello-SDK-compatible API (djitellopy runs unmodified; SILS-verified; hardware-only path). Track record: JUIDA Chairman's Award (2026-06), Kanazawa City "Digital Studies" classes, a technical-high-school training day, conference talks, magazine serials. Notably, the Workshop repo has 55 forks vs 5 stars — evidence that hands-on workshops are the strongest acquisition channel.

**Demand-side gaps:** (1) firmware must be self-built (no prebuilt binaries/releases), (2) no teacher-ready packages (vs CoDrone's PD training, 5E lesson plans, visiting lectures), (3) no class-set procurement path (vs $3,999 12-packs, PO payment, RaaS), (4) no safety/insurance/facility-permission kit, (5) no English materials or papers (explicit English requests exist on forums), (6) near-zero visibility (absent from adoption comparison articles; docs site not deployed).

**Decision structure:** K-12 supplementary materials are legally selected by the school principal; KOSEN/university adoption is an individual instructor's discretionary purchase — the fastest path.

**K-12 client devices (GIGA program):** elementary/junior-high 1:1 devices are now **ChromeOS 60% + iPadOS 31% = 91% browser-centric** (MM Research, 2025-07), with Windows down to 10%; high schools remain Windows-dominant with ~50% BYOD. Consequences: K-12 offerings must be browser-complete (no local installs — Crostini and app installs are typically admin-disabled); browsers cannot speak raw UDP, so the Tello-compatible API is unreachable from these devices; Web Serial flashing works on Chromebooks (subject to admin policy) but is structurally impossible on iPads (WebKit rejects Web Serial/WebUSB). StampFly Edu's micro:bit + MakeCode architecture is validated by this device mix, and re-porting the WebSocket server that already existed in `vehicle_old` (port 80) to the current vehicle firmware would enable a zero-install "browser cockpit" from the drone's SoftAP — also restoring ROS-bridge compatibility (see H2-7).

## 3. Strategic Core

**Positioning: "The platform students never outgrow."** Three pillars: **white-box** (all control/estimation source readable, MIT), **the ladder** (micro:bit → Python/Tello SDK → C++/4-tier API → ESKF/sysid/SILS on the same airframe), and **lightness** (sub-100g regulatory exemption, indoor classes, 1/5 the competitor price). The AI-personalized-curriculum vision (JUIDA award) targets tiers that design their own courses (KOSEN/university instructors, hobbyists); K-12 gets finished packages instead.

**Beachhead: KOSEN + undergraduate.** Highest material completeness, fastest decision-making, strongest existing credibility, and a natural propagation point both downward (outreach classes) and upward (graduates become researchers/engineers).

**Data flywheel:** run the designed-but-never-administered 21-question survey at every event; publish education-effectiveness studies (even CoDrone's curriculum is "not formally evaluated" per third-party review) — a moat closed competitors cannot copy.

## 4. Execution (3 Horizons)

**Horizon 1 (→2027-03): remove friction, secure the beachhead.** P0: prebuilt binaries + browser flashing (ESP Web Tools) + M5Burner; publish the mkdocs site (the landing page has been live since 2026-06-07 but is linked from nowhere — add README/SNS entry points) with a per-tier entry router; connect and verify the existing materials (link syllabus↔notebooks 01–15, refresh stale "planned" READMEs, guarantee the zero-hardware trial path — the `sf sim run` no-controller crash was fixed on 2026-07-08); publish a class-set procurement page (Switch Science already has accounts with the KOSEN organization); ship a safety & operations pack (insurance guidance, facility-permission templates modeled on Maebashi/Adachi forms). P1: regular workshops with mandatory surveys, 5 pilot institutions with loaner kits, a SICE tutorial proposal, a book proposal (30-part blog + magazine serial as manuscript base), and on-hardware verification of the Tello SDK compatibility layer.

**Horizon 2 (FY2027): K-12 partner model + English.** Productize StampFly Edu; build a visiting-lecture/training partner network (HDL model); replicate the "Kanazawa model" (municipal curriculum slot + university partnership) in 2 more municipalities; English workshop slides, GitHub Discussions, one English paper (IEEE EDUCON / IFAC education), re-engage Elektor (their 2025-03 criticisms — no hover / no position hold — are now resolved); establish competitions (existing indoor flight robot contest + own "StampFly Cup"); apply for foundation grants (Murata, Panasonic, Chuden, METI STEAM Library, JST Junior Doctor); and build the zero-install "browser cockpit" by re-porting the `vehicle_old` WebSocket server to the current firmware (the only path from the 91% ChromeOS/iPad K-12 device base to the drone).

**Horizon 3 (FY2028+): scale and institutionalize.** Educational distributor channels, organization-level certifications (JUIDA, KOSEN organization), overseas pilots at 2 universities (targeting tiers priced out of Crazyflie), and community self-sufficiency (maintainers, contribution guide) to resolve the bus-factor-of-one.

## 5. Risks

Supply continuity (the Tello lesson — mitigate via hardware-abstract materials and open BOM), key-person dependency (train-the-trainer, books/videos), a single school accident poisoning the market (safety pack ships *before* scaling), AI-hallucinated curricula damaging trust (teacher-review loop; verified curricula as canon), and version drift between materials and firmware (curriculum↔release compatibility table; semester version pinning).

## 6. KPIs

See the Japanese table in §6: browser-flash onboarding live and docs site public by 2027-03; KOSEN/university adoptions 5 → 12 → 25; K-12 schools 5 → 20 → 50; survey n 60 → 300 → 1,000; first English paper by FY2027; book published by FY2027; GitHub stars 150 → 400 → 1,000.

## 7. Next 90 Days

Revised 2026-07-09, after v2026.07.1 shipped no-build flashing (web flasher + desktop GUI flasher for 3 OSes): the adoption bottleneck has moved from tooling to documentation, so the plan now centers on two axes — (1) the DXH high-school teacher workshop (Jul 18–19, 2 hours; the PCs (Windows 11 / Core i5), StampFly units, controllers, and USB-C cables are all loaned by the university, so the environment can be set up and standardized in advance) and (2) a documentation track that treats the "flash-only user" (teachers and students with no dev environment) as a first-class audience. The 2026-07-09 docs inventory concluded that parameter tuning and log capture already work without a dev environment (`sf log wifi`, serial `param`), but no non-developer documentation says so.

Through Jul 17 (top priority): finalize the 2-hour curriculum (pilot experience → no-build flashing → dev-environment setup → code-modify / motor control, per the university's published program description), produce the handout (doubling as the first screenshot-based flashing guide) and slides, pre-configure the loaner PCs (flashers, USB driver, dev environment — the "install" segment becomes a demo on a pre-built environment), rehearse on the loaner Windows 11 PCs (doubling as the pending .exe verification; beta testers are recruited in parallel to cover ordinary user environments such as non-admin accounts and SmartScreen; one loaned StampFly set per participant is confirmed, so the exercises are one-device-per-person), and measure real setup times. Jul 18–19: run the workshop on both days and collect the first teacher survey (data-flywheel start). Weeks 3–4 (→Aug 7): flash-only-user documentation sprint (entry routing page, first-flight guide, controls/UI reference, beginner parameter tuning, log capture, "after your first landing" next steps, expanded LED/buzzer troubleshooting) plus the institutional procurement page. Weeks 5–8 (→Sep 3): SCI/SICE tutorial prep (Sep 10; participant pre-setup guide, full-day curriculum), materials-connection sprint (syllabus↔notebooks, fix the 65 broken links), fold in Windows beta-test results. Weeks 9–12 (→Oct 8): publish the high-school lesson package informed by the DXH results, safety-pack draft, Tello SDK on-hardware verification, loaner-kit program design, book proposal. Completed before this revision: GitHub Releases (v2026.07.0/.1, CalVer), web + desktop flashers with automatic post-release site redeploy, mkdocs site + landing links.
