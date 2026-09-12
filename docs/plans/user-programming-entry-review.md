# 「独自コードの入口」見直しの起点 — 過去の検討の復元と現状の棚卸し

作成: 2026-09-12。状態: **見直し中**（設計の答えは未定。**本文書は提案を含まない。**）
「StampFly Ecosystem はユーザーにどうやって独自の StampFly プログラムを作ってもらうか」という問いを、現状の実装・前提を捨てて 0 から設計し直す前段として、

1. 過去に同じ問いを、いつ・どの文書で・どう検討し、何を決めたか（第 1〜3 章）
2. 今、ユーザーの目に触れる入口が実際にどうなっているか（第 4 章）
3. 文書と実装、文書と文書の間でどこが食い違っているか（第 5 章）

を事実として並べ、最後に 0 から問い直すための「問い」を列挙する（第 6 章）。

**前提:** Workshop（`firmware/workshop/`、`ws::` 名前空間、`docs/events/stampfly_workshop/`）は旧アーキテクチャで作られており、内容的には廃棄して全面的に書き直す予定である。本文書は Workshop の中身を読まず、存在・位置づけ・使用実績という外形だけを扱う。

**用語:** 本文書で「入口」とは、ユーザーが自分のコードを書き始めるための「雛形（テンプレート: 複製して書き始める出発点のソース一式）＋道具（`sf` コマンド）＋説明（記事・README）」の一組を指す。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 過去の答え | 2026-05-09 に「学習者がレベルに応じて入口を選べる 4 階層（L0〜L3）」を設計。2026-09-07 に「`sf app` = L1（Topic API）の入口」と確定。既定の雛形は「既存 `PidController` に全部委譲し、1 点だけ差し替える」形（2026-09-08） |
| 現状 | 入口は少なくとも 8 系統が並立（例題 01〜10、`sf app`、`custom_program.md`、Blockly、Python SDK、`sf lesson`/Workshop、大学シラバス系、臨時手順書）。講習会 3 件で実際に使われたのは**すべて `sf lesson`（Workshop 系）**で、`sf app` の使用実績は見つからない |
| 直近の緊張 | 2026-09-12 に `custom_program.md` を「`PidController` に頼らずゼロから書く」記事に書き直した結果、`sf app new` の既定雛形と記事の前提が逆転した |
| 利用者像 | 過去文書は少なくとも 6 通りの利用者像を、互いに参照せず別々に定義している（第 2 章） |
| 本文書の到達点 | 第 6 章「0 から問い直すための問い」。答えは書かない |

## 1. 過去の検討 — 年表

日付はコミット日または文書内の日付。主要な節目のみ。

| 日付 | 出典 | 何を決めたか | 書かれていた理由 |
|------|------|------------|----------------|
| 2026-02-14 | `docs/plans/archive/tello-compat.md`（2026-09-12 に main から削除。タグ `archive/2026-09-12` で参照可） | Tello 風の Python 入口を計画。「プロトコル互換より API 名互換を優先」 | 「教育的にはメソッド名が同じなら十分」（同 :24-34） |
| 2026-03-05 | `509d014c`, `bda7e8f8` | `sf app new` 初出。独立 ESP-IDF プロジェクトを複製する方式（当時の唯一の vehicle＝後の vehicle_old 基準） | — |
| 2026-03-05〜07 | `bfc35ba8` 他 | `firmware/my_drone`（`sf app new` の試用成果物）に比例角速度制御を実装。以後放置 | — |
| 2026-04-11〜12 | `2d1f8072`, `4cc3d736` | vehicle_new 要件定義・`coding_and_education.md` 新設。「ファームを作ろうとする人が参考にできる模範コード」「学習教材」を最重要目標に | `coding_and_education.md:10` |
| 2026-05-09 | `3e7032b6`, `e0e97371` | **4 階層アクセス（L0 "Sketch" / L1 Topic / L2 HAL / L3 BSP）初出。** 既存 `firmware/workshop/` を L0 `ws::*` として再構成する計画を確定 | `architecture.md:150-160`（第 3 章に引用） |
| 2026-06-11 | `caca9cc5` | 実際の Tello UDP プロトコルを実装し `djitellopy` 無改変で動く方式へ**転換**（2 月の方針と逆） | 転換理由を明記した文書は未発見 |
| 2026-07-13 | `bf39e634` | Blockly（`sf blocks`）Phase 0。Tello 互換 API の上に「ノーコードに近い入口」を追加 | 教育普及戦略 H2-7 の第一歩（コミット本文） |
| 2026-09-06 | `e1beb1b8` | L0 の名称を「Sketch API」→「Workshop API」に変更。抽象的な階層名が特定教材の実装（`ws::`）と同一視される | — |
| 2026-09-06〜07 | `f67809f8`, `ee6ee82e` | 例題 09（Topic API 読取）・10（IController 差替）追加。いずれも vehicle 本体を起動しない単独ベンチ | — |
| 2026-09-07 昼 | `f9af4730`（同日撤回） | 第一次案: `sf app` を L0（Workshop 骨格）に一本化 | 「workshop 骨格がすでに欲しいものそのもの」（コミット本文） |
| 2026-09-07 夜 | `6d149889`, `7be26a10` | **方針転換: `sf app` = L1 の入口と確定。** 旧 `custom_firmware` 雛形（vehicle_old 構成、ビルド不能）を削除し、例題複製方式に再構築。`custom_program.md` 初版 | 「講習会がうまくいくのは `sf lesson` が『どこを書けばいいか』を教えるから。研究の入口にはそれが無く、サンプルは飾りだった」（`7be26a10` 本文） |
| 2026-09-08 | `2ebd9bf1`, `440d37b4`, `c6ad829c` | vehicle にアプリフック 3 関数（`controller()/estimator()/start()`）と `SF_APP_DIR` を実装。雛形 `11_app_controller`（PidController 全委譲＋`adjust()` 1 点）・`12_app_task_hello` 新設。`sf app new` の既定＝11 | 理由を明記した一次資料は未発見。既定＝11 であることは `sf-app-sils-plan.md` Phase 0 の表に記載 |
| 2026-09-09 | `ae1358a7` | ミキサー差替口 `sf::app::mixer()` を**提案**（未実装）。同時に「`firmware/workshop` のミキサーが vehicle のミキサーを独自複製し R12 に反して乖離」と記録 | `architecture.md:200` |
| 2026-09-12 | `c8a7ed5e`〜`50b08d5f` | `custom_program.md` を「`PidController` に一切頼らず `IController` を最初の 1 行から書く」13 章記事に全面書き直し | `custom_program.md:9` |
| 2026-09-12 | （破棄した草案） | 既定雛形と記事の前提が逆転していることを受け「空の雛形」追加案を起草したが、**雛形の選択は末節であり方向違い**と判断して破棄（main に残さない） | 本文書の直接の契機 |

## 2. 過去文書が想定した利用者像

同じリポジトリ内で、少なくとも 6 通りの利用者像が互いに参照されずに定義されている。

| 文書 | 利用者像 | 出典 |
|------|---------|------|
| `PROJECT_PLAN.md` | 「学生・研究者が迷わず参加できる構造」 | :11 |
| `firmware/vehicle/docs/requirements.md` | 人物像の規定なし。「プログラミング教材」「IoT 教材（全センサにアクセス可能）」という文脈語のみ | :12, :150 |
| `firmware/vehicle/docs/architecture.md` §2.5 | 層ごとに規定。L0: Workshop 受講者・初心者／L1: 推定・制御・ガイダンス学習者／L2: HW 学習者／L3: ファーム実装者 | :153-158 |
| `firmware/vehicle/docs/development_roadmap.md` §1 | 「vehicle の実装に関わる開発者（人間＋AI）」「既存実装をベースに研究・教育を行う学生・研究者」 | :21-22 |
| `docs/plans/education-outreach-strategy.md` | 最も広い「梯子」: 小中（ブラウザ完結）→中高（Python/Tello）→高専・学部（C++/4 階層、**橋頭堡**）→院・研究者（ESKF・sysid・SILS） | :26, :153 |
| `docs/plans/sf-app-sils-plan.md` | 人物像の規定なし。契機のコミットは「研究利用の入口」と性格づけ | `7be26a10` |
| `docs/guides/custom_program.md` §1 | 「C++ の基本文法は分かる／Pub/Sub・名前空間・組み込みは分からない／制御工学は聞いたことがある程度」 | :22-26 |
| `docs/guides/block_programming.md` §1 | 「プログラミング入門者・初めてドローンに触れる生徒・授業で使う教員」 | :13-17 |
| `tools/stampfly_py/README.md` | 規定なし。既存 Tello/`djitellopy` 利用者を暗黙に想定 | :9-12 |
| `docs/university/syllabus.md` | 工学系学部 3〜4 年生、半期 15 回科目 | §1 |

## 3. 過去の答え — 4 階層アクセスと、その外側の入口

### 4 階層（`architecture.md` §2.5、2026-05-09 初出）

> vehicle は **学習者がレベルに応じて入口を選べる** 並列 API を提供する。Workshop 受講者から HW 学習者、ファーム実装者まで、全員が同じファームウェアを共有しつつ、自分のテーマに集中できる。（`architecture.md:151`）

| 層 | 名前空間 | 典型ユーザー | できること（原文要約） | 入口の道具 |
|----|---------|------------|---------------------|-----------|
| L0: Workshop API | `ws::*` | Workshop 受講者・初心者 | `setup()`/`loop_400Hz(dt)` と 30+ 関数で「HW・タスク・Topic 知識ゼロでフライト制御まで」 | `sf lesson`（**廃棄・全面書き換え予定**） |
| L1: Topic API | `sf::api::*` | 推定・制御・ガイダンス学習者 | Topic を読み書きして自分の ESKF/PID/Navigator を実装。`IEstimator`/`IController` を差替え。「入口は `sf app`」 | `sf app` |
| L2: HAL Direct | `stampfly::*Wrapper` | HW 学習者 | センサ・アクチュエータのドライバを直接呼ぶ。Topic を介さない | 例題 01〜08（単独ビルド） |
| L3: BSP Internal | `sf::internal::board` | ファーム実装者・拡張者 | バスハンドル取得、esp-idf 直叩き、起動順序の変更 | （道具なし） |

> 各層は **並列に共存する** — Workshop 受講者は L0 だけ、PID 学習者は L1 だけ、BMI270 の SPI 通信を理解したい学生は L2 まで降りる。**HW を「隠す」のではなく「学べる」** よう、どの層も完成度高く整備する。（`architecture.md:160`）

注: 4 階層は「例題の Level 1〜4（難度）」とは別の軸であると `coding_and_education.md:209-216` が注記している。2 軸が紛れやすかったことの痕跡である。

### `sf app` を L1 の入口と決めた理由（`docs/plans/sf-app-sils-plan.md:9-16`、2026-09-07 ユーザー確認）

> - workshop 骨格（`firmware/workshop`、`setup()` / `loop_400Hz()`）は **L0**（最下層の飛行制御を自分で書けるようになるための初心者の層）であり、`sf app` の対象ではない。
> - `sf app` が担うのは **L1**: vehicle 本体の Pub-Sub（Topic API、`sf::api::`）を使って自分のプログラム（コントローラ・推定器・ガイダンス、トピックを読むタスク）を書き、それを**vehicle 本体に組み込んで**実機と SILS の両方で動かすこと。
> - HAL（L2）や BSP（L3）を自分で書きたい人は現時点では考慮しない。

同計画が採った実装上の判断: 差替口はフック 3 関数のみ（`sf::app::controller()/estimator()/start()`）、取り込みは `SF_APP_DIR` の `*.cpp` を main コンポーネントに直接コンパイル（弱シンボル・同名コンポーネント差替は不採用、同 :84-90）、既定雛形は `11_app_controller`（PidController 全委譲＋`adjust()` 1 点）。

### 4 階層の外側にある入口

| 入口 | 位置づけ | 4 階層との関係 |
|------|---------|--------------|
| Python SDK（`tools/stampfly_py/`、Tello 互換） | Python の逐次コマンド（`takeoff()`, `move_forward()` …）。ファーム内部構造には触れない最外殻 | 体系外。ファーム側の API タスクがテキストコマンドを解釈 |
| Blockly（`sf blocks`） | ブロックを並べる。裏で Tello 互換テキストコマンドを送る | 体系外（Python SDK のさらに外側） |
| 大学シラバス系（`examples/education/`、`lib/stampfly_edu`、`analysis/notebooks/education/`） | Jupyter 上の Python、半期 15 回 | 体系外の独立系統 |

## 4. 現状の棚卸し（2026-09-12、利用者の目に触れる表面）

### 入口の一覧

| # | 入口 | 書くもの | 動かし方 | 実機／SILS | 説明文書 | 備考 |
|---|------|---------|---------|-----------|---------|------|
| 1 | 例題 01〜08 | ESP-IDF/C++、HAL 直叩き | 単独ビルド | 実機のみ | 各 `README.md` | Tier 表記なし。「ここを変えてみよう」節なし |
| 2 | 例題 09 | `sf::api::*` 読取のみ | 単独ビルド（ベンチ） | 実機のみ、SILS 不可 | 同 | README 冒頭で設計文書との番号重複を自己申告 |
| 3 | 例題 10 | `IController`（PidController 委譲＋1 点差替） | 合成入力ベンチ | 「実機を飛ばさない」と明記 | 同 | 実機化は `sf app` へ誘導 |
| 4 | `sf app`（雛形 11・12） | `adjust()` 1 点、または追加タスク | `sf app new/edit/sils/build/flash` | 両対応（embedded 型のみ） | `docs/commands/sf-app.md`、`firmware/apps/README.md` | 書込系 Topic API 未実装（Phase 4） |
| 5 | `docs/guides/custom_program.md` | `IController` をゼロから（ACRO PID 13 章） | `sf app` 一式 | SILS→実機 | 同記事 | 既定雛形の中身を「見せて丸ごと消す」迂回策を含む |
| 6 | Blockly（`sf blocks`） | ブロック | ブラウザ＋ローカル橋渡し | 実機のみ | `docs/guides/block_programming.md` | README 本文から辿れない。`docs/commands/` に専用ページなし |
| 7 | Python SDK | Python | PC 上 | 実機のみ | `tools/stampfly_py/README.md` | `docs/next_step.md` §8 経由でのみ到達 |
| 8 | `sf lesson`（Workshop、`ws::`） | `user_code.cpp` の `setup()`/`loop_400Hz()` | `sf lesson switch/edit/build/flash/sils` | 両対応 | `docs/commands/sf-lesson.md`、各イベント配布資料 | **講習会 3 件の実使用入口**。README 本文に `sf lesson` の語は出ない。廃棄予定 |
| 9 | 大学シラバス系 | Python（Jupyter） | PC 上、実機なければシミュレータへ | 記載どおりなら両対応 | `docs/university/syllabus.md` 等 | README・next_step から**リンクなし**（DOCUMENT_INDEX 経由のみ） |
| 10 | `docs/guides/motor_spin_quickstart.md` | 実体は Workshop Lesson 1 | 手動手順 | 実機 | 同 | 自称「臨時手順書」。`guides/` に置かれているが系統は #8 |
| 11 | 最上位 `examples/`（`pid_tuning/`, `protocol_roundtrip/`） | — | — | — | `examples/README.md` | `.gitkeep` のみの空。`firmware/vehicle/examples/` と同名異物 |

### README から辿ったときの導線

- 「独自プログラム」（#4/#5）と「ワークショップ」（#8）は README で**別の節**として提示され、両者の関係（`firmware/apps/README.md:9-19` が明言する「`sf lesson` と同じ new→edit→build→flash の流れを研究の入口として提供」）は README 本文では説明されない。
- `firmware/apps/README.md`（`--from` に指定できる雛形の一覧と使い分け）と `firmware/vehicle/examples/` 群そのものへは、README・`next_step.md`・`DOCUMENT_INDEX.md` のいずれからも直接リンクがない。
- Blockly と大学シラバス系は `DOCUMENT_INDEX.md` 経由でしか到達できない。
- `landing/index.html` は「制御を自作できる」と訴求するが、具体的な入口へのリンクはない（:495, :692）。

### 講習会で実際に使われた入口

| イベント | 参加者 | 使われた入口 | 出典 |
|---------|-------|------------|------|
| SCI/SICE チュートリアル 2026（2026-09-10） | 制御工学の研究者・教育者 約 30 名 | `sf lesson switch sci2026:N` → `build/flash/sils`（`ws::` API） | `docs/events/sci_tutorial_2026/handson_guide.md:17, 28-31` |
| DXH 高校教員講座（2026-07-18/19） | 高校教員（初心者含む） | `sf lesson edit/build/flash`（Duty 値の書き換え） | `docs/events/dxh2026/handout.md:194-255` |
| StampFly 勉強会（随時） | 大学生・大学院生 | `sf lesson switch <レッスン>`、`setup()`/`loop_400Hz()` | `README.md:230` |

**3 件とも `sf lesson` であり、`sf app`・Blockly・Python SDK・大学シラバス系が講習会で使われた記録は見つからなかった**（見つからなかった＝存在しない、ではない）。

### 道具（`sf` コマンド）の索引の状態

- `sf --help` 実測ではトップレベル 44 コマンド。`docs/commands/README.md` の一覧表は日本語 12 行、英語 11 行で、**英語表には `sf app` の行がない**（同 :25-38, :130-142）。
- `sf blocks`・`sf sils`・`sf trim`・`sf params` には `docs/commands/` の専用ページがない。`sf-flight.md`・`sf-query.md` というページ名に対応する実コマンドはない。

## 5. 文書間・実装間の矛盾（事実の指摘のみ）

1. **既定雛形と最新記事の前提が逆。** `sf app new` の既定（`11_app_controller`）は「PidController 全委譲＋1 点差替」、`custom_program.md`（2026-09-12）は「PidController に一切頼らずゼロから」。記事は生成物を見せて否定し丸ごと消す迂回策で成立している（`custom_program.md` §4〜5）。
2. **教育計画の章立て表が実物を反映していない。** `coding_and_education.md` §4 の Ch.1〜10 表（2026-05-09 確定）は `sf app`・`custom_program.md` を含まず、Ch.5「PID 制御」は未実装の `XX_pid_single_axis` を指す。11・12 番は Level 2 の仮番号から雛形へ転用されたが、`development_roadmap.md:256` は「Level 2（09-13）」のまま。
3. **「独自プログラム」と「ワークショップ」の関係が説明されず、実績は片側のみ。** 設計上 `sf app` は「研究の入口」、`sf lesson` は「講習会の入口」だが、README はこの分担を述べない。使用実績はすべて `sf lesson`。
4. **L0 が「階層」と「特定教材の実装」の両方を指す。** 2026-05 の「Sketch API」（抽象的な最初級層）が 2026-09-06 に「Workshop API」へ改称され `ws::` と同一視された。L0 を再設計するとき、階層としての役割と Workshop という実装を分けるかどうかは未整理。
5. **`sf app` という同一名の下で統合方式が別物。** 2026-03: 独立 ESP-IDF プロジェクト複製（vehicle_old 基準）。2026-09: vehicle の main コンポーネントへ直接コンパイル。名前だけが連続している。
6. **`firmware/my_drone` が 3 回言及されながら未処理だった。** 2026-03 作成 → 2026-09-07 「削除するか決めよ」 → `sf-app-sils-plan.md:197` → 本見直しの調査で再確認。現行構成でビルド不能の可能性が指摘されたまま放置されていた。**2026-09-12 に main から削除**（タグ `archive/2026-09-12` で参照可）。
7. **同名異物・索引欠落。** 最上位 `examples/` と `firmware/vehicle/examples/` の区別説明がない。コマンド索引が 12/44。英語表に `sf app` がない。
8. **Tello 方針の反転理由が未記録。** 2026-02「API 名互換のみ」→ 2026-06「実プロトコル実装」。結びつけて理由を述べる文書は見つからない。
9. **R12（HAL 共有）に対する既知の乖離が未解消。** `firmware/workshop` のミキサーが vehicle のミキサーを独自複製（`architecture.md:200`）。解決案は提案止まり。
10. **L3 に道具がない。** 4 階層のうち L3（ファーム実装者）だけは対応する `sf` コマンドも雛形もなく、「vehicle 本体を直接編集する」以外の入口が定義されていない。「将来 vehicle ファーム全体を一人で書けるようになる」という到達点に対応する層である。

## 6. 0 から問い直すための問い

答えは書かない。各問いに「過去の答え」「現状」を添える。

| # | 問い | 過去の答え | 現状 |
|---|------|-----------|------|
| Q1 | **誰のための入口か。** 一人の利用者像を置くのか、複数の利用者像を段階（梯子）として置くのか | 6 通りの利用者像が別々に定義（第 2 章）。`education-outreach-strategy.md` だけが「梯子」として統合し、橋頭堡＝高専・学部 | 講習会の実参加者は「研究者・教育者」「高校教員」「大学生・院生」。`custom_program.md` は「C++ 既知・Pub/Sub 未経験」 |
| Q2 | **「独自のコード」とは何を書くことか。** ブロック／Python 逐次コマンド／`setup()`+`loop()`／`IController` 差替／Pub-Sub を直接読み書き／HAL／ファーム全体、のどこからどこまでを「独自」と呼ぶか | 4 階層は「層ごとに書くものが違う」と答えた。L1 の既定は「PidController の出力を 1 点いじる」 | ゼロから `IController` を書く記事はできたが、雛形・道具はそれを前提にしていない。L3 には入口がない |
| Q3 | **入口はいくつ必要か。** 1 本の階段として並べるのか、並列の門として置くのか | 「並列に共存する 4 階層」＋外側 2（Blockly・Python）＋別系統 1（大学） | 8 系統以上が並立し、相互関係を説明する文書がない |
| Q4 | **到達点はどこか。** 「飛ばせるところまで一人で行ける」ことを入口の条件にするか。「将来 vehicle ファーム全体を一人で書ける」までの道筋を入口の設計に含めるか | `custom_program.md` は 3 軸 PID で飛ばすまで。`coding_and_education.md` Ch.10「自分だけのコントローラ」は未実装 | 例題 01〜10 は飛ばさない。`sf app` 既定は既存制御で飛ぶ。ゼロから飛ばすのは記事のみ |
| Q5 | **雛形はどういう状態で渡すか。** 空か、動くものか、動くものを削って始めるか | L1 既定は「動くもの（PidController）＋1 点差替」 | 記事は「動くものを丸ごと消して空から」 |
| Q6 | **道具は入口の数だけ必要か。** `sf app` と `sf lesson` は同じ new→edit→build→flash を別名で持つ。ひとつの道具に統合するのか、層ごとに分けるのか | 「講習会は `sf lesson` が『どこを書けばいいか』を教えるからうまくいく」を根拠に `sf app` を同型に再構築（2026-09-07） | 2 系統が並立。README は関係を説明しない |
| Q7 | **SILS で動くことを入口の必須条件にするか。** | `sf-app-sils-plan.md` の契機は「`sf app` の成果物が SILS で動かない」ことだった | 例題 01〜10 は SILS 不可。`sf app`（embedded）と `sf lesson` は可。Blockly・Python は実機のみ |
| Q8 | **教材・道具・雛形の対応をどう一意にするか。** 記事（`custom_program.md`）・レッスン（Workshop）・例題 README・計画表（Ch.1〜10）が、どれを正とするのか | `coding_and_education.md` §4 の章立て表が計画上の正だったが更新されていない | 章立て表・例題番号・雛形・記事が互いにずれている（第 5 章 2） |
| Q9 | **既存資産をどう扱うか。** 例題 01〜12、Workshop（廃棄予定）、Blockly、Python SDK、大学シラバス系、`my_drone`、最上位 `examples/` の空ディレクトリ | 廃棄が決まっているのは Workshop のみ | それ以外は方針未定 |
| Q10 | **用語をどう統一するか。** 「独自プログラム／自分のプロジェクト／自作制御則／カスタムファームアプリ／研究・実験用」、「例題／サンプル／レッスン／実習／課題／学習者テンプレート」 | — | 同じものに 5 つ前後の呼び名（B 報告 §6） |

## 7. 未確認事項

- Workshop の中身（`ws::` の実体、Lesson 0〜13 の構成）は方針により未読。
- Tello 方針反転（2026-02 → 06）の理由。
- `docs/architecture/tello-api-reference.md` 冒頭に現行実装（SoftAP `192.168.10.1`、UDP 8889/8890）と食い違う記述（`TCP 23`、`192.168.4.1`、:41-50）があるが、旧設計の記録か放置かは未確認。
- `lib/stampfly/`（`tools/stampfly_py/` とは別の Python SDK）が現行 vehicle と整合するか。
- 大学シラバス系（`examples/education/`、15 ノートブック）の完成度と実施記録。
- `sf app sils`・`sf lesson sils` は定義と文書の確認のみで、本調査では実行していない（`custom_program.md` 執筆時には `sf app sils` を実行し PASS を確認済み）。
- 教育普及戦略の「梯子」と `architecture.md` の 4 階層の、どちらが先でどちらに影響したか。

## 8. 一次資料

| 分類 | ファイル |
|------|---------|
| 設計 | `PROJECT_PLAN.md`、`firmware/vehicle/docs/requirements.md`、`firmware/vehicle/docs/architecture.md` §2.5、`firmware/vehicle/docs/coding_and_education.md` §3〜4、`firmware/vehicle/docs/development_roadmap.md` §6 |
| 計画 | `docs/plans/sf-app-sils-plan.md`、`docs/plans/education-outreach-strategy.md`、`docs/plans/archive/tello-compat.md`（2026-09-12 に main から削除。`git show archive/2026-09-12:docs/plans/archive/tello-compat.md`） |
| 利用者向け | `README.md`、`docs/next_step.md`、`docs/DOCUMENT_INDEX.md`、`docs/guides/*.md`、`docs/commands/*.md`、`firmware/apps/README.md`、`firmware/vehicle/examples/*/README.md`、`tools/stampfly_py/README.md`、`docs/university/syllabus.md`、`landing/index.html` |
| 道具 | `lib/sfcli/commands/app.py`、`lesson.py`、`blocks.py`、`sils.py`、`sf --help` 実測（2026-09-12、macOS） |
| 講習会 | `docs/events/sci_tutorial_2026/handson_guide.md`、`cheatsheet.md`、`docs/events/dxh2026/handout.md` |
| 履歴 | `git log` の該当コミット（第 1 章の表に記載） |
