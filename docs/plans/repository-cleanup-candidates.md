# 整理対象の候補（リポジトリの棚卸し）

作成: 2026-09-13。状態: **実装中**（同日、オーナー決定: 大学講義用は一度削除して計画だけ残す／ROS2・Tello 互換は必ず取り込むが現状を調査して最新アーキテクチャに合わせる／害悪になり得る記録は削除／`vehicle_old` はタグを打って削除。判断不要の 6 項目は同日実施済み）。

各行の「状態」列は 2026-09-13 時点。**済** = 実施済み、**中** = 実行中、**調査** = 現状調査中、**要** = 判断待ち。

## 0. 判定基準

原典 `PROJECT_PLAN.md` §1「目指す姿」により、リポジトリの構成要素は次のどれかに属する。

| 区分 | 内容 | 代表 |
|------|------|------|
| (a) 学習者用 | `sf app` 系: プログラミングツール・雛形・例題 | `firmware/apps`、`firmware/vehicle/examples`、`docs/guides/custom_program.md` |
| (b) 講師用 | `sf lesson` 系: レッスン・講習資料・運営ツール | `firmware/workshop`、`docs/events`、`sf competition` |
| (c) 資料 | 仕様・パラメータ・実験・分析 | `firmware/vehicle/docs`、`docs/architecture`、`control/models`、`protocol/` |
| (d) 基盤 | ファーム本体・sf CLI・CI・インストーラ・シミュレータ | `firmware/vehicle`、`lib/sfcli`、`simulator/`、`scripts/` |

どれにも属さないもの、同じ役割が二系統あるもの、古いまま残っているものを候補に挙げる。**Workshop（`firmware/workshop/`）は廃棄ではなく現行基盤へのアップグレード対象**であり、本文書の削除候補には入れない。

## 1. 重複・二系統

| # | 対象 | 事実 | 候補の処置 | 判断 |
|---|------|------|-----------|------|
| C1 | Python SDK が 2 系統: `lib/stampfly/`（Python パッケージ）と `tools/stampfly_py/`（配布用サンプル、`djitellopy` 互換） | 照合結果（2026-09-13）: `lib/stampfly/` は TCP 23 CLI＋WebSocket 80 前提で **`vehicle_old` 専用**（現行では接続不能）。`tools/stampfly_py/` は UDP 8889/8890 で現行 vehicle に対応。パッケージ化されているのは動かない方 | `lib/stampfly/` を `vehicle_old` と同時に削除（D1）。SDK は `tools/stampfly_py/` に一本化し、`lib/` へのパッケージ化は別途 | 中 |
| C2 | `TelemetryPacket` の定義が 3 系統（`firmware/vehicle` の `telemetry.hpp`、`vehicle_old`、`firmware/common/protocol/udp_protocol.hpp`） | `messages.yaml` の `TelemetryPacket`（22B）は `vehicle_old` の ESP-NOW 版とのみ一致。現行 vehicle が実送信する 104B の UDP:5005 テレメトリは `messages.yaml` に無い＝正が崩れている | `messages.yaml` の `TelemetryPacket` を現行（UDP:5005、104B）の定義に置き換え、`check_messages.py` の検査対象に加える。`udp_protocol.hpp` は controller が使うなら残す | 要（D1 の後） |
| C3 | 生成物の置き場 `analysis/reports/` と `analysis/out/`（どちらも非追跡） | 同じ役割の別名 | `reports/` に統一（解析スクリプト 4 本の既定出力先・`.gitignore`・原典 §7 を更新） | 済 |
| C4 | `firmware/workshop/` の L0 API 層・ミキサ等に vehicle と重複する実装（HAL は 2026-07-18 から vehicle のコンポーネントを共有） | 横断ルール R12 との乖離（`architecture.md` §2.5 末尾） | Workshop（L0）の API・レッスン更新の中で解消 | 従属 |

## 2. 位置づけが未定のもの

| # | 対象 | 事実 | 候補 | 判断 |
|---|------|------|------|------|
| U1 | 大学シラバス系: `examples/education/`・`lib/stampfly_edu/`・`analysis/notebooks/education/`・`docs/university/`・`docs/setup/education.md`（Python/Jupyter） | 主導線からリンク無し。実施記録は未確認 | **一度削除し、作る予定だけ `university-course-plan.md` に残す**（オーナー決定）。タグ `archive/2026-09-13` | 中 |
| U2 | Blockly（`sf blocks`、`lib/sfcli/assets/vendor/blockly/`、`docs/guides/block_programming.md`） | Phase 0 の試作。実機 E2E 未実施。README から辿れない。Tello 互換 API（UDP 8889）上に載っており現行 vehicle と整合 | **外側「ブロック」の門のプログラミングツールとして位置づけ**（2026-09-13 決定。原典 §1・§16）。置き場は現状維持（UI が育てば `lib/sfblocks/`）。残作業: README 導線、`docs/commands/sf-blocks.md`、10/4 小中学生向け講習前の実機 E2E、`sf lesson` でのブロックレッスン対応 | 済（位置づけ）／残作業あり |
| U3 | `ros/`（ROS2 連携）と `docs/plans/ros2-integration.md`（計画中） | 照合結果（2026-09-13）: `ros/stampfly_bridge` は WebSocket 80＋バイナリ UDP 8888（`ControlPacket`）前提で **`vehicle_old` 専用**。現行 vehicle にはどちらの口も無い。実装は 2026-01-20 で停止 | **ROS2 は必ず取り込む**（オーナー決定）。旧ブリッジ `ros/` と `ros2-udp-debug.md` は **今削除**（2026-09-13 決定。ノード構成・メッセージ定義は削除前に `ros2-integration.md` へ記録）。`ros2-integration.md` を現行 API（UDP 8889 テキスト／8890 状態／5005 テレメトリ）前提の再設計・再実装計画に書き直す | 中 |
| U4 | `docs/guides/motor_spin_quickstart.md` | 自称「臨時手順書」。実体は Workshop Lesson 1 への手動手順 | `docs/commands/sf-lesson.md` §5 に吸収して削除 | 済 |
| U5 | `docs/bonus/`・`docs/experiments/`（LaTeX の番外資料・実験手順） | 役割の説明が薄い | 講習資料（P4）か実験資料（P3）に振り分け | 要 |
| U6 | `docs/telemetry/UDP_TELEMETRY_DESIGN.md` | 設計メモが単独ディレクトリ | `docs/architecture/udp-telemetry-design.md` へ移動 | 済 |
| U7 | `simulator/sandbox/` | STL 分割・WebGL ビューアの実験 | 成果を `shared/` に取り込み、残りを整理 | 要 |
| U8 | `sf app` の既定雛形 `11_app_controller`（PidController 委譲） | 「ゼロから書く」記事と前提が逆 | 全階層対応の雛形設計の中で決める（末節） | 入口の設計に従属 |

## 3. 陳腐化・要更新の資料

| # | 対象 | 事実 | 処置 | 判断 |
|---|------|------|------|------|
| S1 | `firmware/vehicle/docs/coding_and_education.md` §4 の Ch.1〜10 表、`development_roadmap.md` §6.1「Level 2（09-13）」 | 原典 §16 の地図に置き換わった。11・12 番は雛形に転用済み | 表を削除して §16 を参照。Level 2 の範囲を「09・10」に | 済 |
| S2 | `docs/architecture/tello-api-reference.md` | 照合結果（2026-09-13）: 通信方式のレベル（TCP 23＋WebSocket 前提）で現行（UDP ネイティブ、SoftAP `192.168.10.1`、8889/8890/5005）と食い違う。`firmware/vehicle/docs/operation_manual.md` が現行と一致 | 現行実装（`tasks/api_task.cpp`・`sf_telemetry/tello_state`）を正として全面更新。2 月→6 月の方針反転の経緯を 1 段落記す | 中 |
| S3 | `firmware/vehicle/docs/` の必読 6 文書以外 21 件（実装ログ・調査メモ・運用手引・トピック一覧 等） | 記録と生きた資料が混在 | 「生きた資料」「無害な記録」「害悪（削除）」に仕分け中。害悪は削除（タグ `archive/2026-09-13`） | 調査 |
| S4 | `docs/commands/README.md` の一覧（12/44、英語表に `sf app` 無し）、`sf-flight.md`・`sf-query.md`（実コマンド名と不一致）、`sf blocks`・`sf sils`・`sf trim`・`sf params` の専用ページ無し | 索引の欠落 | 索引を 44 コマンドで再構築（日英）。2 ページを `flight-commands.md`・`query-commands.md` に改名。専用ページ無しは `--help` を案内 | 済 |
| S5 | `landing/index.html` の「制御を自作できる」訴求 | 入口へのリンク無し | 入口の設計が決まったら導線を付ける | 入口の設計に従属 |
| S6 | `.mkdocs/mkdocs.yml` の目次「計画 > 現行」 | 2 本のみ | 生きている計画 10 本を掲載 | 済 |
| S7 | `docs/assets/presentation.md`・`firmware/workshop/lessons/lesson_12_python_sdk/README.md` に `sf flight takeoff` 等の記述 | `takeoff`/`land`/`hover` はトップレベルの sf コマンドで、`sf flight` というグループは無い（S4 の作業で判明） | 実コマンド名に直す（Lesson 12 は Workshop の L0 更新時） | 要 |
| S8 | スライド `docs/events/sci_tutorial_2026/slides/chapters/sci_s5_sim_analysis.tex`（`lib/stampfly` のパス誤記 1 か所）、`docs/events/stampfly_workshop/slides/chapters/python_sdk.tex`（章全体が TCP 23／WebSocket の旧 SDK 前提） | `lib/stampfly` は 2026-09-13 に削除。`.tex` の変更は PDF リビルドと目視レビューが必須（`CLAUDE.md` Slide Rules）のため今回は未修正 | スライドレビューの工程として別途実施。`python_sdk.tex` は `tools/stampfly_py`（UDP 8889/8890）前提に書き直す | 要 |

## 4. 記録の扱い（アーカイブ禁止規則との整合）

| # | 対象 | 事実 | 候補 | 判断 |
|---|------|------|------|------|
| R1 | `docs/plans/release-v2026.07.*-notes.md`（5 本、発行済み） | 「タグ作成前のドラフト時点の記述のまま」。v2026.07.2〜.6 はすべて GitHub Releases に発行済み（`gh release list` で確認） | 害悪になり得る記録として削除。GitHub Releases が正 | 中 |
| R2 | `docs/plans/simulator-migration.md`（実装済み、方針は置き換え済み）、`simulator/sils/RESET_PLAN.md`（立ち上げ期の記録。「大学資産は必ず残す」等、現在の決定と矛盾する規則を含む） | 生きた方針は `docs/architecture/simulation-policy.md` | 結論を `simulation-policy.md` に畳んで削除（タグ `archive/2026-09-13`） | 中 |
| R3 | `docs/plans/project-plan-conformance.md`（実装済み） | 原典 §15 規則 7 | 削除 | 中 |
| R4 | `analysis/reports/rate_sysid_reference/`（追跡されている唯一の `reports/`） | 参照値 | 資料（P3）として `control/models/` か `docs/` へ移す | 要 |

## 5. 削除が決まっているもの（時期未定）

| # | 対象 | 決定 | 削除の前提 |
|---|------|------|-----------|
| D1 | `firmware/vehicle_old/`（凍結、実飛行 87 回） | **削除済み**（2026-09-13、コミット `3d9e3cde`、タグ `archive/2026-09-13`） | 同時に外したもの: SILS の `emu_vehicle_old` と接着コード 3 本、専用シナリオ 4 本（退行試験 36→34）、`sf sils/build/doctor/app` の `vehicle_old` ターゲット、`params_manifest.py` の参照 5 件と `EXEMPT_VEHICLE_OLD`、`lib/stampfly`（C1）。`udp_protocol.hpp`・`espnow_protocol.hpp` は controller が使うので残した。検証: `sf params check` ERROR=0、pytest 67、SILS フルビルド＋退行試験 28 PASS/5 既知 FAIL/1 SKIP |

維持するもの: `firmware/legacy/`（`sf flash --legacy` が工場出荷状態への復旧に使う）。

## 6. 確認だけ必要なもの

- `firmware/vehicle/test/` が CI で実行されているか
- `analysis/scripts/eskf_replay.cpp` のビルド経路
- `logs/` に追跡されている 1 ファイルの実体
- `lib/stampfly/` の現行整合（C1 の前提）

## 7. 進め方

| 段階 | 内容 |
|------|------|
| 済（2026-09-13、コミット `3d9e3cde`） | C1、C3、U1、U4、U6、S1、S4、S6、R1、R3、D1 |
| 実行中（2026-09-13、第 2 弾） | S3（害悪 3 文書の削除）、R2（`RESET_PLAN`・`simulator-migration` の統合と削除）、S2（`tello-api-reference.md` の全面更新）、U3（`ros/` の状態注記と `ros2-integration.md` の再設計計画化） |
| 判断待ち | U2（Blockly）、U3 の旧ブリッジコードの削除時期、U5、U7、R4、S7、S8、C2（`TelemetryPacket` を現行定義に置き換え） |
| 従属 | C4（Workshop の L0 更新）、U8・S5（入口の設計） |
