# 整理対象の候補（リポジトリの棚卸し）

作成: 2026-09-13。状態: **計画中**（候補の列挙。処置はオーナーの判断後。判断不要と印を付けたものは順次着手する）。

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
| C1 | Python SDK が 2 系統: `lib/stampfly/`（Python パッケージ）と `tools/stampfly_py/`（配布用サンプル、`djitellopy` 互換） | `lib/stampfly/` が現行 vehicle の API と整合するかは未確認（vehicle_old 期の可能性）。README の「Python SDK」がどちらを指すか曖昧 | 現行に合う方を残して一本化し、残す側を原典 §8/§9 に明記 | 要 |
| C2 | `TelemetryPacket` の定義が 3 系統（`firmware/vehicle` の `telemetry.hpp`、`vehicle_old`、`firmware/common/protocol/udp_protocol.hpp`） | `messages.yaml` の「正」が及んでいない。`check_messages.py` はスキップ扱い | vehicle の定義を `messages.yaml` に取り込み検査対象にする。`udp_protocol.hpp` は vehicle_old 削除（D1）で不要になるか確認 | 要 |
| C3 | 生成物の置き場 `analysis/reports/` と `analysis/out/`（どちらも非追跡） | 同じ役割の別名 | 片方に統一し `.gitignore` と原典 §7 を更新 | 不要（`reports/` に統一を推奨） |
| C4 | `firmware/workshop/` の HAL・ミキサが vehicle の複製 | 横断ルール R12 との乖離 | Workshop アップグレードの中で解消 | アップグレードに従属 |

## 2. 位置づけが未定のもの

| # | 対象 | 事実 | 候補 | 判断 |
|---|------|------|------|------|
| U1 | 大学シラバス系: `examples/education/`・`lib/stampfly_edu/`・`analysis/notebooks/education/`・`docs/university/`（Python/Jupyter） | 主導線からリンク無し。実施記録は未確認 | 原典 §16 で「外側: Python」の門に位置づけた。講習資料（P4）として整備するか、講義専用として分けるか | 要 |
| U2 | Blockly（`sf blocks`、`lib/sfcli/assets/vendor/blockly/`、`docs/guides/block_programming.md`） | Phase 0 の試作。実機 E2E 未実施。README から辿れない | 外側の門として育てるか、試作のまま止めるか | 要 |
| U3 | `ros/`（ROS2 連携、最終更新 2026-07-05）と `docs/plans/ros2-integration.md`（計画中） | 構築中のまま | 続けるか、削除（タグ後）か | 要 |
| U4 | `docs/guides/motor_spin_quickstart.md` | 自称「臨時手順書」。実体は Workshop Lesson 1 への手動手順 | `sf lesson` の手順に吸収して削除 | 不要（吸収を推奨） |
| U5 | `docs/bonus/`・`docs/experiments/`（LaTeX の番外資料・実験手順） | 役割の説明が薄い | 講習資料（P4）か実験資料（P3）に振り分け | 要 |
| U6 | `docs/telemetry/UDP_TELEMETRY_DESIGN.md` | 設計メモが単独ディレクトリ | `docs/architecture/` へ移動 | 不要 |
| U7 | `simulator/sandbox/` | STL 分割・WebGL ビューアの実験 | 成果を `shared/` に取り込み、残りを整理 | 要 |
| U8 | `sf app` の既定雛形 `11_app_controller`（PidController 委譲） | 「ゼロから書く」記事と前提が逆 | 全階層対応の雛形設計の中で決める（末節） | 入口の設計に従属 |

## 3. 陳腐化・要更新の資料

| # | 対象 | 事実 | 処置 | 判断 |
|---|------|------|------|------|
| S1 | `firmware/vehicle/docs/coding_and_education.md` §4 の Ch.1〜10 表、`development_roadmap.md` §6.1「Level 2（09-13）」 | 原典 §16 の地図に置き換わった。11・12 番は雛形に転用済み | 表を削除して §16 を参照。Level 2 の範囲を実態に | 不要 |
| S2 | `docs/architecture/tello-api-reference.md` 冒頭（`192.168.4.1`／TCP 23） | 現行は SoftAP `192.168.10.1`、UDP 8889/8890 | 冒頭を現行に更新し、2 月→6 月の方針反転の経緯を 1 段落記す | 不要 |
| S3 | `firmware/vehicle/docs/` の必読 6 文書以外 21 件（実装ログ・調査メモ・運用手引・トピック一覧 等） | 記録と生きた資料が混在 | 「資料（P2/P3）に昇格」「記録として残す」「削除（タグ後）」に仕分け | 要（一覧は別途） |
| S4 | `docs/commands/README.md` の一覧（12/44、英語表に `sf app` 無し）、`sf-flight.md`・`sf-query.md`（実コマンド名と不一致）、`sf blocks`・`sf sils`・`sf trim`・`sf params` の専用ページ無し | 索引の欠落 | `sf --help` の実測に合わせて索引を作り直す | 不要 |
| S5 | `landing/index.html` の「制御を自作できる」訴求 | 入口へのリンク無し | 入口の設計が決まったら導線を付ける | 入口の設計に従属 |
| S6 | `.mkdocs/mkdocs.yml` の目次「計画 > 現行」 | 2 本のみ | 生きている計画を載せる | 不要 |

## 4. 記録の扱い（アーカイブ禁止規則との整合）

| # | 対象 | 事実 | 候補 | 判断 |
|---|------|------|------|------|
| R1 | `docs/plans/release-v2026.07.*-notes.md`（5 本、発行済み） | 記録。同内容は GitHub Releases にある | GitHub Releases を正として main から削除（タグ後）、または残す | 要 |
| R2 | `docs/plans/simulator-migration.md`（実装済み、方針は置き換え済み）、`simulator/sils/RESET_PLAN.md`（立ち上げ期の記録） | 生きた方針は `docs/architecture/simulation-policy.md` | 結論を `simulation-policy.md` に畳んで削除（タグ後） | 要 |
| R3 | `docs/plans/project-plan-conformance.md`（実装済み） | 原典 §15 規則 7 | 次のタグ付与時に削除 | 不要 |
| R4 | `analysis/reports/rate_sysid_reference/`（追跡されている唯一の `reports/`） | 参照値 | 資料（P3）として `control/models/` か `docs/` へ移す | 要 |

## 5. 削除が決まっているもの（時期未定）

| # | 対象 | 決定 | 削除の前提 |
|---|------|------|-----------|
| D1 | `firmware/vehicle_old/`（凍結、実飛行 87 回） | **いずれ削除**（2026-09-13） | SILS 退行試験の `--target vehicle_old`（CI・`sf sils`）、`firmware/common/protocol/udp_protocol.hpp`（vehicle_old の `sf_svc_udp` のみ）、`sf build vehicle_old`、`CLAUDE.md`・原典 §4・`README.md` の記述を先に外す。削除直前にタグ |

維持するもの: `firmware/legacy/`（`sf flash --legacy` が工場出荷状態への復旧に使う）。

## 6. 確認だけ必要なもの

- `firmware/vehicle/test/` が CI で実行されているか
- `analysis/scripts/eskf_replay.cpp` のビルド経路
- `logs/` に追跡されている 1 ファイルの実体
- `lib/stampfly/` の現行整合（C1 の前提）

## 7. 進め方

| 段階 | 内容 |
|------|------|
| 判断不要・着手可 | C3、U4、U6、S1、S2、S4、S6、R3 |
| 判断要 | C1、C2、U1、U2、U3、U5、U7、R1、R2、R4、S3 |
| 従属 | C4（Workshop アップグレード）、U8・S5（入口の設計）、D1（前提条件の手順書） |
