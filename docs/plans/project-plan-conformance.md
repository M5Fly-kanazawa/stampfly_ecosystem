# PROJECT_PLAN.md（原典）とリポジトリの整合計画

作成: 2026-09-12。状態: **計画中**（棚卸しは完了。是正の方針はプロジェクトオーナーの判断待ち。判断が要らない項目から着手する）。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 目的 | `PROJECT_PLAN.md` を原典とし、リポジトリをそれに合う形にする。実態が意図的な進化なら原典を更新し、そうでなければ実態を原典に合わせる。原典自身の矛盾はすぐ直す |
| 棚卸しの範囲 | 原典 §2〜§12 の全節を実態と突き合わせた（トップレベル／docs／firmware／protocol／control／analysis／tools／simulator／examples／third_party／CI）。Workshop の中身は読んでいない（廃棄・全面書き換え予定） |
| 結果 | 食い違いは 4 種類に分かれる。**C 原典の自己矛盾** 6 件（判断不要、直す）／**D 空の置き場** 10 か所（削除候補）／**A 実態が原典に反する** 6 件／**B 原典が古い**（意図的な進化を原典が未反映）約 30 要素 |
| 一致していたもの | firmware コンポーネント 30 個中 29 個、`vehicle_old` の凍結、`common/protocol` の共有実装、`simulator/` のトップレベル 7 項目、`analysis/reports` の非追跡、`README.md`／`LICENSE`／`docs/overview.md`／`docs/next_step.md`／`docs/university/` |
| 進め方 | 第 6 章。判断不要の是正（C と一部の A、サブ README の陳腐化）→ 空の置き場の削除 → 判断が要る項目 → 原典に「拡張の受け入れ規則」を追加 |

分類の定義: **A** 実態が原典に反する（実態を直す候補）／**B** 原典が古い（実態は意図的な進化。原典を更新する候補）／**C** 原典内部の矛盾（原典と、原典が必読指定する文書との矛盾を含む）／**D** 空の置き場（`.gitkeep` のみ等）。

## 1. 原典の自己矛盾（C）— 判断不要、原典を直す

| # | 箇所 | 矛盾の内容 | 是正 |
|---|------|-----------|------|
| C1 | §3 の木（`PROJECT_PLAN.md:55`） | 木は `docs/workshop/`、直下の本文（:79-80）は `docs/events/`。`docs/workshop/` は存在しない。コミット `36e8f884`（2026-09-06）が本文だけ書き換えた | 木を `events/` に直す |
| C2 | §4 コンポーネント一覧（:136） | `sf_api（Tello風API）` と書くが、`sf_api` は L1 Topic API の読み取り関数群（`sf_api.hpp:10-24`「Public API for learner code」）。Tello 互換 API は `tasks/api_task.cpp`（UDP:8889）で、コンポーネント化されていない。原典が必読指定する `architecture.md:156` とも矛盾。昇格コミット `3d8c33d5`（2026-07-05）の執筆時点からの誤記 | `sf_api（L1 Topic API・読み取り関数群）` に直し、Tello 互換 API は `tasks/api_task.cpp` と `sf_telemetry/tello_state`（状態 10Hz、UDP:8890）にあると書く |
| C3 | §4 `vehicle/main/`（:141） | `config.hpp（全パラメータの一元管理）` と書くが、`config.hpp` はコンパイル時の固定定数のみ（`config.hpp:9-27`）。調整可能パラメータの正は `sf_core/params.cpp`（NVS 永続化）。必読指定の `detailed_design.md:392` が「全パラメータの SSOT は `params.cpp`」と明記 | 「`config.hpp`（固定定数）、調整可能パラメータの正は `sf_core/params.cpp`」に直す |
| C4 | §5（:239）と §2／§3 の木 | §5 は生成先として `lib/sflog/schema.py`・`docs/reference/flight-log-format.md` を名指しするが、`lib/` は §2 の木に、`reference/` は §3 の木に無い。コミット `7ed725a3`（2026-09-11）が §5 だけ更新 | §2 に `lib/`、§3 に `reference/` を追加（第 4 章 B と連動） |
| C5 | §6 `control/simulation/`（:262-263）と §9 | §6 は「`simulation/` — SILS 等の検証環境」、§9 は「SILS 本体は `simulator/sils/`」。同じ機能の置き場を 2 か所で主張。`control/simulation/` は `.gitkeep` のみ。`control/README.md:9` も同じ文言を継承 | §6 から SILS を外す（第 2 章 D と連動）。`control/README.md` も同時に直す |
| C6 | §8 と `CLAUDE.md` | `CLAUDE.md` は「`tools/` 配下は sf CLI のバックエンド。全ツールは sf CLI 経由」と定めるが、原典 §8 にその方針が無い。しかも `tools/slides/`・`tools/udev/`・`tools/terminal_launcher/`・`tools/extract_snippets.py` は sf CLI を経由しない | 方針をどちらに置くかを決めて揃える（第 5 章 Q3） |

## 2. 空の置き場（D）— 削除候補

「アーカイブは作らない」（`CLAUDE.md` Documentation §7）と同じ理由で、中身の無い置き場も残さない。削除するなら原典の該当記述も同時に消す（または「将来の拡張点」として 1 行に留める）。

| # | パス | 原典の記述 | 実態 | 備考 |
|---|------|-----------|------|------|
| D1 | `protocol/generated/` | 「仕様から生成されたコード」 | `.gitkeep` のみ。生成物は消費側（`lib/sflog/schema.py`）と文書側（`docs/reference/`）に置く方針へ転換済み（`flight-log-format-plan.md` Phase 0、コミット `d85b7971`） | 削除し、§5 の `generated/` 記述を「生成物は消費側に置く」に書き換え |
| D2 | `control/simulation/` | 「SILS 等の検証環境」 | `.gitkeep` のみ。SILS は `simulator/sils/` | C5 と連動 |
| D3 | `control/validation/` | 「実機ログとの照合」 | `.gitkeep` のみ | 実態の照合作業は `analysis/scripts/`・`sf sils sysid-gate` 側にある |
| D4 | `examples/protocol_roundtrip/` | 「仕様→エンコード/デコードの最小例」 | `.gitkeep` のみ、初期コミット（2026-01-05）から未着手 | |
| D5 | `examples/pid_tuning/` | 「設計→パラメータ→実機反映」 | `.gitkeep` のみ、同上 | |
| D6 | `third_party/` | 「外部ライブラリ・サブモジュール」 | `.gitkeep` のみ、初期コミットから一度も変更なし | 外部資産の実態は第 3 章 A3 |
| D7 | `docs/protocol/` | 「プロトコルの文章仕様」 | `.gitkeep` のみ。同等の内容は `protocol/README.md`（メッセージ一覧・オフセット表）と `docs/reference/flight-log-format.md` に実在 | 削除し、§3 で `protocol/README.md` を指す |
| D8 | `tools/flashing/` | 「書き込み・DFU・ボード検出」 | `.gitkeep` のみ。実装は `lib/sfcli/commands/flash.py`（352 行）と `tools/flasher_gui/` | 第 5 章 Q3 |
| D9 | `firmware/common/math/`・`utils/` | 原典自身が「未実装プレースホルダ」と明記 | `.gitkeep` のみ。`vehicle` は自前の `sf_math` を持ち依存しない | 削除し、§4 の記述も削除 |
| D10 | `tools/log_capture/` | 「実験ログ取得（PC 側）」 | **ディレクトリ自体が無い**。機能は `sf log capture`／`sf log wifi`（`lib/sfcli/commands/log.py` → `tools/log_analyzer/udp_capture.py`） | 原典 §8 と `tools/README.md` から削除 |

## 3. 実態が原典に反する（A）— 実態を直す候補

| # | 箇所 | 事実 | 是正の選択肢 |
|---|------|------|-------------|
| A1 | §4／§5「`messages.yaml` が SSOT」、§12「プロトコル整合性チェック」 | `espnow_protocol.hpp` は手書き（ヘッダ冒頭「this header is its C++ implementation」）。`messages.yaml` からの生成も、両者の整合を検査する CI も無い。整合検査があるのは `flight_log.yaml` 系のみ（`gen_flight_log.py --check`、`sils-regression.yml:184-189` 経由） | (a) `messages.yaml` のフィールド・オフセット・サイズを `espnow_protocol.hpp` と照合する小さな検査を書き CI に載せる／(b) 原典の「SSOT」を「文書上の正（実装は手動同期）」と書き下げる。**推奨 (a)**（SSOT を名乗るなら検査で担保する） |
| A2 | §12「静的チェック」 | lint・型検査のツール設定が無い（`pyproject.toml` に ruff/black/mypy 無し、5 ワークフローにも無し）。CI の実態は SILS 退行スイート・インストーラ E2E・リリースビルド・Pages 配信 | (a) Python 側に lint を導入／(b) §12 の文言を実態に合わせる。第 5 章 Q6 |
| A3 | §11「ライセンス明記必須」 | 外部 JS 資産は `third_party/` の外に散在: `lib/sfcli/assets/vendor/blockly/`（LICENSE 同梱）、`simulator/shared/assets/vendor/three/`（README に MIT 出典）、`tools/log_analyzer/vendor/plotly.min.js`（**ライセンス全文ファイル無し**、バナー 1 行のみ） | plotly のライセンス全文を同梱する（判断不要）。置き場の方針は第 5 章 Q2 |
| A4 | §8 `tools/` の木 | 原典の 5 分類（flashing／calibration／log_capture／log_analyzer／ci）に対し、実態は 12 ディレクトリ＋単体スクリプト。`flashing/` 空、`log_capture/` 不在。`tools/README.md` は原典と同じ 5 項目のまま陳腐化 | (a) 原典 §8 を実態（sf CLI 中心）に書き換える／(b) `tools/` を原典の分類に再編する。第 5 章 Q3 |
| A5 | §10 `examples/` | `examples/education/`（8 サブフォルダの実働 Python）が原典・`examples/README.md` とも無記載。一方 `firmware/vehicle/examples/`（`sf app new` の雛形群）とは別系統で並立 | 「独自コードの入口」見直し（`user-programming-entry-review.md` Q9）で決める。**今回は保留** |
| A6 | 原典と重複する各所の README | `docs/README.md`（`workshop/` のまま、`commands/`・`setup/`・`reference/`・`contributing/` 等の記載なし）、`tools/README.md`（原典の古い 5 項目のコピー）、`analysis/notebooks/README.md`（実在しないノートブック 11 本を「計画中」と列挙）、`control/README.md`（C5 の文言を継承） | 原典を直すと同時に直す。再発防止に「サブ README は原典の要約に留め、原典と異なる構造を独自に主張しない」を原典の運用規則に入れる（第 6 章 Phase 4） |

## 4. 原典が古い（B）— 原典を更新して受け入れる候補

いずれも決定の出典（文書・コミット）があり、実態は意図的な進化と判断できる。原典への追記案を節ごとに示す。

### §2 トップレベル

| 要素 | 役割 | 決定の出典・最終更新 | 追記案 |
|------|------|-------------------|--------|
| `lib/` | Python パッケージのルート（`pyproject.toml` `package-dir={"":"lib"}`）。`sfcli/`（sf CLI 本体）、`sflog/`（フライトログ列スキーマ）、`stampfly/`（Tello 風 Python SDK）、`stampfly_edu/`（教育用ヘルパ） | `CLAUDE.md`、原典 §5 自身が参照。2026-09-12 も更新 | §2 に追加し、新節「lib/: PC 側の Python 実装」を設ける |
| `scripts/` | `install.sh`／`setup_env.sh` の実装本体 `installer.py` とそのテスト、`analyze_crash_sensors.py` | 2026-09-11 更新 | §2 に追加、または `tools/` へ移す（Q3） |
| `landing/`・`.mkdocs/` | GitHub Pages のランディングと docs サイトの設定（`deploy-pages.yml`） | 2026-09-07／09-12 | §2 に追加、§3 に「公開サイト」の項を設ける |
| `ros/` | ROS2 連携（README に「構築中」）。23 ファイル、最終更新 2026-07-05 | `docs/plans/ros2-integration.md`（状態: 計画中） | Q4 |
| `install.sh`／`.bat`、`setup_env.sh`／`.bat`、`pyproject.toml`、`requirements*.txt`、`.githooks/`、`CLAUDE.md` | 導入・環境・パッケージ定義・フック・AI 向け作業指示 | README／CLAUDE.md が案内 | §2 に「補助ファイル」として列挙 |

### §3 docs/

`docs/` 直下の実態は原典の 6 項目を大きく超える。追記対象: `guides/`（10）、`commands/`（19、sf コマンド参照）、`plans/`（15、状態表記付き）、`setup/`（6）、`contributing/`（7、`CLAUDE.md` の `/commit` が参照）、`reference/`（生成文書）、`assets/`、`telemetry/`、`bonus/`・`experiments/`、`index.md`（サイトのトップ）、`slides.md`、`README.md`、`DOCUMENT_INDEX.md`。`docs/protocol/` は D7 で削除。

### §4 firmware/

| 要素 | 役割 | 決定の出典 | 追記案 |
|------|------|-----------|--------|
| `firmware/apps/` | `sf app new` の生成先（L1 Topic API の入口） | `sf-app-sils-plan.md`（2026-09-07/08） | 追加。ただし入口の設計は見直し中と注記 |
| `firmware/workshop/` | Workshop 骨格（`ws::`、13 レッスン）。`sf lesson`・CI が参照 | `workshop_migration.md`（HAL 二重メンテを自認） | Q8 |
| `firmware/legacy/` | 出荷時バイナリ（`sf flash --legacy`） | コミット `14c05227`／`8050cc1e`（2026-03） | 追加 |
| `sf_app_hooks` | L1 差し替え口（31 個目のコンポーネント） | `architecture.md:134` | コンポーネント一覧に追加 |
| `sf_telemetry` | 400Hz 統一テレメトリに加え Tello 状態（10Hz、UDP:8890） | `tello_state.hpp` | 説明に追記 |
| `vehicle/test/` | Unity 形式のユニットテスト | 昇格時から存在 | 木に追加 |
| `firmware/vehicle/docs/` の必読 6 文書以外 21 件 | 実装ログ・調査メモ・運用手引 | — | 「必読 6 文書」と別カテゴリと明記するだけでよい |

### §5 protocol/

`protocol/README.md`（メッセージ一覧）、`spec/espnow_tdma.yaml`・`websocket.yaml`（実装からの逆文書化。生成・検査の対象外）を追記。`generated/` は D1。

### §6 control/

`models/stampfly_physical.yaml` が機体物理パラメータの SSOT で、`sf params generate` が `tools/sysid/_generated_params.py` を生成し、CI が `--check` で鮮度を検査している（`sils-regression.yml:159`）。`design/loop_shaping_tool/` は原典どおり。

### §7 analysis/

`README.md`・`notes/`・`out/`（非追跡の生成物置き場。`reports/` と並存）、`datasets/` の用途別サブ構造、`notebooks/education/`（教材 16 本）、`scripts/` の案件別サブディレクトリと `eskf_replay.cpp`。

### §8 tools/

原典未記載: `flasher_gui/`、`installer_gui/`、`params_audit/`（`sf params`）、`sysid/`（`sf sysid`）、`stampfly_py/`（配布用 SDK 例）、`slides/`（`docs/events` のスライド HTML 化）、`udev/`、`terminal_launcher/`、`extract_snippets.py`、`test_monitor.py`（参照元なし、孤立の可能性）。

### §12 .github/workflows/

5 本（`deploy-pages`、`macos-linux-e2e`、`windows-e2e`、`release`、`sils-regression`）。原典の一文より大幅に拡充。

## 5. 判断をお願いしたい項目

| # | 問い | 選択肢 | 補足 |
|---|------|--------|------|
| Q1 | 空の置き場 10 か所（第 2 章）を一括で削除してよいか | 一括削除／残すものを指定 | 残す場合は「いつ何を入れるか」を原典に書く |
| Q2 | `third_party/` の扱い | (a) 置き場を廃止し「外部資産はどこに置いてもライセンス全文を同梱」を規則にする／(b) 散在する外部 JS を `third_party/` に集約する | (b) は `sf blocks`・シミュレータ・ログ解析の配信パスを変える必要がある |
| Q3 | `tools/` と `scripts/` | (a) 原典 §8 を実態（sf CLI 中心）に書き換え、`scripts/` は §2 に載せる／(b) `tools/` を原典の分類に再編し `scripts/` を `tools/` 配下へ移す | C6（`CLAUDE.md` の「全ツールは sf CLI 経由」）の扱いも同時に決める |
| Q4 | `ros/`（構築中、最終更新 2026-07-05） | 原典に「構築中」として載せる／削除（履歴に残す） | `ros2-integration.md` は「計画中」 |
| Q5 | `messages.yaml` の整合検査 | 実装する（小さな照合スクリプト＋CI）／原典の SSOT の文言を弱める | A1 |
| Q6 | §12「静的チェック」 | lint を導入／文言を実態に合わせる | A2 |
| Q7 | `examples/education/` と `firmware/vehicle/examples/` の関係 | 入口の見直しで決めるまで保留 | A5 |
| Q8 | `firmware/workshop/` を原典にどう書くか | 「廃棄・全面書き換え予定」と明記／書き換え後まで記載しない | 現在は `sf lesson`・CI が参照する稼働中の実体 |

## 6. 進め方

| Phase | 内容 | 判断 |
|-------|------|------|
| 1 | 判断不要の是正: C1〜C5 の原典修正、A3 の plotly ライセンス同梱、A6 のサブ README 陳腐化の解消、§3／§4／§5／§6／§7／§9／§12 の事実更新（第 4 章の B）。原典に `lib/`・`landing/`・`.mkdocs/`・`firmware/apps`・`firmware/legacy`・`sf_app_hooks`・`vehicle/test/` を追記 | 不要（ただし実施前に本文書を確認いただく） |
| 2 | 空の置き場の削除（D1〜D10）と原典の該当記述の削除。削除前のコミットに `archive/YYYY-MM-DD` タグ | Q1 |
| 3 | 判断が要る項目（Q2〜Q6、Q8） | 個別 |
| 4 | 原典に「拡張の受け入れ規則」を追加: 構造（ディレクトリ・責務・命名）を変えるときは原典を同じコミットで更新する／サブ README は原典の要約に留める／計画文書の状態表記とアーカイブ禁止（`CLAUDE.md` Documentation §7 と同文）／新技術・新用途は「原典に無い＝禁止」ではなく「原典に節を足してから置く」 | 文言の確認 |

「現状や新技術に柔軟に適応できる余地」は Phase 4 の規則で担保する。原典は構造の意図を書く文書であり、ファイル単位の網羅目録ではない（目録は `docs/DOCUMENT_INDEX.md`）——この役割分担も Phase 4 で明記する。

## 7. 未確認事項

- `tools/test_monitor.py` の用途・呼び出し元（リポジトリ内に参照なし）。
- `messages.yaml` と `espnow_protocol.hpp` のバイト単位の一致（今回は冒頭コメントの確認のみ）。
- `analysis/scripts/eskf_replay.cpp` のビルド経路。
- `firmware/vehicle/test/` が CI から実行されているか。
- `logs/` に追跡されている 1 ファイルの実体。

## 8. 一次資料

棚卸しの根拠は 3 本の調査報告（セッション作業領域 `audit_1_toplevel_docs.md`・`audit_2_firmware.md`・`audit_3_protocol_control_analysis_tools_sim.md`、2026-09-12）。本文中の `パス:行番号` はそれらから転記し、主要な主張は原典側のファイルで再確認した。
