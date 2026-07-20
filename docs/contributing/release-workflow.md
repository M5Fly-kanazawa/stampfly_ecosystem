# ファーム更新・パラメータ更新からリリースまでの手順
# Firmware / Parameter Update → Release Workflow

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

ファームウェアの中身（制御則・機能）やパラメータ既定値を変更し、それを新しいリリース
（`vYYYY.MM.P` タグ）として配布するまでの一連の手順をまとめる。タグ作成以降の
チェックリストは [versioning.md §5](versioning.md) が正であり、本ドキュメントは
**その前段（変更作業そのもの）と後段（実機への反映）** を含めた全体の流れを扱う。

### 対象読者

- パラメータ既定値（PIDゲイン・フィルタ定数等）を変更してリリースしたい開発者
- ファームウェアの機能を変更してリリースしたい開発者

### 関連文書

| 文書 | 役割 |
|------|------|
| [versioning.md](versioning.md) | バージョン体系・リリースチェックリスト（§5）・互換表（§6） |
| `firmware/vehicle/docs/development_roadmap.md` | 3原則（Code/Param/Model Identity）・SIL→実機ワークフロー |
| `firmware/vehicle/docs/architecture.md` | アーキテクチャ不変条件（INV）— 制御則変更時は照合必須 |
| `docs/plans/release-vYYYY.MM.P-notes.md` | リリースごとのノート原稿（変更点の整理先） |

## 2. パラメータ既定値を変更する手順

### 2.1 変更箇所は必ず2箇所

パラメータの実体は `firmware/vehicle/components/sf_core/params.cpp` の手書きテーブルであり、
既定値は**同じファイルの2箇所**に現れる。**必ず両方を揃えて変更する**（片方だけ変えると、
初回起動時と `param reset` 後で値が食い違う）。

| 箇所 | 例（`rate.roll.td` の場合） |
|------|---------------------------|
| 変数の初期値 | `float rate_roll_td = 0.002f;` |
| `table[]` の既定値列 | `{"rate.roll.td", ParamType::FLOAT, &rate_roll_td, 0.002f, ...}` |

変更理由・日付・経緯（誰の指示か、どの飛行データに基づくか）を近傍のコメントに
バイリンガルで残す。過去の値の履歴もコメントに残っているので消さない。

### 2.2 数値的裏付け（必須）

制御系パラメータの変更は、**必ず数値的な裏付けを添えて**からコミットする
（プロジェクト規約。定性的な「効くはず」だけでの変更は禁止）。

| 変更の出どころ | 必要な裏付け |
|--------------|-------------|
| 解析・チューニング研究からの提案 | 実フライトログを使ったシミュレーションで効果を定量確認 |
| パイロットの実飛行ハンドチューニング指示 | 指示値をそのまま採用してよいが、**SIL 退行テストの A/B**（変更前後で 39 シナリオの pass/fail 集合が一致すること）は必須 |

SIL A/B の実行例（変更前後を比較する）:

```bash
source setup_env.sh
sf sil build
for scn in simulator/sil/scenarios/*.scn; do sf sil scenario "$scn"; done
# 変更前の結果と pass/fail 集合を突き合わせる
```

### 2.3 既存機体への反映 — NVS の優先関係を理解する

**ファームを書き換えただけでは、既存機体の飛行パラメータは変わらない。**
起動時の `params::load()` は「コンパイル時既定値 → NVS 保存値で上書き」の順で
動作するため、一度でも `param save` した機体では **NVS の保存値が新しい既定値を
覆い隠し続ける**。

| 機体の状態 | 新既定値の反映方法 |
|-----------|------------------|
| NVS 未保存（工場出荷相当・`full` bin を初書き込み） | 何もしなくて良い（既定値がそのまま使われる） |
| `param save` 済みの既存機体 | シリアルコンソール（`sf monitor`）で `param reset` → `param save` を実行 |

`param reset` は RAM 上で既定値に戻すだけ（保存前に再起動すれば元に戻る）。
`param save` は 400Hz ループがストールするため **disarmed 時のみ受け付ける**
（armed 中は拒否される）。個別パラメータだけ反映したい場合は
`param set <name> <value>` → `param save` でもよい。

**既定値を変更したリリースでは、リリースノートに次の2点を必ず明記する:**

1. どのパラメータの既定値が、いくつからいくつへ変わったか
2. 既存機体への反映には `param reset` → `param save` が必要なこと

## 3. ファームウェア本体（制御則・機能）を変更する手順

パラメータだけでなくコードを変更する場合は、`firmware/vehicle/docs/` の6文書
（requirements / architecture / detailed_design / coding_and_education /
development_roadmap / hardware_init）を読んだ上で、以下を守る。

| 手順 | 内容 |
|------|------|
| 1. 設計照合 | 制御則・状態機械・飛行フェーズに関わる変更は `architecture.md` の INV（アーキテクチャ不変条件）に照合。前提が変わる場合は既存コンポーネントへのリップル確認 |
| 2. 実装 | バイリンガルコメント・`@design` タグ・マジックナンバー禁止などのコーディング規約に従う |
| 3. SIL 退行テスト | 39 シナリオ一式を実行し、変更前後で pass/fail 集合が一致することを確認（既存 FAIL は既知として扱う） |
| 4. ビルド | `sf build vehicle`（controller に触れた場合は `sf build controller` も） |
| 5. 実機検証 | 制御則の変更は実飛行での確認まで行う（SIL PASS は実機安全の保証ではない） |
| 6. コミット | `/commit` スキルで Next steps 付きコミット |

## 4. リリース作業の全体フロー

変更が一通り main に揃ったら、以下の順で進める。

| # | 作業 | コマンド／確認内容 |
|---|------|-------------------|
| 1 | リリースノート原稿を更新 | `docs/plans/release-vYYYY.MM.P-notes.md` に変更点を整理（既定値変更は §2.3 の2点を含める） |
| 2 | SIL 退行テストを main の最終状態で一括実行 | 39 シナリオ。個々のコミット時に通していても、タグ直前に1回まとめて実行する |
| 3 | ローカルビルド確認 | `sf build vehicle` / `sf build controller` |
| 4 | CI の事前検証 | GitHub Actions の Release workflow を `workflow_dispatch` で1回実行し、全ジョブ（ファーム2 + フラッシャ4OS）が緑であることを確認。タグ無し実行では Release 発行ジョブだけがスキップされる |
| 5 | タグ作成〜リリース発行 | [versioning.md §5](versioning.md) のチェックリストに従う: `git tag vYYYY.MM.P` → `git push origin vYYYY.MM.P` → Release workflow 完走 → アセット一式（ファーム4 + フラッシャ4 + SHA256SUMS）を確認 |
| 6 | リリースノート加筆 | 自動生成ノートに原稿（#1）の要点を追記 |
| 7 | 互換表更新 | `versioning.md` §6 に新リリースの行を追加 |

## 5. リリース後 — 実機・利用者側への反映

| 対象 | 手順 |
|------|------|
| エコシステム（PC 側） | `sf upgrade`（git 取得・依存再同期・フラッシャ更新提案まで一括） |
| 機体ファームウェア | `sf flash vehicle`、GUI フラッシャ、または配布 `full` bin の書き込み |
| **既定値が変わったパラメータ** | 既存機体では `param reset` → `param save`（§2.3。書き込みだけでは反映されない） |
| 動作確認 | ベンチ確認（ブザー・センサー・テレメトリ）→ 検証飛行 |

---

<a id="english"></a>

## 1. Overview

### About This Document

This document describes the end-to-end procedure for changing firmware behavior
(control laws, features) or parameter defaults and shipping the change as a new
release (`vYYYY.MM.P` tag). The tag-and-after checklist in
[versioning.md §5](versioning.md) remains the source of truth; this document
covers **what comes before it (making the change correctly) and after it
(getting the change onto real craft)**.

### Target Audience

- Developers changing parameter defaults (PID gains, filter constants, ...)
- Developers changing firmware functionality for a release

### Related Documents

| Document | Role |
|----------|------|
| [versioning.md](versioning.md) | Versioning scheme, release checklist (§5), compatibility table (§6) |
| `firmware/vehicle/docs/development_roadmap.md` | The 3 identity principles (Code/Param/Model), SIL→hardware workflow |
| `firmware/vehicle/docs/architecture.md` | Architectural invariants (INV) — mandatory cross-check for control-law changes |
| `docs/plans/release-vYYYY.MM.P-notes.md` | Per-release notes draft (where changes are catalogued) |

## 2. Changing a Parameter Default

### 2.1 Always Two Places to Edit

Parameters live in the hand-written table in
`firmware/vehicle/components/sf_core/params.cpp`, and each default appears in
**two places in that file**. **Always change both** (changing only one makes
first-boot values disagree with post-`param reset` values).

| Location | Example (for `rate.roll.td`) |
|----------|------------------------------|
| Variable initializer | `float rate_roll_td = 0.002f;` |
| Default column in `table[]` | `{"rate.roll.td", ParamType::FLOAT, &rate_roll_td, 0.002f, ...}` |

Record the reason, date, and provenance (whose direction, which flight data) in
the adjacent bilingual comment. Keep the value history already in the comments.

### 2.2 Numerical Backing (Mandatory)

Control-parameter changes must be committed **with numerical backing**
(project rule; qualitative "should help" reasoning alone is not acceptable).

| Origin of the change | Required backing |
|----------------------|------------------|
| Analysis / tuning study proposal | Quantified effect via simulation against real flight logs |
| Pilot's in-flight hand-tune direction | The directed value may be adopted as-is, but an **A/B SIL regression run** (identical pass/fail set across all 39 scenarios before vs. after) is still mandatory |

Example A/B SIL run:

```bash
source setup_env.sh
sf sil build
for scn in simulator/sil/scenarios/*.scn; do sf sil scenario "$scn"; done
# compare the pass/fail set against the pre-change run
```

### 2.3 Reaching Existing Craft — Understand NVS Precedence

**Flashing new firmware alone does not change flight parameters on existing
craft.** At boot, `params::load()` applies compiled-in defaults first, then
overrides them with saved NVS values — so on any craft that has ever run
`param save`, **the saved NVS values keep masking the new defaults**.

| Craft state | How the new default takes effect |
|-------------|----------------------------------|
| No NVS save (factory-fresh / first `full` bin flash) | Nothing to do (defaults apply directly) |
| Existing craft with `param save` history | On the serial console (`sf monitor`): `param reset` → `param save` |

`param reset` only restores defaults in RAM (rebooting before saving undoes
it). `param save` is **accepted only while disarmed** (the NVS flash-sector
erase stalls the 400 Hz loop, so it is refused while armed). To apply a single
parameter instead, `param set <name> <value>` → `param save` also works.

**A release that changes defaults must state both of these in its release
notes:**

1. Which parameter default changed, from what value to what value
2. That existing craft need `param reset` → `param save` to pick it up

## 3. Changing Firmware Behavior (Control Laws / Features)

For code changes (not just defaults), read the six documents under
`firmware/vehicle/docs/` (requirements / architecture / detailed_design /
coding_and_education / development_roadmap / hardware_init) first, then follow:

| Step | Content |
|------|---------|
| 1. Design cross-check | Changes touching control laws, state machines, or flight phases must be checked against the INV section of `architecture.md`; if an assumption changes, enumerate ripple effects on existing components |
| 2. Implementation | Follow the coding rules: bilingual comments, `@design` tags, no magic numbers |
| 3. SIL regression | Run the full 39-scenario suite; the pass/fail set must match the pre-change run (known failures count as known) |
| 4. Build | `sf build vehicle` (and `sf build controller` if touched) |
| 5. Hardware validation | Control-law changes require real-flight verification (SIL PASS does not guarantee hardware safety) |
| 6. Commit | Use the `/commit` skill, including a Next steps section |

## 4. The Release Flow End to End

Once all changes are on main:

| # | Step | Command / check |
|---|------|-----------------|
| 1 | Update the release-notes draft | Catalogue changes in `docs/plans/release-vYYYY.MM.P-notes.md` (default changes must include both points from §2.3) |
| 2 | One consolidated SIL run on final main | All 39 scenarios, once, right before tagging — even if each commit passed individually |
| 3 | Local builds | `sf build vehicle` / `sf build controller` |
| 4 | CI pre-verification | Run the Release workflow once via `workflow_dispatch`; all jobs (2 firmware + 4-OS flasher) must be green. Without a tag, only the Release-publish job is skipped |
| 5 | Tag and publish | Follow [versioning.md §5](versioning.md): `git tag vYYYY.MM.P` → `git push origin vYYYY.MM.P` → workflow completes → verify the full asset set (4 firmware + 4 flasher + SHA256SUMS) |
| 6 | Amend release notes | Add the draft's key points to the auto-generated notes |
| 7 | Compatibility table | Add the release's row to `versioning.md` §6 |

## 5. After the Release — Reaching Craft and Users

| Target | Procedure |
|--------|-----------|
| Ecosystem (PC side) | `sf upgrade` (git update, dependency resync, flasher-update offer in one step) |
| Vehicle firmware | `sf flash vehicle`, the GUI flasher, or writing the released `full` bin |
| **Changed parameter defaults** | On existing craft: `param reset` → `param save` (§2.3 — flashing alone is not enough) |
| Sanity check | Bench check (buzzer, sensors, telemetry) → verification flight |
