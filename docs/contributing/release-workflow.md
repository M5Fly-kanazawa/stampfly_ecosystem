# ファーム更新・パラメータ更新からリリースまでの手順
# Firmware / Parameter Update → Release Workflow

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

ファームウェアの中身（制御則・機能）やパラメータ既定値を変更し、それを新しいリリース
（`vYYYY.MM.P` タグ）として配布するまでの一連の手順をまとめる。リリース時の最終
チェックリスト（タグ前の最終確認〜Release 発行）は
[versioning.md §5](versioning.md#5-リリース手順) が正であり、本ドキュメントは
**その前段（変更作業を正しく行う方法）と後段（実機への反映）** を含めた全体の流れを扱う。
バージョン番号（`P` の増分規則等）の決め方は [versioning.md §2](versioning.md#2-バージョン体系)
を参照。

### 対象読者

- パラメータ既定値（PIDゲイン・フィルタ定数等）を変更してリリースしたい開発者
- ファームウェアの機能を変更してリリースしたい開発者

### 関連文書

| 文書 | 役割 |
|------|------|
| [versioning.md](versioning.md) | バージョン体系（§2）・成果物一覧（§3）・リリースチェックリスト（§5）・互換表（§6） |
| [development_roadmap.md](../../firmware/vehicle/docs/development_roadmap.md) | 3原則（Code/Param/Model Identity）・SILS→実機ワークフロー |
| [architecture.md](../../firmware/vehicle/docs/architecture.md) | アーキテクチャ不変条件（INV）— 制御則変更時は照合必須 |
| [docs/plans/](../plans/) | リリースごとのノート原稿 `release-vYYYY.MM.P-notes.md` の置き場 |

## 2. パラメータ既定値を変更する手順

### 変更箇所は必ず2箇所

パラメータの実体は `firmware/vehicle/components/sf_core/params.cpp` の手書きテーブルであり、
既定値は**同じファイルの2箇所**に現れる。**必ず両方を揃えて変更する**（片方だけ変えると、
初回起動時と `param reset` 後で値が食い違う）。

| 箇所 | 例（`rate.roll.td` の場合） |
|------|---------------------------|
| 変数の初期値 | `float rate_roll_td = 0.002f;` |
| `table[]` の既定値列 | `{"rate.roll.td", ParamType::FLOAT, &rate_roll_td, 0.002f, ...}` |

変更理由・日付・経緯（誰の指示か、どの飛行データに基づくか）を近傍のコメントに
バイリンガルで残す。過去の値の履歴もコメントに残っているので消さない。

### 数値的裏付け（必須）

制御系パラメータの変更は、**必ず数値的な裏付けを添えて**からコミットする
（プロジェクト規約。定性的な「効くはず」だけでの変更は禁止）。

| 変更の出どころ | 必要な裏付け |
|--------------|-------------|
| 解析・チューニング研究からの提案 | 実フライトログを使ったシミュレーションで効果を定量確認する。ログ取得は `sf log wifi` / `sf log convert`、解析・再生シミュレーションは `analysis/` 配下のスタディスクリプト（例: `analysis/scripts/roll_tuning_20260717/`）が先行例。変更前後の定量指標（帯域RMS・追従誤差の改善率等）を、コミットメッセージまたはリリースノート原稿に残す |
| パイロットの実飛行ハンドチューニング指示 | 指示値をそのまま採用してよいが、**SILS 退行テストの A/B**（下記）は必須 |

SILS（Software-In-the-Loop。ファームウェアを PC 上のエミュレータで飛ばす退行テスト）は
**コンパイル時既定値をそのまま使って飛行する**（Param Identity —
[development_roadmap.md](../../firmware/vehicle/docs/development_roadmap.md) 参照）ため、
既定値の変更は SILS の挙動に直接反映される。A/B 一致の確認は「挙動が変わっても、
全シナリオの合否基準を満たし続ける」ことの検証である。

A/B の手順は時系列で次のとおり。個々のシナリオの pass/fail は
`sf sils scenario` の終了コード（0=PASS）で判定できる:

```bash
source setup_env.sh
sf sils build
# ① 変更前の結果をファイルに保存 / save the pre-change results
for scn in simulator/sils/scenarios/*.scn; do
  sf sils scenario "$scn" >/dev/null 2>&1 \
    && echo "PASS $(basename "$scn")" || echo "FAIL $(basename "$scn")"
done | tee /tmp/sils_before.txt

# ② params.cpp を変更 → ③ sf sils build で再ビルドし、同じループを
#    /tmp/sils_after.txt に保存

# ④ 差分ゼロ（pass/fail 集合の一致）を確認
diff /tmp/sils_before.txt /tmp/sils_after.txt && echo "退行なし"
```

対象は `simulator/sils/scenarios/*.scn` の全シナリオ（2026-07 時点で39本。正は
グロブであり、本数は増えてよい）。「既知の FAIL」とは①の変更前実行で既に FAIL
だったものを指す — 変更後に新たに FAIL へ転じたものだけが退行である。

### 既存機体への反映 — NVS の優先関係を理解する

**app だけを書き換えても、既存機体の飛行パラメータは変わらない。**
起動時の `params::load()` は「コンパイル時既定値 → NVS 保存値で上書き」の順で
動作するため、一度でも `param save` した機体では **NVS の保存値が新しい既定値を
覆い隠し続ける**。ただし書き込み方法によって NVS が残るかどうかが変わる:

| 機体の状態・書き込み方法 | 新既定値の反映方法 |
|------------------------|------------------|
| **full bin を書き込む**（GUI フラッシャ、または `esptool.py write_flash 0x0`）。full bin = bootloader・パーティションテーブル・app を1つに結合した書き込みイメージ | 何もしなくてよい。結合イメージは間に挟まる NVS パーティション（オフセット 0x9000）も 0xFF で上書き＝消去するため、既定値がそのまま適用される。**保存済みの調整値・ホバー学習値も消える**点に注意 |
| **`sf flash vehicle` で更新**（app 等のみ書き込み、NVS は温存される） | シリアルコンソール（`sf monitor`）で `param reset` → `param save` を実行 |
| 一度も `param save` していない機体 | 何もしなくてよい（既定値がそのまま使われる） |

`param reset` は RAM 上で既定値に戻すだけ（保存前に再起動すれば元に戻る）。
`param save` は NVS のフラッシュセクタ消去で 400Hz 制御ループがストールするため、
**disarmed 時のみ受け付ける**（armed 中は拒否される）。個別パラメータだけ反映したい
場合は `param set <name> <value>` → `param save` でもよい（`param set` は RAM 反映のみ。
永続化は `param save`）。

**反映確認**: 反映操作の後（`param save` 後または再起動後）に
`param get <変更したパラメータ名>` を実行し、新しい既定値が返ることを確認する。

**既定値を変更したリリースでは、リリースノートに次の2点を必ず明記する:**

1. どのパラメータの既定値が、いくつからいくつへ変わったか
2. 既存機体への反映方法（full bin なら自動、`sf flash` 更新なら
   `param reset` → `param save` が必要なこと）

## 3. ファームウェア本体（制御則・機能）を変更する手順

パラメータだけでなくコードを変更する場合は、`firmware/vehicle/docs/` の6文書
（requirements / architecture / detailed_design / coding_and_education /
development_roadmap / hardware_init）を読んだ上で、以下を守る。

| 手順 | 内容 |
|------|------|
| 1. 設計照合 | 制御則・状態機械・飛行フェーズに関わる変更は `architecture.md` の INV（アーキテクチャ不変条件）に照合。前提が変わる場合は既存コンポーネントへのリップル確認 |
| 2. 実装 | バイリンガルコメント・`@design` タグ・マジックナンバー禁止などのコーディング規約に従う |
| 3. SILS 退行テスト | `simulator/sils/scenarios/*.scn` の全シナリオを実行し、変更前後で pass/fail 集合が一致することを確認（§2 の A/B 手順と同じ。既知 FAIL の判定も同様に変更前の実行結果を基準とする） |
| 4. ビルド | `sf build vehicle`（controller に触れた場合は `sf build controller` も） |
| 5. 実機検証 | 制御則の変更は実飛行での確認まで行う（SILS PASS は実機安全の保証ではない） |
| 6. コミット | `/commit` スキルで Next steps 付きコミット |

## 4. リリース作業の全体フロー

変更が一通り main に揃ったら、以下の順で進める。タグ名は
[versioning.md §2](versioning.md#2-バージョン体系) の規則で決める。

| # | 作業 | コマンド／確認内容 |
|---|------|-------------------|
| 1 | リリースノート原稿を作成・更新 | バージョン番号を決めた時点で `docs/plans/release-vYYYY.MM.P-notes.md` を新規作成する（前回リリースの原稿、例: `release-v2026.07.2-notes.md` をコピーして書き換えると早い）。既定値変更は §2 の2点を含める |
| 2 | SILS 退行テストを main の最終状態で一括実行 | `simulator/sils/scenarios/*.scn` 全シナリオ。個々のコミット時に通していても、タグ直前に1回まとめて実行する |
| 3 | ローカルビルド確認 | `sf build vehicle` / `sf build controller` |
| 4 | CI の事前検証 | GitHub リポジトリの **Actions タブ → `Release firmware binaries` を選択 → Run workflow → ブランチ `main` を指定して実行**。全ジョブ（ファームビルド2 = vehicle/controller + フラッシャ 4OS）が緑であることを確認。タグ無し実行では Release 発行ジョブだけがスキップされる |
| 5 | タグ作成〜リリース発行 | [versioning.md §5](versioning.md#5-リリース手順) のチェックリストに従う: `git tag vYYYY.MM.P` → `git push origin vYYYY.MM.P` → Release workflow 完走 → アセット13点（ファーム4 = vehicle/controller × full/app、フラッシャ4 = Windows/macOS ARM/macOS Intel/Linux、セットアップ4 = Windows/macOS ARM/macOS Intel/Linux、`SHA256SUMS.txt`）の添付を確認 |
| 6 | リリースノート加筆 | 自動生成ノートに原稿（#1）の要点を追記 |
| 7 | 互換表更新 | [versioning.md §6](versioning.md#6-カリキュラム互換表) に新リリースの行を追加 |

## 5. リリース後 — 実機・利用者側への反映

| 対象 | 手順 |
|------|------|
| エコシステム（PC 側） | `sf upgrade`（git 取得・依存再同期・フラッシャ更新提案まで一括） |
| 機体ファームウェア | GUI フラッシャまたは配布 `full` bin の書き込み（NVS ごと初期化）、あるいは `sf flash vehicle`（NVS 温存） |
| **既定値が変わったパラメータ** | `sf flash` で更新した既存機体では `param reset` → `param save`（§2。full bin 書き込みなら不要） |
| 動作確認 | 変更したパラメータを `param get <name>` で確認 → ベンチ確認（ブザー・センサー・テレメトリ）→ 検証飛行 |

---

<a id="english"></a>

## 1. Overview

### About This Document

This document describes the end-to-end procedure for changing firmware behavior
(control laws, features) or parameter defaults and shipping the change as a new
release (`vYYYY.MM.P` tag). The release-time checklist (final pre-tag checks
through Release publication) in
[versioning.md §5](versioning.md#5-release-procedure) remains the source of
truth; this document covers **what comes before it (making the change
correctly) and after it (getting the change onto real craft)**. For how to
choose the version number (the `P` increment rules), see
[versioning.md §2](versioning.md#2-versioning-scheme).

### Target Audience

- Developers changing parameter defaults (PID gains, filter constants, ...)
- Developers changing firmware functionality for a release

### Related Documents

| Document | Role |
|----------|------|
| [versioning.md](versioning.md) | Versioning scheme (§2), artifact list (§3), release checklist (§5), compatibility table (§6) |
| [development_roadmap.md](../../firmware/vehicle/docs/development_roadmap.md) | The 3 identity principles (Code/Param/Model), SILS→hardware workflow |
| [architecture.md](../../firmware/vehicle/docs/architecture.md) | Architectural invariants (INV) — mandatory cross-check for control-law changes |
| [docs/plans/](../plans/) | Home of the per-release notes drafts `release-vYYYY.MM.P-notes.md` |

## 2. Changing a Parameter Default

### Always Two Places to Edit

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

### Numerical Backing (Mandatory)

Control-parameter changes must be committed **with numerical backing**
(project rule; qualitative "should help" reasoning alone is not acceptable).

| Origin of the change | Required backing |
|----------------------|------------------|
| Analysis / tuning study proposal | Quantify the effect via simulation against real flight logs. Capture logs with `sf log wifi` / `sf log convert`; prior art for analysis/replay lives under `analysis/` (e.g. `analysis/scripts/roll_tuning_20260717/`). Record the before/after quantitative metrics (band-limited RMS, tracking-error improvement, ...) in the commit message or the release-notes draft |
| Pilot's in-flight hand-tune direction | The directed value may be adopted as-is, but the **A/B SILS regression run** (below) is still mandatory |

SILS (Software-In-the-Loop: the firmware flying in a PC emulator as a
regression test) **flies on the compiled-in defaults** (Param Identity — see
[development_roadmap.md](../../firmware/vehicle/docs/development_roadmap.md)),
so a default change feeds straight into SILS behavior. The A/B check verifies
that "the behavior may change, but every scenario still meets its pass
criteria."

The A/B procedure, in order — each scenario's pass/fail is the exit code of
`sf sils scenario` (0 = PASS):

```bash
source setup_env.sh
sf sils build
# 1) save the pre-change results
for scn in simulator/sils/scenarios/*.scn; do
  sf sils scenario "$scn" >/dev/null 2>&1 \
    && echo "PASS $(basename "$scn")" || echo "FAIL $(basename "$scn")"
done | tee /tmp/sils_before.txt

# 2) edit params.cpp  3) rebuild with `sf sils build` and rerun the same
#    loop into /tmp/sils_after.txt

# 4) assert an empty diff (identical pass/fail sets)
diff /tmp/sils_before.txt /tmp/sils_after.txt && echo "no regression"
```

The scope is every scenario matching `simulator/sils/scenarios/*.scn`
(39 as of 2026-07; the glob is authoritative, the count may grow). A "known
FAIL" is one that already failed in step 1's pre-change run — only scenarios
that newly flip to FAIL count as regressions.

### Reaching Existing Craft — Understand NVS Precedence

**Rewriting the app alone does not change flight parameters on existing
craft.** At boot, `params::load()` applies compiled-in defaults first, then
overrides them with saved NVS values — so on any craft that has ever run
`param save`, **the saved NVS values keep masking the new defaults**. Whether
NVS survives depends on how you flash:

| Craft state / flashing method | How the new default takes effect |
|-------------------------------|----------------------------------|
| **Flashing the full bin** (GUI flasher, or `esptool.py write_flash 0x0`). The full bin is a single image combining bootloader + partition table + app | Nothing to do. The merged image also overwrites the NVS partition sandwiched in between (offset 0x9000) with 0xFF — i.e. erases it — so the defaults apply directly. Note that **saved tuning values and hover-learning data are wiped too** |
| **Updating via `sf flash vehicle`** (writes app etc. only; NVS survives) | On the serial console (`sf monitor`): `param reset` → `param save` |
| Craft that never ran `param save` | Nothing to do (defaults apply directly) |

`param reset` only restores defaults in RAM (rebooting before saving undoes
it). `param save` is **accepted only while disarmed**, because the NVS
flash-sector erase stalls the 400 Hz control loop (it is refused while armed).
To apply a single parameter instead, `param set <name> <value>` →
`param save` also works (`param set` updates RAM only; persistence is
`param save`).

**Verify the result**: after applying (post-`param save` or post-reboot), run
`param get <changed-parameter>` and confirm it returns the new default.

**A release that changes defaults must state both of these in its release
notes:**

1. Which parameter default changed, from what value to what value
2. How existing craft pick it up (automatic with a full-bin flash; `param
   reset` → `param save` after an `sf flash` update)

## 3. Changing Firmware Behavior (Control Laws / Features)

For code changes (not just defaults), read the six documents under
`firmware/vehicle/docs/` (requirements / architecture / detailed_design /
coding_and_education / development_roadmap / hardware_init) first, then follow:

| Step | Content |
|------|---------|
| 1. Design cross-check | Changes touching control laws, state machines, or flight phases must be checked against the INV section of `architecture.md`; if an assumption changes, enumerate ripple effects on existing components |
| 2. Implementation | Follow the coding rules: bilingual comments, `@design` tags, no magic numbers |
| 3. SILS regression | Run every scenario in `simulator/sils/scenarios/*.scn`; the pass/fail set must match the pre-change run (same A/B procedure as §2, including the known-FAIL definition) |
| 4. Build | `sf build vehicle` (and `sf build controller` if touched) |
| 5. Hardware validation | Control-law changes require real-flight verification (SILS PASS does not guarantee hardware safety) |
| 6. Commit | Use the `/commit` skill, including a Next steps section |

## 4. The Release Flow End to End

Once all changes are on main, proceed in this order. Choose the tag name per
[versioning.md §2](versioning.md#2-versioning-scheme).

| # | Step | Command / check |
|---|------|-----------------|
| 1 | Create/update the release-notes draft | When the version number is decided, create `docs/plans/release-vYYYY.MM.P-notes.md` (copying the previous release's draft, e.g. `release-v2026.07.2-notes.md`, is the fast path). Default changes must include both points from §2 |
| 2 | One consolidated SILS run on final main | Every scenario in `simulator/sils/scenarios/*.scn`, once, right before tagging — even if each commit passed individually |
| 3 | Local builds | `sf build vehicle` / `sf build controller` |
| 4 | CI pre-verification | On GitHub: **Actions tab → select `Release firmware binaries` → Run workflow → choose branch `main`**. All jobs must be green (2 firmware builds = vehicle/controller + 4-OS flasher). Without a tag, only the Release-publish job is skipped |
| 5 | Tag and publish | Follow [versioning.md §5](versioning.md#5-release-procedure): `git tag vYYYY.MM.P` → `git push origin vYYYY.MM.P` → workflow completes → confirm all 13 assets are attached (4 firmware = vehicle/controller × full/app, 4 flasher = Windows / macOS ARM / macOS Intel / Linux, 4 setup = Windows / macOS ARM / macOS Intel / Linux, plus `SHA256SUMS.txt`) |
| 6 | Amend release notes | Add the draft's key points to the auto-generated notes |
| 7 | Compatibility table | Add the release's row to [versioning.md §6](versioning.md#6-curriculum-compatibility-table) |

## 5. After the Release — Reaching Craft and Users

| Target | Procedure |
|--------|-----------|
| Ecosystem (PC side) | `sf upgrade` (git update, dependency resync, flasher-update offer in one step) |
| Vehicle firmware | GUI flasher or the released `full` bin (re-initializes NVS too), or `sf flash vehicle` (NVS survives) |
| **Changed parameter defaults** | On craft updated via `sf flash`: `param reset` → `param save` (§2; not needed after a full-bin flash) |
| Sanity check | `param get <name>` on changed parameters → bench check (buzzer, sensors, telemetry) → verification flight |
