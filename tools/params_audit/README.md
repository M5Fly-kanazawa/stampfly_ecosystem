# 物理パラメータ整合検査ツール（params_audit）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`tools/params_audit` は、StampFly の機体物理パラメータ（推力係数 C_T、トルク係数
C_Q、トルク/推力比 kappa、慣性モーメント 等）に関する2つのツールをまとめた
ディレクトリである: (1) `sf params generate` — 唯一の正である
`control/models/stampfly_physical.yaml` からコードを機械生成する（Phase 1）、
(2) `sf params check` — 生成の対象になっていない残りの手動コピー箇所を決定論的に
検査する（Phase 0）。

### なぜ必要か

**Phase 1（コード生成、2026-07-26 一部着手）:** `control/models/
stampfly_physical.yaml` を唯一の正とし、`sf params generate` が
`tools/sysid/_generated_params.py`・`simulator/sils/plant/generated_params.hpp`・
`docs/architecture/stampfly-parameters.md` のマーカー表を機械生成する。この3箇所は
もう手書きの数値リテラルを持たない — YAML を編集して `sf params generate` を
実行するだけで全て揃う。

**Phase 0（監査、現役）:** 上記3箇所以外——`simulator/genesis/motor_model.py`・
`simulator/vpython/core/motors.py`・`firmware/vehicle/components/sf_actuator/
actuator.cpp`（firmware は生成対象外）・MuJoCo XML・URDF・
`stampfly-parameters.md` の既存手書き表 等——は、引き続き同じ物理定数を手書きで
複製している（10箇所以上）。一度でも実測値が更新されると、更新漏れの箇所だけ
旧値のまま取り残される — これを人手でのレビューではなく機械的に検出するのが
`sf params check` の目的。`sf params generate` が対象を広げるほど、この監査対象は
減っていく設計。

### 対象読者

- 機体物理パラメータ（モータ・プロペラ係数、慣性、質量 等）を再測定・更新する開発者
- SILS・シミュレータの物理モデルを改修する開発者
- CI にパラメータ整合チェックを組み込みたい開発者

## 2. 使い方

### 値を変更する（Phase 1 対象箇所）

```bash
source setup_env.sh   # ESP-IDF + sf CLI を有効化

# 1. control/models/stampfly_physical.yaml を編集
# 2. コードを再生成
sf params generate
# 3. sf params check --strict で全体の整合を確認してからコミット
sf params check --strict
```

`sf params generate --check` は何も書き込まず、生成物が YAML から乖離していないか
（YAML を編集したのに再生成を忘れていないか）だけを確認する（CI 用、exit 1 で失敗）。

### 整合を検査する（Phase 0 監査、Phase 1 対象箇所も含め全体）

```bash
sf params check              # テキスト表で結果を表示
sf params check --json       # 機械可読 JSON で出力
sf params check --strict     # UNRESOLVED（未確定パラメータ）も失敗扱いにする
```

`tools/params_audit/check_params.py` を直接実行することもできる（標準ライブラリ
のみに依存するため、`sf` CLI を経由しなくても動く）:

```bash
python3 tools/params_audit/check_params.py --json
```

### 判定の種類

| 判定 | 意味 |
|------|------|
| `OK` | 現在値が期待値と相対誤差 1e-6 以内で一致 |
| `MISMATCH` | 現在値が期待値と食い違っている（要修正） |
| `UNRESOLVED` | 正解値がまだ決まっておらず複数の候補値が並立中（`--strict` でのみ失敗扱い） |
| `EXEMPT` | 不一致が既知・意図的で許容されている（理由付き） |
| `ERROR` | ファイルが存在しない、または正規表現が1件もマッチしない（ファイル改変でパターンが失効した可能性。**黙ってスキップせず必ず失敗扱い**） |

終了コード: `MISMATCH` または `ERROR` が1件でもあれば 1、それ以外は 0
（`--strict` 指定時は `UNRESOLVED` も 1 扱いに追加される）。

### 退行検出への組み込み

本検査は `sf sils regression`（`lib/sfcli/commands/sils.py` の `run_regression()`）
の最初のステップとして自動実行され、シナリオ実行前に `--strict` 相当で判定
する。不合格なら回帰全体が即座に失敗する。CI（`.github/workflows/
sils-regression.yml`）にも同じ検査を早期化する専用ステップがある。

## 3. マニフェストの拡張方法

検査対象は `tools/params_audit/params_manifest.py` の `MANIFEST` 辞書で定義する。
パラメータ名（例 `"C_T"`）をキーに、`ParamCheck` のリストを値とする:

```python
"Ixx": [
    ParamCheck(
        file="tools/sysid/defaults.py",
        regex=r'"Ixx":\s*\{\s*"value":\s*([0-9eE.+-]+),',
        expected=EXPECTED_IXX,
        note="DEFAULT_PARAMS.inertia.Ixx",
    ),
    ...
],
```

（`"C_T"`/`"C_Q"`/`"J_mp"`/`"Rm"`/`"kappa"` の一部の行は Phase 1 で `sf params
generate` の生成先——`tools/sysid/_generated_params.py`・`simulator/sils/plant/
generated_params.hpp`——を指すようになった。それらの値を変更するときは
`params_manifest.py` を編集するのではなく `control/models/
stampfly_physical.yaml` を編集して `sf params generate` を実行すること。）

新しいコピー箇所を追加する手順:

1. **対象ファイルを実際に読み、現在の表記に正確にマッチする正規表現を書く**
   （推測で書かない）。捕捉グループ（`(...)`）は数値トークンの周りに正確に1つだけ置く。
2. 正規表現は**変数名・定数名にアンカーし、値そのものにアンカーしない**こと。
   値にアンカーすると、その値と一致するときしかマッチせず、食い違いを原理的に
   検出できなくなる。
3. 期待値は `EXPECTED_*` 定数として本ファイル冒頭にまとめ、出所（実測日・
   参照文書）をコメントで残す。正解がまだ決まっていないパラメータは
   `Unresolved(candidates="...")` を使う。既知・意図的な不一致は
   `Exempt(reason="...")` を使い、理由（元コメントの引用等）を残す。
4. 同一ファイル内に同じパラメータが複数箇所ある場合は `note=` で場所を区別する
   （例 `"DEFAULT_PARAMS"` と `"module constant"`）。
5. `sf params check` を実行し、新しい行が `ERROR` にならず意図した判定
   （`OK`/`MISMATCH`/`UNRESOLVED`/`EXEMPT`）になることを確認する。

### Phase 1 の状況

`control/models/stampfly_physical.yaml`（spec YAML）から `sf params generate`
がコードを生成する方式が、`tools/sysid/_generated_params.py`・
`simulator/sils/plant/generated_params.hpp`・`docs/architecture/
stampfly-parameters.md` のマーカー表について 2026-07-26 に着手済み——この3箇所は
もう本マニフェストによる「後追い検査」の対象ではなく、YAML そのものが正で
生成が保証する（`sf params generate --check` で検出、CI 組み込み済み）。
残りの手動コピー箇所（`simulator/genesis/*`・`simulator/vpython/*`・
firmware（生成対象外）・MuJoCo XML・URDF・docs の既存手書き表）は引き続き
Phase 0 の本マニフェストで監査する。生成対象を広げるたびに、対応するマニフェスト
の行を `tools/sysid/_generated_params.py` や生成先ファイルへ差し替えていく。

---

<a id="english"></a>

## 1. Overview

### About This Document

`tools/params_audit` bundles two tools for StampFly's vehicle physical
parameters (thrust coefficient C_T, torque coefficient C_Q, torque/thrust
ratio kappa, moments of inertia, etc.): (1) `sf params generate` — machine-
generates code from the single source of truth `control/models/
stampfly_physical.yaml` (Phase 1), and (2) `sf params check` —
deterministically audits the remaining hand-copied locations that generation
does not yet cover (Phase 0).

### Why This Is Needed

**Phase 1 (code generation, started 2026-07-26):** `control/models/
stampfly_physical.yaml` is the single source of truth. `sf params generate`
machine-generates `tools/sysid/_generated_params.py`,
`simulator/sils/plant/generated_params.hpp`, and the marker table in
`docs/architecture/stampfly-parameters.md`. These three locations no longer
hold hand-typed numeric literals -- edit the YAML and run `sf params
generate`.

**Phase 0 (audit, still active):** Everywhere else -- `simulator/genesis/
motor_model.py`, `simulator/vpython/core/motors.py`,
`firmware/vehicle/components/sf_actuator/actuator.cpp` (firmware is out of
generation scope), MuJoCo XML, URDF files, and the pre-existing hand-written
tables in `stampfly-parameters.md` -- still hand-duplicates the same physical
constants in 10+ places. Whenever a measured value is updated, any copy that
was missed is silently left stale. `sf params check`'s purpose is to detect
that mechanically instead of relying on manual review. This audited surface
shrinks as `sf params generate` covers more consumers.

### Target Audience

- Developers re-measuring or updating vehicle physical parameters (motor/prop
  coefficients, inertia, mass, etc.)
- Developers modifying the SILS/simulator physics models
- Developers wiring a parameter-consistency check into CI

## 2. Usage

### Changing a value (Phase 1 locations)

```bash
source setup_env.sh   # activate ESP-IDF + sf CLI

# 1. Edit control/models/stampfly_physical.yaml
# 2. Regenerate the code
sf params generate
# 3. Confirm everything agrees before committing
sf params check --strict
```

`sf params generate --check` writes nothing; it only confirms the generated
files have not drifted from the YAML (i.e. you didn't forget to regenerate
after editing it) -- used in CI, exits 1 on staleness.

### Auditing consistency (Phase 0, covers the whole surface including Phase 1 locations)

```bash
sf params check              # text table
sf params check --json       # machine-readable JSON
sf params check --strict     # also fail on UNRESOLVED parameters
```

`tools/params_audit/check_params.py` can also be run directly (standard
library only, so it works without the `sf` CLI):

```bash
python3 tools/params_audit/check_params.py --json
```

### Verdicts

| Verdict | Meaning |
|---------|---------|
| `OK` | Current value matches the expected value within 1e-6 relative error |
| `MISMATCH` | Current value diverges from the expected value (needs fixing) |
| `UNRESOLVED` | No single correct value has been decided; multiple candidates coexist (fails only under `--strict`) |
| `EXEMPT` | The mismatch is known, intentional, and accepted (with a reason) |
| `ERROR` | File not found, or the regex matched zero times (the pattern may have gone stale after an edit — **never silently skipped, always a failure**) |

Exit code: 1 if any `MISMATCH` or `ERROR`, 0 otherwise (`--strict` also
counts `UNRESOLVED` toward failure).

### Wired into Regression Detection

This audit runs automatically as the first step of `sf sils regression`
(`run_regression()` in `lib/sfcli/commands/sils.py`), gated at `--strict`
severity before any scenario runs — a failure fails the whole regression
immediately. CI (`.github/workflows/sils-regression.yml`) also has a
dedicated step that runs the same check earlier, for a faster fail.

## 3. Extending the Manifest

Checked locations are declared in the `MANIFEST` dict in
`tools/params_audit/params_manifest.py`, keyed by parameter name (e.g.
`"C_T"`), each mapping to a list of `ParamCheck` entries:

```python
"Ixx": [
    ParamCheck(
        file="tools/sysid/defaults.py",
        regex=r'"Ixx":\s*\{\s*"value":\s*([0-9eE.+-]+),',
        expected=EXPECTED_IXX,
        note="DEFAULT_PARAMS.inertia.Ixx",
    ),
    ...
],
```

(Some `"C_T"`/`"C_Q"`/`"J_mp"`/`"Rm"`/`"kappa"` entries now point at `sf
params generate`'s output -- `tools/sysid/_generated_params.py` and
`simulator/sils/plant/generated_params.hpp` (Phase 1). To change one of those
values, edit `control/models/stampfly_physical.yaml` and run `sf params
generate`, rather than editing `params_manifest.py`.)

To add a new copy location:

1. **Read the target file's actual current text and write a regex that
   matches it exactly** (do not guess). Include exactly one capturing group
   around the numeric token.
2. Anchor the regex on the **variable/constant name, never on the value
   itself** — anchoring on the value only ever matches when it's already
   correct, which makes divergence undetectable by construction.
3. Collect expected values as `EXPECTED_*` constants at the top of the file,
   with a source comment (measurement date, reference document). For a
   parameter with no confirmed value yet, use
   `Unresolved(candidates="...")`. For a known, intentional mismatch, use
   `Exempt(reason="...")` and cite the source (e.g. quote the original code
   comment).
4. If the same parameter appears more than once in one file, disambiguate
   with `note=` (e.g. `"DEFAULT_PARAMS"` vs. `"module constant"`).
5. Run `sf params check` and confirm the new row is not `ERROR` and lands on
   the intended verdict (`OK`/`MISMATCH`/`UNRESOLVED`/`EXEMPT`).

### Phase 1 Status

Generating code from `control/models/stampfly_physical.yaml` (the spec YAML)
via `sf params generate` started 2026-07-26, covering
`tools/sysid/_generated_params.py`, `simulator/sils/plant/generated_params.hpp`,
and the marker table in `docs/architecture/stampfly-parameters.md`. These
three locations are no longer audited after the fact by this manifest -- the
YAML is authoritative and generation guarantees agreement (caught by `sf
params generate --check`, wired into CI). The remaining hand-copied locations
(`simulator/genesis/*`, `simulator/vpython/*`, firmware -- out of generation
scope --, MuJoCo XML, URDF, the docs' pre-existing hand-written tables) are
still audited by this Phase-0 manifest. As generation coverage grows, the
corresponding manifest rows get repointed at `tools/sysid/_generated_params.py`
or the relevant generated file.
