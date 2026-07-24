# 物理パラメータ整合検査ツール（params_audit）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

`tools/params_audit` は、StampFly の機体物理パラメータ（推力係数 C_T、トルク係数
C_Q、トルク/推力比 kappa、慣性モーメント 等）がファームウェア・SIL・複数の
シミュレータへ手動でコピペされていることによる食い違いを、決定論的に検出する
検査ツールである。`sf params check` として CLI から実行する。

### なぜ必要か

コード生成パイプライン（spec YAML → 各言語のコードを自動生成する仕組み）が
まだ無い（Phase 1、`docs/architecture/simulation-policy.md` 参照）ため、同じ
物理定数が `tools/sysid/defaults.py`・`simulator/genesis/motor_model.py`・
`simulator/vpython/core/motors.py`・`firmware/vehicle/components/sf_actuator/
actuator.cpp` 等、10箇所以上に手書きで複製されている。一度でも実測値が更新
されると、更新漏れの箇所だけ旧値のまま取り残される — これを人手でのレビュー
ではなく機械的に検出するのが本ツールの目的（Phase 0）。

### 対象読者

- 機体物理パラメータ（モータ・プロペラ係数、慣性、質量 等）を再測定・更新する開発者
- SIL・シミュレータの物理モデルを改修する開発者
- CI にパラメータ整合チェックを組み込みたい開発者

## 2. 使い方

```bash
source setup_env.sh   # ESP-IDF + sf CLI を有効化

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

## 3. マニフェストの拡張方法

検査対象は `tools/params_audit/params_manifest.py` の `MANIFEST` 辞書で定義する。
パラメータ名（例 `"C_T"`）をキーに、`ParamCheck` のリストを値とする:

```python
"C_T": [
    ParamCheck(
        file="tools/sysid/defaults.py",
        regex=r'_CT_VALUE\s*=\s*([0-9eE.+-]+)',
        expected=EXPECTED_CT,
        note="module constant _CT_VALUE",
    ),
    ...
],
```

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

### Phase 1 への発展

将来的には spec YAML（`protocol/spec/` に倣った機械可読定義）から各言語の
コードを自動生成する方式へ移行する計画があり、実現すれば本マニフェストによる
「後追い検査」自体が不要になる。それまでの Phase 0 として、本ツールは
食い違いの早期発見に用いる。

---

<a id="english"></a>

## 1. Overview

### About This Document

`tools/params_audit` deterministically detects divergence caused by
StampFly's vehicle physical parameters (thrust coefficient C_T, torque
coefficient C_Q, torque/thrust ratio kappa, moments of inertia, etc.) being
hand-copied across the firmware, the SIL, and several simulators. It runs
from the CLI as `sf params check`.

### Why This Is Needed

There is no code-generation pipeline yet (Phase 1 — generating per-language
code from a spec YAML, see `docs/architecture/simulation-policy.md`), so the
same physical constants are hand-duplicated in 10+ places, including
`tools/sysid/defaults.py`, `simulator/genesis/motor_model.py`,
`simulator/vpython/core/motors.py`, and
`firmware/vehicle/components/sf_actuator/actuator.cpp`. Whenever a measured
value is updated, any copy that was missed is silently left stale. This
tool's purpose (Phase 0) is to detect that mechanically instead of relying
on manual review.

### Target Audience

- Developers re-measuring or updating vehicle physical parameters (motor/prop
  coefficients, inertia, mass, etc.)
- Developers modifying the SIL/simulator physics models
- Developers wiring a parameter-consistency check into CI

## 2. Usage

```bash
source setup_env.sh   # activate ESP-IDF + sf CLI

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

## 3. Extending the Manifest

Checked locations are declared in the `MANIFEST` dict in
`tools/params_audit/params_manifest.py`, keyed by parameter name (e.g.
`"C_T"`), each mapping to a list of `ParamCheck` entries:

```python
"C_T": [
    ParamCheck(
        file="tools/sysid/defaults.py",
        regex=r'_CT_VALUE\s*=\s*([0-9eE.+-]+)',
        expected=EXPECTED_CT,
        note="module constant _CT_VALUE",
    ),
    ...
],
```

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

### Path to Phase 1

The long-term plan is to generate per-language code from a machine-readable
spec YAML (following the pattern in `protocol/spec/`), which would make this
after-the-fact manifest check unnecessary. Until then, this Phase-0 tool is
used to catch divergence early.
