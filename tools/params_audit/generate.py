#!/usr/bin/env python3
"""
sf params generate — regenerate physical-parameter code from the SSOT YAML.
sf params generate — SSOT YAML から物理パラメータのコードを再生成する。

Reads control/models/stampfly_physical.yaml (the Single Source of Truth for
StampFly's physical parameters) and machine-generates three artifacts:

  1. tools/sysid/_generated_params.py         -- flat, stdlib-importable
     Python constants (consumed by tools/sysid/defaults.py and
     tools/params_audit/params_manifest.py).
  2. simulator/sils/plant/generated_params.hpp -- C++ constexpr constants
     (consumed by simulator/sils/plant/plant.hpp).
  3. The <!-- AUTO-GENERATED:params --> marker table inside
     docs/architecture/stampfly-parameters.md (both the JP and EN sections).

`--check` renders the same three artifacts in memory and compares them
against what is currently on disk WITHOUT writing anything — used to catch
"edited the YAML but forgot to regenerate" (wired into CI, see
.github/workflows/sils-regression.yml).

control/models/stampfly_physical.yaml（StampFly 物理パラメータの唯一の正）を
読み込み、以下の3種類の生成物を機械生成する:

  1. tools/sysid/_generated_params.py         -- フラットな、標準ライブラリ
     だけで import 可能な Python 定数（tools/sysid/defaults.py と
     tools/params_audit/params_manifest.py が使用）。
  2. simulator/sils/plant/generated_params.hpp -- C++ constexpr 定数
     （simulator/sils/plant/plant.hpp が使用）。
  3. docs/architecture/stampfly-parameters.md 内の
     <!-- AUTO-GENERATED:params --> マーカー表（日本語・英語セクション両方）。

`--check` は同じ3生成物をメモリ上でレンダリングし、ディスク上の現在の内容と
比較するだけで書き込みは行わない — 「YAMLを変えたのに再生成を忘れた」を
検出する（CI に配線済み、.github/workflows/sils-regression.yml 参照）。

Depends on PyYAML (available in the ESP-IDF python env that `sf` runs under;
see setup_env.sh). Unlike check_params.py, this module is NOT required to be
stdlib-only -- only the GENERATED _generated_params.py must be
stdlib-importable, which it is (plain float constants, no imports).
PyYAML に依存する（`sf` が動く ESP-IDF python 環境に同梱、setup_env.sh
参照）。check_params.py と異なり本モジュール自体は標準ライブラリのみに
限定されない -- 標準ライブラリだけで import 可能でなければならないのは
「生成される」_generated_params.py 側であり、実際にそうなっている
（単純な float 定数のみ、import なし）。
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - environment problem, not a code bug
    print(
        "[ERROR] PyYAML is required (`import yaml` failed). Run under the "
        "sf CLI's Python env: `source setup_env.sh`.\n"
        "[ERROR] PyYAML が必要（`import yaml` に失敗）。sf CLI の Python "
        "環境で実行すること: `source setup_env.sh`。",
        file=sys.stderr,
    )
    raise

SSOT_REL = "control/models/stampfly_physical.yaml"
GEN_PY_REL = "tools/sysid/_generated_params.py"
GEN_HPP_REL = "simulator/sils/plant/generated_params.hpp"
DOCS_REL = "docs/architecture/stampfly-parameters.md"

MARKER_BEGIN = "<!-- AUTO-GENERATED:params BEGIN -->"
MARKER_END = "<!-- AUTO-GENERATED:params END -->"

# Anchor line each language section's marker block is inserted after, the
# first time the marker doesn't exist yet in the file.
# マーカーがファイルにまだ存在しない初回のみ、この行の直後に挿入する。
JP_ANCHOR = "## 3. モーター・プロペラパラメータ"
EN_ANCHOR = "## 3. Motor & Propeller Parameters"
EN_SPLIT = '<a id="english"></a>'

_SUPERSCRIPT = str.maketrans("0123456789-", "⁰¹²³⁴⁵⁶⁷⁸⁹⁻")


def find_repo_root() -> Path:
    """Walk up from this file looking for a repo marker (.git/CLAUDE.md).
    このファイルから上へ辿りリポジトリマーカー(.git/CLAUDE.md)を探す。

    Mirrors check_params.find_repo_root() (duplicated, not imported, to keep
    this module runnable standalone without params_audit's package context).
    check_params.find_repo_root() と同じロジック（standalone 実行できるよう
    import ではなく複製）。
    """
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        if (parent / ".git").exists() or (parent / "CLAUDE.md").exists():
            return parent
    return here.parents[2]


# =============================================================================
# YAML loading + derived-value evaluation
# YAML 読み込み＋派生値の評価
# =============================================================================
def load_spec(repo_root: Path) -> Dict[str, Any]:
    """Load and parse the SSOT YAML. / SSOT YAML を読み込みパースする。"""
    path = repo_root / SSOT_REL
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def eval_derived(formula: str, params: Dict[str, Dict[str, Any]]) -> float:
    """Evaluate a "derived" formula string (e.g. "Cq / Ct") against a
    calibration set's params dict. No builtins available -- the formula
    strings live in the trusted, developer-edited SSOT YAML, not user input.
    "derived" の式文字列（例 "Cq / Ct"）を較正セットの params 辞書に対して
    評価する。builtins は与えない -- 式文字列は開発者が編集する信頼された
    SSOT YAML 内にあり、ユーザー入力ではない。
    """
    namespace = {name: float(entry["value"]) for name, entry in params.items()}
    return float(eval(formula, {"__builtins__": {}}, namespace))  # noqa: S307


def build_view(spec: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize the raw YAML dict into the flat values every renderer needs.
    生の YAML 辞書を、各レンダラーが必要とするフラットな値へ正規化する。
    """
    constants = {name: entry["value"] for name, entry in spec["constants"].items()}

    measured = spec["calibration_sets"]["measured_2026_07"]
    measured_params = {name: entry["value"] for name, entry in measured["params"].items()}
    derived = {
        name: eval_derived(formula, measured["params"])
        for name, formula in measured["derived"].items()
    }

    legacy = spec["calibration_sets"]["legacy_motor_curve"]
    legacy_params = {name: entry["value"] for name, entry in legacy["params"].items()}

    return {
        "constants": constants,
        "measured_params": measured_params,
        "measured_derived": derived,
        "measured_entries": measured["params"],
        "legacy_params": legacy_params,
    }


# =============================================================================
# Formatting helpers
# 整形ヘルパー
# =============================================================================
def _py_float(value: float) -> str:
    """Full-round-trip-precision Python float literal via repr().
    repr() によるラウンドトリップ完全精度の Python float リテラル。"""
    return repr(float(value))


def _cpp_float(value: float) -> str:
    """Same precision as _py_float, with the 'f' (float) suffix C++ needs.
    _py_float と同精度、C++ の float サフィックス 'f' 付き。"""
    return f"{_py_float(value)}f"


def _doc_value(value: float) -> str:
    """Repo-style display: unicode-superscript scientific notation for
    |value| < 0.01, plain decimal otherwise (matches the existing hand-written
    tables' convention in docs/architecture/stampfly-parameters.md).
    リポジトリ流の表示: |値| < 0.01 は unicode 上付き指数表記、それ以外は
    普通の10進表記（stampfly-parameters.md の既存手書き表の慣例に合わせる）。
    """
    if value == 0 or abs(value) >= 0.01:
        text = f"{value:g}"
        return text
    exponent = math.floor(math.log10(abs(value)))
    mantissa = value / (10 ** exponent)
    # Guard against float rounding pushing the mantissa to exactly 10.
    # float の丸めでmantissaがちょうど10になる場合の保護。
    if round(mantissa, 6) >= 10:
        mantissa /= 10
        exponent += 1
    mantissa_str = f"{mantissa:.4g}"
    exponent_str = str(exponent).translate(_SUPERSCRIPT)
    return f"{mantissa_str}×10{exponent_str}"


def _header_block(comment_open: str, comment_close: str, line_prefix: str = "") -> str:
    """Bilingual "AUTO-GENERATED -- DO NOT EDIT" banner.
    バイリンガルの「自動生成 -- 編集しないこと」バナー。"""
    lines = [
        "AUTO-GENERATED -- DO NOT EDIT.",
        f"Source: {SSOT_REL}",
        "Regenerate: sf params generate",
        "",
        "自動生成 -- 編集しないこと。",
        f"元データ: {SSOT_REL}",
        "再生成: sf params generate",
    ]
    body = "\n".join(f"{line_prefix}{line}".rstrip() for line in lines)
    return f"{comment_open}\n{body}\n{comment_close}\n"


# =============================================================================
# Renderer 1: tools/sysid/_generated_params.py
# =============================================================================
def render_python(view: Dict[str, Any]) -> str:
    m = view["measured_params"]
    d = view["measured_derived"]
    c = view["constants"]

    out: List[str] = []
    out.append('"""')
    out.append(_header_block("", "").rstrip())
    out.append('"""')
    out.append("")
    out.append("# --- calibration_sets.measured_2026_07 (status: adopted) ---")
    out.append("# tools/sysid/defaults.py's module constants and")
    out.append("# tools/params_audit/params_manifest.py's EXPECTED_* both import from here.")
    out.append(f"_CT_VALUE = {_py_float(m['Ct'])}   # N/(rad/s)^2 -- thrust coeff")
    out.append(f"_CQ_VALUE = {_py_float(m['Cq'])}   # N*m/(rad/s)^2 -- torque coeff")
    out.append(f"_JMP_VALUE = {_py_float(m['Jmp'])}   # kg*m^2 -- rotor inertia")
    out.append(
        f"_DM_VALUE = {_py_float(m['Dm'])}   # N*m*s/rad -- viscous damping "
        "(2026-07-26 coast-down 3-term refit, b term ~0)"
    )
    out.append(
        f"_QF_VALUE = {_py_float(m['Qf'])}   # N*m -- Coulomb friction torque "
        "(2026-07-26 coast-down 3-term refit, c term x Jmp)"
    )
    out.append(f"_RM_VALUE = {_py_float(m['Rm'])}   # ohm -- winding resistance")
    out.append(f"_KM_VALUE = {_py_float(m['Km'])}   # V/(rad/s) -- back-EMF constant")
    out.append(
        f"_KAPPA_VALUE = {_py_float(d['kappa'])}   # m -- kappa = Cq/Ct, full precision (derived)"
    )
    out.append("")
    out.append("# Rounded/adopted kappa (3 sig figs), hand-written into firmware's B^-1")
    out.append("# mixer (actuator.cpp KAPPA) and simulator/sils/plant Config::kappa on")
    out.append("# 2026-07-17. Differs from _KAPPA_VALUE (full precision) by ~1e-4 relative --")
    out.append("# both are kept (see control/models/stampfly_physical.yaml kappa_adopted note)")
    out.append("# to avoid changing that existing rounding (behavior neutrality).")
    out.append(f"KAPPA_ADOPTED = {_py_float(m['kappa_adopted'])}")
    out.append("")
    out.append("# --- constants (calibration-independent) ---")
    out.append(f"EXPECTED_MASS = {_py_float(c['mass'])}")
    out.append(f"EXPECTED_IXX = {_py_float(c['Ixx'])}")
    out.append(f"EXPECTED_IYY = {_py_float(c['Iyy'])}")
    out.append(f"EXPECTED_IZZ = {_py_float(c['Izz'])}")
    out.append(f"EXPECTED_ARM = {_py_float(c['arm_offset'])}")
    out.append(f"EXPECTED_URDF_BASE_MASS = {_py_float(c['urdf_base_mass'])}")
    out.append("")
    out.append("# --- cross-file comparison values (tools/params_audit/params_manifest.py) ---")
    out.append("EXPECTED_CT = _CT_VALUE")
    out.append("EXPECTED_CQ = _CQ_VALUE")
    out.append("EXPECTED_JMP = _JMP_VALUE")
    out.append("EXPECTED_DM = _DM_VALUE")
    out.append("EXPECTED_QF = _QF_VALUE")
    out.append("EXPECTED_RM = _RM_VALUE")
    out.append("EXPECTED_KM = _KM_VALUE")
    out.append("EXPECTED_KAPPA = KAPPA_ADOPTED")
    out.append("")
    return "\n".join(out)


# =============================================================================
# Renderer 2: simulator/sils/plant/generated_params.hpp
# =============================================================================
def render_cpp(view: Dict[str, Any]) -> str:
    c = view["constants"]
    m = view["measured_params"]
    d = view["measured_derived"]
    lg = view["legacy_params"]

    out: List[str] = []
    out.append(_header_block("/*", " */", line_prefix=" * ").rstrip())
    out.append("#pragma once")
    out.append("")
    out.append("namespace sils_params {")
    out.append("")
    out.append("// --- constants (calibration-independent) / 較正非依存の定数 ---")
    out.append(f"constexpr float MASS = {_cpp_float(c['mass'])};  ///< vehicle mass [kg]")
    out.append(f"constexpr float IXX = {_cpp_float(c['Ixx'])};  ///< roll moment of inertia [kg*m^2]")
    out.append(f"constexpr float IYY = {_cpp_float(c['Iyy'])};  ///< pitch moment of inertia [kg*m^2]")
    out.append(f"constexpr float IZZ = {_cpp_float(c['Izz'])};  ///< yaw moment of inertia [kg*m^2]")
    out.append(f"constexpr float ARM_OFFSET = {_cpp_float(c['arm_offset'])};  ///< moment arm [m]")
    out.append("")
    out.append("// --- calibration_sets.measured_2026_07 (status: adopted) ---")
    out.append("// Consumed directly by simulator/sils/plant/plant.hpp::Config (backlog #2,")
    out.append("// 2026-07-26 motor ODE): Ct/Cq/Jmp/Dm/Qf/Rm/Km all feed the electromechanical")
    out.append("// ODE dw/dt = [-(Dm+Km^2/Rm)*w - Cq*w^2 - Qf + Km*V/Rm] / Jmp directly (no")
    out.append("// longer 'reference only' -- the static Am/Bm/Cm curve they used to sit")
    out.append("// beside is retired).")
    out.append("namespace adopted {")
    out.append(
        f"constexpr float CT = {_cpp_float(m['Ct'])};  ///< N/(rad/s)^2, Config::Ct source "
        "(T = Ct*omega^2)"
    )
    out.append(
        f"constexpr float CQ = {_cpp_float(m['Cq'])};  ///< N*m/(rad/s)^2, Config::Cq source "
        "(Q = Cq*omega^2, also the ODE's aero-drag term)"
    )
    out.append(
        f"constexpr float JMP = {_cpp_float(m['Jmp'])};  ///< kg*m^2, Config::Jmp source "
        "(ODE rotor inertia)"
    )
    out.append(
        f"constexpr float DM = {_cpp_float(m['Dm'])};  ///< N*m*s/rad, Config::Dm source "
        "(viscous damping, ~0 -- 2026-07-26 coast-down refit)"
    )
    out.append(
        f"constexpr float QF = {_cpp_float(m['Qf'])};  ///< N*m, Config::Qf source "
        "(Coulomb friction torque)"
    )
    out.append(f"constexpr float RM = {_cpp_float(m['Rm'])};  ///< ohm, winding resistance")
    out.append(f"constexpr float KM = {_cpp_float(m['Km'])};  ///< V/(rad/s), back-EMF constant")
    out.append(
        f"constexpr float KAPPA = {_cpp_float(m['kappa_adopted'])};  "
        "///< m, adopted/rounded (matches firmware actuator.cpp KAPPA)"
    )
    out.append(
        f"constexpr float KAPPA_EXACT = {_cpp_float(d['kappa'])};  "
        "///< m, full-precision Cq/Ct (reference only, unused)"
    )
    out.append("}  // namespace adopted")
    out.append("")
    out.append(
        "// --- calibration_sets.legacy_motor_curve (status: frozen_for_implementation, "
        "backlog #3) ---"
    )
    out.append("// Not consumed by any implementation as of 2026-07-26 -- plant.hpp switched")
    out.append("// its motor model to the electromechanical ODE (backlog #2) and no longer")
    out.append("// references Am/Bm/Cm/motor-curve-Ct at all. The only surviving copy of this")
    out.append("// family's Ct is firmware actuator.cpp's MOTOR_CT, a hand literal (NOT")
    out.append("// sourced from here). Kept for historical reference and for backlog #3")
    out.append("// (firmware-side switch) to consult.")
    out.append("namespace legacy {")
    out.append(f"constexpr float CT = {_cpp_float(lg['Ct'])};")
    out.append(f"constexpr float CQ = {_cpp_float(lg['Cq'])};")
    out.append(f"constexpr float AM = {_cpp_float(lg['Am'])};")
    out.append(f"constexpr float BM = {_cpp_float(lg['Bm'])};")
    out.append(f"constexpr float CM = {_cpp_float(lg['Cm'])};")
    out.append(f"constexpr float RM = {_cpp_float(lg['Rm'])};")
    out.append(f"constexpr float KM = {_cpp_float(lg['Km'])};")
    out.append(f"constexpr float DM = {_cpp_float(lg['Dm'])};")
    out.append(f"constexpr float QF = {_cpp_float(lg['Qf'])};")
    out.append("}  // namespace legacy")
    out.append("")
    out.append("}  // namespace sils_params")
    out.append("")
    return "\n".join(out)


# =============================================================================
# Renderer 3: docs/architecture/stampfly-parameters.md marker table
# =============================================================================
_DOC_ROWS_JP: Tuple[Tuple[str, str, str, str], ...] = (
    # (label, symbol, constants-key-or-measured-key, kind)
    ("機体質量", "mass", "mass", "const"),
    ("Roll慣性モーメント", "Ixx", "Ixx", "const"),
    ("Pitch慣性モーメント", "Iyy", "Iyy", "const"),
    ("Yaw慣性モーメント", "Izz", "Izz", "const"),
    ("モーメントアーム", "arm_offset", "arm_offset", "const"),
    ("推力係数", "Ct", "Ct", "measured"),
    ("トルク係数", "Cq", "Cq", "measured"),
    ("回転子慣性モーメント", "Jmp", "Jmp", "measured"),
    ("巻線抵抗", "Rm", "Rm", "measured"),
    ("逆起電力定数", "Km", "Km", "measured"),
    ("トルク/推力比（採用値）", "κ", "kappa_adopted", "measured"),
)

_DOC_ROWS_EN: Tuple[Tuple[str, str, str, str], ...] = (
    ("Vehicle mass", "mass", "mass", "const"),
    ("Roll moment of inertia", "Ixx", "Ixx", "const"),
    ("Pitch moment of inertia", "Iyy", "Iyy", "const"),
    ("Yaw moment of inertia", "Izz", "Izz", "const"),
    ("Moment arm", "arm_offset", "arm_offset", "const"),
    ("Thrust coefficient", "Ct", "Ct", "measured"),
    ("Torque coefficient", "Cq", "Cq", "measured"),
    ("Rotor inertia", "Jmp", "Jmp", "measured"),
    ("Winding resistance", "Rm", "Rm", "measured"),
    ("Back-EMF constant", "Km", "Km", "measured"),
    ("Torque/thrust ratio (adopted)", "κ", "kappa_adopted", "measured"),
)


def render_docs_block(view: Dict[str, Any], lang: str) -> str:
    """Render just the inner content (between markers) for one language.
    1言語分の内側コンテンツ（マーカー間）のみをレンダリングする。

    Column order is deliberately SYMBOL-then-LABEL (not label-then-symbol):
    tools/params_audit/params_manifest.py's existing hand-table ParamCheck
    regexes anchor on "<JP-or-EN label>\\s*\\|\\s*<symbol>\\s*\\|" against
    this same doc file's pre-existing hand-written tables further down. If
    this generated table used the same label-then-symbol column order with
    matching label text (e.g. "推力係数 | Ct |"), `re.search` would match
    THIS block first (it appears earlier in the file) instead of the
    intended hand table, silently checking the wrong row. Reversing the
    column order breaks that accidental collision structurally, without
    relying on label-text differences that are easy to lose in a future edit.
    列順を意図的に「記号→ラベル」にしている（ラベル→記号ではない）:
    tools/params_audit/params_manifest.py の既存の手書き表用 ParamCheck
    正規表現は、この同じ文書内のさらに下にある既存手書き表に対して
    "<日本語/英語ラベル>\\s*\\|\\s*<記号>\\s*\\|" にアンカーしている。
    もし本生成表が同じ「ラベル→記号」の列順でラベル文言まで一致すると
    （例: "推力係数 | Ct |"）、re.search はファイル中で先に現れる本ブロックに
    先にマッチしてしまい、意図した手書き表ではなく誤った行を検査すること
    になる。列順を逆にすることで、将来の編集で失われやすい「ラベル文言の
    違い」に頼らず、この偶発的な衝突を構造的に防ぐ。
    """
    rows = _DOC_ROWS_JP if lang == "ja" else _DOC_ROWS_EN
    c = view["constants"]
    m = view["measured_params"]
    entries = view["measured_entries"]

    if lang == "ja":
        intro = (
            f"> 以下は `{SSOT_REL}`（Single Source of Truth）から "
            "`sf params generate` が生成した確定値の一覧である。手編集しないこと。"
        )
        headers = ("記号", "パラメータ", "値", "単位", "実測日", "方法")
    else:
        intro = (
            f"> The table below is generated from `{SSOT_REL}` "
            "(Single Source of Truth) by `sf params generate`. Do not hand-edit it."
        )
        headers = ("Symbol", "Parameter", "Value", "Unit", "Date", "Method")

    lines = [intro, "", "| " + " | ".join(headers) + " |",
             "|" + "|".join(["---"] * len(headers)) + "|"]

    for label, symbol, key, kind in rows:
        if kind == "const":
            value = c[key]
            unit = view["_const_units"][key]
            date = view["_const_dates"][key]
            method = "-"
        else:
            value = m[key]
            entry = entries[key]
            unit = entry.get("unit", "-")
            date = entry.get("date", "-")
            if lang == "en":
                method = entry.get("method_en", entry.get("method", entry.get("decision", "-")))
            else:
                method = entry.get("method", entry.get("decision", "-"))
        lines.append(
            f"| {symbol} | {label} | {_doc_value(value)} | {unit} | {date} | {method} |"
        )

    return "\n".join(lines)


def _attach_doc_metadata(spec: Dict[str, Any], view: Dict[str, Any]) -> None:
    """Stash per-constant unit/date onto the view for render_docs_block().
    render_docs_block() 用に定数ごとの単位/日付を view へ格納する。"""
    view["_const_units"] = {name: e.get("unit", "-") for name, e in spec["constants"].items()}
    view["_const_dates"] = {name: e.get("date", "-") for name, e in spec["constants"].items()}


def splice_markers(text: str, anchor: str, body: str) -> str:
    """Insert/replace the marker block in one language-section substring.
    1言語セクションの部分文字列内でマーカーブロックを挿入/置換する。

    If MARKER_BEGIN/END already exist, replace everything between them
    (inclusive). Otherwise, insert a fresh block right after the anchor
    heading's line. Idempotent: running twice with the same body is a no-op
    the second time.
    MARKER_BEGIN/END が既に存在すればその間（両端含む）を置換する。
    存在しなければアンカー見出し行の直後に新規挿入する。冪等: 同じ body で
    2回実行しても2回目は変化しない。
    """
    block = f"{MARKER_BEGIN}\n{body}\n{MARKER_END}"
    if MARKER_BEGIN in text and MARKER_END in text:
        start = text.index(MARKER_BEGIN)
        end = text.index(MARKER_END) + len(MARKER_END)
        return text[:start] + block + text[end:]

    anchor_idx = text.index(anchor)
    line_end = text.index("\n", anchor_idx)
    return text[: line_end + 1] + "\n" + block + "\n" + text[line_end + 1 :]


def render_docs_full(view: Dict[str, Any], current_text: str) -> str:
    """Splice both the JP and EN marker blocks into the full doc text.
    JP・EN 両方のマーカーブロックを文書全体のテキストへ挿入する。"""
    jp_body = render_docs_block(view, "ja")
    en_body = render_docs_block(view, "en")

    split_idx = current_text.index(EN_SPLIT)
    jp_part = current_text[:split_idx]
    en_part = current_text[split_idx:]

    jp_part = splice_markers(jp_part, JP_ANCHOR, jp_body)
    en_part = splice_markers(en_part, EN_ANCHOR, en_body)
    return jp_part + en_part


# =============================================================================
# Orchestration: generate / --check
# 統括処理: 生成 / --check
# =============================================================================
def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8") if path.exists() else ""


def run_generate(repo_root: Path, check_only: bool) -> int:
    spec = load_spec(repo_root)
    view = build_view(spec)
    _attach_doc_metadata(spec, view)

    py_path = repo_root / GEN_PY_REL
    hpp_path = repo_root / GEN_HPP_REL
    docs_path = repo_root / DOCS_REL

    new_py = render_python(view)
    new_hpp = render_cpp(view)
    new_docs = render_docs_full(view, _read(docs_path))

    targets = [
        (py_path, new_py),
        (hpp_path, new_hpp),
        (docs_path, new_docs),
    ]

    if check_only:
        stale = [str(path.relative_to(repo_root)) for path, new in targets if _read(path) != new]
        if stale:
            print("[FAIL] generated files are stale (SSOT YAML changed without regenerating):")
            for rel in stale:
                print(f"  - {rel}")
            print("Run `sf params generate` and commit the result.")
            return 1
        print("[OK] all generated files match control/models/stampfly_physical.yaml")
        return 0

    changed = []
    for path, new in targets:
        if _read(path) != new:
            changed.append(str(path.relative_to(repo_root)))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(new, encoding="utf-8")

    if changed:
        print("[OK] regenerated (changed):")
        for rel in changed:
            print(f"  - {rel}")
    else:
        print("[OK] regenerated (no changes -- already up to date)")
    return 0


def main(argv: List[str] = None) -> int:
    """CLI entry point -- also called directly by `sf params generate`.
    CLI エントリポイント -- `sf params generate` からも直接呼ばれる。"""
    parser = argparse.ArgumentParser(
        description="Generate physical-parameter code from control/models/stampfly_physical.yaml.",
    )
    parser.add_argument(
        "--check", action="store_true",
        help="do not write anything; exit 1 if generated files are stale",
    )
    args = parser.parse_args(argv)

    repo_root = find_repo_root()
    return run_generate(repo_root, args.check)


if __name__ == "__main__":
    sys.exit(main())
