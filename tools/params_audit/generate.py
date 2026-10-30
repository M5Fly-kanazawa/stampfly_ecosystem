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
     (consumed by simulator/sils/plant/plant.hpp). Two namespaces carry the
     calibration sets in use: sils_params::plant (bench_2026_10_15, incl. the
     R_out(D) table) and sils_params::firmware_curve (flight_anchored_motor_curve,
     the firmware thrust->duty constants).
  3. The <!-- AUTO-GENERATED:params --> marker table inside
     docs/architecture/stampfly-parameters.md (both the JP and EN sections).

`--check` renders the same three artifacts in memory and compares them
against what is currently on disk WITHOUT writing anything — used to catch
"edited the YAML but forgot to regenerate" (wired into CI, see
.github/workflows/sils-regression.yml).

control/models/stampfly_physical.yaml（StampFly 物理パラメータの基準となる文書）を
読み込み、以下の3種類の生成物を機械生成する:

  1. tools/sysid/_generated_params.py         -- フラットな、標準ライブラリ
     だけで import 可能な Python 定数（tools/sysid/defaults.py と
     tools/params_audit/params_manifest.py が使用）。
  2. simulator/sils/plant/generated_params.hpp -- C++ constexpr 定数
     （simulator/sils/plant/plant.hpp が使用）。使用中の較正セットは2つの名前空間に
     載る: sils_params::plant（bench_2026_10_15、R_out(D) 表を含む）と
     sils_params::firmware_curve（flight_anchored_motor_curve、ファームの
     推力→duty 換算の定数）。
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


def _interp(x: float, xs: List[float], ys: List[float]) -> float:
    """Piecewise-linear interpolation, held constant outside [xs[0], xs[-1]].
    Same semantics as numpy.interp (xs ascending). Pure Python on purpose.
    区分線形補間。[xs[0], xs[-1]] の外では端の値で一定（numpy.interp と同じ
    意味、xs は昇順）。意図的に標準ライブラリだけで書く。"""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            frac = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + frac * (ys[i + 1] - ys[i])
    raise AssertionError("unreachable")  # pragma: no cover


def compute_rout_tables(rout: Dict[str, Any]) -> Dict[str, Any]:
    """Build the R_out(D) layers from the YAML's measured points (a) and the
    calculated table (b); compute the adopted table (c) = calc x r(D).
    YAML の実測点 (a) と計算表 (b) から R_out(D) の各層を作り、採用表
    (c) = calc × r(D) を計算する。

    r(D) is piecewise-linear through the measured/calc ratios at the measured D
    values (the two D = 0.5 ratios are averaged), held constant below the lowest
    and above the highest measured D. The returned *_table lists have 101
    entries for D = 0.00 .. 1.00 (index = round(D * 100)); entry 0 is NOT a
    physical value (the plant treats D < 0.01 as "no drive", motor current 0,
    explicitly) and simply repeats the D = 0.01 value so the table stays
    monotone and index-aligned.
    r(D) は実測 D での実測/計算比を通る区分線形（D=0.5 の 2 点は平均）で、最小
    実測 D 未満と最大実測 D 超は定数で保持。返す *_table は D=0.00..1.00 の 101
    要素（添字 = round(D×100)）。要素 0 は物理値ではない（プラントは D<0.01 を
    「駆動なし」＝モータ電流 0 と明示的に扱う）ので D=0.01 の値を繰り返すだけで、
    表を単調かつ添字整合のまま保つ。
    """
    calc_d = [float(row[0]) for row in rout["calc"]["points"]]
    calc_r = [float(row[1]) for row in rout["calc"]["points"]]

    ratios_by_d: Dict[float, List[float]] = {}
    for d_cmd, r_meas, _vdd, _tag in rout["measured_points"]:
        ratio = float(r_meas) / _interp(float(d_cmd), calc_d, calc_r)
        ratios_by_d.setdefault(float(d_cmd), []).append(ratio)
    knots_d = sorted(ratios_by_d)
    knots_r = [sum(ratios_by_d[d]) / len(ratios_by_d[d]) for d in knots_d]

    adopted_r = [r * _interp(d, knots_d, knots_r) for d, r in zip(calc_d, calc_r)]

    grid_d = [0.0] + calc_d  # 101 entries; index 0 = the D < 0.01 "no drive" slot
    calc_table = [calc_r[0]] + calc_r
    adopted_table = [adopted_r[0]] + adopted_r
    return {
        "grid_d": grid_d,
        "calc_table": calc_table,
        "adopted_table": adopted_table,
        "ratio_knots_d": knots_d,
        "ratio_knots": knots_r,
    }


def build_view(spec: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize the raw YAML dict into the flat values every renderer needs.
    生の YAML 辞書を、各レンダラーが必要とするフラットな値へ正規化する。

    Five calibration sets are consumed: bench_2026_10_15 (SILS plant),
    flight_anchored_motor_curve (firmware thrust->duty constants), the
    superseded measured_2026_07 (for the not-yet-migrated D*V drive models),
    legacy_motor_curve (docs history table) and plant_fit_flip_2026_10 (the
    opt-in plant profile).
    5つの較正セットを使う: bench_2026_10_15（SILS プラント）、
    flight_anchored_motor_curve（ファームの推力→duty 換算定数）、置き換え済みの
    measured_2026_07（移行前の D×V 駆動モデル向け）、legacy_motor_curve（docs の
    履歴表向け）、plant_fit_flip_2026_10（オプトインのプラントプロファイル向け）。
    """
    constants = {name: entry["value"] for name, entry in spec["constants"].items()}
    sets = spec["calibration_sets"]

    measured = sets["measured_2026_07"]
    measured_params = {name: entry["value"] for name, entry in measured["params"].items()}
    derived = {
        name: eval_derived(formula, measured["params"])
        for name, formula in measured["derived"].items()
    }

    legacy = sets["legacy_motor_curve"]
    legacy_params = {name: entry["value"] for name, entry in legacy["params"].items()}

    plant_fit = sets["plant_fit_flip_2026_10"]
    plant_fit_params = {name: entry["value"] for name, entry in plant_fit["params"].items()}

    bench = sets["bench_2026_10_15"]
    plant_params = {name: entry["value"] for name, entry in bench["params"].items()}
    rout = compute_rout_tables(bench["rout_table"])

    firmware = sets["flight_anchored_motor_curve"]
    firmware_params = {name: entry["value"] for name, entry in firmware["params"].items()}

    return {
        "constants": constants,
        "plant_fit_params": plant_fit_params,
        "measured_params": measured_params,
        "measured_derived": derived,
        "measured_entries": measured["params"],
        "legacy_params": legacy_params,
        "plant_params": plant_params,
        "plant_entries": bench["params"],
        "rout": rout,
        "firmware_params": firmware_params,
        "firmware_entries": firmware["params"],
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
def _py_tuple(values: List[float], per_line: int = 5) -> List[str]:
    """Render a float list as an indented Python tuple literal body.
    float リストをインデント付き Python タプルリテラルの本体として整形する。"""
    lines = []
    for i in range(0, len(values), per_line):
        chunk = ", ".join(_py_float(v) for v in values[i:i + per_line])
        lines.append(f"    {chunk},")
    return lines


def render_python(view: Dict[str, Any]) -> str:
    m = view["measured_params"]
    d = view["measured_derived"]
    c = view["constants"]
    lg = view["legacy_params"]
    pl = view["plant_params"]
    fw = view["firmware_params"]
    rout = view["rout"]

    out: List[str] = []
    out.append('"""')
    out.append(_header_block("", "").rstrip())
    out.append('"""')
    out.append("")
    out.append("# =====================================================================")
    out.append("# Which constant set? / どの較正セットか")
    out.append("#   EXPECTED_FW_*       firmware thrust->duty constants (flight_anchored_motor_curve,")
    out.append("#                       what actuator.cpp holds) -- use when a tool INVERTS firmware duty.")
    out.append("#   EXPECTED_PLANT_*    SILS plant propulsion (bench_2026_10_15) -- use when a tool")
    out.append("#                       MODELS the plant physics the way simulator/sils/plant does.")
    out.append("#   EXPECTED_MEASURED_* superseded measured_2026_07 (own D*V drive model: Genesis,")
    out.append("#                       VPython, loop_shaping_tool, sysid defaults, docs hand tables).")
    out.append("#   EXPECTED_LEGACY_*   legacy_motor_curve (docs history table).")
    out.append("#   EXPECTED_FW_* は actuator.cpp が持つ換算定数、EXPECTED_PLANT_* は SILS プラントの")
    out.append("#   推進系、EXPECTED_MEASURED_* は D×V 駆動のまま移行待ちの置き換え済みセット。")
    out.append("# =====================================================================")
    out.append("")
    out.append("# --- calibration_sets.measured_2026_07 (status: superseded 2026-10-30) ---")
    out.append("# tools/sysid/defaults.py's module constants import these underscore names.")
    out.append("# The SILS plant no longer reads this set; Genesis/VPython/loop_shaping_tool/")
    out.append("# sysid defaults still use it (own D*V drive model, migration pending).")
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
    out.append("# --- firmware thrust->duty constants (flight_anchored_motor_curve, adopted) ---")
    out.append("# What firmware/vehicle/components/sf_actuator/actuator.cpp holds (MOTOR_CT,")
    out.append("# KAPPA, MOTOR_AM/BM/CM). tools/sysid/plant_fit.py inverts exactly this curve.")
    out.append(f"EXPECTED_FW_CT = {_py_float(fw['Ct_gauge'])}   # N/(rad/s)^2 -- thrust gauge (MOTOR_CT)")
    out.append(f"EXPECTED_FW_KAPPA = {_py_float(fw['kappa'])}   # m -- torque/thrust ratio (KAPPA)")
    out.append(f"EXPECTED_FW_AM = {_py_float(fw['Am'])}   # V/(rad/s)^2 (MOTOR_AM)")
    out.append(f"EXPECTED_FW_BM = {_py_float(fw['Bm'])}   # V/(rad/s) (MOTOR_BM)")
    out.append(f"EXPECTED_FW_CM = {_py_float(fw['Cm'])}   # V (MOTOR_CM)")
    out.append("# Mixer kappa under its historical name (simulator/tests/test_control_allocation_")
    out.append("# firmware_compat.py imports it): the firmware kappa above.")
    out.append("KAPPA_ADOPTED = EXPECTED_FW_KAPPA")
    out.append("")
    out.append("# --- SILS plant propulsion (bench_2026_10_15, adopted) ---")
    out.append("# I_m = (V - Ke*w)/R_out(D); J dw/dt = Kt*I_m - Cq*w^2 - Kt*(I0a + I0b*w^2); T = eta*Ct*w^2")
    out.append(f"EXPECTED_PLANT_KE = {_py_float(pl['Ke'])}   # V*s/rad")
    out.append(f"EXPECTED_PLANT_KT = {_py_float(pl['Kt'])}   # N*m/A")
    out.append(f"EXPECTED_PLANT_CQ = {_py_float(pl['Cq'])}   # N*m/(rad/s)^2")
    out.append(f"EXPECTED_PLANT_CT = {_py_float(pl['Ct'])}   # N/(rad/s)^2")
    out.append(f"EXPECTED_PLANT_J = {_py_float(pl['J'])}   # kg*m^2")
    out.append(f"EXPECTED_PLANT_I0A = {_py_float(pl['I0a'])}   # A")
    out.append(f"EXPECTED_PLANT_I0B = {_py_float(pl['I0b'])}   # A/(rad/s)^2")
    out.append(f"EXPECTED_PLANT_C_C = {_py_float(pl['C_C'])}   # F (informational)")
    out.append(f"EXPECTED_PLANT_F_PWM = {_py_float(pl['f_pwm'])}   # Hz (informational)")
    out.append(f"EXPECTED_PLANT_THRUST_EFFICIENCY = {_py_float(pl['thrust_efficiency'])}")
    out.append("")
    out.append("# R_out(D) [ohm], 101 entries for D = 0.00 .. 1.00 (index = round(D*100)).")
    out.append("# Entry 0 is a placeholder (D < 0.01 means no drive); see generate.py.")
    out.append("PLANT_ROUT_D = (")
    out.extend(_py_tuple(rout["grid_d"]))
    out.append(")")
    out.append("PLANT_ROUT_OHM = (")
    out.extend(_py_tuple(rout["adopted_table"]))
    out.append(")")
    out.append("")
    out.append("# --- legacy_motor_curve (history; docs table only) ---")
    out.append(f"EXPECTED_LEGACY_AM = {_py_float(lg['Am'])}   # V/(rad/s)^2")
    out.append(f"EXPECTED_LEGACY_BM = {_py_float(lg['Bm'])}   # V/(rad/s)")
    out.append(f"EXPECTED_LEGACY_CM = {_py_float(lg['Cm'])}   # V")
    out.append("")
    out.append("# --- constants (calibration-independent) ---")
    out.append(f"EXPECTED_MASS = {_py_float(c['mass'])}")
    out.append(f"EXPECTED_IXX = {_py_float(c['Ixx'])}")
    out.append(f"EXPECTED_IYY = {_py_float(c['Iyy'])}")
    out.append(f"EXPECTED_IZZ = {_py_float(c['Izz'])}")
    out.append(f"EXPECTED_ARM = {_py_float(c['arm_offset'])}")
    out.append(f"EXPECTED_URDF_BASE_MASS = {_py_float(c['urdf_base_mass'])}")
    out.append("")
    out.append("# --- superseded measured_2026_07 comparison values (tools/params_audit) ---")
    out.append("# Used for the D*V-drive copies that have not migrated yet (Genesis, VPython,")
    out.append("# loop_shaping_tool, sysid defaults, docs hand tables).")
    out.append("EXPECTED_MEASURED_CT = _CT_VALUE")
    out.append("EXPECTED_MEASURED_CQ = _CQ_VALUE")
    out.append("EXPECTED_MEASURED_JMP = _JMP_VALUE")
    out.append("EXPECTED_MEASURED_DM = _DM_VALUE")
    out.append("EXPECTED_MEASURED_QF = _QF_VALUE")
    out.append("EXPECTED_MEASURED_RM = _RM_VALUE")
    out.append("EXPECTED_MEASURED_KM = _KM_VALUE")
    out.append(f"EXPECTED_MEASURED_KAPPA = {_py_float(m['kappa_adopted'])}   # kappa_adopted (docs table kappa)")
    out.append("")
    return "\n".join(out)


# =============================================================================
# Renderer 2: simulator/sils/plant/generated_params.hpp
# =============================================================================
def _cpp_array(values: List[float], per_line: int = 5) -> List[str]:
    """Render a float list as the body lines of a C++ initializer list.
    float リストを C++ 初期化子リストの本体行として整形する。"""
    lines = []
    for i in range(0, len(values), per_line):
        chunk = ", ".join(_cpp_float(v) for v in values[i:i + per_line])
        lines.append(f"    {chunk},")
    return lines


def render_cpp(view: Dict[str, Any]) -> str:
    c = view["constants"]
    lg = view["legacy_params"]
    pf = view["plant_fit_params"]
    pl = view["plant_params"]
    fw = view["firmware_params"]
    rout = view["rout"]

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
    out.append("// --- calibration_sets.bench_2026_10_15 (status: adopted, SILS plant propulsion) ---")
    out.append("// Averaged capacitor-recirculation drive model, consumed by")
    out.append("// simulator/sils/plant/plant.hpp / plant.cpp:")
    out.append("//   I_m = (V - KE*w) / R_out(D)           (I_m = 0 when D < 0.01)")
    out.append("//   J dw/dt = KT*I_m - CQ*w^2 - KT*(I0A + I0B*w^2)")
    out.append("//   T = THRUST_EFFICIENCY * CT * w^2")
    out.append("// 平均化したコンデンサ還流駆動モデル（plant.hpp / plant.cpp が使用）。式は上の英文参照。")
    out.append("// Scalars come first and the R_out arrays last, so the audit regexes that anchor")
    out.append("// on this namespace find each scalar before the first closing brace.")
    out.append("// スカラーを先、R_out 配列を最後に置く（この名前空間に固定した検査用正規表現が")
    out.append("// 最初の閉じ括弧より前でスカラーを見つけられるように）。")
    out.append("namespace plant {")
    out.append(f"constexpr float KE = {_cpp_float(pl['Ke'])};  ///< back-EMF constant [V*s/rad]")
    out.append(f"constexpr float KT = {_cpp_float(pl['Kt'])};  ///< torque constant [N*m/A]")
    out.append(f"constexpr float CQ = {_cpp_float(pl['Cq'])};  ///< torque coefficient [N*m/(rad/s)^2]")
    out.append(f"constexpr float CT = {_cpp_float(pl['Ct'])};  ///< thrust coefficient [N/(rad/s)^2]")
    out.append(f"constexpr float J = {_cpp_float(pl['J'])};  ///< rotor inertia incl. propeller [kg*m^2]")
    out.append(f"constexpr float I0A = {_cpp_float(pl['I0a'])};  ///< no-load current, constant part [A]")
    out.append(f"constexpr float I0B = {_cpp_float(pl['I0b'])};  ///< no-load current, w^2 part [A/(rad/s)^2]")
    out.append(f"constexpr float C_C = {_cpp_float(pl['C_C'])};  ///< recirculation capacitor [F] (informational)")
    out.append(f"constexpr float F_PWM = {_cpp_float(pl['f_pwm'])};  ///< PWM frequency [Hz] (informational)")
    out.append(
        f"constexpr float THRUST_EFFICIENCY = {_cpp_float(pl['thrust_efficiency'])};  "
        "///< Config::thrust_efficiency default (eta)"
    )
    out.append("")
    out.append("// R_out(D) [ohm], D = 0.00 .. 1.00 in 0.01 steps (ROUT_N entries, index = round(D*100)).")
    out.append("// ROUT_OHM is the ADOPTED table = calculated table x r(D), computed by the generator")
    out.append("// from the measured points (see the YAML). Entry 0 (D = 0.00) is a placeholder equal to")
    out.append("// the D = 0.01 entry and is never used: the plant treats D < 0.01 as 'no drive' (motor")
    out.append("// current 0) explicitly, before looking at the table.")
    out.append("// R_out(D)[Ω]。D=0.00〜1.00、0.01 刻み（ROUT_N 要素、添字=round(D×100)）。ROUT_OHM は")
    out.append("// 採用表（計算表×r(D)、生成器が実測点から計算）。要素 0（D=0.00）は D=0.01 と同値の")
    out.append("// 穴埋めで使われない: プラントは表を見る前に D<0.01 を「駆動なし」（モータ電流 0）と")
    out.append("// 明示的に扱う。")
    out.append(f"constexpr int ROUT_N = {len(rout['grid_d'])};")
    out.append("constexpr float ROUT_D[ROUT_N] = {")
    out.extend(_cpp_array(rout["grid_d"]))
    out.append("};")
    out.append("constexpr float ROUT_OHM[ROUT_N] = {")
    out.extend(_cpp_array(rout["adopted_table"]))
    out.append("};")
    out.append("}  // namespace plant")
    out.append("")
    out.append("// --- calibration_sets.flight_anchored_motor_curve (status: adopted, role: firmware) ---")
    out.append("// The firmware thrust->duty constants (actuator.cpp MOTOR_CT, KAPPA, MOTOR_AM/BM/CM).")
    out.append("// Not read by the plant; emitted so every copy of them is audited against one set.")
    out.append("// ファームの推力→duty 換算定数。プラントは読まない（コピーを1セットに対して検査するため生成）。")
    out.append("namespace firmware_curve {")
    out.append(f"constexpr float CT_GAUGE = {_cpp_float(fw['Ct_gauge'])};  ///< thrust gauge [N/(rad/s)^2]")
    out.append(f"constexpr float KAPPA = {_cpp_float(fw['kappa'])};  ///< torque/thrust ratio [m]")
    out.append(f"constexpr float AM = {_cpp_float(fw['Am'])};  ///< V = AM*w^2 + BM*w + CM [V/(rad/s)^2]")
    out.append(f"constexpr float BM = {_cpp_float(fw['Bm'])};  ///< [V/(rad/s)]")
    out.append(f"constexpr float CM = {_cpp_float(fw['Cm'])};  ///< [V]")
    out.append("}  // namespace firmware_curve")
    out.append("")
    out.append(
        "// --- calibration_sets.legacy_motor_curve (status: frozen_for_implementation, "
        "history) ---"
    )
    out.append("// Not consumed by any implementation. Kept for historical reference.")
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
    out.append("// --- calibration_sets.plant_fit_flip_2026_10 (status: SUPERSEDED 2026-10-30) ---")
    out.append("// Identified against the old V0 motor model; double-counts with the R_out drive.")
    out.append("// Emitted only so the opt-in profile SILS_EMU_PLANT_PROFILE=flip_fit_2026_10")
    out.append("// (emu_main.cpp) keeps building until Phase C replaces the *_fit scenarios.")
    out.append("// 旧 V0 モータモデルに対して同定した値で、R_out 駆動と二重に効く。オプトインの")
    out.append("// プロファイル flip_fit_2026_10 がビルドできるよう、Phase C で *_fit シナリオを")
    out.append("// 置き換えるまで生成する。")
    out.append("namespace plant_fit {")
    out.append(
        f"constexpr float TORQUE_AUTHORITY = {_cpp_float(pf['torque_authority'])};  "
        "///< Config::torque_authority (roll/pitch differential-thrust ratio)"
    )
    out.append(
        f"constexpr float MOTOR_DELAY_MS = {_cpp_float(pf['motor_delay_ms'])};  "
        "///< Config::motor_delay_ms (duty-path transport delay [ms])"
    )
    out.append(
        f"constexpr float MOTOR_SLEW_PER_S = {_cpp_float(pf['motor_slew_per_s'])};  "
        "///< Config::motor_slew_per_s (duty slew limit [duty/s])"
    )
    out.append("}  // namespace plant_fit")
    out.append("")
    out.append("}  // namespace sils_params")
    out.append("")
    return "\n".join(out)


# =============================================================================
# Renderer 3: docs/architecture/stampfly-parameters.md marker table
# =============================================================================
_DOC_ROWS_JP: Tuple[Tuple[str, str, str, str], ...] = (
    # (label, symbol, key into the kind's table, kind)
    # kind: const = constants / plant = bench_2026_10_15 / firmware = flight_anchored_motor_curve
    ("機体質量", "mass", "mass", "const"),
    ("Roll慣性モーメント", "Ixx", "Ixx", "const"),
    ("Pitch慣性モーメント", "Iyy", "Iyy", "const"),
    ("Yaw慣性モーメント", "Izz", "Izz", "const"),
    ("モーメントアーム", "arm_offset", "arm_offset", "const"),
    ("逆起電力定数（SILS プラント）", "Ke", "Ke", "plant"),
    ("トルク定数（SILS プラント）", "Kt", "Kt", "plant"),
    ("トルク係数（SILS プラント）", "Cq", "Cq", "plant"),
    ("推力係数（SILS プラント）", "Ct", "Ct", "plant"),
    ("ロータ慣性（SILS プラント）", "J", "J", "plant"),
    ("無負荷電流・定数部（SILS プラント）", "I0a", "I0a", "plant"),
    ("無負荷電流・ω² 部（SILS プラント）", "I0b", "I0b", "plant"),
    ("還流コンデンサ（SILS プラント）", "C_C", "C_C", "plant"),
    ("PWM 周波数（SILS プラント）", "f_pwm", "f_pwm", "plant"),
    ("推力効率（SILS プラント）", "η", "thrust_efficiency", "plant"),
    ("推力ゲージ係数（ファーム）", "Ct_gauge", "Ct_gauge", "firmware"),
    ("トルク/推力比（ファーム）", "κ", "kappa", "firmware"),
    ("2次係数（ファーム）", "Am", "Am", "firmware"),
    ("1次係数（ファーム）", "Bm", "Bm", "firmware"),
    ("定数項（ファーム）", "Cm", "Cm", "firmware"),
)

_DOC_ROWS_EN: Tuple[Tuple[str, str, str, str], ...] = (
    ("Vehicle mass", "mass", "mass", "const"),
    ("Roll moment of inertia", "Ixx", "Ixx", "const"),
    ("Pitch moment of inertia", "Iyy", "Iyy", "const"),
    ("Yaw moment of inertia", "Izz", "Izz", "const"),
    ("Moment arm", "arm_offset", "arm_offset", "const"),
    ("Back-EMF constant (SILS plant)", "Ke", "Ke", "plant"),
    ("Torque constant (SILS plant)", "Kt", "Kt", "plant"),
    ("Torque coefficient (SILS plant)", "Cq", "Cq", "plant"),
    ("Thrust coefficient (SILS plant)", "Ct", "Ct", "plant"),
    ("Rotor inertia (SILS plant)", "J", "J", "plant"),
    ("No-load current, constant part (SILS plant)", "I0a", "I0a", "plant"),
    ("No-load current, w^2 part (SILS plant)", "I0b", "I0b", "plant"),
    ("Recirculation capacitor (SILS plant)", "C_C", "C_C", "plant"),
    ("PWM frequency (SILS plant)", "f_pwm", "f_pwm", "plant"),
    ("Thrust efficiency (SILS plant)", "η", "thrust_efficiency", "plant"),
    ("Thrust gauge coefficient (firmware)", "Ct_gauge", "Ct_gauge", "firmware"),
    ("Torque/thrust ratio (firmware)", "κ", "kappa", "firmware"),
    ("Quadratic coefficient (firmware)", "Am", "Am", "firmware"),
    ("Linear coefficient (firmware)", "Bm", "Bm", "firmware"),
    ("Constant term (firmware)", "Cm", "Cm", "firmware"),
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
    tables = {
        "plant": (view["plant_params"], view["plant_entries"]),
        "firmware": (view["firmware_params"], view["firmware_entries"]),
    }

    if lang == "ja":
        intro = (
            f"> 以下は `{SSOT_REL}`（Single Source of Truth）から "
            "`sf params generate` が生成した確定値の一覧である。手編集しないこと。"
            "SILS プラントの推進系（`bench_2026_10_15`、2026-10-15 のベンチ実測。"
            "駆動回路の等価出力抵抗 R_out(D) の表は同ファイルにあり、プラントは D·V でなく"
            " R_out を介して電流を求める）と、ファームの推力→duty 換算の定数"
            "（`flight_anchored_motor_curve`）を別々に示す。"
            "旧セット `measured_2026_07` は SILS プラントについては置き換え済みで、"
            "独自の D·V 駆動モデルを持つ Genesis・VPython 等が移行までなお参照する"
            "（本節の下の手書き表）。"
        )
        headers = ("記号", "パラメータ", "値", "単位", "実測日", "方法")
    else:
        intro = (
            f"> The table below is generated from `{SSOT_REL}` "
            "(Single Source of Truth) by `sf params generate`. Do not hand-edit it. "
            "It lists the SILS plant propulsion constants (`bench_2026_10_15`, bench "
            "measurement of 2026-10-15; the drive circuit's equivalent output resistance "
            "R_out(D) is tabulated in the same file, and the plant derives the current "
            "from R_out rather than from D*V) and the firmware thrust-to-duty constants "
            "(`flight_anchored_motor_curve`) separately. The old set `measured_2026_07` "
            "is superseded for the SILS plant; Genesis, VPython and others, which keep "
            "their own D*V drive model, still use it until migrated (the hand tables "
            "in section 3 below)."
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
            params, entries = tables[kind]
            value = params[key]
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
