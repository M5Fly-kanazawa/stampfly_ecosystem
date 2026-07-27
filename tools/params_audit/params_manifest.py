"""
Physical parameter audit manifest — Single Source of Truth for expected values.
物理パラメータ検査マニフェスト — 期待値の正典。

This module declares WHERE each physical parameter is hand-copied across the
repository (file + regex) and WHAT value it should hold there. It does not
run anything itself; check_params.py reads this table and does the checking.
このモジュールは、各物理パラメータがリポジトリ内のどこに手動コピペされて
いるか（ファイル＋正規表現）と、そこにあるべき値を宣言するだけで、検査
自体は行わない。検査は check_params.py がこの表を読んで実行する。

Why a manifest instead of "the tool re-derives every value itself": the whole
point of this Phase-0 tool is to catch DIVERGENCE between copies that a human
(or an agent) pasted by hand. A manifest of (file, regex, expected) triples is
the simplest structure that can express "these N places should agree" without
requiring each source file to expose a machine-readable API. Phase 1 (spec
YAML → code generation, see docs/architecture/simulation-policy.md) removes
the need for this manifest entirely by generating the copies instead of
auditing them.
「ツール自身が値を再導出する」のではなくマニフェストを使う理由: この
Phase 0 ツールの目的は、人間（またはエージェント）が手作業で貼り付けた
コピー間の食い違いを検出することそのものにある。(ファイル, 正規表現, 期待値)
の3つ組の一覧は、各ソースファイルに機械可読 API を持たせずとも「この N 箇所は
一致すべき」を表現できる最も単純な構造である。Phase 1（spec YAML→コード生成、
docs/architecture/simulation-policy.md 参照）ではコピーを検査するのではなく
生成することで、本マニフェスト自体が不要になる。

IMPORTANT — regex authoring rule (2026-07-24, see the task that created this
file): each regex below was written by actually reading the target file's
CURRENT text, not guessed. Anchor on the constant/variable NAME, never on the
numeric value itself — otherwise the regex only matches the value it expects
and can never observe a mismatch.
重要 — 正規表現作成規則: 以下の各正規表現は対象ファイルの「現在の」テキストを
実際に読んで書いた（推測ではない）。定数名・変数名にアンカーし、値そのものに
アンカーしないこと — でなければ正規表現は期待値としか一致せず、食い違いを
決して観測できない。
"""

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Union

# Import the generated EXPECTED_* constants (Phase 1: spec YAML -> code
# generation, see docs/architecture/simulation-policy.md). `tools/` is added
# to sys.path defensively here (not just relied upon from the caller) so this
# module keeps working both when `sf params check` puts tools/ on sys.path
# AND when check_params.py is run directly as a script (python auto-adds only
# tools/params_audit/, its own directory, to sys.path in that mode -- see
# check_params.py's find_repo_root()/module docstring for the two contexts).
# 生成された EXPECTED_* 定数を import する（Phase 1: spec YAML→コード生成、
# docs/architecture/simulation-policy.md 参照）。`tools/` はここでも
# 防御的に sys.path へ追加する（呼び出し元任せにしない）— `sf params check`
# が tools/ を sys.path に載せる場合と、check_params.py を直接スクリプト
# 実行する場合（この場合 Python は自身のディレクトリ tools/params_audit/ の
# みを自動追加する — 2通りの実行文脈は check_params.py の
# find_repo_root()/モジュール docstring 参照）の両方で動き続けるようにする。
_TOOLS_DIR = str(Path(__file__).resolve().parent.parent)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from sysid._generated_params import (  # noqa: E402
    EXPECTED_CT,
    EXPECTED_CQ,
    EXPECTED_KAPPA,
    EXPECTED_JMP,
    EXPECTED_DM,
    EXPECTED_QF,
    EXPECTED_IXX,
    EXPECTED_IYY,
    EXPECTED_IZZ,
    EXPECTED_ARM,
    EXPECTED_RM,
    EXPECTED_KM,
    EXPECTED_MASS,
    EXPECTED_URDF_BASE_MASS,
)

# HTML display-scale variants of EXPECTED_* used by control/design/
# loop_shaping_tool/index.html's Calculation tab, whose <input> fields show
# Ct/Cq pre-divided by a fixed power-of-ten (labelled "×10⁻⁸" etc. next to
# the field) rather than the raw SI value. Never hand-adjust these except to
# match a scale-label change in index.html itself.
# control/design/loop_shaping_tool/index.html の Calculation タブが使う
# EXPECTED_* の表示スケール版。Ct/Cq の <input> はSI値そのものではなく、
# 固定の10のべき乗（フィールド脇に「×10⁻⁸」等と表示）で割った値を表示する。
# index.html 自体の表示スケール表記が変わらない限り手で調整しないこと。
EXPECTED_CT_SCALED_1EM8 = EXPECTED_CT / 1e-8  # calc-ct field ("×10⁻⁸ N/(rad/s)²")


# =============================================================================
# Judgement markers — used in place of a numeric "expected" value.
# 判定マーカー — 数値の「期待値」の代わりに使う。
# =============================================================================
@dataclass(frozen=True)
class Unresolved:
    """No single correct value has been decided yet; multiple candidates
    coexist in the repository. The checker only COLLECTS the current value
    here and reports it — it never compares it to anything.
    正解値がまだ決まっていない（複数の候補値が並立中）ことを表す。検査器は
    現在値を収集して報告するだけで、比較は一切行わない。"""
    candidates: str = ""  # human-readable summary of the competing values / 並立候補の要約


@dataclass(frozen=True)
class Exempt:
    """The value at this location is KNOWN to differ from the confirmed
    physical value, and that is intentional (e.g. a coupled 3-value set
    pending a joint recalibration). Always reported as EXEMPT, never MISMATCH.
    この箇所の値が確定物理値と異なることが分かっており、それは意図的
    である（例: 連動する3点セットで再較正待ち）ことを表す。常に MISMATCH
    ではなく EXEMPT として報告する。"""
    reason: str  # why the mismatch is accepted, with a source citation / 許容理由（出所引用付き）


UNRESOLVED = Unresolved  # re-exported for convenience / 利便のため再エクスポート


@dataclass(frozen=True)
class ParamCheck:
    """One (file, regex, expected) triple. / 1件の (ファイル, 正規表現, 期待値) 3つ組。

    Attributes:
        file: Path relative to the repository root. / リポジトリルート相対パス。
        regex: Must contain EXACTLY ONE capturing group around the numeric
            token (plain float/scientific notation, or the markdown doc's
            unicode-superscript "1.00×10⁻⁸" form — see check_params.parse_value).
            捕捉グループを正確に1つだけ含み、数値トークン（通常の浮動小数点/
            指数表記、または Markdown 文書の unicode 上付き指数表記
            "1.00×10⁻⁸"）を囲むこと。詳細は check_params.parse_value 参照。
        expected: A float (numeric check), Unresolved (collect only), or
            Exempt (known-and-accepted mismatch).
            float（数値検査）、Unresolved（収集のみ）、Exempt（既知で許容
            された不一致）のいずれか。
        note: Short label disambiguating multiple occurrences of the same
            parameter within one file (e.g. "DEFAULT_PARAMS" vs "module
            constant"). Shown in the report's location column.
            同一ファイル内に同じパラメータが複数箇所ある場合の識別ラベル
            （例: "DEFAULT_PARAMS" と "module constant"）。レポートの場所欄に表示。
    """
    file: str
    regex: str
    expected: Union[float, Unresolved, Exempt]
    note: str = ""


# =============================================================================
# Confirmed measured values — the SSOT for every "expected" float below.
#
# Phase 1 (spec YAML -> code generation, see docs/architecture/
# simulation-policy.md): these EXPECTED_* values are no longer hand-typed
# literals in this file. They are imported (see top of file) from
# tools/sysid/_generated_params.py, which `sf params generate` machine-
# generates from the single YAML source of truth
# control/models/stampfly_physical.yaml. To change one, edit that YAML, run
# `sf params generate`, then commit both. Do not hand-edit a numeric literal
# into a ParamCheck row below, and do not redefine EXPECTED_* here.
#
# 確定済み実測値 — 以下の全"期待値"の正典。
#
# Phase 1（spec YAML→コード生成、docs/architecture/simulation-policy.md
# 参照）: これらの EXPECTED_* は、もうこのファイルへ手書きされたリテラルでは
# ない。（ファイル冒頭で）唯一の正である YAML
# （control/models/stampfly_physical.yaml）から `sf params generate` が
# 機械生成する tools/sysid/_generated_params.py から import する。値を
# 変更する場合はその YAML を編集し `sf params generate` を実行、両方を
# コミットすること。以下の ParamCheck 行に数値リテラルを直接書き込んだり、
# ここで EXPECTED_* を再定義したりしないこと。
# =============================================================================

# --- EXEMPT markers (known-and-accepted mismatch, with reason) ---
# --- EXEMPT マーカー（既知で許容された不一致、理由付き） ---
# Phase 1 (2026-07-26, backlog #2 motor ODE): simulator/sil/plant/plant.hpp's
# Config::Ct switched to the adopted (measured) value -- EXEMPT_PLANT_CT (which
# used to cover it) is retired. The ONLY remaining known-and-accepted mismatch
# is firmware's still-legacy MOTOR_CT (backlog #3, firmware out of scope here).
# Phase 1（2026-07-26、バックログ#2モータODE化）: simulator/sil/plant/plant.hpp
# の Config::Ct は adopted（実測）値へ切替済み -- それをカバーしていた
# EXEMPT_PLANT_CT は撤廃。唯一残る既知許容の不一致は firmware の旧 MOTOR_CT
# （バックログ#3、本タスクでは firmware は対象外）。
EXEMPT_FIRMWARE_MOTOR_CT = Exempt(
    reason="firmware/vehicle/components/sf_actuator/actuator.cpp の MOTOR_CT は"
           "実機ファーム現状の鏡写し（backlog #2 では firmware/ を変更しない）。"
           "simulator/sil/plant/plant.hpp の Config::Ct は 2026-07-26 に adopted 値へ"
           "切替済み（backlog #2, motor ODE 化）。ファーム側の切替は "
           "docs/architecture/simulation-policy.md backlog #3 で別途実施する。"
)

# firmware/vehicle_old is FROZEN legacy (87 real flights, no new development,
# see firmware/vehicle_old/README.md banner). Its physical-parameter literals
# are intentionally left at their old-generation values -- this manifest exists
# to document that fact (and catch accidental edits), not to demand they be
# updated to the adopted family.
# firmware/vehicle_old は凍結されたレガシー（実飛行87回、新規開発なし、
# firmware/vehicle_old/README.md 冒頭バナー参照）。物理パラメータのリテラルは
# 意図的に旧世代の値のまま据え置く -- 本マニフェストはその事実を記録し
# （誤編集を検知するため）登録するのであり、採用ファミリへの更新を求める
# ものではない。
EXEMPT_VEHICLE_OLD = Exempt(
    reason="firmware/vehicle_old は凍結されたレガシーファーム（実飛行87回、"
           "新規開発なし）。現行ファームは firmware/vehicle/。現在の確定値は "
           "docs/architecture/stampfly-parameters.md / "
           "control/models/stampfly_physical.yaml を参照。"
)


# =============================================================================
# The manifest / マニフェスト本体
# =============================================================================
MANIFEST: Dict[str, List[ParamCheck]] = {
    # -------------------------------------------------------------------
    # C_T — thrust coefficient / 推力係数
    # -------------------------------------------------------------------
    "C_T": [
        ParamCheck(
            # Phase 1: _CT_VALUE moved from tools/sysid/defaults.py (hand-typed
            # literal) into the generated tools/sysid/_generated_params.py.
            # Phase 1: _CT_VALUE は手書きリテラルだった tools/sysid/defaults.py
            # から、生成物 tools/sysid/_generated_params.py へ移動した。
            file="tools/sysid/_generated_params.py",
            regex=r'_CT_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CT,
            note="module constant _CT_VALUE (generated)",
        ),
        ParamCheck(
            file="lib/stampfly_edu/sim/plants.py",
            regex=r'"Ct":\s*([0-9eE.+-]+),',
            expected=EXPECTED_CT,
            note="ImportError fallback dict",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Ct:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CT,
            note="MotorParams.Ct",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Ct\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CT,
            note="motor_prop.Ct",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'推力係数\s*\|\s*Ct\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_CT,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Thrust coefficient\s*\|\s*Ct\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_CT,
            note="body table (EN)",
        ),
        ParamCheck(
            # Phase 1 (2026-07-26, backlog #2): plant.hpp's Config::Ct now reads
            # sil_params::adopted::CT (generated_params.hpp) -- a symbolic
            # reference, not a numeric literal, so there is nothing left to
            # regex-match in plant.hpp itself (same pattern as kappa/RM/mass
            # below). The literal moved to generated_params.hpp.
            # Phase 1（2026-07-26、バックログ#2）: plant.hpp の Config::Ct は
            # sil_params::adopted::CT（generated_params.hpp）を参照する記号参照に
            # なり、plant.hpp 自体には正規表現で捕捉すべき数値リテラルが無くなった
            # （下の kappa/RM/mass と同じパターン）。リテラルは generated_params.hpp
            # へ移動した。
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float CT = ([0-9eE.+-]+)f;',
            expected=EXPECTED_CT,
            note="sil_params::adopted::CT (Config::Ct source, backlog #2 ODE)",
        ),
        ParamCheck(
            # Firmware is out of scope for backlog #2 (motor ODE, SIL-only); the
            # firmware mixer's thrust curve still uses the legacy Ct pending
            # backlog #3's separate firmware-side switch.
            # firmware はバックログ#2（モータODE化、SIL限定）の対象外——ファーム
            # ミキサーの推力曲線は backlog #3 の別途ファーム側切替待ちで旧 Ct のまま。
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_CT\s*=\s*([0-9eE.+-]+)f;',
            expected=EXEMPT_FIRMWARE_MOTOR_CT,
            note="mixerCompute() thrust curve MOTOR_CT (legacy, backlog #3 pending)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'def thrust_to_duty\(thrust: float, Ct: float = ([0-9eE.+-]+)',
            expected=EXPECTED_CT,
            note="thrust_to_duty() default Ct",
        ),
        ParamCheck(
            file="control/design/loop_shaping_tool/index.html",
            regex=r'id="calc-ct" value="([0-9eE.+-]+)"',
            expected=EXPECTED_CT_SCALED_1EM8,
            note="Calculation tab calc-ct input (×10⁻⁸ display scale)",
        ),
        ParamCheck(
            # firmware_old is EXEMPT (frozen legacy), not MISMATCH -- see
            # EXEMPT_VEHICLE_OLD above and firmware/vehicle_old/README.md banner.
            # firmware_old はEXEMPT（凍結レガシー）——MISMATCHではない。上記
            # EXEMPT_VEHICLE_OLD と firmware/vehicle_old/README.md 冒頭バナー参照。
            file="firmware/vehicle_old/components/sf_algo_control/motor_model.cpp",
            regex=r'\.Ct = ([0-9eE.+-]+)f,',
            expected=EXEMPT_VEHICLE_OLD,
            note="DEFAULT_MOTOR_PARAMS.Ct (frozen legacy firmware)",
        ),
    ],

    # -------------------------------------------------------------------
    # C_Q — torque coefficient / トルク係数
    # -------------------------------------------------------------------
    "C_Q": [
        ParamCheck(
            # Phase 1: moved to the generated tools/sysid/_generated_params.py
            # (Phase 1: 生成物 tools/sysid/_generated_params.py へ移動)
            file="tools/sysid/_generated_params.py",
            regex=r'_CQ_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CQ,
            note="module constant _CQ_VALUE (generated)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Cq:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CQ,
            note="MotorParams.Cq",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Cq\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CQ,
            note="motor_prop.Cq",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'トルク係数\s*\|\s*Cq\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_CQ,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Torque coefficient\s*\|\s*Cq\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_CQ,
            note="body table (EN)",
        ),
        ParamCheck(
            # Phase 1 (2026-07-26, backlog #2): plant.hpp now HAS a standalone Cq
            # (Config::Cq, sourced from sil_params::adopted::CQ) -- the reaction
            # torque model switched from kappa*T to a direct per-motor
            # Cq*omega^2 + Jmp*omega_dot in substep() (supersedes the note that
            # used to live here).
            # Phase 1（2026-07-26、バックログ#2）: plant.hpp は独立した Cq
            # （Config::Cq、sil_params::adopted::CQ 由来）を持つようになった——
            # 反トルクモデルが kappa*T からモータ毎の直接 Cq*omega^2+Jmp*omega_dot
            # （substep()）へ切替（旧注記を置換）。
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float CQ = ([0-9eE.+-]+)f;',
            expected=EXPECTED_CQ,
            note="sil_params::adopted::CQ (Config::Cq source, backlog #2 ODE)",
        ),
    ],

    # -------------------------------------------------------------------
    # kappa — torque/thrust ratio Cq/Ct / トルク推力比
    # -------------------------------------------------------------------
    "kappa": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'KAPPA\s*=\s*([0-9eE.+-]+)f;',
            expected=EXPECTED_KAPPA,
            note="mixerCompute() B^-1 KAPPA",
        ),
        ParamCheck(
            # Phase 1: plant.hpp's Config::kappa now reads
            # sil_params::adopted::KAPPA (simulator/sil/plant/generated_params.hpp)
            # instead of a hand-written literal -- the literal itself moved there.
            # Phase 1: plant.hpp の Config::kappa は手書きリテラルではなく
            # sil_params::adopted::KAPPA
            # （simulator/sil/plant/generated_params.hpp）を参照するようになった
            # -- リテラル自体がそちらへ移動した。
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float KAPPA = ([0-9eE.+-]+)f;',
            expected=EXPECTED_KAPPA,
            note="sil_params::adopted::KAPPA (independent of the deferred Ct set)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'kappa:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_KAPPA,
            note="QuadConfig.kappa",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'トルク/推力比\s*\|\s*κ\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_KAPPA,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Torque/Thrust ratio\s*\|\s*κ\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_KAPPA,
            note="body table (EN)",
        ),
        # NOTE: tools/sysid/defaults.py's _KAPPA_VALUE is now a DERIVED
        # expression (_KAPPA_VALUE = _CQ_VALUE / _CT_VALUE), not a numeric
        # literal, so there is nothing to regex-match — it is structurally
        # guaranteed correct once the C_T/C_Q entries above are OK. Checking
        # C_T and C_Q there already covers this location transitively.
        # defaults.py の _KAPPA_VALUE は数値リテラルではなく導出式
        # (_KAPPA_VALUE = _CQ_VALUE / _CT_VALUE) になったため、正規表現で
        # 値を捕捉する対象が無い — 上の C_T/C_Q エントリが OK であれば
        # 構造的に正しいことが保証される（この箇所は間接的にカバー済み）。
        ParamCheck(
            # firmware_old is EXEMPT (frozen legacy) -- see EXEMPT_VEHICLE_OLD
            # above and firmware/vehicle_old/README.md banner.
            # firmware_old はEXEMPT（凍結レガシー）——上記 EXEMPT_VEHICLE_OLD と
            # firmware/vehicle_old/README.md 冒頭バナー参照。
            file="firmware/vehicle_old/components/sf_algo_control/include/control_allocation.hpp",
            regex=r'float kappa = ([0-9eE.+-]+)f;',
            expected=EXEMPT_VEHICLE_OLD,
            note="QuadConfig::kappa member default (frozen legacy firmware)",
        ),
    ],

    # -------------------------------------------------------------------
    # J_mp — rotor inertia / ローター慣性モーメント
    # -------------------------------------------------------------------
    "J_mp": [
        ParamCheck(
            # Phase 1: moved to the generated tools/sysid/_generated_params.py
            # (Phase 1: 生成物 tools/sysid/_generated_params.py へ移動)
            file="tools/sysid/_generated_params.py",
            regex=r'_JMP_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_JMP,
            note="module constant _JMP_VALUE (generated)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Jmp:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_JMP,
            note="MotorParams.Jmp",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Jmp\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_JMP,
            note="motor_prop.Jmp",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'回転子慣性モーメント\s*\|\s*Jmp\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_JMP,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Rotor Inertia\s*\|\s*Jmp\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_JMP,
            note="body table (EN)",
        ),
        ParamCheck(
            # Phase 1 (2026-07-26, backlog #2): plant.hpp's Config::Jmp (ODE rotor
            # inertia) now sources this directly.
            # Phase 1（2026-07-26、バックログ#2）: plant.hpp の Config::Jmp
            # （ODEローター慣性）は本値を直接参照する。
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float JMP = ([0-9eE.+-]+)f;',
            expected=EXPECTED_JMP,
            note="sil_params::adopted::JMP (Config::Jmp source, backlog #2 ODE)",
        ),
    ],

    # -------------------------------------------------------------------
    # D_m — viscous damping (2026-07-26 coast-down 3-term refit, backlog #2)
    # 粘性摩擦係数（2026-07-26 コーストダウン3項再フィット、バックログ#2）
    # -------------------------------------------------------------------
    "D_m": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_DM_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_DM,
            note="module constant _DM_VALUE (generated)",
        ),
        ParamCheck(
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float DM = ([0-9eE.+-]+)f;',
            expected=EXPECTED_DM,
            note="sil_params::adopted::DM (Config::Dm source, backlog #2 ODE)",
        ),
    ],

    # -------------------------------------------------------------------
    # Q_f — Coulomb friction torque (2026-07-26 coast-down 3-term refit, backlog #2)
    # 静止摩擦トルク（2026-07-26 コーストダウン3項再フィット、バックログ#2）
    # -------------------------------------------------------------------
    "Q_f": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_QF_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_QF,
            note="module constant _QF_VALUE (generated)",
        ),
        ParamCheck(
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float QF = ([0-9eE.+-]+)f;',
            expected=EXPECTED_QF,
            note="sil_params::adopted::QF (Config::Qf source, backlog #2 ODE)",
        ),
    ],

    # -------------------------------------------------------------------
    # Ixx / Iyy / Izz — body moments of inertia / 機体慣性モーメント
    # (already-consistent reference case — this group should read all OK)
    # （全実装一致済みの正常参照ケース — 全て OK が期待される）
    # -------------------------------------------------------------------
    "Ixx": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'"Ixx":\s*\{\s*"value":\s*([0-9eE.+-]+),',
            expected=EXPECTED_IXX,
            note="DEFAULT_PARAMS.inertia.Ixx",
        ),
        ParamCheck(
            file="simulator/sil/models/stampfly.xml",
            regex=r'diaginertia="([0-9.eE+-]+) ',
            expected=EXPECTED_IXX,
            note="<inertial diaginertia> [0]",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_sim.py",
            regex=r'inersia=\[\[([0-9.eE+-]+),\s*0\.0,\s*0\.0\]',
            expected=EXPECTED_IXX,
            note="multicopter(inersia=...) [0][0]",
        ),
        ParamCheck(
            file="tools/log_analyzer/rate_sysid.py",
            regex=r'SPEC_INERTIA = \{"roll":\s*([0-9eE.+-]+),',
            expected=EXPECTED_IXX,
            note="SPEC_INERTIA['roll']",
        ),
    ],
    "Iyy": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'"Iyy":\s*\{\s*"value":\s*([0-9eE.+-]+),',
            expected=EXPECTED_IYY,
            note="DEFAULT_PARAMS.inertia.Iyy",
        ),
        ParamCheck(
            file="simulator/sil/models/stampfly.xml",
            regex=r'diaginertia="[0-9.eE+-]+ ([0-9.eE+-]+) ',
            expected=EXPECTED_IYY,
            note="<inertial diaginertia> [1]",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_sim.py",
            regex=r'inersia=\[\[[0-9.eE+-]+,\s*0\.0,\s*0\.0\],\s*\[0\.0,\s*([0-9.eE+-]+),\s*0\.0\]',
            expected=EXPECTED_IYY,
            note="multicopter(inersia=...) [1][1]",
        ),
        ParamCheck(
            file="tools/log_analyzer/rate_sysid.py",
            regex=r'"pitch":\s*([0-9eE.+-]+),\s*"yaw"',
            expected=EXPECTED_IYY,
            note="SPEC_INERTIA['pitch']",
        ),
    ],
    "Izz": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'"Izz":\s*\{\s*"value":\s*([0-9eE.+-]+),',
            expected=EXPECTED_IZZ,
            note="DEFAULT_PARAMS.inertia.Izz",
        ),
        ParamCheck(
            file="simulator/sil/models/stampfly.xml",
            regex=r'diaginertia="[0-9.eE+-]+ [0-9.eE+-]+ ([0-9.eE+-]+)"',
            expected=EXPECTED_IZZ,
            note="<inertial diaginertia> [2]",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_sim.py",
            regex=r'\[0\.0,\s*0\.0,\s*([0-9.eE+-]+)\]\]',
            expected=EXPECTED_IZZ,
            note="multicopter(inersia=...) [2][2]",
        ),
        ParamCheck(
            file="tools/log_analyzer/rate_sysid.py",
            regex=r'"yaw":\s*([0-9eE.+-]+)\}',
            expected=EXPECTED_IZZ,
            note="SPEC_INERTIA['yaw']",
        ),
    ],

    # -------------------------------------------------------------------
    # arm — moment arm (X/Y offset, center to motor) / モーメントアーム
    # -------------------------------------------------------------------
    "arm": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'"arm_length":\s*\{\s*"value":\s*([0-9.eE+-]+),',
            expected=EXPECTED_ARM,
            note="DEFAULT_PARAMS.geometry.arm_length",
        ),
        ParamCheck(
            file="simulator/sil/models/stampfly.xml",
            regex=r'name="rotor1" pos="\s*([0-9.]+)',
            expected=EXPECTED_ARM,
            note="<site name=rotor1> X offset",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_sim.py",
            regex=r'self\.d\s*=\s*([0-9.]+)\s*#',
            expected=EXPECTED_ARM,
            note="ControlAllocator.d",
        ),
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'ARM_D\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_ARM,
            note="mixerCompute() B^-1 ARM_D",
        ),
    ],

    # -------------------------------------------------------------------
    # Rm — winding resistance (RESOLVED 2026-07-24: 0.593, see EXPECTED_RM)
    # 巻線抵抗（2026-07-24決着: 0.593、EXPECTED_RM 参照）
    # -------------------------------------------------------------------
    "Rm": [
        ParamCheck(
            # Phase 1: moved to the generated tools/sysid/_generated_params.py
            # (Phase 1: 生成物 tools/sysid/_generated_params.py へ移動)
            file="tools/sysid/_generated_params.py",
            regex=r'_RM_VALUE\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_RM,
            note="module constant _RM_VALUE (generated)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Rm:\s*float\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_RM,
            note="MotorParams.Rm",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Rm\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_RM,
            note="motor_prop.Rm",
        ),
        ParamCheck(
            # Phase 1: plant.hpp's Config::motor_Rm now reads
            # sil_params::adopted::RM (generated_params.hpp) -- literal moved there.
            # Phase 1: plant.hpp の Config::motor_Rm は sil_params::adopted::RM
            # （generated_params.hpp）を参照する -- リテラルはそちらへ移動した。
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float RM = ([0-9.eE+-]+)f;',
            expected=EXPECTED_RM,
            note="sil_params::adopted::RM (battery model source, Config::motor_Rm)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'巻線抵抗\s*\|\s*Rm\s*\|\s*([0-9.eE+-]+)\s*\|',
            expected=EXPECTED_RM,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Winding Resistance\s*\|\s*Rm\s*\|\s*([0-9.eE+-]+)\s*\|',
            expected=EXPECTED_RM,
            note="body table (EN)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'Km: float = [0-9eE.+-]+,\s*Rm: float = ([0-9eE.+-]+)',
            expected=EXPECTED_RM,
            note="thrust_to_duty() default Rm",
        ),
        ParamCheck(
            file="control/design/loop_shaping_tool/index.html",
            regex=r'id="calc-rm" value="([0-9eE.+-]+)"',
            expected=EXPECTED_RM,
            note="Calculation tab calc-rm input (unscaled, Ω)",
        ),
    ],

    # -------------------------------------------------------------------
    # Km — back-EMF constant / 逆起電力定数
    # -------------------------------------------------------------------
    "Km": [
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'def thrust_to_duty\([^)]*Km: float = ([0-9eE.+-]+)',
            expected=EXPECTED_KM,
            note="thrust_to_duty() default Km",
        ),
    ],

    # -------------------------------------------------------------------
    # mass — vehicle mass (RESOLVED 2026-07-24: 0.037, see EXPECTED_MASS)
    # 機体質量（2026-07-24決着: 0.037、EXPECTED_MASS 参照）
    # -------------------------------------------------------------------
    "mass": [
        ParamCheck(
            # Phase 1: plant.hpp's Config::mass now reads sil_params::MASS
            # (generated_params.hpp) -- the literal moved there.
            # Phase 1: plant.hpp の Config::mass は sil_params::MASS
            # （generated_params.hpp）を参照する -- リテラルはそちらへ移動した。
            file="simulator/sil/plant/generated_params.hpp",
            regex=r'constexpr float MASS = ([0-9.eE+-]+)f;',
            expected=EXPECTED_MASS,
            note="sil_params::MASS (Config::mass source)",
        ),
        ParamCheck(
            file="simulator/sil/models/stampfly.xml",
            regex=r'<inertial[^>]*mass="([0-9.eE+-]+)"',
            expected=EXPECTED_MASS,
            note="<inertial mass>",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_sim.py",
            regex=r'mass = ([0-9.eE+-]+)\n\s*weight = mass',
            expected=EXPECTED_MASS,
            note="Drone Setup mass",
        ),
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'"mass":\s*\{\s*"value":\s*([0-9.eE+-]+),',
            expected=EXPECTED_MASS,
            note="DEFAULT_PARAMS.mass",
        ),
        ParamCheck(
            file="simulator/shared/assets/meshes/parts/stampfly_fixed.urdf",
            regex=r'<mass value="([0-9.eE+-]+)"/>\s*<!--',
            expected=EXPECTED_MASS,
            note="single-link total mass (fixed-prop variant)",
        ),
        ParamCheck(
            file="simulator/shared/assets/meshes/parts/stampfly.urdf",
            regex=r'<mass value="([0-9.eE+-]+)"/>\s*\n\s*<inertia ixx="9\.16e-6"',
            expected=EXPECTED_URDF_BASE_MASS,
            note="base_link mass only (base 0.033 + 4x1g props = 0.037 total)",
        ),
        ParamCheck(
            file="simulator/genesis/scripts/run_genesis_sim.py",
            regex=r'MASS = ([0-9.eE+-]+)\s*#\s*kg \(37g\)',
            expected=EXPECTED_MASS,
            note="Physical Constants MASS",
        ),
        ParamCheck(
            file="simulator/genesis/scripts/run_genesis_headless.py",
            regex=r'MASS = ([0-9.eE+-]+)\nWEIGHT = MASS \* GRAVITY',
            expected=EXPECTED_MASS,
            note="Physical Constants MASS",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_vpython_headless.py",
            regex=r'mass = ([0-9.eE+-]+)\n\s*weight = mass \* 9\.81',
            expected=EXPECTED_MASS,
            note="Create drone mass",
        ),
        ParamCheck(
            file="control/design/loop_shaping_tool/index.html",
            regex=r'id="calc-mass" value="([0-9.eE+-]+)"',
            expected=EXPECTED_MASS,
            note="Calculation tab calc-mass input (unscaled, kg)",
        ),
    ],
}
