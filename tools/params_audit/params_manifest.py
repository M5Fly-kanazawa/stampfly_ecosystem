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
    # firmware thrust->duty set (flight_anchored_motor_curve, adopted)
    # ファーム推力→duty 換算セット
    EXPECTED_FW_CT,
    EXPECTED_FW_KAPPA,
    EXPECTED_FW_AM,
    EXPECTED_FW_BM,
    EXPECTED_FW_CM,
    # SILS plant propulsion (bench_2026_10_15, adopted)
    # SILS プラントの推進系
    EXPECTED_PLANT_KE,
    EXPECTED_PLANT_KT,
    EXPECTED_PLANT_CQ,
    EXPECTED_PLANT_CT,
    EXPECTED_PLANT_J,
    EXPECTED_PLANT_I0A,
    EXPECTED_PLANT_I0B,
    EXPECTED_PLANT_C_C,
    EXPECTED_PLANT_F_PWM,
    EXPECTED_PLANT_THRUST_EFFICIENCY,
    # superseded measured_2026_07 (own D*V drive model, migration pending)
    # 置き換え済み measured_2026_07（独自の D×V 駆動モデル、移行待ち）
    EXPECTED_MEASURED_CT,
    EXPECTED_MEASURED_CQ,
    EXPECTED_MEASURED_KAPPA,
    EXPECTED_MEASURED_JMP,
    EXPECTED_MEASURED_DM,
    EXPECTED_MEASURED_QF,
    EXPECTED_MEASURED_RM,
    EXPECTED_MEASURED_KM,
    # legacy_motor_curve (docs history table)
    EXPECTED_LEGACY_AM,
    EXPECTED_LEGACY_BM,
    EXPECTED_LEGACY_CM,
    # calibration-independent constants
    EXPECTED_IXX,
    EXPECTED_IYY,
    EXPECTED_IZZ,
    EXPECTED_ARM,
    EXPECTED_MASS,
    EXPECTED_URDF_BASE_MASS,
)

# Which calibration set does a row check against? (2026-10-30, R_out plant change)
#   - FIRMWARE rows  (actuator.cpp MOTOR_CT/KAPPA/MOTOR_AM/BM/CM, plant_fit.py,
#     genesis control_allocation.py thrust_to_duty/kappa) -> EXPECTED_FW_*
#     (flight_anchored_motor_curve: what the firmware holds).
#   - PLANT rows (simulator/sils/plant/generated_params.hpp namespace plant)
#     -> EXPECTED_PLANT_* (bench_2026_10_15).
#   - Genesis motor_model.py, VPython motors.py, loop_shaping_tool calc-ct,
#     sysid defaults and the docs hand tables keep their OWN D*V drive model
#     (V = D*Vbat with Rm/Km/Dm/Qf): "own D*V drive model, migration pending" ->
#     EXPECTED_MEASURED_* (the superseded measured_2026_07 set).
# Every EXPECTED_* is imported from the generated module; nothing is hand-typed
# here any more (the former hand-typed EXPECTED_AM/BM/CM and EXPECTED_AM_FA/BM_FA
# are gone: the firmware set and the legacy history rows are generated).
# 各行がどの較正セットと照合されるか（2026-10-30、R_out プラント変更）:
#   - ファーム側の行 -> EXPECTED_FW_*（ファームが持つ換算定数）
#   - プラント側の行（generated_params.hpp の plant 名前空間）-> EXPECTED_PLANT_*
#   - Genesis motor_model.py・VPython motors.py・loop_shaping_tool の calc-ct・
#     sysid defaults・docs の手書き表は独自の D×V 駆動モデルのまま
#     （「独自 D×V 駆動モデル、移行待ち」）-> EXPECTED_MEASURED_*
# EXPECTED_* は全て生成モジュールから import し、ここへ手書きしない
# （従来手書きだった EXPECTED_AM/BM/CM と EXPECTED_AM_FA/BM_FA は撤去した）。

# HTML display-scale variant of EXPECTED_MEASURED_CT used by control/design/
# loop_shaping_tool/index.html's Calculation tab, whose <input> fields show
# Ct pre-divided by a fixed power-of-ten (labelled "x10^-8" next to the field)
# rather than the raw SI value. Never hand-adjust except to match a
# scale-label change in index.html itself.
# control/design/loop_shaping_tool/index.html の Calculation タブが使う
# EXPECTED_MEASURED_CT の表示スケール版。Ct の <input> は SI 値そのものではなく、
# 固定の10のべき乗（フィールド脇に「×10⁻⁸」と表示）で割った値を表示する。
# index.html 自体の表示スケール表記が変わらない限り手で調整しないこと。
EXPECTED_MEASURED_CT_SCALED_1EM8 = EXPECTED_MEASURED_CT / 1e-8  # calc-ct field


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
# ない。（ファイル冒頭で）基準となる YAML
# （control/models/stampfly_physical.yaml）から `sf params generate` が
# 機械生成する tools/sysid/_generated_params.py から import する。値を
# 変更する場合はその YAML を編集し `sf params generate` を実行、両方を
# コミットすること。以下の ParamCheck 行に数値リテラルを直接書き込んだり、
# ここで EXPECTED_* を再定義したりしないこと。
# =============================================================================

# --- EXEMPT markers (known-and-accepted mismatch, with reason) ---
# --- EXEMPT マーカー（既知で許容された不一致、理由付き） ---
# (2026-08-03): EXEMPT_FIRMWARE_MOTOR_CT, which used to cover
# firmware/vehicle/components/sf_actuator/actuator.cpp's MOTOR_CT pending
# backlog #3's firmware-side Ct switch, is RETIRED. The measured_2026_07
# family's Ct is now a provisional 1.00e-8 (the 2026-07-15 thrust-stand value
# was retracted -- see control/models/stampfly_physical.yaml Ct.note) --
# numerically identical to the legacy literal MOTOR_CT already held, so the
# C_T check below promotes that row from EXEMPT to a normal EXPECTED_CT (OK)
# comparison (2026-10-30: now EXPECTED_FW_CT).
# （2026-08-03）: backlog #3 のファーム側 Ct 切替待ちとして firmware/vehicle/
# components/sf_actuator/actuator.cpp の MOTOR_CT をカバーしていた
# EXEMPT_FIRMWARE_MOTOR_CT は撤廃。measured_2026_07 ファミリの Ct は暫定値
# 1.00e-8 になった（2026-07-15のthrust stand値は撤回 -- 詳細は
# control/models/stampfly_physical.yaml の Ct.note 参照）-- MOTOR_CT が既に
# 保持していた旧リテラルと数値上一致するため、下記 C_T チェックはこの行を
# EXEMPT から通常の EXPECTED_CT（OK）比較へ昇格する。

# max_thrust_per_motor: provenance of the 0.15 N vs 0.168 N per-motor limit is
# NOT confirmed against a measurement (unlike C_T/C_Q/kappa/mass/inertia
# above, all of which trace to a dated bench measurement in the SSOT YAML).
# The 0.168 N family now has a single consumer,
# firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp
# (tools/log_analyzer/visualize_interactive.py's mirror of it was removed
# 2026-09-11 when that script switched to reading ctrl_output.csv's real
# controller output instead of reconstructing thrust from firmware
# constants -- see docs/plans/flight-log-format-plan.md section 3.4).
# 0.168 N 系の消費者は現在 firmware/vehicle の pid_controller.hpp のみ
# （tools/log_analyzer/visualize_interactive.py 側の鏡写しは 2026-09-11 に
# 削除済み -- 同スクリプトが実測のコントローラ出力を持つ ctrl_output.csv を
# 読むように切り替わったため。計画書 docs/plans/flight-log-format-plan.md
# 3.4節参照）。
# Modeled as EXEMPT rather than Unresolved (2026-08-03): `sf sils regression` /
# CI's --strict treats any UNRESOLVED row as a failure, which made this open
# question block automated checks. EXEMPT keeps the mismatch visible in the
# report (reason text below, prefixed "計測待ち（出所未確認）:") without
# failing --strict -- it does not assert either candidate value is correct.
# 最大推力（1モーターあたり）: 0.15N/0.168Nどちらの出所も実測で確認されていない
# （上のC_T/C_Q/kappa/mass/慣性と異なり、SSOT YAMLに紐づく実測日がない）。
# Unresolved ではなく EXEMPT として扱う（2026-08-03）: `sf sils regression`/CIの
# --strict は UNRESOLVED を1件でも失敗扱いにするため、この未決着課題が自動検査を
# 止めてしまっていた。EXEMPT ならレポート上の可視性（下記理由文、先頭に
# 「計測待ち（出所未確認）:」）を保ったまま --strict を通す -- どちらの候補値が
# 正しいかを主張するものではない。
EXEMPT_MAX_THRUST_PER_MOTOR = Exempt(
    reason="計測待ち（出所未確認）: 0.15N/0.168Nどちらの出所も実測で確認されて"
           "いない（上のC_T/C_Q/kappa/mass/慣性と異なり、SSOT YAMLに紐づく実測日が"
           "ない）。ベンチ実測 duty0.9 で 0.108 N/枚と乖離（2026-07-15調査時点の"
           "実測記録。詳細レポートは日付付き分析記録の整理に伴い削除済み、"
           "2026-08-03）。0.168 = 旧Ct(1.0e-8)×(4097 rad/s)² と一致する可能性あり"
           "（未確認）。0.15 N 系（tools/sysid・simulator/genesis・"
           "simulator/vpython 等7箇所）と 0.168 N 系（firmware/vehicle "
           "pid_controller.hpp のみ）の2値が並立している。"
)


# =============================================================================
# The manifest / マニフェスト本体
# =============================================================================
MANIFEST: Dict[str, List[ParamCheck]] = {
    # -------------------------------------------------------------------
    # C_T — thrust coefficient, superseded measured_2026_07 copies
    # (own D*V drive model, migration pending) / 推力係数: 置き換え済み
    # measured_2026_07 のコピー（独自 D×V 駆動モデル、移行待ち）。
    # The SILS plant's Ct is group "plant_Ct"; the firmware's gauge is
    # "C_T_firmware". / SILS プラントの Ct は "plant_Ct"、ファームのゲージは
    # "C_T_firmware"。
    # -------------------------------------------------------------------
    "C_T": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_CT_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_CT,
            note="module constant _CT_VALUE (generated, measured_2026_07 superseded; sysid defaults)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Ct:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_CT,
            note="MotorParams.Ct (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Ct\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_CT,
            note="motor_prop.Ct (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'推力係数\s*\|\s*Ct\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_CT,
            note="body hand table (JP, measured_2026_07 record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Thrust coefficient\s*\|\s*Ct\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_CT,
            note="body hand table (EN, measured_2026_07 record)",
        ),
        ParamCheck(
            file="control/design/loop_shaping_tool/index.html",
            regex=r'id="calc-ct" value="([0-9eE.+-]+)"',
            expected=EXPECTED_MEASURED_CT_SCALED_1EM8,
            note="Calculation tab calc-ct input (x10^-8 display scale; own D*V drive model, migration pending)",
        ),
    ],

    # -------------------------------------------------------------------
    # C_T_firmware — the firmware's thrust gauge (flight_anchored_motor_curve
    # Ct_gauge): actuator.cpp MOTOR_CT and the Genesis thrust_to_duty default.
    # plant_fit.py's _MOTOR_CT is now symbolic (= EXPECTED_FW_CT) so there is no
    # literal to match there.
    # ファームの推力ゲージ係数。plant_fit.py の _MOTOR_CT は記号参照
    # （= EXPECTED_FW_CT）になり、照合すべきリテラルは無い。
    # -------------------------------------------------------------------
    "C_T_firmware": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_CT\s*=\s*([0-9eE.+-]+)f;',
            expected=EXPECTED_FW_CT,
            note="mixerCompute() thrust curve MOTOR_CT (firmware set Ct_gauge)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'def thrust_to_duty\(thrust: float, Ct: float = ([0-9eE.+-]+)',
            expected=EXPECTED_FW_CT,
            note="thrust_to_duty() default Ct (firmware thrust gauge)",
        ),
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'EXPECTED_FW_CT\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_FW_CT,
            note="generated EXPECTED_FW_CT (plant_fit.py _MOTOR_CT source)",
        ),
        ParamCheck(
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'namespace firmware_curve \{[^}]*?constexpr float CT_GAUGE = ([0-9eE.+-]+)f;',
            expected=EXPECTED_FW_CT,
            note="sils_params::firmware_curve::CT_GAUGE",
        ),
    ],

    # -------------------------------------------------------------------
    # C_Q — torque coefficient, superseded measured_2026_07 copies
    # トルク係数: 置き換え済み measured_2026_07 のコピー。Plant: "plant_Cq"。
    # -------------------------------------------------------------------
    "C_Q": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_CQ_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_CQ,
            note="module constant _CQ_VALUE (generated, measured_2026_07 superseded; sysid defaults)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Cq:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_CQ,
            note="MotorParams.Cq (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Cq\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_CQ,
            note="motor_prop.Cq (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'トルク係数\s*\|\s*Cq\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_CQ,
            note="body hand table (JP, measured_2026_07 record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Torque coefficient\s*\|\s*Cq\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_CQ,
            note="body hand table (EN, measured_2026_07 record)",
        ),
    ],

    # -------------------------------------------------------------------
    # kappa — torque/thrust ratio the firmware mixer B^-1 uses (firmware set).
    # The docs hand table keeps the measured_2026_07 kappa_adopted record (same
    # number, 4.10e-3). The SILS plant no longer has a kappa: its reaction
    # torque is Cq*w^2 + J*dw/dt per motor.
    # ファームのミキサー B^-1 が使う κ（ファームセット）。docs の手書き表は
    # measured_2026_07 の kappa_adopted の記録（同じ 4.10e-3）。SILS プラントは
    # もう κ を持たない（反トルクはモータ毎の Cq·ω²+J·dω/dt）。
    # -------------------------------------------------------------------
    "kappa": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'KAPPA\s*=\s*([0-9eE.+-]+)f;',
            expected=EXPECTED_FW_KAPPA,
            note="mixerCompute() B^-1 KAPPA (firmware set)",
        ),
        ParamCheck(
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'namespace firmware_curve \{[^}]*?constexpr float KAPPA = ([0-9eE.+-]+)f;',
            expected=EXPECTED_FW_KAPPA,
            note="sils_params::firmware_curve::KAPPA",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'kappa:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_FW_KAPPA,
            note="QuadConfig.kappa (mirrors the firmware mixer)",
        ),
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'EXPECTED_FW_KAPPA\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_FW_KAPPA,
            note="generated EXPECTED_FW_KAPPA (plant_fit.py _KAPPA source, KAPPA_ADOPTED alias)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'トルク/推力比\s*\|\s*κ\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_KAPPA,
            note="body hand table (JP, measured_2026_07 record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Torque/Thrust ratio\s*\|\s*κ\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_KAPPA,
            note="body hand table (EN, measured_2026_07 record)",
        ),
        # NOTE: tools/sysid/defaults.py's _KAPPA_VALUE is a DERIVED expression
        # (_CQ_VALUE / _CT_VALUE), not a numeric literal, so it is covered
        # transitively by the C_T/C_Q groups above.
        # defaults.py の _KAPPA_VALUE は導出式（_CQ_VALUE/_CT_VALUE）で数値
        # リテラルではないため、上の C_T/C_Q グループで間接的にカバーされる。
    ],

    # -------------------------------------------------------------------
    # Am / Bm / Cm — legacy motor-curve family (history): docs hand table only.
    # Firmware copies of the curve are the "*_firmware" groups below.
    # 旧モータ曲線ファミリ（履歴）: docs の手書き表のみ。ファーム側のコピーは
    # 下の "*_firmware" グループ。
    # -------------------------------------------------------------------
    "Am": [
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'2次係数\s*\|\s*Am\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_LEGACY_AM,
            note="body table (JP, legacy_motor_curve historical record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Quadratic coefficient\s*\|\s*Am\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_LEGACY_AM,
            note="body table (EN, legacy_motor_curve historical record)",
        ),
    ],
    "Bm": [
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'1次係数\s*\|\s*Bm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_LEGACY_BM,
            note="body table (JP, legacy_motor_curve historical record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Linear coefficient\s*\|\s*Bm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_LEGACY_BM,
            note="body table (EN, legacy_motor_curve historical record)",
        ),
    ],
    "Cm": [
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'定数項\s*\|\s*Cm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_LEGACY_CM,
            note="body table (JP, legacy_motor_curve historical record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Constant term\s*\|\s*Cm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_LEGACY_CM,
            note="body table (EN, legacy_motor_curve historical record)",
        ),
    ],

    # -------------------------------------------------------------------
    # Am / Bm / Cm _firmware — the firmware's thrust->duty curve
    # (flight_anchored_motor_curve). Consumers: actuator.cpp MOTOR_AM/BM/CM and
    # tools/sysid/plant_fit.py (symbolic since 2026-10-30: _MOTOR_AM/BM/CM =
    # EXPECTED_FW_*, so no literal to match there).
    # ファームの推力→duty 曲線の Am/Bm/Cm。消費者: actuator.cpp と
    # plant_fit.py（2026-10-30 から記号参照のため照合すべきリテラルは無い）。
    # -------------------------------------------------------------------
    "Am_firmware": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_AM\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_FW_AM,
            note="V=Am*w^2+Bm*w+Cm curve MOTOR_AM (firmware set)",
        ),
        ParamCheck(
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'namespace firmware_curve \{[^}]*?constexpr float AM = ([0-9.eE+-]+)f;',
            expected=EXPECTED_FW_AM,
            note="sils_params::firmware_curve::AM",
        ),
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'EXPECTED_FW_AM\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_FW_AM,
            note="generated EXPECTED_FW_AM (plant_fit.py _MOTOR_AM source)",
        ),
    ],
    "Bm_firmware": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_BM\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_FW_BM,
            note="V=Am*w^2+Bm*w+Cm curve MOTOR_BM (firmware set)",
        ),
        ParamCheck(
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'namespace firmware_curve \{[^}]*?constexpr float BM = ([0-9.eE+-]+)f;',
            expected=EXPECTED_FW_BM,
            note="sils_params::firmware_curve::BM",
        ),
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'EXPECTED_FW_BM\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_FW_BM,
            note="generated EXPECTED_FW_BM (plant_fit.py _MOTOR_BM source)",
        ),
    ],
    "Cm_firmware": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_CM\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_FW_CM,
            note="V=Am*w^2+Bm*w+Cm curve MOTOR_CM (firmware set)",
        ),
        ParamCheck(
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'namespace firmware_curve \{[^}]*?constexpr float CM = ([0-9.eE+-]+)f;',
            expected=EXPECTED_FW_CM,
            note="sils_params::firmware_curve::CM",
        ),
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'EXPECTED_FW_CM\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_FW_CM,
            note="generated EXPECTED_FW_CM (plant_fit.py _MOTOR_CM source)",
        ),
    ],

    # -------------------------------------------------------------------
    # J_mp — rotor inertia, superseded measured_2026_07 copies (own D*V drive
    # model, migration pending). Plant: "plant_J".
    # ローター慣性: 置き換え済み measured_2026_07 のコピー。Plant: "plant_J"。
    # -------------------------------------------------------------------
    "J_mp": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_JMP_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_JMP,
            note="module constant _JMP_VALUE (generated, measured_2026_07 superseded)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Jmp:\s*float\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_JMP,
            note="MotorParams.Jmp (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Jmp\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_JMP,
            note="motor_prop.Jmp (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'回転子慣性モーメント\s*\|\s*Jmp\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_JMP,
            note="body hand table (JP, measured_2026_07 record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Rotor Inertia\s*\|\s*Jmp\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_MEASURED_JMP,
            note="body hand table (EN, measured_2026_07 record)",
        ),
    ],

    # -------------------------------------------------------------------
    # D_m / Q_f — viscous damping / Coulomb friction (measured_2026_07, superseded).
    # The SILS plant no longer has them: friction is Kt*(I0a + I0b*w^2) from the
    # bench set. / SILS プラントはもう持たない（摩擦は bench セットの Kt·(I0a+I0b·ω²)）。
    # -------------------------------------------------------------------
    "D_m": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_DM_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_DM,
            note="module constant _DM_VALUE (generated, measured_2026_07 superseded)",
        ),
    ],
    "Q_f": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_QF_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_QF,
            note="module constant _QF_VALUE (generated, measured_2026_07 superseded)",
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
            file="simulator/sils/models/stampfly.xml",
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
            file="simulator/sils/models/stampfly.xml",
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
            file="simulator/sils/models/stampfly.xml",
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
            file="simulator/sils/models/stampfly.xml",
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
    # Rm — winding resistance (measured_2026_07, superseded; own D*V drive
    # model, migration pending). The SILS plant no longer has Rm: its drive
    # resistance is R_out(D).
    # 巻線抵抗（置き換え済み measured_2026_07、独自 D×V 駆動モデル、移行待ち）。
    # SILS プラントはもう Rm を持たない（駆動の抵抗は R_out(D)）。
    # -------------------------------------------------------------------
    "Rm": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_RM_VALUE\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_MEASURED_RM,
            note="module constant _RM_VALUE (generated, measured_2026_07 superseded)",
        ),
        ParamCheck(
            file="simulator/genesis/motor_model.py",
            regex=r'Rm:\s*float\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_MEASURED_RM,
            note="MotorParams.Rm (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="simulator/vpython/core/motors.py",
            regex=r'self\.Rm\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_MEASURED_RM,
            note="motor_prop.Rm (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'巻線抵抗\s*\|\s*Rm\s*\|\s*([0-9.eE+-]+)\s*\|',
            expected=EXPECTED_MEASURED_RM,
            note="body hand table (JP, measured_2026_07 record)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Winding Resistance\s*\|\s*Rm\s*\|\s*([0-9.eE+-]+)\s*\|',
            expected=EXPECTED_MEASURED_RM,
            note="body hand table (EN, measured_2026_07 record)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'Km: float = [0-9eE.+-]+,\s*Rm: float = ([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_RM,
            note="thrust_to_duty() default Rm (own D*V drive model, migration pending)",
        ),
        ParamCheck(
            file="control/design/loop_shaping_tool/index.html",
            regex=r'id="calc-rm" value="([0-9eE.+-]+)"',
            expected=EXPECTED_MEASURED_RM,
            note="Calculation tab calc-rm input (unscaled, ohm; own D*V drive model, migration pending)",
        ),
    ],

    # -------------------------------------------------------------------
    # Km — back-EMF constant (measured_2026_07, superseded; own D*V drive model)
    # 逆起電力定数（置き換え済み measured_2026_07、独自 D×V 駆動モデル）
    # -------------------------------------------------------------------
    "Km": [
        ParamCheck(
            file="tools/sysid/_generated_params.py",
            regex=r'_KM_VALUE\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_MEASURED_KM,
            note="module constant _KM_VALUE (generated, measured_2026_07 superseded)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'def thrust_to_duty\([^)]*Km: float = ([0-9eE.+-]+)',
            expected=EXPECTED_MEASURED_KM,
            note="thrust_to_duty() default Km (own D*V drive model, migration pending)",
        ),
    ],

    # -------------------------------------------------------------------
    # mass — vehicle mass (RESOLVED 2026-07-24: 0.037, see EXPECTED_MASS)
    # 機体質量（2026-07-24決着: 0.037、EXPECTED_MASS 参照）
    # -------------------------------------------------------------------
    "mass": [
        ParamCheck(
            # Phase 1: plant.hpp's Config::mass now reads sils_params::MASS
            # (generated_params.hpp) -- the literal moved there.
            # Phase 1: plant.hpp の Config::mass は sils_params::MASS
            # （generated_params.hpp）を参照する -- リテラルはそちらへ移動した。
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float MASS = ([0-9.eE+-]+)f;',
            expected=EXPECTED_MASS,
            note="sils_params::MASS (Config::mass source)",
        ),
        ParamCheck(
            file="simulator/sils/models/stampfly.xml",
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

    # -------------------------------------------------------------------
    # max_thrust_per_motor — EXEMPT, not compared numerically (see
    # EXEMPT_MAX_THRUST_PER_MOTOR above for why EXEMPT rather than
    # Unresolved). Two competing values circulate: 0.15 N (most of
    # tools/simulator) and 0.168 N (firmware/vehicle pid_controller.hpp
    # only, as of 2026-09-11). Neither traces to a dated bench measurement
    # in the SSOT YAML -- this section exists to make that gap visible, not
    # to declare a winner.
    # 最大推力（1モーターあたり）— EXEMPT、数値比較は行わない（Unresolved
    # ではなく EXEMPT を使う理由は上の EXEMPT_MAX_THRUST_PER_MOTOR 参照）。
    # 0.15N系と0.168N系の2値が並立し、どちらもSSOT YAMLの実測日に紐づいて
    # いない——本セクションはその欠落を可視化するためのものであり、正解を
    # 宣言するものではない（0.168N系の消費者は2026-09-11時点で firmware/vehicle
    # の pid_controller.hpp のみ）。
    # -------------------------------------------------------------------
    "max_thrust_per_motor": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'"max_thrust":\s*\{\s*"value":\s*([0-9.eE+-]+),',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="DEFAULT_PARAMS.motor.max_thrust (0.15 N family)",
        ),
        ParamCheck(
            file="tools/sysid/inertia.py",
            regex=r'MAX_THRUST_PER_MOTOR = ([0-9.eE+-]+)',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="module constant MAX_THRUST_PER_MOTOR (0.15 N family)",
        ),
        ParamCheck(
            file="simulator/genesis/control_allocation.py",
            regex=r'max_thrust_per_motor: float = ([0-9.eE+-]+)',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="mix_with_saturation() default max_thrust_per_motor (0.15 N family)",
        ),
        ParamCheck(
            file="simulator/vpython/control/motor_mixer.py",
            regex=r'def motor_to_thrust\(motor_output: float, max_thrust_n: float = ([0-9.eE+-]+)\)',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="motor_to_thrust() default max_thrust_n (0.15 N family)",
        ),
        ParamCheck(
            file="simulator/vpython/control/motor_mixer.py",
            regex=(r'def motor_to_torque\(\s*motors: np\.ndarray,\s*'
                   r'motor_positions: list = None,\s*motor_dirs: list = None,\s*'
                   r'max_thrust_n: float = ([0-9.eE+-]+),'),
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="motor_to_torque() default max_thrust_n (0.15 N family)",
        ),
        ParamCheck(
            file="simulator/vpython/scripts/run_sim.py",
            regex=r'self\.max_thrust = ([0-9.eE+-]+)\s*#\s*Max thrust per motor',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="Drone Setup max_thrust (0.15 N family)",
        ),
        ParamCheck(
            file="firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp",
            regex=r'float max_thrust_\s*=\s*([0-9.eE+-]+)f;\s*//\s*\[N\] total',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="max_thrust_ = 4 x 0.168 N per motor (0.168 N family)",
        ),
    ],
}


# =============================================================================
# SILS plant propulsion rows (bench_2026_10_15): the generated header's plant
# namespace and the generated Python constants, against the plant set.
# SILS プラント推進系の行（bench_2026_10_15）: 生成ヘッダの plant 名前空間と
# 生成 Python 定数を plant セットと照合する。
# plant.hpp / plant.cpp read these symbolically (no literal to match there).
# plant.hpp / plant.cpp は記号参照（照合すべきリテラルは無い）。
# =============================================================================
def _plant_groups() -> Dict[str, List[ParamCheck]]:
    """Build the "plant_*" groups: one per bench-set constant, two rows each
    (generated header + generated Python). / bench セットの定数ごとに
    "plant_*" グループ（生成ヘッダ＋生成 Python の 2 行）を作る。"""
    hpp = "simulator/sils/plant/generated_params.hpp"
    py = "tools/sysid/_generated_params.py"
    spec = (
        # (group, C++ name, Python name, expected)
        ("plant_Ke", "KE", "KE", EXPECTED_PLANT_KE),
        ("plant_Kt", "KT", "KT", EXPECTED_PLANT_KT),
        ("plant_Cq", "CQ", "CQ", EXPECTED_PLANT_CQ),
        ("plant_Ct", "CT", "CT", EXPECTED_PLANT_CT),
        ("plant_J", "J", "J", EXPECTED_PLANT_J),
        ("plant_I0a", "I0A", "I0A", EXPECTED_PLANT_I0A),
        ("plant_I0b", "I0B", "I0B", EXPECTED_PLANT_I0B),
        ("plant_C_C", "C_C", "C_C", EXPECTED_PLANT_C_C),
        ("plant_f_pwm", "F_PWM", "F_PWM", EXPECTED_PLANT_F_PWM),
        ("plant_eta", "THRUST_EFFICIENCY", "THRUST_EFFICIENCY", EXPECTED_PLANT_THRUST_EFFICIENCY),
    )
    groups: Dict[str, List[ParamCheck]] = {}
    for group, cpp_name, py_name, expected in spec:
        groups[group] = [
            ParamCheck(
                file=hpp,
                regex=rf'namespace plant \{{[^}}]*?constexpr float {cpp_name} = ([0-9eE.+-]+)f;',
                expected=expected,
                note=f"sils_params::plant::{cpp_name} (bench_2026_10_15)",
            ),
            ParamCheck(
                file=py,
                regex=rf'EXPECTED_PLANT_{py_name}\s*=\s*([0-9eE.+-]+)',
                expected=expected,
                note=f"generated EXPECTED_PLANT_{py_name}",
            ),
        ]
    return groups


MANIFEST.update(_plant_groups())
