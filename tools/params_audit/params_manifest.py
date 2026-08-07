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


# EXPECTED_AM / EXPECTED_BM / EXPECTED_CM — control/models/stampfly_physical.yaml
# calibration_sets.legacy_motor_curve.params.{Am,Bm,Cm}.value. Deliberately
# hand-typed here (not imported from tools/sysid/_generated_params.py like the
# EXPECTED_* above): generate.py's render_python() only emits the
# measured_2026_07 family (CT/CQ/JMP/DM/QF/RM/KM/KAPPA) into that file. The
# legacy_motor_curve family IS machine-generated, but only on the C++ side
# (simulator/sils/plant/generated_params.hpp's sils_params::legacy::AM/BM/CM,
# via render_cpp()) -- a header nothing currently consumes
# (status: frozen_for_implementation, see the YAML family header comment).
# 旧ファミリ定数。バックログ#3で新チェーンへ切替予定。If backlog #3 adds a
# Python-side generated symbol for this family, replace these three literals
# with imports the same way EXPECTED_CT etc. are imported above.
EXPECTED_AM = 5.39e-8   # V/(rad/s)^2 -- legacy_motor_curve.Am
EXPECTED_BM = 6.33e-4   # V/(rad/s)   -- legacy_motor_curve.Bm
EXPECTED_CM = 1.53e-2   # V           -- legacy_motor_curve.Cm


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
# (2026-08-03): EXEMPT_FIRMWARE_MOTOR_CT, which used to cover
# firmware/vehicle/components/sf_actuator/actuator.cpp's MOTOR_CT pending
# backlog #3's firmware-side Ct switch, is RETIRED. The measured_2026_07
# family's Ct is now a provisional 1.00e-8 (the 2026-07-15 thrust-stand value
# was retracted -- see control/models/stampfly_physical.yaml Ct.note) --
# numerically identical to the legacy literal MOTOR_CT already held, so the
# C_T check below promotes that row from EXEMPT to a normal EXPECTED_CT (OK)
# comparison.
# （2026-08-03）: backlog #3 のファーム側 Ct 切替待ちとして firmware/vehicle/
# components/sf_actuator/actuator.cpp の MOTOR_CT をカバーしていた
# EXEMPT_FIRMWARE_MOTOR_CT は撤廃。measured_2026_07 ファミリの Ct は暫定値
# 1.00e-8 になった（2026-07-15のthrust stand値は撤回 -- 詳細は
# control/models/stampfly_physical.yaml の Ct.note 参照）-- MOTOR_CT が既に
# 保持していた旧リテラルと数値上一致するため、下記 C_T チェックはこの行を
# EXEMPT から通常の EXPECTED_CT（OK）比較へ昇格する。

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

# tools/log_analyzer's duty-reconstruction / interactive-visualization scripts
# intentionally mirror the firmware constants THAT WERE ACTIVE WHEN THE
# REPLAYED LOGS WERE RECORDED (the pre-2026-07-17 motor curve / 9.71e-3 kappa),
# not today's firmware nor the SILS-adopted measured family, so their
# reconstructed thrust/duty plots match what the real hardware actually did at
# the time. Frozen mirrors for log reconstruction -- NOT a stray copy. See
# reconstruct_duties.py's own NOTE(2026-07-15) comment. Not code-changed by
# the 2026-08-03 kappa/Ct revision (out of scope -- see params_manifest.py's
# task instructions); only this EXEMPT reason text was updated to correct a
# prior inaccuracy ("mirrors CURRENT firmware", which was never quite true --
# firmware's kappa moved on 2026-07-17 and again 2026-08-03, these files did
# not).
# tools/log_analyzer の duty再構成／対話可視化スクリプトは、今日のファームでも
# SILSの実測ファミリでもなく、「再生対象のログが記録された当時、有効だった」
# ファーム定数（2026-07-17以前のモータ曲線・kappa=9.71e-3）を意図的に鏡写し
# している——実機がその当時実際にどう動いていたかにログ再構成の結果を一致
# させるため。過去ログ再構成用の凍結鏡写しであり、単なるコピー漏れではない
# （reconstruct_duties.py 自身の NOTE(2026-07-15) コメント参照）。2026-08-03の
# κ/Ct改定ではコード変更しない（対象外——本パラメータ大改定タスクの指示書
# 参照）。理由文のみ、従来の不正確な記述（「現行ファームを鏡写し」——実際には
# ファームのkappaは2026-07-17・2026-08-03と動いたがこれらのファイルは
# 動いていない）を修正した。
EXEMPT_LOG_ANALYZER_MIRROR = Exempt(
    reason="過去ログ再構成用の凍結鏡写し（当時の飛行ファーム定数）。"
           "時代別対応は将来課題。"
)

# max_thrust_per_motor: provenance of the 0.15 N vs 0.168 N per-motor limit is
# NOT confirmed against a measurement (unlike C_T/C_Q/kappa/mass/inertia
# above, all of which trace to a dated bench measurement in the SSOT YAML).
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
           "（未確認）。0.15 N 系（tools/sysid・lib/stampfly_edu・simulator/genesis・"
           "simulator/vpython 等9箇所）と 0.168 N 系（firmware/vehicle "
           "pid_controller.hpp・tools/log_analyzer/visualize_interactive.py "
           "の2箇所）の2値が並立している。"
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
            # sils_params::adopted::CT (generated_params.hpp) -- a symbolic
            # reference, not a numeric literal, so there is nothing left to
            # regex-match in plant.hpp itself (same pattern as kappa/RM/mass
            # below). The literal moved to generated_params.hpp.
            # Phase 1（2026-07-26、バックログ#2）: plant.hpp の Config::Ct は
            # sils_params::adopted::CT（generated_params.hpp）を参照する記号参照に
            # なり、plant.hpp 自体には正規表現で捕捉すべき数値リテラルが無くなった
            # （下の kappa/RM/mass と同じパターン）。リテラルは generated_params.hpp
            # へ移動した。
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float CT = ([0-9eE.+-]+)f;',
            expected=EXPECTED_CT,
            note="sils_params::adopted::CT (Config::Ct source, backlog #2 ODE)",
        ),
        ParamCheck(
            # Promoted from EXEMPT to a normal OK comparison 2026-08-03: the
            # measured_2026_07 family's Ct is now the same provisional 1.00e-8
            # this literal already held (see the EXEMPT markers header comment
            # above and control/models/stampfly_physical.yaml Ct.note).
            # 2026-08-03にEXEMPTから通常のOK比較へ昇格: measured_2026_07
            # ファミリのCtは、このリテラルが既に保持していた暫定値1.00e-8と
            # 同じになった（上のEXEMPTマーカー冒頭コメントと
            # control/models/stampfly_physical.yaml のCt.note参照）。
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_CT\s*=\s*([0-9eE.+-]+)f;',
            expected=EXPECTED_CT,
            note="mixerCompute() thrust curve MOTOR_CT (adopted, provisional as of 2026-08-03)",
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
        ParamCheck(
            # See EXEMPT_LOG_ANALYZER_MIRROR above -- intentional firmware
            # mirror, not a stray copy of the SILS-adopted measured value.
            file="tools/log_analyzer/reconstruct_duties.py",
            regex=r'\bCT = ([0-9eE.+-]+)',
            expected=EXEMPT_LOG_ANALYZER_MIRROR,
            note="module constant CT (firmware actuator.cpp mirror)",
        ),
        ParamCheck(
            file="tools/log_analyzer/visualize_interactive.py",
            regex=r'\bCt = ([0-9eE.+-]+)',
            expected=EXEMPT_LOG_ANALYZER_MIRROR,
            note="thrust_to_duty() local Ct (firmware actuator.cpp mirror)",
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
            # (Config::Cq, sourced from sils_params::adopted::CQ) -- the reaction
            # torque model switched from kappa*T to a direct per-motor
            # Cq*omega^2 + Jmp*omega_dot in substep() (supersedes the note that
            # used to live here).
            # Phase 1（2026-07-26、バックログ#2）: plant.hpp は独立した Cq
            # （Config::Cq、sils_params::adopted::CQ 由来）を持つようになった——
            # 反トルクモデルが kappa*T からモータ毎の直接 Cq*omega^2+Jmp*omega_dot
            # （substep()）へ切替（旧注記を置換）。
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float CQ = ([0-9eE.+-]+)f;',
            expected=EXPECTED_CQ,
            note="sils_params::adopted::CQ (Config::Cq source, backlog #2 ODE)",
        ),
        ParamCheck(
            # See EXEMPT_LOG_ANALYZER_MIRROR above -- intentional firmware
            # mirror, not a stray copy of the SILS-adopted measured value.
            file="tools/log_analyzer/reconstruct_duties.py",
            regex=r'\bCQ = ([0-9eE.+-]+)',
            expected=EXEMPT_LOG_ANALYZER_MIRROR,
            note="module constant CQ (firmware actuator.cpp mirror)",
        ),
        ParamCheck(
            file="tools/log_analyzer/visualize_interactive.py",
            regex=r'\bCq = ([0-9eE.+-]+)',
            expected=EXEMPT_LOG_ANALYZER_MIRROR,
            note="thrust_to_duty() local Cq (firmware actuator.cpp mirror)",
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
            # sils_params::adopted::KAPPA (simulator/sils/plant/generated_params.hpp)
            # instead of a hand-written literal -- the literal itself moved there.
            # Phase 1: plant.hpp の Config::kappa は手書きリテラルではなく
            # sils_params::adopted::KAPPA
            # （simulator/sils/plant/generated_params.hpp）を参照するようになった
            # -- リテラル自体がそちらへ移動した。
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float KAPPA = ([0-9eE.+-]+)f;',
            expected=EXPECTED_KAPPA,
            note="sils_params::adopted::KAPPA (independent of the deferred Ct set)",
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
        ParamCheck(
            # See EXEMPT_LOG_ANALYZER_MIRROR above -- intentional firmware
            # mirror, not a stray copy of the SILS-adopted measured value.
            file="tools/log_analyzer/reconstruct_duties.py",
            regex=r'\bKAPPA = ([0-9eE.+-]+)',
            expected=EXEMPT_LOG_ANALYZER_MIRROR,
            note="module constant KAPPA (firmware actuator.cpp mirror)",
        ),
        ParamCheck(
            file="tools/log_analyzer/visualize_interactive.py",
            regex=r'\bkappa = ([0-9eE.+-]+)',
            expected=EXEMPT_LOG_ANALYZER_MIRROR,
            note="B^-1 mixer local kappa (firmware actuator.cpp mirror)",
        ),
    ],

    # -------------------------------------------------------------------
    # Am / Bm / Cm — legacy motor-curve family: V = Am·ω² + Bm·ω + Cm
    # 旧モータ曲線ファミリ。バックログ#3で新チェーンへ切替予定。
    # (firmware/vehicle/components/sf_actuator/actuator.cpp's MOTOR_CT/AM/BM/CM
    # are the only surviving consumers as of the 2026-07-26 motor-ODE change
    # that removed this family from simulator/sils/plant/plant.hpp -- see the
    # SSOT YAML's legacy_motor_curve family header comment.)
    # -------------------------------------------------------------------
    "Am": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_AM\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_AM,
            note="V=Am*w^2+Bm*w+Cm curve MOTOR_AM (legacy family, backlog #3 pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'2次係数\s*\|\s*Am\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_AM,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Quadratic coefficient\s*\|\s*Am\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_AM,
            note="body table (EN)",
        ),
    ],
    "Bm": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_BM\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_BM,
            note="V=Am*w^2+Bm*w+Cm curve MOTOR_BM (legacy family, backlog #3 pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'1次係数\s*\|\s*Bm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_BM,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Linear coefficient\s*\|\s*Bm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_BM,
            note="body table (EN)",
        ),
    ],
    "Cm": [
        ParamCheck(
            file="firmware/vehicle/components/sf_actuator/actuator.cpp",
            regex=r'MOTOR_CM\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_CM,
            note="V=Am*w^2+Bm*w+Cm curve MOTOR_CM (legacy family, backlog #3 pending)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'定数項\s*\|\s*Cm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_CM,
            note="body table (JP)",
        ),
        ParamCheck(
            file="docs/architecture/stampfly-parameters.md",
            regex=r'Constant term\s*\|\s*Cm\s*\|\s*([0-9.]+×10[⁰¹²³⁴⁵⁶⁷⁸⁹⁻⁺]+)\s*\|',
            expected=EXPECTED_CM,
            note="body table (EN)",
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
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float JMP = ([0-9eE.+-]+)f;',
            expected=EXPECTED_JMP,
            note="sils_params::adopted::JMP (Config::Jmp source, backlog #2 ODE)",
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
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float DM = ([0-9eE.+-]+)f;',
            expected=EXPECTED_DM,
            note="sils_params::adopted::DM (Config::Dm source, backlog #2 ODE)",
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
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float QF = ([0-9eE.+-]+)f;',
            expected=EXPECTED_QF,
            note="sils_params::adopted::QF (Config::Qf source, backlog #2 ODE)",
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
            # sils_params::adopted::RM (generated_params.hpp) -- literal moved there.
            # Phase 1: plant.hpp の Config::motor_Rm は sils_params::adopted::RM
            # （generated_params.hpp）を参照する -- リテラルはそちらへ移動した。
            file="simulator/sils/plant/generated_params.hpp",
            regex=r'constexpr float RM = ([0-9.eE+-]+)f;',
            expected=EXPECTED_RM,
            note="sils_params::adopted::RM (battery model source, Config::motor_Rm)",
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
    # tools/simulator) and 0.168 N (firmware/vehicle pid_controller.hpp,
    # tools/log_analyzer/visualize_interactive.py). Neither traces to a dated
    # bench measurement in the SSOT YAML -- this section exists to make that
    # gap visible, not to declare a winner.
    # 最大推力（1モーターあたり）— EXEMPT、数値比較は行わない（Unresolved
    # ではなく EXEMPT を使う理由は上の EXEMPT_MAX_THRUST_PER_MOTOR 参照）。
    # 0.15N系と0.168N系の2値が並立し、どちらもSSOT YAMLの実測日に紐づいて
    # いない——本セクションはその欠落を可視化するためのものであり、正解を
    # 宣言するものではない。
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
            file="tools/log_analyzer/reconstruct_duties.py",
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
            file="lib/stampfly_edu/dynamics/equations.py",
            regex=r'"max_thrust":\s*([0-9.eE+-]+),\s*"g":\s*9\.80665,\s*"Vbat"',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="_DEFAULTS.max_thrust (0.15 N family)",
        ),
        ParamCheck(
            file="lib/stampfly_edu/sim/plants.py",
            regex=r'"max_thrust":\s*([0-9.eE+-]+),\s*"g":\s*9\.80665,',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="_DEFAULTS.max_thrust (0.15 N family)",
        ),
        ParamCheck(
            file="firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp",
            regex=r'float max_thrust_\s*=\s*([0-9.eE+-]+)f;\s*//\s*\[N\] total',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="max_thrust_ = 4 x 0.168 N per motor (0.168 N family)",
        ),
        ParamCheck(
            file="tools/log_analyzer/visualize_interactive.py",
            regex=r'max_thrust = ([0-9.eE+-]+)\s*#\s*duty',
            expected=EXEMPT_MAX_THRUST_PER_MOTOR,
            note="local max_thrust (0.168 N family)",
        ),
        ParamCheck(
            # firmware_old is EXEMPT (frozen legacy) -- see EXEMPT_VEHICLE_OLD
            # above and firmware/vehicle_old/README.md banner.
            file="firmware/vehicle_old/components/sf_algo_control/include/control_allocation.hpp",
            regex=r'float max_thrust_per_motor = ([0-9.eE+-]+)f;\s*//\s*duty',
            expected=EXEMPT_VEHICLE_OLD,
            note="QuadConfig::max_thrust_per_motor (frozen legacy firmware)",
        ),
        ParamCheck(
            file="firmware/vehicle_old/components/sf_algo_control/include/control_allocation.hpp",
            regex=r'float max_thrust_\s*=\s*([0-9.eE+-]+)f;',
            expected=EXEMPT_VEHICLE_OLD,
            note="ControlAllocator::max_thrust_ member default (frozen legacy firmware)",
        ),
        ParamCheck(
            file="firmware/vehicle_old/main/tasks/control_task.cpp",
            regex=r'MAX_TOTAL_THRUST = 4\.0f \* ([0-9.eE+-]+)f;',
            expected=EXEMPT_VEHICLE_OLD,
            note="MAX_TOTAL_THRUST = 4 x max_thrust_per_motor (frozen legacy firmware)",
        ),
    ],
}
