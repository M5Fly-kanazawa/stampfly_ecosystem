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

from dataclasses import dataclass, field
from typing import Dict, List, Union


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
# Confirmed measured values (2026-07-15/17 measurement campaign) — the SSOT
# for every "expected" float below. Do not hand-edit a numeric literal into a
# ParamCheck row; add/adjust the constant here instead, with its source.
# 確定済み実測値（2026-07-15/17 測定キャンペーン）— 以下の全"期待値"の正典。
# ParamCheck の行に数値リテラルを直接書き込まず、ここの定数を追加・修正し、
# 出所を添えること。
# =============================================================================
# Thrust coefficient C_T [N/(rad/s)^2], thrust-stand measurement, 2026-07-15.
# 推力係数 [N/(rad/s)^2]、推力測定、2026-07-15。
# Source: docs/architecture/simulation-policy.md backlog #3;
#         docs/architecture/stampfly-parameters.md header note.
EXPECTED_CT = 6.7e-9

# Torque coefficient C_Q [N*m/(rad/s)^2], coast-down measurement, 2026-07-15.
# トルク係数 [N・m/(rad/s)^2]、コーストダウン法、2026-07-15。
EXPECTED_CQ = 4.10e-11

# Torque/thrust ratio kappa = C_Q / C_T [m]. Adopted in the firmware B^-1
# mixer on 2026-07-17 (firmware/vehicle/components/sf_actuator/actuator.cpp:99).
# トルク/推力比 kappa = C_Q/C_T [m]。2026-07-17 にファーム B^-1 ミキサーへ採用済み。
EXPECTED_KAPPA = 6.12e-3

# Rotor inertia J_mp [kg*m^2], photographic + spec-sheet method, 2026-07-15.
# ローター慣性 [kg・m^2]、写真法+諸元法、2026-07-15。
EXPECTED_JMP = 1.375e-8

# Body inertia [kg*m^2] — already consistent everywhere; kept here so this
# manifest is the single place a future re-identification would touch.
# 機体慣性 [kg・m^2] — 全実装で既に一致済み。将来の再同定で触る箇所を
# ここ1箇所に集約しておく。
EXPECTED_IXX = 9.16e-6
EXPECTED_IYY = 13.3e-6
EXPECTED_IZZ = 20.4e-6

# Moment arm (X/Y offset from CG to motor) [m] — already consistent everywhere.
# モーメントアーム（重心→モータの X/Y オフセット）[m] — 全実装で既に一致済み。
EXPECTED_ARM = 0.023

# Winding resistance Rm [Ω] — RESOLVED 2026-07-24 (teacher decision, commit
# 9a656a9f 2026-07-15): adopt the paper's LCR measurement. The other two
# candidates (0.34 = older/possibly-different-unit LCR, 0.63 = vpython's own
# stale initial value) were update gaps, not competing measurements.
# 巻線抵抗 Rm [Ω] — 2026-07-24 決着（先生決定、コミット 9a656a9f 2026-07-15）:
# 論文LCR実測を採用。他の2値（0.34=旧/別個体疑いのLCR、0.63=vpython側の
# 更新漏れの旧初期値）は競合する実測ではなく単なる更新漏れだった。
EXPECTED_RM = 0.593

# Vehicle mass [kg] — RESOLVED 2026-07-24: adopt the measured 36.8g (firmware
# reflected this 2026-03-31, commit 43841314; simulator copies were the update
# gap). See docs/architecture/stampfly-parameters.md §"質量特性".
# 機体質量 [kg] — 2026-07-24 決着: 実測36.8gを採用（ファームは2026-03-31に
# 反映済み、コミット43841314；シミュレータ側が更新漏れだった）。
EXPECTED_MASS = 0.037

# URDF base_link mass [kg] — simulator/shared/assets/meshes/parts/stampfly.urdf
# models the vehicle as base_link (body) + 4 separate propeller links (0.001 kg
# each), so base_link alone is NOT the full vehicle mass. 0.033 + 4*0.001 =
# 0.037 = EXPECTED_MASS. This constant exists only because the manifest checks
# one regex-captured number per row; it is not an independent measurement.
# URDF base_link 質量 [kg] — stampfly.urdf は機体を base_link（本体）＋
# プロペラ4リンク（各0.001kg）で表現するため、base_link 単体は機体全質量では
# ない。0.033 + 4*0.001 = 0.037 = EXPECTED_MASS。この定数はマニフェストが
# 1行1数値しか検査できないために存在するだけで、独立した実測値ではない。
EXPECTED_URDF_BASE_MASS = 0.033

# --- EXEMPT markers (known-and-accepted mismatch, with reason) ---
# --- EXEMPT マーカー（既知で許容された不一致、理由付き） ---
EXEMPT_PLANT_CT = Exempt(
    reason="simulator/sil/plant/plant.hpp の TODO(2026-07-15): Ct・Am(∝Cq)・"
           "thrust_efficiency は Model Identity（飛行ログ較正）で連動しており、"
           "単独差し替えはホバ推力を壊す。更新は3点セット "
           "(Am_new=Rm·Cq_new/Km=2.28e-8, Ct_new=6.7e-9, thrust_efficiency再フィット) "
           "とセットで実施すること — docs/architecture/simulation-policy.md backlog #3。"
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
            file="tools/sysid/defaults.py",
            regex=r'_CT_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CT,
            note="module constant _CT_VALUE",
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
            file="simulator/sil/plant/plant.hpp",
            regex=r'float Ct\s*=\s*([0-9eE.+-]+)f;',
            expected=EXEMPT_PLANT_CT,
            note="Config::Ct (deferred 3-point set)",
        ),
    ],

    # -------------------------------------------------------------------
    # C_Q — torque coefficient / トルク係数
    # -------------------------------------------------------------------
    "C_Q": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'_CQ_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_CQ,
            note="module constant _CQ_VALUE",
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
        # NOTE: simulator/sil/plant/plant.hpp has no standalone Cq constant —
        # its torque model is kappa*T, not Cq*omega^2 — so it is intentionally
        # NOT in this list (task spec: "無ければ対象外").
        # plant.hpp は独立した Cq 定数を持たない（反トルクは kappa*T で計算、
        # Cq*omega^2 ではない）ため意図的に対象外。
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
            file="simulator/sil/plant/plant.hpp",
            regex=r'float kappa\s*=\s*([0-9eE.+-]+)f;',
            expected=EXPECTED_KAPPA,
            note="Config::kappa (independent of the deferred Ct set)",
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
    ],

    # -------------------------------------------------------------------
    # J_mp — rotor inertia / ローター慣性モーメント
    # -------------------------------------------------------------------
    "J_mp": [
        ParamCheck(
            file="tools/sysid/defaults.py",
            regex=r'_JMP_VALUE\s*=\s*([0-9eE.+-]+)',
            expected=EXPECTED_JMP,
            note="module constant _JMP_VALUE",
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
            file="tools/sysid/defaults.py",
            regex=r'_RM_VALUE\s*=\s*([0-9.eE+-]+)',
            expected=EXPECTED_RM,
            note="module constant _RM_VALUE",
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
            file="simulator/sil/plant/plant.hpp",
            regex=r'motor_Rm\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_RM,
            note="Config::motor_Rm (battery model)",
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
    ],

    # -------------------------------------------------------------------
    # mass — vehicle mass (RESOLVED 2026-07-24: 0.037, see EXPECTED_MASS)
    # 機体質量（2026-07-24決着: 0.037、EXPECTED_MASS 参照）
    # -------------------------------------------------------------------
    "mass": [
        ParamCheck(
            file="simulator/sil/plant/plant.hpp",
            regex=r'float mass\s*=\s*([0-9.eE+-]+)f;',
            expected=EXPECTED_MASS,
            note="Config::mass (battery model)",
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
    ],
}
