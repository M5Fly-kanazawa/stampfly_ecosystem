#!/usr/bin/env python3
"""
check_messages.py - Cross-check protocol/spec/messages.yaml (the ESP-NOW
wire-format SSOT) against its hand-written C++ implementation in
firmware/common/protocol/include/espnow_protocol.hpp.

check_messages.py - protocol/spec/messages.yaml（ESP-NOW 電文形式の
唯一の正 = SSOT）と、その手書き C++ 実装
firmware/common/protocol/include/espnow_protocol.hpp を突き合わせて検査する。

WHAT IS COMPARED / 何を比較するか
----------------------------------
`espnow_protocol.hpp` only implements two of the five messages declared in
`messages.yaml` -- `ControlPacket` as a full packed struct, and
`PairingPacket` as two standalone constants (`kPairingPacketSize`,
`kPairingSignature`) rather than a struct. This matches the header's own
docstring, which states it covers only "the primary ESP-NOW + TDMA link
between Controller and Vehicle". Accordingly:

`espnow_protocol.hpp` は messages.yaml が定義する5メッセージのうち2つしか
実装していない -- `ControlPacket` は完全な packed struct として、
`PairingPacket` は struct ではなく単体の定数（`kPairingPacketSize`、
`kPairingSignature`）としてのみ実装されている。これはヘッダ自身の
docstring（"Controller-Vehicle間の主系統ESP-NOW+TDMAリンク" のみを扱うと
明記）と整合する。よって:

  * ControlPacket (full struct in header):
      - total size: messages.yaml `size:` vs the header's
        `static_assert(sizeof(ControlPacket) == N)` AND vs the size
        computed by summing each struct member's C type size in
        declaration order (the struct is `__attribute__((packed))`, so
        there is no compiler-inserted padding to account for).
      - per field, in declaration order: name, C type (derived from the
        yaml field's `type:`, resolving custom `types:` entries such as
        `mac_address` -> `bytes` first), byte size, and cumulative byte
        offset (derived the same packed-struct way) vs the yaml field's
        `offset:`.
      - the `flags` bitfield: each yaml `bits:` entry (`name`, `bit`) vs
        the header's `CTRL_FLAG_<NAME>` constants (expected value
        `1 << bit`). This also flags a constant that exists in the header
        but has no corresponding yaml bit (or vice versa).

  * ControlPacket (ヘッダに完全な struct あり):
      - 総サイズ: messages.yaml の `size:` と、ヘッダの
        `static_assert(sizeof(ControlPacket) == N)`、および struct の各
        メンバーの C 型サイズを宣言順に積算して求めたサイズを突き合わせる
        （`__attribute__((packed))` のためコンパイラが挿入するパディングは
        無い）。
      - 各フィールド（宣言順）: 名前、C型（yaml の `type:` から解決。
        `mac_address` 等のカスタム `types:` は先に `bytes` 等へ解決する）、
        バイトサイズ、累積バイトオフセット（同様に packed 前提で算出）を
        yaml の `offset:` と突き合わせる。
      - `flags` ビットフィールド: yaml の各 `bits:` エントリ（`name`,
        `bit`）とヘッダの `CTRL_FLAG_<NAME>` 定数（期待値 `1 << bit`）を
        突き合わせる。ヘッダにだけ存在し yaml に無いビット（逆も同様）も
        検出する。

  * PairingPacket (constants only in header, no struct):
      - total size: yaml `size:` vs `kPairingPacketSize`.
      - the `signature` field's yaml `value:` byte list vs the
        `kPairingSignature` array (order-sensitive, element by element).
      - NOT compared: per-field name/order/offset for `channel` and
        `drone_mac`, because the header defines no struct for
        PairingPacket -- there is nothing on the C++ side to check field
        layout against. This is a deliberate scope decision, not an
        oversight.

  * PairingPacket（ヘッダには定数のみ、struct 無し）:
      - 総サイズ: yaml の `size:` と `kPairingPacketSize`。
      - `signature` フィールドの yaml `value:` バイト列と
        `kPairingSignature` 配列（順序込みで要素ごとに比較）。
      - 比較しない: `channel` と `drone_mac` のフィールド名・順序・
        オフセット。ヘッダに PairingPacket 用の struct が無く、C++側に
        比較対象のフィールドレイアウトが存在しないため。意図的なスコープ
        判断であり、見落としではない。

WHAT IS NOT COMPARED, AND WHY / 比較しない項目とその理由
--------------------------------------------------------
  * TelemetryPacket, TelemetryWSPacket, TDMABeacon: not implemented in
    `espnow_protocol.hpp` at all (confirmed: no struct, no matching
    constants). Per the header's own docstring this is out of scope for
    this file -- TelemetryPacket has independent, non-shared definitions
    in firmware/vehicle/.../telemetry.hpp, firmware/vehicle_old/...
    (controller_comm.hpp / telemetry.hpp) and
    firmware/common/protocol/include/udp_protocol.hpp, and TDMABeacon has
    no C++ byte-layout definition anywhere in the tree. These messages are
    reported as "skipped" (not a mismatch) rather than silently ignored.
  * TelemetryPacket・TelemetryWSPacket・TDMABeacon: `espnow_protocol.hpp`
    に一切実装が無い（struct も対応する定数も無いことを確認済み）。ヘッダ
    自身の docstring からもこのファイルの対象外である --
    TelemetryPacket は firmware/vehicle/.../telemetry.hpp や
    firmware/vehicle_old/...（controller_comm.hpp / telemetry.hpp）、
    firmware/common/protocol/include/udp_protocol.hpp に、共有されない
    個別の定義を持つ。TDMABeacon はツリー中どこにも C++ のバイトレイアウト
    定義が無い。これらは（無言で無視するのではなく）「スキップ」として
    明示的に報告する。
  * `description` / `transport` / `rate_hz` / `unit` / `frame` / `range`
    / `enum` (TelemetryPacket's `state` field): documentation-only or
    informational metadata with no corresponding C++ symbol to check
    against; comparing them would just be comparing yaml prose to itself.
  * `description`・`transport`・`rate_hz`・`unit`・`frame`・`range`・
    `enum`（TelemetryPacket の `state` フィールド）: ドキュメント用途・
    参考情報であり、突き合わせるべき C++ 側のシンボルが存在しない。
    比較しても yaml の説明文を自分自身と比べるだけになる。

HOW THE HEADER IS PARSED / ヘッダの解析方法
--------------------------------------------
A full C++ parser is not needed for a header this small and regular:
comments are stripped first, then `struct X { ... } __attribute__((packed));`
blocks, `static_assert(sizeof(X) == N, ...)` guards, and top-level
`constexpr <type> <name> [= value];` / `constexpr <type> <name>[N] = {...};`
declarations are located with regular expressions. Struct member
declarations (including array members like `uint8_t x[3];`) are parsed one
statement at a time.
このサイズ・書式のヘッダには本格的な C++ パーサは不要: まずコメントを除去し、
`struct X { ... } __attribute__((packed));` ブロック、
`static_assert(sizeof(X) == N, ...)` ガード、トップレベルの
`constexpr <型> <名前> [= 値];` / `constexpr <型> <名前>[N] = {...};` 宣言を
正規表現で見つける。struct メンバー宣言（`uint8_t x[3];` のような配列を
含む）は1文ずつ解析する。

Exit codes / 終了コード:
  0  matched -- no mismatch found (may still report skipped messages)
     一致 -- 不一致なし（スキップしたメッセージがあっても0）
  1  mismatch found between messages.yaml and espnow_protocol.hpp
     messages.yaml と espnow_protocol.hpp の間に不一致あり
  2  could not parse messages.yaml or espnow_protocol.hpp (unexpected
     structure), or another setup error
     messages.yaml / espnow_protocol.hpp の構造が想定外でパース不能、
     その他のセットアップ異常
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

# =============================================================================
# Paths
# パス
# =============================================================================

REPO_ROOT = Path(__file__).resolve().parents[2]
YAML_PATH = REPO_ROOT / "protocol" / "spec" / "messages.yaml"
HEADER_PATH = REPO_ROOT / "firmware" / "common" / "protocol" / "include" / "espnow_protocol.hpp"

# =============================================================================
# Type tables
# 型テーブル
# =============================================================================

# C type name -> size in bytes (packed layout: no alignment padding).
# C 型名 -> バイトサイズ（packed レイアウトのためアライメントパディング無し）。
C_TYPE_SIZES: dict[str, int] = {
    "uint8_t": 1,
    "int8_t": 1,
    "uint16_t": 2,
    "int16_t": 2,
    "uint32_t": 4,
    "int32_t": 4,
    "uint64_t": 8,
    "int64_t": 8,
    "float": 4,
    "double": 8,
}

# yaml base type name -> expected C type. "bytes" fields map to an array of
# uint8_t and are handled separately (they always carry an explicit length).
# yaml の基底型名 -> 期待される C 型。"bytes" 型は uint8_t の配列にマップされ、
# 別途処理する（常に明示的な長さを伴う）。
YAML_TO_C_TYPE: dict[str, str] = {
    "uint8": "uint8_t",
    "int8": "int8_t",
    "uint16": "uint16_t",
    "int16": "int16_t",
    "uint32": "uint32_t",
    "int32": "int32_t",
    "float32": "float",
}


class CheckError(Exception):
    """Raised when messages.yaml or espnow_protocol.hpp has an unexpected
    structure that this script does not know how to compare.
    messages.yaml / espnow_protocol.hpp が本スクリプトの想定外の構造を
    持っていた場合に送出する。
    """


# =============================================================================
# Data classes
# データクラス
# =============================================================================


@dataclass
class ResolvedYamlField:
    """One messages.yaml field, resolved to a concrete C type/size/offset.
    messages.yaml のフィールドを具体的な C型・サイズ・オフセットへ解決したもの。
    """

    name: str
    offset: int
    c_type: str
    size: int
    array_len: int | None  # None for scalar fields / スカラーなら None


@dataclass
class HeaderField:
    """One struct member parsed from espnow_protocol.hpp.
    espnow_protocol.hpp から解析した struct メンバー1個。
    """

    name: str
    c_type: str
    offset: int
    size: int
    array_len: int | None


@dataclass
class HeaderStruct:
    """One `struct ... {} __attribute__((packed));` block.
    `struct ... {} __attribute__((packed));` ブロック1個。
    """

    name: str
    fields: list[HeaderField]
    computed_size: int  # sum of field sizes, packed layout / 積算サイズ
    asserted_size: int | None = None  # from static_assert, if present / static_assert 由来


@dataclass
class HeaderConstant:
    """One top-level `constexpr` declaration (scalar or array).
    トップレベルの `constexpr` 宣言1個（スカラーまたは配列）。
    """

    name: str
    c_type: str
    value: Any  # int for scalar, list[int] for array
    is_array: bool


@dataclass
class Mismatch:
    """One reportable disagreement between messages.yaml and the header.
    messages.yaml とヘッダの間の1件の不一致。
    """

    message: str
    field_name: str | None
    expected: Any  # yaml side / yaml 側
    actual: Any  # header side / ヘッダ側
    detail: str = ""

    def format(self) -> str:
        loc = self.message if self.field_name is None else f"{self.message}.{self.field_name}"
        line = f"  {loc}: expected(yaml)={self.expected!r} actual(header)={self.actual!r}"
        if self.detail:
            line += f"  -- {self.detail}"
        return line


# =============================================================================
# messages.yaml parsing
# messages.yaml の解析
# =============================================================================


def load_yaml_spec() -> dict:
    """Parse protocol/spec/messages.yaml.
    protocol/spec/messages.yaml を読み込みパースする。
    """
    with open(YAML_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_yaml_field(fld: dict, custom_types: dict, msg_name: str) -> ResolvedYamlField:
    """Resolve one yaml field dict to a concrete C type, byte size, and
    (for byte-array fields) element count.
    yaml のフィールド dict1個を、具体的な C型・バイトサイズ・（バイト配列
    フィールドなら）要素数へ解決する。
    """
    name = fld["name"]
    type_name = fld["type"]
    explicit_size = fld.get("size")

    if type_name in custom_types:
        base = custom_types[type_name].get("base")
        base_declared_size = custom_types[type_name].get("size")
    else:
        base = type_name
        base_declared_size = None

    if base == "bytes":
        length = explicit_size if explicit_size is not None else base_declared_size
        if length is None:
            raise CheckError(
                f"{msg_name}.{name}: 'bytes' field has no size (neither the field nor "
                f"its referenced type declares one)"
            )
        return ResolvedYamlField(name=name, offset=fld["offset"], c_type="uint8_t", size=length, array_len=length)

    c_type = YAML_TO_C_TYPE.get(base)
    if c_type is None:
        raise CheckError(
            f"{msg_name}.{name}: unknown yaml base type {base!r} "
            f"(extend YAML_TO_C_TYPE in check_messages.py if this is intentional)"
        )
    return ResolvedYamlField(
        name=name, offset=fld["offset"], c_type=c_type, size=C_TYPE_SIZES[c_type], array_len=None
    )


# =============================================================================
# espnow_protocol.hpp parsing
# espnow_protocol.hpp の解析
# =============================================================================

STRUCT_RE = re.compile(r"struct\s+(\w+)\s*\{(.*?)\}\s*__attribute__\(\(packed\)\)\s*;", re.DOTALL)
FIELD_RE = re.compile(r"^([A-Za-z_]\w*)\s+([A-Za-z_]\w*)\s*(\[\s*(\d+)\s*\])?$")
ASSERT_RE = re.compile(r"static_assert\s*\(\s*sizeof\s*\(\s*(\w+)\s*\)\s*==\s*(\d+)")
CONST_ARRAY_RE = re.compile(r"constexpr\s+([A-Za-z_]\w*)\s+([A-Za-z_]\w*)\s*\[\s*\d+\s*\]\s*=\s*\{([^}]*)\}\s*;")
CONST_SCALAR_RE = re.compile(r"constexpr\s+([A-Za-z_]\w*)\s+([A-Za-z_]\w*)\s*=\s*([^;{]+);")


def strip_comments(text: str) -> str:
    """Remove // line comments and /* ... */ block comments so the regexes
    below never accidentally match text inside a comment.
    // 行コメントと /* ... */ ブロックコメントを除去し、以下の正規表現が
    コメント内の文字列に誤って一致しないようにする。
    """
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def parse_structs(text: str) -> dict[str, HeaderStruct]:
    """Find every packed struct and parse its members in declaration order,
    deriving cumulative byte offsets (packed => no padding between members).
    packed struct を全て見つけ、宣言順にメンバーを解析し、累積バイト
    オフセットを導出する（packed のためメンバー間にパディングは無い）。
    """
    structs: dict[str, HeaderStruct] = {}
    for m in STRUCT_RE.finditer(text):
        name, body = m.group(1), m.group(2)
        fields: list[HeaderField] = []
        offset = 0
        for stmt in body.split(";"):
            stmt = stmt.strip()
            if not stmt:
                continue
            fm = FIELD_RE.match(stmt)
            if not fm:
                raise CheckError(f"struct {name}: could not parse member declaration: {stmt!r}")
            c_type, member_name, _, array_len_str = fm.groups()
            if c_type not in C_TYPE_SIZES:
                raise CheckError(
                    f"struct {name}.{member_name}: unknown C type {c_type!r} "
                    f"(extend C_TYPE_SIZES in check_messages.py if this is intentional)"
                )
            base_size = C_TYPE_SIZES[c_type]
            array_len = int(array_len_str) if array_len_str else None
            size = base_size * (array_len if array_len is not None else 1)
            fields.append(
                HeaderField(name=member_name, c_type=c_type, offset=offset, size=size, array_len=array_len)
            )
            offset += size
        structs[name] = HeaderStruct(name=name, fields=fields, computed_size=offset)
    return structs


def parse_static_asserts(text: str) -> dict[str, int]:
    """Map struct name -> the size asserted by static_assert(sizeof(X)==N).
    struct 名 -> static_assert(sizeof(X)==N) が主張するサイズ、の対応表。
    """
    return {m.group(1): int(m.group(2)) for m in ASSERT_RE.finditer(text)}


def parse_constants(text: str) -> dict[str, HeaderConstant]:
    """Parse top-level `constexpr` scalar and array declarations.
    トップレベルの `constexpr` スカラー・配列宣言を解析する。
    """
    consts: dict[str, HeaderConstant] = {}
    for m in CONST_ARRAY_RE.finditer(text):
        c_type, name, values_str = m.groups()
        values = [int(v.strip(), 0) for v in values_str.split(",") if v.strip()]
        consts[name] = HeaderConstant(name=name, c_type=c_type, value=values, is_array=True)
    for m in CONST_SCALAR_RE.finditer(text):
        c_type, name, value_str = m.groups()
        if name in consts:
            # Already captured as an array (array regex runs first); a bare
            # scalar match on the same name should not happen, but skip it
            # defensively rather than overwrite a good result.
            # 既に配列として取得済み（配列用正規表現を先に実行しているため）。
            # 同名でスカラー一致が起きることは通常無いが、念のため上書きせず
            # スキップする。
            continue
        try:
            value = int(value_str.strip(), 0)
        except ValueError:
            continue  # Not an integer literal (e.g. an expression) -- not needed for our checks.
        consts[name] = HeaderConstant(name=name, c_type=c_type, value=value, is_array=False)
    return consts


# =============================================================================
# Comparisons
# 比較処理
# =============================================================================


def compare_full_struct(
    msg_name: str,
    yaml_fields: list[ResolvedYamlField],
    yaml_size: int,
    header_struct: HeaderStruct,
    mismatches: list[Mismatch],
) -> None:
    """Compare a yaml message that has a matching full C++ struct: total
    size (both the static_assert and the size computed from the struct's
    own members) and each field's name/type/size/offset in order.
    対応する完全な C++ struct を持つ yaml メッセージを比較する: 総サイズ
    （static_assert と、struct 自身のメンバーから計算したサイズの両方）と、
    各フィールドの名前・型・サイズ・オフセットを順に比較する。
    """
    if header_struct.asserted_size is None:
        mismatches.append(
            Mismatch(
                msg_name,
                "size (static_assert)",
                yaml_size,
                None,
                "no static_assert(sizeof(...)) guard found for this struct in the header",
            )
        )
    elif yaml_size != header_struct.asserted_size:
        mismatches.append(Mismatch(msg_name, "size (static_assert)", yaml_size, header_struct.asserted_size))

    if yaml_size != header_struct.computed_size:
        mismatches.append(
            Mismatch(
                msg_name,
                "size (sum of struct members)",
                yaml_size,
                header_struct.computed_size,
                "computed by summing each struct member's C type size in declaration order",
            )
        )

    max_len = max(len(yaml_fields), len(header_struct.fields))
    for i in range(max_len):
        yf = yaml_fields[i] if i < len(yaml_fields) else None
        hf = header_struct.fields[i] if i < len(header_struct.fields) else None

        if yf is None:
            mismatches.append(
                Mismatch(msg_name, f"field[{i}]", "<no field>", hf.name, "extra struct member not in messages.yaml")
            )
            continue
        if hf is None:
            mismatches.append(
                Mismatch(msg_name, f"field[{i}]", yf.name, "<no field>", "yaml field missing from header struct")
            )
            continue

        label = f"{yf.name}" if yf.name == hf.name else f"field[{i}] ({yf.name} vs {hf.name})"
        if yf.name != hf.name:
            mismatches.append(Mismatch(msg_name, f"{label}.name", yf.name, hf.name))
        if yf.c_type != hf.c_type:
            mismatches.append(Mismatch(msg_name, f"{label}.type", yf.c_type, hf.c_type))
        if yf.offset != hf.offset:
            mismatches.append(Mismatch(msg_name, f"{label}.offset", yf.offset, hf.offset))
        if yf.size != hf.size:
            mismatches.append(Mismatch(msg_name, f"{label}.size", yf.size, hf.size))
        if yf.array_len != hf.array_len:
            mismatches.append(Mismatch(msg_name, f"{label}.array_len", yf.array_len, hf.array_len))


def compare_flag_bits(
    msg_name: str,
    field_name: str,
    bits: list[dict],
    header_constants: dict[str, HeaderConstant],
    prefix: str,
    mismatches: list[Mismatch],
) -> None:
    """Compare a yaml `bits:` list (bitfield flag names) against header
    `constexpr uint8_t <prefix><NAME> = ...;` constants, expecting each
    constant's value to equal `1 << bit`.
    yaml の `bits:` リスト（ビットフィールドのフラグ名）と、ヘッダの
    `constexpr uint8_t <prefix><NAME> = ...;` 定数を比較する。各定数の値は
    `1 << bit` であることを期待する。
    """
    yaml_bits = {b["name"]: b["bit"] for b in bits}
    header_flags = {
        name[len(prefix) :]: c.value
        for name, c in header_constants.items()
        if name.startswith(prefix) and not c.is_array
    }

    for name in sorted(set(yaml_bits) | set(header_flags)):
        in_yaml = name in yaml_bits
        in_header = name in header_flags
        if in_yaml and in_header:
            expected = 1 << yaml_bits[name]
            actual = header_flags[name]
            if expected != actual:
                mismatches.append(
                    Mismatch(msg_name, f"{field_name}.{name}", f"0x{expected:02X}", f"0x{actual:02X}")
                )
        elif in_yaml and not in_header:
            expected = 1 << yaml_bits[name]
            mismatches.append(
                Mismatch(
                    msg_name,
                    f"{field_name}.{name}",
                    f"bit {yaml_bits[name]} (0x{expected:02X})",
                    None,
                    f"no matching header constant {prefix}{name}",
                )
            )
        else:  # in_header and not in_yaml
            mismatches.append(
                Mismatch(
                    msg_name,
                    f"{field_name}.{name}",
                    None,
                    f"{prefix}{name} = 0x{header_flags[name]:02X}",
                    "constant exists in header but this bit is not documented in messages.yaml bits:",
                )
            )


def compare_pairing_packet(yaml_msg: dict, header_constants: dict[str, HeaderConstant], mismatches: list[Mismatch]) -> None:
    """Compare the parts of PairingPacket that the header actually
    implements: total size (`kPairingPacketSize`) and the signature byte
    constant (`kPairingSignature`). See module docstring for what is
    deliberately not compared and why.
    PairingPacket のうちヘッダが実際に実装している部分（総サイズ
    `kPairingPacketSize` と signature 定数 `kPairingSignature`）のみを
    比較する。意図的に比較しない項目とその理由はモジュール docstring 参照。
    """
    msg_name = "PairingPacket"
    yaml_size = yaml_msg["size"]

    size_const = header_constants.get("kPairingPacketSize")
    if size_const is None or size_const.is_array:
        mismatches.append(Mismatch(msg_name, "size", yaml_size, None, "constant kPairingPacketSize not found in header"))
    elif size_const.value != yaml_size:
        mismatches.append(Mismatch(msg_name, "size", yaml_size, size_const.value, "kPairingPacketSize"))

    sig_field = next((f for f in yaml_msg["fields"] if f["name"] == "signature"), None)
    if sig_field is None:
        raise CheckError("PairingPacket.signature field not found in messages.yaml")
    yaml_sig = sig_field.get("value")

    sig_const = header_constants.get("kPairingSignature")
    if sig_const is None or not sig_const.is_array:
        mismatches.append(
            Mismatch(msg_name, "signature", yaml_sig, None, "constant kPairingSignature not found in header")
        )
    elif sig_const.value != yaml_sig:
        mismatches.append(Mismatch(msg_name, "signature", yaml_sig, sig_const.value, "kPairingSignature"))


# =============================================================================
# Main
# メイン処理
# =============================================================================


def run_check() -> tuple[list[Mismatch], list[str], list[str]]:
    """Run every comparison and return (mismatches, compared_summaries,
    skipped_message_names). Raises CheckError on unexpected structure.
    全ての比較を実行し、(不一致リスト, 比較済みの要約リスト,
    スキップしたメッセージ名リスト) を返す。想定外の構造なら CheckError。
    """
    spec = load_yaml_spec()
    if "messages" not in spec:
        raise CheckError(f"{YAML_PATH}: no top-level 'messages:' key")
    messages: dict = spec["messages"]
    custom_types: dict = spec.get("types", {})

    header_text_raw = HEADER_PATH.read_text(encoding="utf-8")
    header_text = strip_comments(header_text_raw)
    structs = parse_structs(header_text)
    asserts = parse_static_asserts(header_text)
    constants = parse_constants(header_text)
    for name, st in structs.items():
        st.asserted_size = asserts.get(name)

    mismatches: list[Mismatch] = []
    compared: list[str] = []
    skipped: list[str] = []

    # ---- ControlPacket: full struct + flag bits ----
    if "ControlPacket" in messages:
        cp = messages["ControlPacket"]
        yaml_fields = [resolve_yaml_field(f, custom_types, "ControlPacket") for f in cp["fields"]]
        header_struct = structs.get("ControlPacket")
        if header_struct is None:
            mismatches.append(Mismatch("ControlPacket", None, "struct ControlPacket {...}", None, "no matching struct found in header"))
        else:
            compare_full_struct("ControlPacket", yaml_fields, cp["size"], header_struct, mismatches)

        flags_field = next((f for f in cp["fields"] if f["name"] == "flags"), None)
        if flags_field is not None and "bits" in flags_field:
            compare_flag_bits("ControlPacket", "flags", flags_field["bits"], constants, "CTRL_FLAG_", mismatches)
        compared.append(f"ControlPacket ({len(cp['fields'])} fields, total size, flags bitfield)")
    else:
        raise CheckError(f"{YAML_PATH}: 'ControlPacket' message not found under messages:")

    # ---- PairingPacket: size + signature constant only ----
    if "PairingPacket" in messages:
        compare_pairing_packet(messages["PairingPacket"], constants, mismatches)
        compared.append("PairingPacket (size, signature constant)")
    else:
        raise CheckError(f"{YAML_PATH}: 'PairingPacket' message not found under messages:")

    # ---- Messages with no C++ representation in this header: skip, don't fail ----
    for name in ("TelemetryPacket", "TelemetryWSPacket", "TDMABeacon"):
        if name in messages:
            skipped.append(name)

    return mismatches, compared, skipped


def main() -> int:
    try:
        mismatches, compared, skipped = run_check()
    except CheckError as exc:
        print(f"check_messages.py: ERROR: {exc}", file=sys.stderr)
        return 2
    except (OSError, yaml.YAMLError) as exc:
        print(f"check_messages.py: ERROR reading/parsing input files: {exc}", file=sys.stderr)
        return 2
    except (KeyError, TypeError, ValueError) as exc:
        print(f"check_messages.py: ERROR: unexpected structure in messages.yaml or the header: {exc!r}", file=sys.stderr)
        return 2

    skip_note = f" | skipped (no C++ representation in this header): {', '.join(skipped)}" if skipped else ""

    if mismatches:
        print(
            "check_messages.py: MISMATCH between protocol/spec/messages.yaml (expected) "
            "and firmware/common/protocol/include/espnow_protocol.hpp (actual):"
        )
        for m in mismatches:
            print(m.format())
        if skipped:
            print(f"  (also skipped, no C++ representation in this header: {', '.join(skipped)})")
        return 1

    print(f"check_messages.py: OK ({'; '.join(compared)}{skip_note})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
