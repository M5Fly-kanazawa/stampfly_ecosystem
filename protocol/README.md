# protocol

通信プロトコルの仕様定義（Single Source of Truth）。

## ディレクトリ構成

- `spec/` - 機械可読なプロトコル仕様（YAML, proto 等）
- `generated/` - 仕様から生成されたコード（教育用途ではコミット可）
- `tools/` - 仕様検証、コード生成ツール

## 設計原則

エコシステム内の全ての通信実装はこの仕様から派生する。
firmware/common/protocol はここで定義された仕様の組込み向け実装である。

---

# protocol

Communication protocol specification (Single Source of Truth).

## Directory Structure

- `spec/` - Machine-readable protocol specification (YAML, proto, etc.)
- `generated/` - Code generated from specification (commit allowed for educational purposes)
- `tools/` - Specification validation and code generation tools

## Design Principle

All communication implementations in the ecosystem derive from this specification.
firmware/common/protocol is the embedded implementation of the specification defined here.
